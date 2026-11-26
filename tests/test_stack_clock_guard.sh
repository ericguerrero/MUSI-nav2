#!/bin/bash
# The robot stack must never run while the wall clock can jump.
#
# On 2026-11-14 the ROSbot's Raspberry Pi lost power about 30 s after NTP
# stepped the clock by 20 hours with the driver running (2 of 2 trials), and
# did not when the driver was stopped. robot/stack.sh is the guard: it syncs
# the clock only while the driver is down and leaves NTP off afterwards.
#
# This test runs stack.sh against stand-in commands that record every call, so
# it checks the order of operations without a robot. Runs on the host.
#
# NTP is switched with `timedatectl set-ntp`, never by naming a service: the
# robot's image masks systemd-timesyncd and runs chrony, and timedatectl
# drives whichever one is installed.
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STACK="${REPO}/robot/stack.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT
PASS=0
FAIL=0

# Stand-ins. Each appends its own name and arguments to calls.log.
mkdir -p "${WORK}/bin"
printf '#!/bin/bash\necho "nmcli $*" >> "%s/calls.log"\n' "${WORK}" > "${WORK}/bin/nmcli"
# sudo records that a password would have been asked for, then runs the command.
printf '#!/bin/bash\necho "sudo $*" >> "%s/calls.log"\nexec "$@"\n' "${WORK}" > "${WORK}/bin/sudo"
# timedatectl remembers whether NTP is on (NTP_ACTIVE=1: on at the start). The
# clock counts as synchronised once NTP has been switched on in this run and
# SYNC_AFTER polls have passed; SYNC_AFTER=never is a network with no time source.
cat > "${WORK}/bin/timedatectl" <<EOF
#!/bin/bash
state="${WORK}/ntp_state"
[ -f "\${state}" ] || { [ "\${NTP_ACTIVE}" = "1" ] && echo on > "\${state}" || echo off > "\${state}"; }
case "\$*" in
    "set-ntp true") echo on > "\${state}"; touch "${WORK}/ntp_was_started"
                    echo "timedatectl \$*" >> "${WORK}/calls.log" ;;
    "set-ntp false") echo off > "\${state}"; echo "timedatectl \$*" >> "${WORK}/calls.log" ;;
    "show -p NTP --value") [ "\$(cat "\${state}")" = "on" ] && echo yes || echo no ;;
    "show -p NTPSynchronized --value")
        count=\$(cat "${WORK}/polls" 2>/dev/null || echo 0)
        echo \$((count + 1)) > "${WORK}/polls"
        if [ "\${SYNC_AFTER}" != "never" ] && [ -f "${WORK}/ntp_was_started" ] \
            && [ "\${count}" -ge "\${SYNC_AFTER}" ]; then echo yes; else echo no; fi ;;
    *) echo "timedatectl stand-in: unexpected arguments: \$*" >&2; exit 64 ;;
esac
EOF
# docker reports the driver as running when DRIVER_UP=1.
cat > "${WORK}/bin/docker" <<EOF
#!/bin/bash
echo "docker \$*" >> "${WORK}/calls.log"
if [ "\$1" = "ps" ] && [ "\${DRIVER_UP}" = "1" ]; then echo abc123; fi
EOF
chmod +x "${WORK}"/bin/*

run_stack() {  # environment assignments, then stack.sh arguments after --
    local env=()
    while [ "$1" != "--" ]; do env+=("$1"); shift; done
    shift
    rm -f "${WORK}/calls.log" "${WORK}/polls" "${WORK}/ntp_state" "${WORK}/ntp_was_started"
    env PATH="${WORK}/bin:${PATH}" CLOCK_WAIT=3 CLOCK_POLL=1 NTP_ACTIVE=0 "${env[@]}" \
        bash "${STACK}" "$@" > "${WORK}/out.txt" 2>&1
    echo $? > "${WORK}/rc"
}

line_of() { grep -n -m1 -- "$1" "${WORK}/calls.log" 2>/dev/null | cut -d: -f1; }

check() {  # label, command...
    local label="$1"; shift
    if "$@"; then echo "  PASS: ${label}"; PASS=$((PASS + 1))
    else echo "  FAIL: ${label}"; FAIL=$((FAIL + 1)); fi
}

called() { grep -q -- "$1" "${WORK}/calls.log" 2>/dev/null; }
not_called() { ! called "$1"; }
before() { local a b; a=$(line_of "$1"); b=$(line_of "$2"); [ -n "$a" ] && [ -n "$b" ] && [ "$a" -lt "$b" ]; }
said() { grep -q -- "$1" "${WORK}/out.txt"; }
not_said() { ! said "$1"; }
exit_code_is() { [ "$(cat "${WORK}/rc")" = "$1" ]; }
sudo_calls_are() { [ "$(grep -c '^sudo ' "${WORK}/calls.log" 2>/dev/null)" = "$1" ]; }

echo "driver down, time source reachable:"
run_stack SYNC_AFTER=1 DRIVER_UP=0 -- up
# A clock step makes sudo forget the password it was just given. Seen on the
# robot: a second sudo after the step sat at a prompt nobody was watching, with
# NTP still on. So the whole clock step runs under one sudo.
check "asks for the password once" sudo_calls_are 1
check "switches NTP on to set the clock" called "timedatectl set-ntp true"
check "switches NTP off again" called "timedatectl set-ntp false"
check "switches NTP off before the driver starts" before "timedatectl set-ntp false" "docker compose up -d"
check "starts the stack" called "docker compose up -d"
check "says the clock is synchronised" said "clock synchronised"
check "uses no command the stand-ins do not know" not_said "unexpected arguments"
check "exits 0" exit_code_is 0

echo "driver down, no time source:"
run_stack SYNC_AFTER=never DRIVER_UP=0 -- up
check "asks for the password once" sudo_calls_are 1
check "still switches NTP off so the clock cannot step later" called "timedatectl set-ntp false"
check "switches NTP off before the driver starts" before "timedatectl set-ntp false" "docker compose up -d"
check "starts the stack anyway" called "docker compose up -d"
check "warns that the clock is not synchronised" said "NOT synchronised"
check "exits 0" exit_code_is 0

echo "driver already running, NTP found on:"
run_stack SYNC_AFTER=1 DRIVER_UP=1 NTP_ACTIVE=1 -- up
check "never switches NTP on" not_called "timedatectl set-ntp true"
check "never changes the network" not_called "nmcli"
check "switches NTP off" called "timedatectl set-ntp false"
check "still applies the compose file" called "docker compose up -d"

echo "driver already running, NTP already off:"
run_stack SYNC_AFTER=1 DRIVER_UP=1 NTP_ACTIVE=0 -- up
check "asks for no password at all" not_called "sudo"
check "still applies the compose file" called "docker compose up -d"

echo "clock step asked for directly while the driver runs:"
run_stack SYNC_AFTER=1 DRIVER_UP=1 -- sync-clock
check "is refused" exit_code_is 1
check "never switches NTP on" not_called "timedatectl set-ntp true"

echo "compose options and other commands:"
run_stack SYNC_AFTER=1 DRIVER_UP=0 -- up --profile camera
check "passes global compose options before the subcommand" called "docker compose --profile camera up -d"
run_stack SYNC_AFTER=1 DRIVER_UP=0 -- down
check "down stops every service, the camera too" called "docker compose --profile camera stop"
check "down does not touch NTP" not_called "set-ntp"
run_stack SYNC_AFTER=1 DRIVER_UP=0 -- frobnicate
check "an unknown command is refused" exit_code_is 2

echo "passed: ${PASS}  failed: ${FAIL}"
[ "${FAIL}" -eq 0 ]
