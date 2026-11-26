#!/bin/bash
# Start and stop the ROSbot 2R stack. Runs on the robot, from the directory
# that holds compose.yaml. Use this instead of `docker compose up`.
#
#   ./stack.sh up                    driver and lidar
#   ./stack.sh up --profile camera   also the depth camera
#   ./stack.sh down                  stop everything
#
# Why a wrapper: the wall clock must never jump while the driver runs. The
# Raspberry Pi has no battery-backed clock, so after a power cut it boots hours
# in the past. On 2026-11-14, each time NTP corrected that with the driver
# running, the Pi lost power about 30 s later (2 of 2 trials); with the driver
# stopped the same correction did nothing. So NTP stays disabled on the robot,
# and the clock is set here, only while the driver is down. Network arriving
# later cannot move the clock, because nothing is left running that would.
#
# Setting the clock needs root, so `up` asks for the sudo password when the
# driver is down. Run it from a terminal (over ssh: `ssh -t`).
set -uo pipefail

DRIVER="${DRIVER:-rosbot}"
CLOCK_WAIT="${CLOCK_WAIT:-45}"   # seconds to wait for a time source
CLOCK_POLL="${CLOCK_POLL:-1}"
WIRED_ADDRESS="192.168.77.2/24"

driver_running() {
    [ -n "$(docker ps -q -f "name=^${DRIVER}$")" ]
}

clock_synchronised() {
    [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ]
}

ntp_on() {
    [ "$(timedatectl show -p NTP --value)" = "yes" ]
}

# The privileged part, run as root through one sudo (see `up`). NTP on, wait
# for the clock, NTP off. It refuses while the driver runs: that is the one
# state in which the clock must not move.
sync_clock() {
    if driver_running; then
        echo "stack: refusing to set the clock while the driver runs" >&2
        return 1
    fi
    # DHCP beside the fixed address gives the robot a route to a time server
    # when its cable is on a network that has one. Runtime only: with DHCP in
    # the saved profile and no DHCP server on the cable, the fixed address
    # that is the way back into the robot could be dropped.
    nmcli device modify eth0 ipv4.method auto ipv4.addresses "${WIRED_ADDRESS}" \
        > /dev/null || echo "stack: could not enable DHCP on eth0"
    # Through timedatectl, never by naming a service: this image masks
    # systemd-timesyncd and runs chrony, and timedatectl drives whichever
    # NTP service is installed.
    timedatectl set-ntp true
    local waited=0
    until clock_synchronised || [ "${waited}" -ge "${CLOCK_WAIT}" ]; do
        sleep "${CLOCK_POLL}"
        waited=$((waited + CLOCK_POLL))
    done
    if clock_synchronised; then
        echo "stack: clock synchronised: $(date -u +%FT%TZ)"
    else
        echo "stack: clock NOT synchronised after ${CLOCK_WAIT} s (no time source)." \
             "It stays as it is for this boot: $(date -u +%FT%TZ)"
    fi
    timedatectl set-ntp false
}

up() {
    if driver_running; then
        echo "stack: driver is running, the clock is left alone"
        # NTP should already be off; asked about first so that the usual case
        # needs no password.
        if ntp_on; then
            sudo timedatectl set-ntp false
        fi
    else
        # One sudo for the whole step. A clock step makes sudo forget the
        # password it was just given, so a second sudo after the step would
        # stop at a prompt with NTP still on.
        sudo "$0" sync-clock
    fi
    if ntp_on; then
        echo "stack: NTP is still on, so the driver is NOT started." >&2
        echo "stack: run 'sudo timedatectl set-ntp false', then '$0 up' again." >&2
        return 1
    fi
    docker compose "$@" up -d
}

case "${1:-}" in
    up)
        shift
        up "$@"
        ;;
    sync-clock)
        sync_clock
        ;;
    down)
        # Names every profile so that optional services stop as well.
        docker compose --profile camera stop
        ;;
    *)
        echo "usage: $0 up [compose options] | down" >&2
        exit 2
        ;;
esac
