#!/bin/bash
# Robot-side steps of one sensor test. Copied to ~/capture and called over SSH
# by run_sensor_test.sh, so that no step depends on quoting through SSH.
#
#   robot_side.sh prepare
#   robot_side.sh probe [serial] [dmesg]
#   robot_side.sh start <service> <capture delay s> <capture.py options...>
#   robot_side.sh stop <service>
#   robot_side.sh finish
set -uo pipefail

IMAGE="husarion/rosbot:humble-0.14.0-20250319"
OUT="${HOME}/probe_out"

# A mark is the robot's wall clock at a step, to line the probe log up with it.
mark() {
    date +%s.%N > "${OUT}/mark_$1"
    sync "${OUT}/mark_$1"
}

case "${1:-}" in
    prepare)
        mkdir -p "${HOME}/capture_out" "${OUT}"
        pkill -f capture/probe.py
        docker rm -f capture > /dev/null 2>&1
        rm -f "${HOME}"/capture_out/* "${OUT}"/*
        # The scripts were copied a moment ago; without this a cut in the next
        # seconds loses them and the directories with them.
        sync
        uptime
        vcgencmd get_throttled
        lsusb | grep -E "2bc5|10c4"
        ;;
    probe)
        # Words after "probe" switch the optional parts on: serial, dmesg.
        # Each is a suspect of its own, so a control run leaves both off.
        serial=()
        case " $* " in *" serial "*) serial=(--serial /dev/ttySERIAL) ;; esac
        setsid nohup python3 "${HOME}/capture/probe.py" --out "${OUT}/probe.jsonl" \
            "${serial[@]}" > "${OUT}/probe.err" 2>&1 < /dev/null &
        case " $* " in *" dmesg "*)
            # Kernel messages from now on (USB drops, under-voltage), synced
            # line by line. Read through the powerlog container: the host
            # restricts dmesg to root.
            setsid nohup bash -c 'docker exec powerlog dmesg -W | while IFS= read -r line; do
                printf "%s\n" "$line" >> "$0"; sync "$0"; done' "${OUT}/dmesg.log" \
                > /dev/null 2>&1 < /dev/null &
            ;;
        esac
        ;;
    start)
        service="$2"
        delay="$3"
        shift 3
        mark start
        cd "${HOME}" && docker compose --profile camera up -d "${service}" 2>&1 | tail -2
        mark started
        # The capture container joins later, so that a cut can be put down to
        # the sensor alone or to the sensor with a subscriber.
        export -f mark
        export OUT IMAGE
        setsid nohup bash -c 'sleep "$0"; mark capture
            docker run -d --name capture --network host --ipc host \
                -v "${HOME}/capture:/capture:ro" -v "${HOME}/capture_out:/out" "${IMAGE}" \
                python3 /capture/capture.py --out /out "$@"' "${delay}" "$@" \
            > /dev/null 2>&1 < /dev/null &
        ;;
    stop)
        mark stop
        cd "${HOME}" && docker compose --profile camera stop "$2" 2>&1 | tail -2
        mark stopped
        ;;
    finish)
        pkill -f capture/probe.py
        pkill -f "dmesg -W"
        docker logs capture 2>&1 | tail -3
        docker rm -f capture > /dev/null 2>&1
        docker ps --format "{{.Names}} {{.Status}}"
        uptime
        vcgencmd get_throttled
        ;;
    *)
        echo "usage: $0 prepare | probe | start <service> <options> | stop <service> | finish" >&2
        exit 2
        ;;
esac
