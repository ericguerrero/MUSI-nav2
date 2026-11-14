# ROSbot 2R power log

A record that survives a power cut, so that after a dead or reset robot you can say what
happened. Design: `docs/superpowers/specs/2026-11-14-robot-powerlog-design.md`.

## What runs on the robot

A container named `powerlog`, started by Docker at every boot. Every 2 s it appends one line
to `~/powerlog/log/powerlog.jsonl` and syncs it to the card:

- `uv`: the Pi saw under-voltage in the last 2 s (kernel `rpi_volt`).
- `batt`: the 12 V input as the STM32 reports it on `/battery`; `null` when the STM32 or the
  driver is silent.
- `drv`: the driver's `controller_manager` node is visible.
- `boot` and `up`: boot ID and seconds since boot. Use these to order events.
  `wall` is the robot's clock, which is wrong after a boot until NTP answers.

Kernel under-voltage messages are copied as `uv` records with the kernel's own time.
The log rotates at 20 MB and keeps ten files.

## Deploy

```bash
./robot/powerlog/deploy.sh          # ROBOT_HOST=user@host overrides husarion@192.168.77.2
```

It copies this directory to `~/powerlog/` and recreates the container. It is a separate
Compose project and does not touch the driver stack.

## Read it

```bash
ssh husarion@192.168.77.2 'python3 ~/powerlog/report.py --boots 5'
```

For each boot: duration, how it ended, battery min, max and last reading, under-voltage
events, periods without `/battery`, and holes in the samples. How a boot ended:

| End | Meaning |
|---|---|
| `stop` | The logger was told to stop: a shutdown, or someone stopped the container. |
| `cut` | No stop record, and a later boot exists: power was lost or the Pi reset. |
| `open` | The newest boot, still running (or cut, if you read a copy of a dead robot's log). |

A `start` record also carries `prev.end`; `restart` there means only the container restarted.

## Check the firmware

```bash
ssh husarion@192.168.77.2 '~/powerlog/fwcheck.sh'
```

It stops the driver, reads the STM32 flash back through its bootloader, compares it with
`firmware.bin` of the driver image, and starts the driver again. It never writes to the chip.
The driver is down for about a minute, so do not run it while the robot is in use.

| Result | Exit | Meaning |
|---|---|---|
| `MATCH` | 0 | The chip holds the pinned image. A fault is not corrupted firmware. |
| `MISMATCH` | 1 | The chip differs; the line gives how many bytes and the first offset. |
| `READ FAILED` | 2 | The bootloader did not answer or refused the read. Not a verdict. |

Results are appended to `~/powerlog/log/fwcheck.jsonl`. If power is lost during a check the
driver stays stopped after the next boot: start it with `docker start rosbot`.

## Limits

- The last 2 s before a cut are lost. Sags shorter than 2 s show only as kernel events.
- The log shows that power died and the trend before it, not the electrical cause.
- A `stop` record cannot tell a clean shutdown from a logger stopped by hand.
