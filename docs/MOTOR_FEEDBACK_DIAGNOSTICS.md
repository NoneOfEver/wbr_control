# Motor feedback diagnostics

The production VOFA stream retains a compact view of motor-feedback freshness.
Start capture before enabling the robot so the transition into active control is
present in the recording.

## VOFA channel map

Channels 0-19 keep their normal chassis meanings. Channels 20-32 report:

| Channel | Meaning | Unit |
| --- | --- | --- |
| 20 | left wheel current feedback age | us |
| 21 | right wheel current feedback age | us |
| 22 | left B motor current feedback age | us |
| 23 | left D motor current feedback age | us |
| 24 | right B motor current feedback age | us |
| 25 | right D motor current feedback age | us |
| 26 | current motor-stale mask | bit mask |
| 27-32 | age latched on the latest fresh-to-stale transition, same order | us |

The mask uses bits 0-5 in the same order as channels 20-25. Zero means all six
feedback sources were fresh in that chassis cycle. An age of `UINT32_MAX`
means that no valid frame has been decoded.

The MPC Link state frame carries the same mask in `motor_stale_mask`. When CSV
logging is enabled, the production `usb_mpc` process writes that field so a
host-side event can be correlated with the lower-controller VOFA capture.
