# Power consumption measurements
The following measurements were taken from an SDS011 attached to a Heltec WIFI
LoRa 32 V2 Board which was connected to a standard household USB charger.

## Modes
### Idle
The device is in idle, waiting for its next measuring period with laser and
fans off, but not in sleep mode.

### During Measurement
The fans are running and the laser is taking measurments.

### Sleep Mode
The device has been set to sleep mode via command.

## Current Measurements (in mA)

|Mode       | Measurement Duration |Minimum|Average|Maximum|
|-----------|----------------------|-------|-------|-------|
|Idle       |                3 min | 15.54 | 16.25 | 16.98 |
|Measurement|                30 s  | 72.29 | 73.00 | 72.29 |
|Sleep      |                3 min | 3.13  | 3.19  | 6.63  |

## Conclusion
For low power applications, it is preferable to set the sensor into sleep mode
while it is not in use.

The idle current of the active measurement mode with a waiting period set is
significantly higher.
