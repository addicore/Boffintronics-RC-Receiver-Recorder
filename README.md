# Boffintronics-RC-Receiver-Recorder

This is the Arduino code for the Boffintronics RC Receiver Recorder found at https://www.addicore.com/Boffintronics-RC-Receiver-Recorder-p/ad539.htm

Adds support for the new Boffintronics RC Receiver Recorder Sound Add-On Module found at https://www.addicore.com/Boffintronics-RC-Receiver-Recorder-Sound-Module-p/ad580.htm

Notes:

1. Any sequences that have been recorded with prior versions of firmware must be re-recorded with v 3.1 firmware
2. This sketch has been verified with the library versions noted in the includes section of the Arduino sketch. Versions other than those may cause eratic operation.

## Current Version - 3.1 - Released 8/25/2021

Change Log

v 3.1 - 8/25/2021
- Added sound board support
- Added support for servos 7 and 8

v 2.1 - 8/24/2020
- Added multi track recording
- Fixed retrigger after lockout bug when using digital I/O as trigger
- Fixed digital output polarity bug
- Added flash code version by pressing PGM & REC on power up

v 1.0 - 6/16/2020
- Initial production Release


Copyright (C) 2021  Boffintronics, LLC and Addicore
