**Converts a HEX file to UF2 format.**
Check link above here for latest version and more documentation.
https://github.com/microsoft/uf2/blob/master/utils/uf2conv.py

Useful after compiling SAMD21/51 firmware in HEX format, to convert into UF2 format for easy firmware updates without an IDE.
The UF2 output can then be overwritted to/drag-dropped to the bootloader disk drive or sent to customer for in field upgrade of the respective devices.

**Python3 required**

command line usage:

`python3 source.hex -o output.uf2`
