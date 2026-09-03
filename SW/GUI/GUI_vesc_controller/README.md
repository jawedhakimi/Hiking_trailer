# Arduino project for Elecrow DIS07050H ESP32 HMI with 5" Capacitive Touch Display


## Prerequisites

1. Arduino-IDE should be installed on your system. (The build and board was tested with v1.8.19 and v2 versions too.)
2. In `Tools`/`Boards Manager` select and install `esp32` board-support package. (Contains the ESP toolchains for the board's Tensilica Xtensa ESP32 MCU. The board was tested with 2.0.17, version 3.0.0 and above is probably not liked by the included LovyanGFX.)
3. (The essence of `LovyanGFX` with the display-driver is included in the local `libraries` folder of this board-template, but if you want to us Arduino's built-in one you can delete it and install to Arduino-IDE with `Tools`/`Manage Libraries`.)


## Board settings (retained between Arduino-IDE sessions)

1. In `Tools` select `ESP32 Arduino` / `ESP32S3 Dev Module` as board
2. Set the parameters for your board: Flash-mode to `QIO 80MHz`, Flash-size to `4MB`, PSRAM to `OPI PSRAM`, (Partition-scheme to `Huge App` with 3MB app and 1MB SPIFFS partitions)
3. If you'll want to upload to the device from Arduino-IDE, select the programmer tool and port in `Tools` menu: Arduino-IDE uses the `esptool` of ESP-IDF in the background.
4. (The `partitions.csv` file in `ui` subfolder of this sketch-folder beside `ui.ino` overrides the `Huge App`/`Custom`/etc. partitioning scheme setting. If you really want those you need to delete this file, or edit it for different partition sizes.)


## Compilation

1. Open the `.ino` file in Arduino
2. Go to File/Preferences and set Sketchbook location to the path of your UI project (where this README is located). The setting is retained between Arduino-IDE sessions.
3. Build the project


## Burning

If the built-in Arduino-IDE chip-programmer is working you can go with it.

(If you want to make a backup before overwriting the flash in the device, you can use esptool's read command like: `esptool.py -b 460800 read_flash 0 ALL Backup.bin`.)

If it's not working on your system, there are two ways depending on esp32 package version used:

### esp32-3.0.0 (and above):
You can find the built merged .bin file in the folder (like `tmp`) where Arduino-IDE puts the builds.
The whole .bin (the merged 4MB one containing all partitions) can be burnt to the board by the ESP-IDF command: `esptool.py -b 460800 write_flash 0 ui.ino.merged.bin`

### A faster way, working with esp32-2.0.17 too (flashing just the application):
You can find the built ELF's flashable version in a `ui.ino.bin` file in the folder (like `tmp`) where Arduino-IDE puts the builds.
This `ui.ino.bin` file can be burnt to the board's app0 partition at offset 0x10000 by the ESP-IDF command: `esptool.py  write_flash  0x10000 ui.ino.bin`. (Offset might differ with other patition scheme, get app0 offset by `esptool.py read_flash 0x8000 0xc00 ptable.img` and `gen_esp32part.py ptable.img`.)


## Misc. notes

With this board-template you might encounter some Parallel-RGB vs LVGL-flush synchronization issues if you use computing-heavy or alpha-channel graphic contents like big shadows. According to our research it comes from low-level IO code in Arduino-IDE related to the display-driver. (ESP-IDF version doesn't have this problem.)


2024 SquareLine
