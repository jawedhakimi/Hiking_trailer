# ESP32-S3 CAN (TWAI) Hardware Test

Quick PlatformIO project to verify a TJA1051 CAN transceiver wired to an
ESP32-S3 N16R8 is working, using the ESP32's built-in TWAI (CAN)
controller — no extra CAN library needed.

## Wiring

| ESP32-S3 | TJA1051 |
|----------|---------|
| GPIO17   | TXD     |
| GPIO16   | RXD     |
| 3V3      | VIO / VCC (check your board's voltage — some TJA1051 boards run VIO at 3.3V and VCC at 5V) |
| GND      | GND     |

CANH/CANL from the TJA1051 go to your CAN bus. If you only have this one
board and nothing else on the bus, you don't need termination resistors
for a basic loopback/self-test — see "Testing with only one board" below.
If you have a real 2+ node bus (e.g. this board + a VESC), put a 120 ohm
resistor across CANH/CANL at **each physical end** of the bus.

## What the sketch does

`src/main.cpp`:
- Initializes the TWAI (CAN) driver at 500 kbit/s (VESC's default) on
  TX=GPIO17 / RX=GPIO16.
- Every 500 ms, transmits a dummy CAN frame (ID `0x123`, 8 bytes of
  counting data).
- Continuously checks for and prints any received CAN frames.
- Prints driver alerts (bus errors, TX failures, bus-off, etc.) so
  problems are visible immediately instead of silently failing.

Open the Serial Monitor at **115200 baud** after flashing.

## Testing with only one board (no second CAN node yet)

By default the sketch uses `TWAI_MODE_NO_ACK`. This lets the ESP32
transmit CAN frames without needing another node on the bus to
acknowledge them — normally a CAN transmitter treats a missing ACK as
an error and keeps retrying, which would just spam
`[ALERT] TX failed`. With `NO_ACK` mode you can:

1. Flash this sketch to a single board.
2. Probe CANH/CANL with a scope or logic analyzer — you should see a
   valid differential CAN waveform every 500 ms even with nothing else
   connected. This alone confirms the TJA1051 + wiring + ESP32 TWAI
   controller are all functioning.
3. Watch the Serial Monitor — you should see `[TX] ID=0x123 ...` lines
   printing every 500 ms with no `[ALERT]` errors.

### Testing RX / two-board loopback

To actually verify the receive path end-to-end, use two boards (or one
board + a USB-CAN adapter / another CAN device):

1. Flash this same sketch to both boards.
2. Wire CANH-to-CANH and CANL-to-CANL between the two boards' TJA1051
   transceivers, and add a 120 ohm resistor across CANH/CANL at each
   board (2 total).
3. Change `TWAI_TEST_MODE` to `TWAI_MODE_NORMAL` in `src/main.cpp` on
   both boards (recompile/flash) since you now have a real second node
   to provide ACKs.
4. Each board's Serial Monitor should show its own `[TX]` lines and the
   `[RX]` lines it receives from the other board, with matching data.

### Testing against your VESC

Once basic TX/RX is confirmed, wire CANH/CANL to your VESC's CAN bus
(with proper termination), set `TWAI_TEST_MODE` to `TWAI_MODE_NORMAL`,
and re-flash. You should start seeing `[RX]` frames from the VESC's
periodic status broadcasts (VESC broadcasts status frames by default at
its configured CAN bit rate — make sure `CAN_BITRATE_KBPS` here matches
the VESC's configured CAN baud rate, typically 500 kbit/s).

## Troubleshooting

- **Nothing prints on Serial, but you see the ROM boot banner
  (`ESP-ROM:esp32s3-...`, `rst:0x15 (USB_UART_CHIP_RESET)`)**: this is
  the normal signature of a board with no separate USB-UART chip — the
  banner comes from the ROM's own USB-Serial-JTAG peripheral, which
  exists independently of your firmware. Your app's `Serial` was still
  pointed at the (unconnected) UART0 pins. This project's
  `platformio.ini` already has `ARDUINO_USB_MODE=1` and
  `ARDUINO_USB_CDC_ON_BOOT=1` enabled for this reason — if you still see
  nothing, do a clean rebuild (`pio run -t clean` then upload again) and
  make sure the Serial Monitor is pointed at the port that appears
  *after* upload (native USB CDC can enumerate as a different COM port
  than the one used for flashing).
- **Nothing prints on Serial at all, no boot banner either**: your board
  likely has a separate USB-UART bridge chip (CP2102/CH340/etc). Comment
  the two `ARDUINO_USB_*` build flags back out in `platformio.ini`.
- **`[ALERT] TX failed` even in `NO_ACK` mode**: check TX/RX aren't
  swapped, check TJA1051 power (VIO/VCC/GND), check the transceiver's
  `STB`/`EN` (standby/enable) pin — many TJA1051 breakout boards need
  this pulled to the correct level to leave standby mode.
- **`[ALERT] Bus-off!` / lots of bus errors in NORMAL mode**: usually
  missing termination resistors, a bit-rate mismatch with the other
  node, or a wiring fault (CANH/CANL swapped or shorted).
- **Scope shows no signal on CANH/CANL at all**: double-check the
  transceiver has power and is out of standby, and that GPIO17/16 are
  actually wired to TXD/RXD (not swapped).

## Building & flashing

```
pio run -t upload
pio device monitor -b 115200
```

or with the PlatformIO IDE extension: open this folder, use the PIO
toolbar's Upload and Monitor buttons.
