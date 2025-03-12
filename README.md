# MultiCDC
This is a simple RP2040-based 6-channel USB to UART converter.

This is my first project using the Zephyr RTOS, so there might be lots of
mistakes. However, it seems to work.


## Operation
The device creates 7 `/dev/ttyACM*` devices when connected to a linux host.
The first 6 are linked to hardware UARTs, and `/dev/ttyACM6` is used for
logging and shell.

The RP2040 only has 2 hardware UARTs. The other 4 channels use the PIOs.

Unfortunately, the `pico-uart-pio` driver doesn't allow changing the baud
rate at runtime. For this reason, baud rates (`current-speed`) for those
channels need to be set in the device tree (`app.overlay`).

Pin map (most of these can be changed arbitrarily):
| pin | function |
|:---:|:--------:|
| P0  | Tx0      |
| P1  | Rx0      |
| P2  | Tx1      |
| P3  | Rx1      |
| P4  | Tx2      |
| P5  | Rx2      |
| P6  | Tx3      |
| P7  | Rx3      |
| P8  | Tx4      |
| P9  | Rx4      |
| P10 | Tx5      |
| P11 | Tx5      |


## Build
```sh
. ~/zephyrproject/.venv/bin/activate
. ~/zephyrproject/zephyr/zephyr-env.sh 
west build

# enter bootloader: press BOOT button, press & release RESET, release BOOT
west flash
```
