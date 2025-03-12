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


## Build
```sh
. ~/zephyrproject/.venv/bin/activate
. ~/zephyrproject/zephyr/zephyr-env.sh 
west build

# enter bootloader: press BOOT button, press & release RESET, release BOOT
west flash
```
