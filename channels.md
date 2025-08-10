| channel | can change baudrate | baudrate  | voltage | device                    |
|:-------:|:-------------------:|----------:|--------:|:--------------------------|
| CH0     | yes                 | 38400     | 5V      | lab-io (needs bootloader) |
| CH1     | no                  | 9600      | 5V      | alarmclock                |
| CH2     | yes                 | 115200    | ext     | window-closer (bootloader)|
| CH3     | no                  | 9600      | 5V      | ZPH02                     |
| CH4     | no                  | 9600      | 3V3     | S88 (needs 5V power)      |
| CH5     | no                  | 115200    |         | reserved                  |

window-closer could teoretically go through a PIO channel now that we use
115200 baud both for the firmware and the bootloader, but avrdude upload via
pio was extremely unreliable.
