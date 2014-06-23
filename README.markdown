# Parking Sensor

Garage Parking Sensor is a gizmo that helps you park your car the perfect distance from the wall every time. It can also optionally serve as a presence sensor.

## Usage

To save the sweet spot, using whatever means necessary - your spouse, a tennis ball suspended from the ceiling, a bunch of mirrors with lasers - park the car the disared distance away from the wall where the sensor is mounted (for best results, aim the sensor at your car's bumper). Once satisfied with the location of your car, push the Save button on the gizmo taking care not to interfere with the ultrasound beam between the sensor and the car. The distance will be saved into the non-volatile memory (meaning, it will survive the reboot or the loss of power).

To park in the same perfect spot, drive *slowly* until the red LED starts to blink, then stop. The LEDs of the gizmo indicate the relative distance to your perfect spot. The first three yellow LEDs cover the range from approximately 3 meters (max detectable distance) to within about 20% of the target spot. Two green LEDs are for the 20% to 5% range. Finally, the red LED indicates that you're less than 5% away from your target spot (we're talking millimeters at this point). In practical terms, the sensor forces you to slow down fast and puts you within your target zone with the precision of a sharp shooter. It is *very* accurate!

## Schematics

The circuit is Arduino-compatible. That means it uses the ATmega microprocessor unit with the Arduino bootloader. However, there is no USB on board, the only way to communicate with the board is to use an FTDI breakout board similar to [this](https://www.sparkfun.com/products/9716). The ultrasound sensor used ("PING)))") is probably the most expensive component of this gizmo (I bought mine for about $30). I used [Eagle](http://www.cadsoftusa.com/) to design and draw the schematics and the PCB.

## Firmware

Firmware is Arduino-based, so you'll have to install and know how to use the Arduino software in order to tinker and upload the firmware to the board.

To be continued...
