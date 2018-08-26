# attiny85_front_i2c_slave
attiny85 acting as an I2C slave (for ESP8266) for doing light/temperature/battery monitoring

ESP8266 is not very battery efficient, it also doesn't handle external interrupting / waking very well.

The HW design for this runs off 3x 1.5V AAA batteries and is used primarily to monitor a door being opened/closed, but it also includes
the ability to monitor a photocell (light level).

         ATTINY85
        +----U----+
     NC |1       8| VCC (+)
  SW_IN |2       7| SCL
  PC_IN |3       6| RST_OUT
    GND |4       5| SDA
        +---------+

- GND/VCC connected to 4.5V alkaline battery pack
- SW_IN connected to magnetic reed switch between SW_IN and GND
- PC_IN connected to photocell between PC_IN and VCC
- RST_OUT connected to RST pin of ESP8266
- SCL/SDA connected to I2C pins of ESP8266

This code allows an ATTINY85 to serve as a controller and I2C Slave for an ESP8266.

The ATTINY will monitor for various conditions you may want to monitor -- Switch input, light-level, battery voltage, timer (heartbeat).
When a wakeable condition exists it will wake the ESP8266 using an output connected to the ESP8266 RESET pin.  

The ATTINY85 then turns itself into an I2C Slave and makes the details of the various conditions available to be read by the ESP8266.

The ESP8266 companion code sets up that device to act as an I2C Master to read the state of the ATTINY85 so that it can make some report
to a remote service via WiFi.

When the ESP8266 has completed, it makes an I2C Write to register 0x5E which signals the ATTINY85 to put the ESP8266 into RESET.

Given strict wake up conditions for the ESP8266, the battery life can be very long.

TODO: Disable everything and shutdown when the battery voltage reaches critically low levels -- This will help reduce the instance
of battery leakage

TODO: Make the code more dynamic to handle the heartbeat or PC inputs being disabled (turn it strictly into a door switch monitor)
