# mysensors_panasonic_heatpump

Original location: https://github.com/ToniA/arduino-heatpumpir
                   https://github.com/ToniA/Raw-IR-decoder-for-Arduino 

This project is a rather quick and dirty copy from the above two projects to create an mysensors node (for use in domoticz) to emulate the remote controller for Panasonic NZ9SKE heatpump. 

It creates sensors/controllers for controlling the heatpump. It also listens for ir-codes from the remote, allowing Domoticz to view the current state of the heatpump, even if the original remote is used. 

## Instructions

* Download the heatpumpir library, and place it under your personal Arduino 'libraries' directory, under directory 'HeatpumpIR'
* This sketch will present the following sensors on the mysensors network:
  Status: Power on/power off
  Setpoint: The temperature for the heat pump
  Mode: 10=Auto, 20=Heat, 30=Cool, 40=Dry, 50=Fan, 60=Maintenance heat
  Fan speed: 10=Auto, 20=Fan 1, 30=Fan 2, 40=Fan 3, 50=Fan 4, 60=Fan 5
  Air Swing (Vertical): 10=Auto, 20=Straight, 30=Down 1, 40=Down 2, 50=Down 3, 60=Down 4
  Profile: 10=Normal, 20=Quiet, 30=Powerful 

### ESP8266 support

This library also supports ESP8266. Just change the IR send method from 'IRSenderPWM' to 'IRSenderBitBang':

    IRSenderBitBang irSender(1);     // IR led on ESP8266 digital pin 1

* Note that depending on your board, certain GPIO's might not be available. For example on NodeMCU, the usable GPIO's are D1 (GPIO5), D2 (GPIO4), D6 (GPIO12), D7 (GPIO13) and D3 (GPIO0).

