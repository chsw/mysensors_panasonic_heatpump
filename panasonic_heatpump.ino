/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - chsw - https://github.com/chsw/mysensors_panasonic_heatpump
 *
 * DESCRIPTION
 * Heatpump controller
 */


 /*
  * This sketch is for the cabin project, to control a heatpump, running on Sensebender
  * https://www.mysensors.org/hardware/micro
  *
  * The sketch is based on the 'HeatpumpIRController' example of the MySensors project
  */



// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Channel 1
//#define MY_RF24_CHANNEL  76

// Safe baud rate for a 3.3V device
#define MY_BAUD_RATE 9600

// From library at https://github.com/ToniA/arduino-heatpumpir
#include <IRSender.h>
#include <PanasonicHeatpumpIR.h>

// Humidity & temperature sensor
#include <Wire.h>

#define DOMO_UNDEFINED 0
#define DOMO_MODE_AUTO 10
#define DOMO_MODE_HEAT 20
#define DOMO_MODE_COOL 30
#define DOMO_MODE_DRY 40
#define DOMO_MODE_FAN 50
#define DOMO_MODE_MAINTENANCE 60
#define DOMO_FAN_AUTO 10
#define DOMO_FAN_1 20
#define DOMO_FAN_2 30
#define DOMO_FAN_3 40
#define DOMO_FAN_4 50
#define DOMO_FAN_5 60
#define DOMO_PROFILE_NORMAL 10
#define DOMO_PROFILE_POWERFUL 30
#define DOMO_PROFILE_QUIET 20
#define DOMO_VERTICAL_AIR_AUTO 10
#define DOMO_VERTICAL_AIR_STRAIGHT 20
#define DOMO_VERTICAL_AIR_DOWN1 30
#define DOMO_VERTICAL_AIR_DOWN2 40
#define DOMO_VERTICAL_AIR_DOWN3 50
#define DOMO_VERTICAL_AIR_DOWN4 60


#define PANASONIC_AIRCON2_PROFILE_NORMAL 0x00


uint32_t mark_header_avg = 0;
uint16_t mark_header_cnt = 0;
uint32_t mark_bit_avg = 0;
uint16_t mark_bit_cnt = 0;
uint32_t space_zero_avg = 0;
uint16_t space_zero_cnt = 0;
uint32_t space_one_avg = 0;
uint16_t space_one_cnt = 0;
uint32_t space_header_avg = 0;
uint16_t space_header_cnt = 0;
uint32_t space_pause_avg = 0;
uint16_t space_pause_cnt = 0;
// we will store up to 1024 symbols
//char symbols[600];  // decoded symbols
uint16_t currentpulse = 0; // index for pulses we're storing
// Decoded bytes
byte byteCount = 0;
//byte bytes[64];
#define bufferLength 300
byte irbytes[bufferLength];

int rbytes;


#define IRpin_PIN      PIND
#define IRpin 2
// the maximum pulse we'll listen for - 65 milliseconds is a long time
#define MAXPULSE 65000
// The thresholds for different symbols
//uint16_t MARK_THRESHOLD_BIT_HEADER    = 0; // Value between BIT MARK and HEADER MARK
//uint16_t SPACE_THRESHOLD_ZERO_ONE     = 0; // Value between ZERO SPACE and ONE SPACE
//uint16_t SPACE_THRESHOLD_ONE_HEADER   = 0; // Value between ONE SPACE and HEADER SPACE
//uint16_t SPACE_THRESHOLD_HEADER_PAUSE = 0; // Value between HEADER SPACE and PAUSE SPACE (Panasonic/Midea only)
    // Panasonic cs
#define    MARK_THRESHOLD_BIT_HEADER     2000
#define     SPACE_THRESHOLD_ZERO_ONE      800
#define     SPACE_THRESHOLD_ONE_HEADER   1500
#define    SPACE_THRESHOLD_HEADER_PAUSE 8000


// what our timing resolution should be, larger is better
// as its more 'precise' - but too large and you wont get
// accurate timing
uint16_t RESOLUTION=20;



// MySensors libraries
#include <SPI.h>
#include <MySensors.h>

// Child ID's of this node
#define CHILD_STATUS     1
#define CHILD_SETPOINT   2
#define CHILD_MODE 3
#define CHILD_FAN   4
#define CHILD_AIRSWING    5
#define CHILD_PROFILE 6
#define CHILD_VAR1 7
    long lastUpdateSent = 0;
class hvac
{ 
  private:
    bool isMaintenance = false;
    void triggerMaintenanceDirty()
    {
        mode_dirty = true;
        fan_dirty=true;
        setpoint_dirty=true;
        profile_dirty=true;
    }
  public:
    bool status = 1;
    bool status_dirty = false;
    int mode= 10;
    bool mode_dirty = false;
    float setpoint_auto= 20;
    float setpoint_heat=19;
    float setpoint_cool=25;
    float setpoint_dry=16;
    float setpoint_maintenance=10;
    bool setpoint_dirty=false;
    int fan_auto=10;
    int fan_heat=10;
    int fan_cool=10;
    int fan_dry=10;
    int fan_fan=20;
    bool fan_dirty=false;
    int airSwing=0;
    bool airSwing_dirty=false;
    int profile=10;
    bool profile_dirty=false;
    bool var1_dirty=false;
    bool var2_dirty=false;


    void setSetpoint(float temp)
    {
        if(temp > 30.0 || temp < 8.0)
         {
          setpoint_dirty=true;
          return;
         }
        if(temp < 16.0)
        {
          switch(mode)
          {
            case DOMO_MODE_COOL:
            case DOMO_MODE_DRY:
            case DOMO_MODE_FAN:
              setpoint_dirty = true;
              return;
          }
          setpoint_maintenance = temp;
          var2_dirty=true;
          if(isMaintenance == false)
          {
            isMaintenance = true;
            triggerMaintenanceDirty();
          }
        }
        else
        {
          if(isMaintenance == true)
          {
            isMaintenance = false;
            triggerMaintenanceDirty();
            if(!(mode== DOMO_MODE_AUTO || mode == DOMO_MODE_HEAT))
              setMode(DOMO_MODE_HEAT);
          }

          switch(mode)
          {
            case DOMO_MODE_AUTO:
              setpoint_auto=temp;
              break;
            case DOMO_MODE_HEAT:
              setpoint_heat=temp;
              break;
            case DOMO_MODE_COOL:
              setpoint_cool=temp;
              break;
            case DOMO_MODE_DRY:
              setpoint_dry=temp;
              break;
            case DOMO_MODE_FAN:
              break;
          }
          var1_dirty = true;
        }
        setpoint_dirty=true;
    }

    float getSetpoint()
    {
      if(isMaintenance)
        return setpoint_maintenance;
      switch(mode)
      {
        case DOMO_MODE_AUTO:
          return setpoint_auto;
        case DOMO_MODE_HEAT:
          return setpoint_heat;
        case DOMO_MODE_COOL:
          return setpoint_cool;
        case DOMO_MODE_DRY:
          return setpoint_dry;
        case DOMO_MODE_FAN:
          return 27;
      }
      
    }

    int getFan()
    {
      if(isMaintenance)
        return DOMO_FAN_5;
      switch(mode)
      {
        case DOMO_MODE_AUTO:
          return fan_auto;
        case DOMO_MODE_HEAT:
          return fan_heat;
        case DOMO_MODE_COOL:
          return fan_cool;
        case DOMO_MODE_DRY:
          return fan_dry;
        case DOMO_MODE_FAN:
          return fan_fan;
      }
    }

    void setFan(int newfan)
    {
      fan_dirty = true;
      var2_dirty=true;
      switch(mode)
      {
        case DOMO_MODE_AUTO:
          fan_auto=newfan;
          break;
        case DOMO_MODE_HEAT:
          fan_heat=newfan;
          break;
        case DOMO_MODE_COOL:
          fan_cool=newfan;
          break;
        case DOMO_MODE_DRY:
          fan_dry=newfan;
          break;
        case DOMO_MODE_FAN:
          fan_fan=newfan;
          break;
      }
    }

    int getProfile()
    {
      if(isMaintenance)
        return DOMO_PROFILE_NORMAL;
      return profile;
    }
    void setProfile(int newProfile)
    {
      profile_dirty=true;
      if(isMaintenance)
        return;
        profile = newProfile;
    }

    int getMode()
    {
      if(isMaintenance)
        return DOMO_MODE_MAINTENANCE;
      return mode;
    }
    void setMode(int newMode)
    {
      if(newMode == DOMO_MODE_MAINTENANCE)
      {
        if(isMaintenance)
          return;
        isMaintenance = true;
        triggerMaintenanceDirty();
      }
      else
      {
        if(isMaintenance)
        {
          isMaintenance = false;
          triggerMaintenanceDirty();
        }
        mode=newMode;
        mode_dirty=true;
      }
      setpoint_dirty = true;
      fan_dirty = true;
    }
} current;

struct VAR1_DATA
{
    byte setpoint_auto;
    byte setpoint_heat;
    byte setpoint_cool;
    byte setpoint_dry;
} ;
struct VAR2_DATA
{
  byte setpoint_maintenance;
  byte fan_auto:4;
  byte fan_heat:4;
  byte fan_cool:4;
  byte fan_dry:4;
  byte fan_fan:4;
};

long lastTransmitt = 0;
long lastCommandReceived = 0;
// MySensors messages of this node
MyMessage msgSetpoint(CHILD_SETPOINT, V_HVAC_SETPOINT_HEAT);
MyMessage msgStatus(CHILD_STATUS, V_STATUS);
MyMessage msgMode(CHILD_MODE,V_PERCENTAGE);
MyMessage msgFan(CHILD_FAN,V_PERCENTAGE);
MyMessage msgAirSwing(CHILD_AIRSWING,V_PERCENTAGE);
MyMessage msgProfile(CHILD_PROFILE,V_PERCENTAGE);
MyMessage msgVar1(CHILD_VAR1, V_VAR1);
MyMessage msgVar2(CHILD_VAR1, V_VAR2);

// IR led on PWM output-capable digital pin 3
IRSenderPWM irSender(3);


void setup()
{
  Serial.println(F("HeatpumpIR sensor starting up..."));
    request(msgVar1.sensor, msgVar1.type);
    request(msgVar2.sensor, msgVar2.type);
    request(msgStatus.sensor, msgStatus.type);
    request(msgMode.sensor, msgMode.type);
//    request(msgSetpoint.sensor, msgSetpoint.type);
//    request(msgFan.sensor, msgFan.type);
    request(msgAirSwing.sensor, msgAirSwing.type);
    request(msgProfile.sensor, msgProfile.type);
}


void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Panasonic heat pump", "1.0");

  // Register the sensors to the MySensors Gateway

  present(CHILD_STATUS,S_HVAC); // onoff
  present(CHILD_SETPOINT,S_HVAC, "Temperature"); // setpoint
  present(CHILD_MODE,S_DIMMER, "Mode"); // mode
  present(CHILD_FAN,S_DIMMER, "Fan speed"); // Fan speed
  present(CHILD_AIRSWING,S_DIMMER, "Air Swing"); // Air swing
  present(CHILD_PROFILE,S_DIMMER, "Profile"); // profile


  //send(msgSetpoint.set(16,1));

  attachInterrupt(digitalPinToInterrupt(IRpin), interuptHandler, RISING);
}
bool  interuptTriggered = false;
void interuptHandler()
{
  interuptTriggered = true;
//  detachInterrupt(digitalPinToInterrupt(IRpin));
}

bool initialSend = false;
bool valuesRequested = false;
void loop()
{
  if(!valuesRequested)
  {
    valuesRequested = true;

  }
  if((millis()>10000) && initialSend == false){
    initialSend=true;
/*    send(msgStatus.set(current.status));
    send(msgFan.set(current.fan));
    send(msgSetpoint.set(current.setpoint,1));
    send(msgMode.set(current.mode));
    send(msgAirSwing.set(current.airSwing));
    send(msgProfile.set(current.profile));
*/    
  }

    if(interuptTriggered)
    {
        currentpulse=0;
      byteCount=0;
      Serial.println("Start listening for pulses");
       // memset(symbols, 0, sizeof(symbols));
      long beforePulses = millis();
      
      int count = receivePulses();
      long afterPulses = millis();

//      attachInterrupt(digitalPinToInterrupt(IRpin), interuptHandler, CHANGE);
      interuptTriggered = false;

      Serial.print(F("Pulses received: "));
      Serial.print(afterPulses - beforePulses);
      Serial.println(F("ms"));
      Serial.print(F("Bytes received: "));
      Serial.println(count);
      
//      printPulses();
      if(count >= 27 && decodePanasonicCS(irbytes, count))
      {
      
      fetchValues(irbytes);


    }
    
    else
     Serial.println(F("Expected 27 bytes, skipping parse"));
    }
    if(lastTransmitt < lastCommandReceived)
    {
      lastTransmitt = millis();
      // Transmitt new values
      sendHeatpumpIRCommand(0);
      delay(10);
      interuptTriggered = false; // Trigga inte på våra egna pulser
    }
      // Send updates
      if(current.status_dirty)
      {
        send(msgStatus.set(current.status));
        current.status_dirty = false;
      }
      if(current.mode_dirty)
      {
        send(msgMode.set(current.getMode()));
        current.mode_dirty = false;
      }
      if(current.setpoint_dirty || (millis()-lastUpdateSent)>(25*60000))
      {
        send(msgSetpoint.set(current.getSetpoint(), 1));
        current.setpoint_dirty = false;
        lastUpdateSent = millis();
      }
      if(current.fan_dirty)
      {
        send(msgFan.set(current.getFan()));
        current.fan_dirty = false;
      }
      if(current.profile_dirty)
      {
        send(msgProfile.set(current.getProfile()));
        current.profile_dirty = false;
      }
      if(current.airSwing_dirty)
      {
        send(msgAirSwing.set(current.airSwing));
        current.airSwing_dirty = false;
      }
      if(current.var1_dirty)
      {
        uint32_t value1 = 0;
        VAR1_DATA *var1 = (VAR1_DATA*)(&value1);
        var1->setpoint_auto=current.setpoint_auto*2.0;
        var1->setpoint_heat=current.setpoint_heat*2.0;
        var1->setpoint_cool=current.setpoint_cool*2.0;
        var1->setpoint_dry=current.setpoint_dry*2.0;
        send(msgVar1.set(value1));
        current.var1_dirty = false;
      }
      if(current.var2_dirty)
      {
        uint32_t value2 = 0;
        VAR2_DATA *var2 = (VAR2_DATA*)(&value2);
        var2->setpoint_maintenance=current.setpoint_maintenance*2.0;
        var2->fan_auto = current.fan_auto/10;
        var2->fan_heat = current.fan_heat/10;
        var2->fan_cool = current.fan_cool/10;
        var2->fan_dry = current.fan_dry/10;
        var2->fan_fan = current.fan_fan/10;
        send(msgVar2.set(value2));
        current.var2_dirty = false; 
      }
}


// Handle incoming messages from the MySensors Gateway
void receive(const MyMessage &message) {

  const char *irData;

  // V_IR type message
  if (message.type==V_IR_SEND) {
    Serial.println(F("Received IR send command..."));
    irData = message.getString();
    Serial.print(F("Code: 0x"));
    Serial.println(irData);
    sendHeatpumpIRCommand(irData);
  }

  bool triggerSend = false;
  if(message.sensor == msgStatus.sensor)
  {
      current.status = message.getBool();
      current.status_dirty = true;
      triggerSend = true;
  }
  else if(message.sensor == msgMode.sensor)
  {
      current.setMode(message.getInt());
      triggerSend = true;
  }
  else if(message.sensor == msgFan.sensor)
  {
      current.setFan(message.getInt());
      triggerSend = true;
  }
  else if(message.sensor == msgAirSwing.sensor)
  { 
      current.airSwing = message.getInt();
      current.airSwing_dirty = true;
      triggerSend = true;
  }
  else if(message.sensor == msgProfile.sensor)
  {
      current.profile = message.getInt();
      current.profile_dirty = true;
      triggerSend = true;
  }
  else if(message.sensor == msgSetpoint.sensor)
  {
      Serial.print(F("Received new setpoint: "));
      Serial.println(message.getFloat());
      current.setSetpoint(message.getFloat());
      triggerSend = true;
  }
  if(triggerSend)
  {
    lastCommandReceived = millis();
  }

  if(message.sensor == msgVar1.sensor && message.type == msgVar1.type)
  {
    uint32_t value = message.getLong();
    if(message.getLong()>0)
    {
      VAR1_DATA *var1 = (VAR1_DATA*)(&value);
      //long value = message.getLong();
      float tmp = var1->setpoint_auto/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        current.setpoint_auto = tmp;
      tmp = var1->setpoint_heat/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        current.setpoint_heat = tmp;
      tmp = var1->setpoint_cool/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        current.setpoint_cool = tmp;
      tmp = var1->setpoint_dry/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        current.setpoint_dry = tmp;
      Serial.println(F("Received saved values:"));
      Serial.print(F("  setpoint_auto: "));
      Serial.println(current.setpoint_auto);
      Serial.print(F("  setpoint_heat: "));
      Serial.println(current.setpoint_heat);
      Serial.print(F("  setpoint_cool: "));
      Serial.println(current.setpoint_cool);
      Serial.print(F("  setpoint_dry: "));
      Serial.println(current.setpoint_dry);
      current.setpoint_dirty = true;
    }
    
  }
  if(message.sensor == msgVar2.sensor && message.type == msgVar2.type)
  {
    uint32_t value = message.getLong();
    if(message.getLong()>0)
    {
      VAR2_DATA *var2 = (VAR2_DATA*)(&value);
      float tmp = var2->setpoint_maintenance/2.0f;
      if(tmp >=8 && tmp < 16)
        current.setpoint_maintenance = tmp;
      int fan = var2->fan_auto*10;
      current.fan_auto=fan;
      fan = var2->fan_heat*10;
      current.fan_heat=fan;
      fan = var2->fan_cool*10;
      current.fan_cool=fan;
      fan = var2->fan_dry*10;
      current.fan_dry=fan;
      fan = var2->fan_fan*10;
      current.fan_fan=fan;
      Serial.println(F("Received saved values:"));
      Serial.print("  setpoint_maintenance: ");
      Serial.println(current.setpoint_maintenance);
      Serial.print(F("  fan_auto: "));
      Serial.println(current.fan_auto);      
      Serial.print(F("  fan_heat: "));
      Serial.println(current.fan_heat);      
      Serial.print(F("  fan_cool: "));
      Serial.println(current.fan_cool);      
      Serial.print(F("  fan_dry: "));
      Serial.println(current.fan_dry);      
      Serial.print(F("  fan_fan: "));
      Serial.println(current.fan_fan);      

    }
  }

}


int receivePulses(void) {
  if((IRpin_PIN & (1 << IRpin))==0)
    return 0;
  
  uint16_t highpulse, lowpulse;  // temporary storage timing
  char symbolBuffer[2];
   memset(symbolBuffer, 0, sizeof(symbolBuffer));
  // Initialize the averages every time
  mark_header_avg = 0;
  mark_header_cnt = 0;
  mark_bit_avg = 0;
  mark_bit_cnt = 0;
  space_zero_avg = 0;
  space_zero_cnt = 0;
  space_one_avg = 0;
  space_one_cnt = 0;
  space_header_avg = 0;
  space_header_cnt = 0;
  space_pause_avg = 0;
  space_pause_cnt = 0;

  // Only Panasonic seems to use the pause
  space_pause_avg = 0;
  space_pause_cnt = 0;

  int currbyte = 0; 
  int currbit = 0;
  memset(irbytes, 0, sizeof(bufferLength));
  rbytes=0;
  while (/*currentpulse < sizeof(symbols)*/true)
  {
     highpulse = 0;
     while (IRpin_PIN & (1 << IRpin)) {
       // pin is still HIGH

       // count off another few microseconds
       highpulse++;
       delayMicroseconds(RESOLUTION);

       // If the pulse is too long, we 'timed out' - either nothing
       // was received or the code is finished, so print what
       // we've grabbed so far, and then reset
       if ((highpulse >= (MAXPULSE/RESOLUTION)) /*&& (currentpulse != 0)*/) {
         return rbytes;
       }
    }

    highpulse = highpulse * RESOLUTION;
    // dont read over memory
    if(rbytes>=bufferLength)
      return currbyte;
    if (currentpulse > 0)
    {
      // this is a SPACE
      if ( highpulse > SPACE_THRESHOLD_HEADER_PAUSE ) {
        symbolBuffer[0] = 'W';
        // Cumulative moving average, see http://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average
        space_pause_avg = (highpulse + space_pause_cnt * space_pause_avg) / ++space_pause_cnt;
      } else if ( (currentpulse > 0 && symbolBuffer[1] == 'H') || highpulse > SPACE_THRESHOLD_ONE_HEADER ) {
        symbolBuffer[0] = 'h';
        // Cumulative moving average, see http://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average
        space_header_avg = (highpulse + space_header_cnt * space_header_avg) / ++space_header_cnt;
      } else if ( highpulse > SPACE_THRESHOLD_ZERO_ONE ) {
        symbolBuffer[0] = '1';
        irbytes[currbyte]= ((irbytes[currbyte]>>1)|0x80);
        currbit ++;
        space_one_avg = (highpulse + space_one_cnt * space_one_avg) / ++space_one_cnt;
      } else {
        symbolBuffer[0] = '0';
        irbytes[currbyte]= (irbytes[currbyte]>>1);
        currbit++;
        space_zero_avg = (highpulse + space_zero_cnt * space_zero_avg) / ++space_zero_cnt;
      }
      if(currbit==8){
      currbit = 0;
      currbyte++;
      rbytes++;
    }
    }
    currentpulse++;
    // remember previos symbol
    symbolBuffer[1]=symbolBuffer[0];


    // same as above
    lowpulse = 0;
    while (! (IRpin_PIN & _BV(IRpin))) {
       // pin is still LOW
       lowpulse++;
       delayMicroseconds(RESOLUTION);
       if ((lowpulse >= (MAXPULSE/RESOLUTION))  && (currentpulse != 0)) {
         return rbytes;
       }
    }

    // this is a MARK

    lowpulse = lowpulse * RESOLUTION;

    if ( lowpulse > MARK_THRESHOLD_BIT_HEADER ) {
      symbolBuffer[0] = 'H';
      currentpulse++;
      symbolBuffer[1]=symbolBuffer[0];
      mark_header_avg = (lowpulse + mark_header_cnt * mark_header_avg) / ++mark_header_cnt;
    } else {
      mark_bit_avg = (lowpulse + mark_bit_cnt * mark_bit_avg) / ++mark_bit_cnt;
    }

    // we read one high-low pulse successfully, continue!
  }
}

// Decode the IR command and send the IR command to the heatpump
void sendHeatpumpIRCommand(const char *irCommandString) {
/*
  // irCommandString is an 8-digit hex digit
  long irCommand = 0;
  int sscanfStatus = sscanf(irCommandString, "%lx", &irCommand);

  if (sscanfStatus == 1) {
    Serial.print(F("IR code conversion OK: 0x"));
    Serial.println(irCommand, HEX);
  } else {
    Serial.println(F("Failed to convert IR hex code to number"));
  }
*/
/*
The heatpump command is packed into a 32-bit hex number, see
libraries\HeatpumpIR\HeatpumpIR.h for the constants

12345678
  3 MODEL
   4 POWER
    5 OPERATING MODE
     6 FAN SPEED
      78 TEMPERATURE IN HEX

00213416 (as an example of a valid code)
  2 = PanasonicJKE
   1 = Power ON
    3 = COOL
     4 = FAN 4
      16 = Temperature 22 degrees (0x16 = 22)
 */
/*
  byte model = (irCommand & 0x00F00000) >> 20;
  byte power = (irCommand & 0x00010000) >> 16;
  byte mode  = (irCommand & 0x0000F000) >> 12;
  byte fan   = (irCommand & 0x00000F00) >> 8;
  byte temp  = (irCommand & 0x000000FF);
*/
  const char* buf;
/*  Serial.print(F("Model: "));

  buf = heatpumpIR[2]->model();
  // 'model' is a PROGMEM pointer, so need to write a byte at a time
  while (char modelChar = pgm_read_byte(buf++))
  {
    Serial.print(modelChar);
  }
  Serial.println();
*/
/*  Serial.print(F("Model #: ")); Serial.println(model);
  Serial.print(F("Power: ")); Serial.println(power);
  Serial.print(F("Mode: ")); Serial.println(mode);
  Serial.print(F("Fan: ")); Serial.println(fan);
  Serial.print(F("Temp: ")); Serial.println(temp);
*/
  // Heatpump models start from 0, i.e. model number is always less than the number of different models
  //if (model < models) 
  {
    // This is a supported model

  static const uint8_t irtemplate[27] PROGMEM = {
    0x02, 0x20, 0xE0, 0x04, 0x00, 0x00, 0x00, 0x06, 0x02, 0x20, 0xE0, 0x04, 0x00, 0x08, 0x00, 0x80, 0x00, 0x00, 0x00, 0x0E, 0xE0, 0x00, 0x00, 0x89, 0x00, 0x00, 0x00
  //   0     1     2     3     4     5     6     7     8     9    10    11    12    13    14   15     16    17    18    19    20    21    22    23    24    25    26
  };
  uint8_t buff[27];
  memcpy_P(buff, irtemplate, sizeof(irtemplate));

    setStatus(buff, current.status);
    setMode(buff, current.getMode());
    setSetpoint(buff, current.getSetpoint());
    setAirSwingVertical(buff, current.airSwing);
    setFan(buff, current.getFan());
    setProfile(buff, current.profile);
    
    Serial.println(F("Sending IR code to heatpump: "));
    //heatpumpIR[2]->send(irSender, current.status,current.mode, current.fan, current.setpoint, VDIR_UP, HDIR_AUTO);
    sendPanasonic(irSender, buff);
    printBuffer(buff);

    //uint8_t *buffer = heatpumpIR[2]->getLastBuffer();
    verifyBuffer( readStatus(buff), current.status, "status");
    verifyBuffer( readMode(buff), current.mode, "mode");
    verifyBuffer( readSetpoint(buff), current.getSetpoint(), "setpoint");
    verifyBuffer( readFan(buff), current.getFan(), "fan");
    verifyBuffer( readAirSwingVertical(buff), current.airSwing, "airSwing");
    verifyBuffer( readProfile(buff), current.profile, "profile");

  }
}

void printBuffer(byte *buffer)
{
  Serial.print(F("  Power status: "));
  Serial.println(readStatus(buffer)?"On":"Off");
  Serial.print(F("  Mode: "));
  switch(readMode(buffer))
  {
    case DOMO_MODE_AUTO:
      Serial.println(F("AUTO"));
      break;
    case DOMO_MODE_HEAT:
      Serial.println(F("HEAT"));
      break;
    case DOMO_MODE_COOL:
      Serial.println(F("COOL"));
      break;
    case DOMO_MODE_DRY:
      Serial.println(F("DRY"));
      break;
    case DOMO_MODE_FAN:
      Serial.println(F("FAN"));
      break;
    default:
      Serial.print(F("UNDEFINED: "));
      Serial.println(readMode(buffer));
      
  }
  Serial.print(F("  Temperature: "));
  Serial.println(readSetpoint(buffer));
  Serial.print(F("  Fan speed: "));
  switch(readFan(buffer))
  {
    case DOMO_FAN_AUTO:
      Serial.println(F("Auto"));
      break;
    case DOMO_FAN_1:
      Serial.println(F("1"));
      break;
    case DOMO_FAN_2:
      Serial.println(F("2"));
      break;
    case DOMO_FAN_3:
      Serial.println(F("3"));
      break;
    case DOMO_FAN_4:
      Serial.println(F("4"));
      break;
    case DOMO_FAN_5:
      Serial.println(F("5"));
      break;
    default:
      Serial.print(F("UNDEFINED; "));
      Serial.println(readFan(buffer));
  }
  Serial.print(F("  Vertical Air Swing: "));
  switch(readAirSwingVertical(buffer))
  {
    case DOMO_VERTICAL_AIR_AUTO:
      Serial.println(F("Auto"));
      break;
    case DOMO_VERTICAL_AIR_STRAIGHT:
      Serial.println(F("Straight"));
      break;
    case DOMO_VERTICAL_AIR_DOWN1:
      Serial.println(F("Down 1"));
      break;
    case DOMO_VERTICAL_AIR_DOWN2:
      Serial.println(F("Down 2"));
      break;
    case DOMO_VERTICAL_AIR_DOWN3:
      Serial.println(F("Down 3"));
      break;
    case DOMO_VERTICAL_AIR_DOWN4:
      Serial.println(F("Down 4"));
      break;
    default:
      Serial.print(F("UNDEFINED: "));
      Serial.println(readAirSwingVertical(buffer));
  }
  Serial.print(F("  Profile: "));
  switch(readProfile(buffer))
  {
    case DOMO_PROFILE_NORMAL: 
      Serial.println(F("Normal"));
      break;  
    case DOMO_PROFILE_QUIET: 
      Serial.println(F("Quiet"));
      break;  
    case DOMO_PROFILE_POWERFUL: 
      Serial.println(F("Powerful"));
      break;  
    default:
      Serial.print(F("UNDEFINED: "));
      Serial.println(readProfile(buffer));
  }
}
void verifyBuffer(int was, int expected, char *name)
{
    if(was != expected)
    {
      Serial.print(F("Test failed. Difference found in sent ir code for "));
      Serial.print(name);
      Serial.print(F(". Was: "));
      Serial.print(was);
      Serial.print(F(", Expected: "));
      Serial.println(expected);
    }
}
void verifyBuffer(float was, float expected, char *name)
{
    if(was != expected)
    {
      Serial.print(F("Test failed. Difference found in sent ir code for "));
      Serial.print(name);
      Serial.print(F(". Was: "));
      Serial.print(was);
      Serial.print(F(", Expected: "));
      Serial.println(expected);
    }
}
void verifyBuffer(bool was, bool expected, char *name)
{
    if(was != expected)
    {
      Serial.print(F("Test failed. Difference found in sent ir code for "));
      Serial.print(name);
      Serial.print(F(". Was: "));
      Serial.print(was);
      Serial.print(F(", Expected: "));
      Serial.println(expected);
    }
}

bool decodePanasonicCS(byte *bytes, int byteCount)
{
    // check if panasonic
    if(byteCount != 27){
      Serial.print(F("Received ir signal not 27 bytes! ("));
      Serial.print(byteCount);
      Serial.println(F("bytes)."));
      Serial.println(F("Using last 27 bytes"));
      bytes += byteCount-27;
      //return false;
    }

        // Print the byte array
  for (int i = 0; i < 27; i++) {
    if (bytes[i] < 0x10) {
      Serial.print(F("0"));
    }
    Serial.print(bytes[i],HEX);
    if ( i < byteCount - 1 ) {
      Serial.print(F(","));
    }
  }
  Serial.println(F(""));
            // checksum algorithm
        // Assumption: bitwise sum of payload bytes (so ignore header, and last byte)
        byte checksum = 0x06; // found differce, so start with 0x06
        //Serial.print(F("Add bytes: "));
        for (int i = 13; i < 26; i++){
            if (bytes[i] < 0x10) {
          //    Serial.print(F("0"));
            }
            //Serial.print(bytes[i],HEX);
            //Serial.print(F("+"));
            checksum = checksum + bytes[i];
        }
//        Serial.println(F("."));
  //      Serial.print(F("Checksum Truncate: "));
        checksum = (checksum & 0xFF); // mask out only first byte)
        Serial.print(F("Calculated checksum: "));
        Serial.print(checksum,HEX);
        if (checksum == bytes[26]){
            Serial.println(F(". Checksum OK"));
        }
        else
        {
            Serial.print(F(". Checksum not OK. Should be: "));
            Serial.println(bytes[26]);
            return false;
        }
    // Test if looks like Pansonic code
    // check via byteCount and fixed values
    if (bytes[10 == 0xE0]){ // I just picked a value here
        Serial.println(F("Look like a Panasonic CS protocol"));

        // check on-off
        // 13th byte from start, first byte with info
        // this is not a toggle, but a state
        if ((bytes[13] & 0x01) == 0x01) { // check if the right bit set 
            Serial.println(F("Powering ON!"));
        } else {
            Serial.println(F("Powering OFF"));
        }

      
        // check mode
        bool maintenanceHeat = !(bytes[14] & 0x20);
        
        // if this byte is used for other things, first mask the correct bits
        Serial.print(F("Mode: "));
        if(maintenanceHeat)
          Serial.println(F("Maintenance Heat"));
        else
          switch ((bytes[13]) & 0xF0){ // masks the first 4 bits 1111 0000
            case 0x00:
                Serial.println(F("Auto"));
                break;
            case 0x40:
                Serial.println(F("Heat"));
                break;
            case 0x30:
                Serial.println(F("Cool"));
                break;
            case 0x20:
                Serial.println(F("Dry"));
                break;
            case 0x60: 
                Serial.println(F("Fan"));
                break;
            default:
                Serial.println(F("Error"));
                break;
          }
        // check temp
        Serial.print(F("Temperature: "));
        byte temp = (bytes[14] & 0x1E);
        byte halftemp = (bytes[14] & 0x01);
        temp = temp >> 1;
        //Serial.print(offset);Serial.print(temp);Serial.println(bytes[14]);
        Serial.print((temp+(maintenanceHeat?0:16))); // masks the middle 5 bits: 0x1E = 0001 1110
        if(halftemp > 0)
          Serial.println(F(".5"));
        else
          Serial.println();

        // check fanspeed
        Serial.print(F("Fan speed: "));
        switch (bytes[16] & 0xF0){ // 0xF0 = 1111 0000 eerste 4 bits
            case 0xA0:
                Serial.println(F("FAN AUTO"));
                break;
            case 0x30:
                Serial.println(F("FAN 1"));
                break;
            case 0x40:
                Serial.println(F("FAN 2"));
                break;
            case 0x50:
                Serial.println(F("FAN 3"));
                break;    
            case 0x60:
                Serial.println(F("FAN 4"));
                break;    
            case 0x70:
                Serial.println(F("FAN 5"));
                break;
            default:
                Serial.println(F("Error"));
                break;
        }

        // check vertical swing
        Serial.print(F("Vertical swing: "));
        switch (bytes[16] & 0x0F){ // 0x0F = 0000 1111
            case 0x0F:
                Serial.println(F("AUTO"));
                break;
            case 0x01:
                Serial.println(F("Straight"));
                break;
            case 0x02:
                Serial.println(F("Down 1"));
                break;
            case 0x03:
                Serial.println(F("Down 2"));
                break;    
            case 0x04:
                Serial.println(F("Down 3"));
                break;    
            case 0x05:
                Serial.println(F("Down 4"));
                break;
            default:
                Serial.println(F("Error"));
                break;
        }

        // check horizontal swing
        Serial.print(F("Horizontal swing: "));
        switch (bytes[17] & 0x0F){ // 0x0F = 0000 1111
            case 0x0D:
                Serial.println(F("AUTO"));
                break;
            case 0x06:
                Serial.println(F("Center"));
                break;
            case 0x09:
                Serial.println(F("Left"));
                break;
            case 0x0A:
                Serial.println(F("Left center"));
                break;    
            case 0x0B:
                Serial.println(F("Right center"));
                break;    
            case 0x0C:
                Serial.println(F("Right"));
                break;
            default:
                Serial.println(F("Error"));
                break;
        }
        // Check profile
        Serial.print(F("Profile: "));
        switch(bytes[21]){
           case 0x00:
              Serial.println(F("Auto"));
              break;
           case 0x01:
              Serial.println(F("Powerful"));
              break;
           case 0x20:
              Serial.println(F("Quiet"));
              break;
           default:
              Serial.println(F("Error"));
        }

        // timer A (start)
        Serial.print(F("Timer A active: "));
        if((bytes[13] & 0x02) == 0x02){ //check if second bit in byte is active
            Serial.println(F("Yes"));
        } else {
            Serial.println(F("No"));
        }        

        // timer B (end)
        Serial.print(F("Timer B active: "));
        if((bytes[13] & 0x04) == 0x04){ //check if third bit in byte is active
            Serial.println(F("Yes"));
        } else {
            Serial.println(F("No"));
        }
        // no need to investigate actual timings if you're going to automate :-)



        // good end
        return true;
    }

    return false;
}

void fetchValues(byte *bytes)
{
  bool status = readStatus(bytes);
  if(current.status != status)
  {
      current.status=status;
      current.status_dirty=true;
  }

  int mode = readMode(bytes);
  if(current.getMode() != mode)
  {
    current.setMode(mode);
  }

  float setpoint = readSetpoint(bytes);
  if(current.getSetpoint() != setpoint)
    current.setSetpoint(setpoint);
/*  if(current.mode == DOMO_MODE_MAINTENANCE)
  {
    if(current.setpoint != setpoint)
    {
      current.setpoint_maintenance=setpoint;
      current.setpoint_dirty=true;
    }
  }
  else
  {
    if(current.setpoint != setpoint)
    {
      current.setpoint=setpoint;
      current.setpoint_dirty=true;
    }
  }*/

  int fan = readFan(bytes);
  if(current.getFan() != fan)
  {
    current.setFan(fan);
  }

  int profile = readProfile(bytes);
  if(current.profile != profile)
  {
    current.profile = profile;
    current.profile_dirty = true;  
  }
  int airSwing = readAirSwingVertical(bytes);
  if(current.airSwing != airSwing)
  {
    current.airSwing = airSwing;
    current.airSwing_dirty = true;
  }


}

bool readStatus(byte *bytes)
{
    return (bytes[13] &0x01)==0x01;
}

void setStatus(byte *bytes, bool status)
{
    bytes[13] = (bytes[13] &0xFE)|(status?0x01:0x00);
}


int readMode(byte *bytes)
{
        // check mode
        bool maintenanceHeat = !(bytes[14] & 0x20);

        if(maintenanceHeat)
          return DOMO_MODE_MAINTENANCE;

        // if this byte is used for other things, first mask the correct bits
          switch ((bytes[13]) & 0xF0){ // masks the first 4 bits 1111 0000
            case PANASONIC_AIRCON2_MODE_AUTO: // auto
                return DOMO_MODE_AUTO;
            case PANASONIC_AIRCON2_MODE_HEAT: // heat
                return DOMO_MODE_HEAT;
            case PANASONIC_AIRCON2_MODE_COOL: // cool
                return DOMO_MODE_COOL;
            case PANASONIC_AIRCON2_MODE_DRY: // dry
                return DOMO_MODE_DRY;
            case PANASONIC_AIRCON2_MODE_FAN: // fan
                return DOMO_MODE_FAN;
            default:
                return DOMO_UNDEFINED;
          }
  
}

void setMode(byte *bytes, int mode)
{
  switch(mode)
  {
    case DOMO_MODE_AUTO:
      bytes[13] = (bytes[13]&0xF)|PANASONIC_AIRCON2_MODE_AUTO;
      break;
    case DOMO_MODE_HEAT:
    case DOMO_MODE_MAINTENANCE:
      bytes[13] = (bytes[13]&0xF)|PANASONIC_AIRCON2_MODE_HEAT;
      break;
    case DOMO_MODE_COOL:
      bytes[13] = (bytes[13]&0xF)|PANASONIC_AIRCON2_MODE_COOL;
      break;
    case DOMO_MODE_DRY:
      bytes[13] = (bytes[13]&0xF)|PANASONIC_AIRCON2_MODE_DRY;
      break;
    case DOMO_MODE_FAN:
      bytes[13] = (bytes[13]&0xF)|PANASONIC_AIRCON2_MODE_FAN;
      break;
  }
}
float readSetpoint(byte *bytes)
{
            // check mode
    //    bool maintenanceHeat = !(bytes[14] & 0x20);

        // check temp
        return ((int)bytes[14])/2.0f;
/*        byte temp = (bytes[14] & 0x1E);
        byte halftemp = (bytes[14] & 0x01);
        temp = temp >> 1;
        float value = (temp+(maintenanceHeat?0:16));
        if(halftemp > 0)
          value+=0.5;
        return value;*/
}

void setSetpoint(byte *bytes, float setpoint)
{
  // Make sure its valid
  byte data =((int)(setpoint*2.0f));
  bytes[14]=data;
  // if temp < 16.0
        // check mode
/*        bool maintenanceHeat = !(bytes[14] & 0x20);

        // check temp
        byte temp = (bytes[14] & 0x1E);
        byte halftemp = (bytes[14] & 0x01);
        temp = temp >> 1;
        float value = (temp+(maintenanceHeat?0:16));
        if(halftemp > 0)
          value+=0.5;
        return value;*/
}


int readFan(byte *bytes)
{
          // check fanspeed
        switch (bytes[16] & 0xF0){ 
            case PANASONIC_AIRCON2_FAN_AUTO:
                return DOMO_FAN_AUTO; 
            case PANASONIC_AIRCON2_FAN1:
                return DOMO_FAN_1; 
            case PANASONIC_AIRCON2_FAN2:
                return DOMO_FAN_2; 
            case PANASONIC_AIRCON2_FAN3:
                return DOMO_FAN_3; 
            case PANASONIC_AIRCON2_FAN4:
                return DOMO_FAN_4; 
            case PANASONIC_AIRCON2_FAN5:
                return DOMO_FAN_5; 
            default:
              return DOMO_UNDEFINED;
        }
}
void setFan(byte *bytes, int fan)
{
    switch(fan)
  {
    case DOMO_FAN_AUTO:
      bytes[16] = (bytes[16]&0xF)|PANASONIC_AIRCON2_FAN_AUTO;
      break;
    case DOMO_FAN_1:
      bytes[16] = (bytes[16]&0xF)|PANASONIC_AIRCON2_FAN1;
      break;
    case DOMO_FAN_2:
      bytes[16] = (bytes[16]&0xF)|PANASONIC_AIRCON2_FAN2;
      break;
    case DOMO_FAN_3:
      bytes[16] = (bytes[16]&0xF)|PANASONIC_AIRCON2_FAN3;
      break;
    case DOMO_FAN_4:
      bytes[16] = (bytes[16]&0xF)|PANASONIC_AIRCON2_FAN4;
      break;
    case DOMO_FAN_5:
      bytes[16] = (bytes[16]&0xF)|PANASONIC_AIRCON2_FAN5;
      break;
  }
}

int readProfile(byte *bytes)
{
          // Check profile
        switch(bytes[21]){
           case PANASONIC_AIRCON2_PROFILE_NORMAL:
              return DOMO_PROFILE_NORMAL; 
           case PANASONIC_AIRCON2_POWERFUL:
              return DOMO_PROFILE_POWERFUL;
           case PANASONIC_AIRCON2_QUIET:
              return DOMO_PROFILE_QUIET; 
           default:
              return DOMO_UNDEFINED;
        }
}
void setProfile(byte *bytes, int profile)
{
          // Check profile
        switch(profile){
           case DOMO_PROFILE_NORMAL:
              bytes[21] = PANASONIC_AIRCON2_PROFILE_NORMAL;
              break;
           case DOMO_PROFILE_POWERFUL:
              bytes[21] = PANASONIC_AIRCON2_POWERFUL;
              break;
           case DOMO_PROFILE_QUIET:
              bytes[21] = PANASONIC_AIRCON2_QUIET;
              break;
        }
}


int readAirSwingVertical(byte *bytes)
{
          // check vertical swing
        switch (bytes[16] & 0x0F){ // 0x0F = 0000 1111
            case PANASONIC_AIRCON2_VS_AUTO:
                return DOMO_VERTICAL_AIR_AUTO; // auto
            case PANASONIC_AIRCON2_VS_UP:
                return DOMO_VERTICAL_AIR_STRAIGHT; // Straight
            case PANASONIC_AIRCON2_VS_MUP:
                return DOMO_VERTICAL_AIR_DOWN1; // down 1
            case PANASONIC_AIRCON2_VS_MIDDLE:
                return DOMO_VERTICAL_AIR_DOWN2; // down 2
            case PANASONIC_AIRCON2_VS_MDOWN:
                return DOMO_VERTICAL_AIR_DOWN3; // down 3
            case PANASONIC_AIRCON2_VS_DOWN:
                return DOMO_VERTICAL_AIR_DOWN4; // down 4
            default:
                return DOMO_UNDEFINED;
        }
}
void setAirSwingVertical(byte *bytes, int airswing)
{

          // check vertical swing
        switch (airswing){
            case DOMO_VERTICAL_AIR_AUTO:
                bytes[16] = (bytes[16] & 0xF0) | PANASONIC_AIRCON2_VS_AUTO;
                break;
            case DOMO_VERTICAL_AIR_STRAIGHT:
                bytes[16] = (bytes[16] & 0xF0) | PANASONIC_AIRCON2_VS_UP;
                break;
            case DOMO_VERTICAL_AIR_DOWN1:
                bytes[16] = (bytes[16] & 0xF0) | PANASONIC_AIRCON2_VS_MUP;
                break;
            case DOMO_VERTICAL_AIR_DOWN2:
                bytes[16] = (bytes[16] & 0xF0) | PANASONIC_AIRCON2_VS_MIDDLE;
                break;
            case DOMO_VERTICAL_AIR_DOWN3:
                bytes[16] = (bytes[16] & 0xF0) | PANASONIC_AIRCON2_VS_MDOWN;
                break;
            case DOMO_VERTICAL_AIR_DOWN4:
                bytes[16] = (bytes[16] & 0xF0) | PANASONIC_AIRCON2_VS_DOWN;
                break;
        }
}

int readAirSwingHorizontal(byte *bytes)
{
          // check horizontal swing
        switch (bytes[17] & 0x0F){ // 0x0F = 0000 1111
            case 0x0D:
                return 0; // auto
            case 0x06:
                return 30; // center;
            case 0x09:
                return 10; // left
            case 0x0A:
                return 20; // left center
            case 0x0B:
                return 40; // right center
            case 0x0C:
                return 50; // right
            default:
                return 100;
        }
}

void sendPanasonic(IRSender& IR, byte *buffer)
{
/*  switch(_panasonicModel)
  {
    case PANASONIC_DKE:
      panasonicTemplate[17] = swingH; // Only the DKE model has a setting for the horizontal air flow
      panasonicTemplate[23] = 0x01;
      panasonicTemplate[25] = 0x06;
      break;
    case PANASONIC_JKE:
      break;
    case PANASONIC_NKE:
      panasonicTemplate[17] = 0x06;
      break;
    case PANASONIC_LKE:
      panasonicTemplate[17] = 0x06;
      panasonicTemplate[13] = 0x02;
      break;
  }
*/

  // Checksum calculation
  uint8_t checksum = 0xF4;

  for (int i=0; i<26; i++) {
    checksum += buffer[i];
  }

  buffer[26] = checksum;

  // Print the byte array
  Serial.print(F("  IR-byte array: "));
  for (int i = 0; i < 27; i++) {
    if (buffer[i] < 0x10) {
      Serial.print(F("0"));
    }
    Serial.print(buffer[i],HEX);
    if ( i < 27 - 1 ) {
      Serial.print(F(","));
    }
  }
  Serial.println(F(""));
  
  // 38 kHz PWM frequency
  IR.setFrequency(38);

  // Header
  IR.mark(PANASONIC_AIRCON2_HDR_MARK);
  IR.space(PANASONIC_AIRCON2_HDR_SPACE);

  // First 8 bytes
  for (int i=0; i<8; i++) {
    IR.sendIRbyte(buffer[i], PANASONIC_AIRCON2_BIT_MARK, PANASONIC_AIRCON2_ZERO_SPACE, PANASONIC_AIRCON2_ONE_SPACE);
  }

  // Pause
  IR.mark(PANASONIC_AIRCON2_BIT_MARK);
  IR.space(PANASONIC_AIRCON2_MSG_SPACE);

  // Header
  IR.mark(PANASONIC_AIRCON2_HDR_MARK);
  IR.space(PANASONIC_AIRCON2_HDR_SPACE);

  // Last 19 bytes
  for (int i=8; i<27; i++) {
    IR.sendIRbyte(buffer[i], PANASONIC_AIRCON2_BIT_MARK, PANASONIC_AIRCON2_ZERO_SPACE, PANASONIC_AIRCON2_ONE_SPACE);
  }

  IR.mark(PANASONIC_AIRCON2_BIT_MARK);
  IR.space(0);
  
//  memcpy_P(lastSendBuffer, panasonicTemplate, sizeof(panasonicTemplate));
}

