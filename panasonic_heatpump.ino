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
 * Mysensor node thats emulate a Panasonic heat pump remote for Domoticz
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Channel 1
//#define MY_RF24_CHANNEL  76

// Safe baud rate for a 3.3V device
#define MY_BAUD_RATE 9600

// MySensors libraries
#include <SPI.h>
#include <MySensors.h>

// From library at https://github.com/ToniA/arduino-heatpumpir
#include <IRSender.h>
#include <PanasonicHeatpumpIR.h>

// LCD
#include <LiquidCrystal_I2C.h>

// Initialize display. Google the correct settings for your display. 
// The follwoing setting should work for the recommended display in the MySensors "shop".
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

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

// Buffer for received ir data
#define bufferLength 300
byte irbytes[bufferLength];

#define IRpin_PIN   PIND
#define IRpin       2
// the maximum pulse we'll listen for - 65 milliseconds is a long time
#define MAXPULSE 65000
// The thresholds for different symbols
// Panasonic cs
#define    MARK_THRESHOLD_BIT_HEADER     2000
#define     SPACE_THRESHOLD_ZERO_ONE      800
#define     SPACE_THRESHOLD_ONE_HEADER   1500
#define    SPACE_THRESHOLD_HEADER_PAUSE 8000


// what our timing resolution should be, larger is better
// as its more 'precise' - but too large and you wont get
// accurate timing
uint16_t RESOLUTION=20;

// Child ID's of this node
#define CHILD_STATUS     1
#define CHILD_SETPOINT   2
#define CHILD_MODE       3
#define CHILD_FAN        4
#define CHILD_AIRSWING   5
#define CHILD_PROFILE    6
#define CHILD_VAR        7
    
class HeatpumpModel
{ 
  private:
    bool isMaintenance = false;
    void triggerMaintenanceDirty()
    {
        modeIsDirty = true;
        fanIsDirty=true;
        setpointIsDirty=true;
        profileIsDirty=true;
    }
  public:
    bool status = 1;
    bool statusIsDirty = false;
    
    int mode = 10;
    bool modeIsDirty = false;
    
    float setpointAuto = 20;
    float setpointHeat = 19;
    float setpointCool = 25;
    float setpointDry = 16;
    float setpointMaintenance = 10;
    bool setpointIsDirty = false;
    
    int fanAuto = 10;
    int fanHeat = 10;
    int fanCool = 10;
    int fanDry = 10;
    int fanFan = 20;
    bool fanIsDirty = false;
    
    int airSwing = DOMO_VERTICAL_AIR_AUTO;
    bool airSwingIsDirty = false;
    
    int profile = 10;
    bool profileIsDirty = false;
    
    bool var1IsDirty = false;
    bool var2IsDirty = false;

    void setSetpoint(float temp)
    {
        if(temp > 30.0 || temp < 8.0)
         {
          setpointIsDirty=true;
          return;
         }
        if(temp < 16.0)
        {
          switch(mode)
          {
            case DOMO_MODE_COOL:
            case DOMO_MODE_DRY:
            case DOMO_MODE_FAN:
              setpointIsDirty = true;
              return;
          }
          setpointMaintenance = temp;
          var2IsDirty=true;
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
              setpointAuto=temp;
              break;
            case DOMO_MODE_HEAT:
              setpointHeat=temp;
              break;
            case DOMO_MODE_COOL:
              setpointCool=temp;
              break;
            case DOMO_MODE_DRY:
              setpointDry=temp;
              break;
            case DOMO_MODE_FAN:
              break;
          }
          var1IsDirty = true;
        }
        setpointIsDirty=true;
    }

    float getSetpoint()
    {
      if(isMaintenance)
        return setpointMaintenance;
      switch(mode)
      {
        case DOMO_MODE_AUTO:
          return setpointAuto;
        case DOMO_MODE_HEAT:
          return setpointHeat;
        case DOMO_MODE_COOL:
          return setpointCool;
        case DOMO_MODE_DRY:
          return setpointDry;
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
          return fanAuto;
        case DOMO_MODE_HEAT:
          return fanHeat;
        case DOMO_MODE_COOL:
          return fanCool;
        case DOMO_MODE_DRY:
          return fanDry;
        case DOMO_MODE_FAN:
          return fanFan;
      }
    }

    void setFan(int newfan)
    {
      fanIsDirty = true;
      var2IsDirty=true;
      switch(mode)
      {
        case DOMO_MODE_AUTO:
          fanAuto=newfan;
          break;
        case DOMO_MODE_HEAT:
          fanHeat=newfan;
          break;
        case DOMO_MODE_COOL:
          fanCool=newfan;
          break;
        case DOMO_MODE_DRY:
          fanDry=newfan;
          break;
        case DOMO_MODE_FAN:
          fanFan=newfan;
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
      profileIsDirty=true;
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
        modeIsDirty=true;
      }
      setpointIsDirty = true;
      fanIsDirty = true;
    }
} model;

struct VAR1_DATA
{
    byte setpointAuto;
    byte setpointHeat;
    byte setpointCool;
    byte setpointDry;
} ;
struct VAR2_DATA
{
  byte setpointMaintenance;
  byte fanAuto:4;
  byte fanHeat:4;
  byte fanCool:4;
  byte fanDry:4;
  byte fanFan:4;
};

long lastTransmitt = 0;
long lastUpdateSent = 0;
long lastCommandReceived = 0;
bool interuptTriggered = false;

// MySensors messages of this node
MyMessage msgSetpoint(CHILD_SETPOINT, V_HVAC_SETPOINT_HEAT);
MyMessage msgStatus(CHILD_STATUS, V_STATUS);
MyMessage msgMode(CHILD_MODE,V_PERCENTAGE);
MyMessage msgFan(CHILD_FAN,V_PERCENTAGE);
MyMessage msgAirSwing(CHILD_AIRSWING,V_PERCENTAGE);
MyMessage msgProfile(CHILD_PROFILE,V_PERCENTAGE);
MyMessage msgVar1(CHILD_VAR, V_VAR1);
MyMessage msgVar2(CHILD_VAR, V_VAR2);

// IR led on PWM output-capable digital pin 3
IRSenderPWM irSender(3);


void setup()
{
  Serial.println(F("Panasonic Heatpump Remote starting up..."));

  // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.begin(16,2); 
  lcd.home();
  lcd.print(F("Panasonic"));
  lcd.setCursor(0,1);
  lcd.print(F("Starting up"));
  
  request(msgVar1.sensor, msgVar1.type);
  request(msgVar2.sensor, msgVar2.type);
  request(msgStatus.sensor, msgStatus.type);
  request(msgMode.sensor, msgMode.type);
  request(msgAirSwing.sensor, msgAirSwing.type);
  request(msgProfile.sensor, msgProfile.type);
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Panasonic heatpump remote", "1.0");

  // Register the sensors to the MySensors Gateway
  present(CHILD_STATUS,S_HVAC); // onoff
  present(CHILD_SETPOINT,S_HVAC, "Temperature"); // setpoint
  present(CHILD_MODE,S_DIMMER, "Mode"); // mode
  present(CHILD_FAN,S_DIMMER, "Fan speed"); // Fan speed
  present(CHILD_AIRSWING,S_DIMMER, "Air Swing"); // Air swing
  present(CHILD_PROFILE,S_DIMMER, "Profile"); // profile

  // Listen for ir
  attachInterrupt(digitalPinToInterrupt(IRpin), interuptHandler, RISING);
}

void interuptHandler()
{
  interuptTriggered = true;
}

void loop()
{
  // Interrupt triggered - detected possible IR code
  if(interuptTriggered)
  {
    Serial.println("Start listening for IR codes");
    long beforePulses = millis();
    int count = receivePulses();
    long afterPulses = millis();
    interuptTriggered = false;

    Serial.print(F("  Received "));
    Serial.print(count);
    Serial.print(F(" bytes in "));
    Serial.print(afterPulses - beforePulses);
    Serial.println(F("ms."));

    if(count > 0 && decodePanasonicCS(irbytes, count))
    {
      transferRecivedDataToModel(irbytes);
      updateDisplay();
    }
  }
  // If we received new command since last IR transmitt - transmitt model to heat pump
  if(lastTransmitt < lastCommandReceived)
  {
    lastTransmitt = millis();
    // Transmitt new values
    sendHeatpumpIRCommand();
    // Dont trigger on our own transmitt
    delay(10); // This delay is needed as transmitted signals might be picked up by our receiver (sensor delay)
    interuptTriggered = false; 
    updateDisplay();
  }
  // Send updates
  if(model.statusIsDirty)
  {
    send(msgStatus.set(model.status));
    model.statusIsDirty = false;
  }
  if(model.modeIsDirty)
  {
    send(msgMode.set(model.getMode()));
    model.modeIsDirty = false;
  }
  // If dirty or not updated in 25 minutes (to prevent value from beeing "lost" in domoticz)
  if(model.setpointIsDirty || (millis()-lastUpdateSent)>(25*60000))
  {
    send(msgSetpoint.set(model.getSetpoint(), 1));
    model.setpointIsDirty = false;
    lastUpdateSent = millis();
  }
  if(model.fanIsDirty)
  {
    send(msgFan.set(model.getFan()));
    model.fanIsDirty = false;
  }
  if(model.profileIsDirty)
  {
    send(msgProfile.set(model.getProfile()));
    model.profileIsDirty = false;
  }
  if(model.airSwingIsDirty)
  {
    send(msgAirSwing.set(model.airSwing));
    model.airSwingIsDirty = false;
  }
  if(model.var1IsDirty)
  {
    uint32_t value1 = 0;
    VAR1_DATA *var1 = (VAR1_DATA*)(&value1);
    var1->setpointAuto=model.setpointAuto*2.0;
    var1->setpointHeat=model.setpointHeat*2.0;
    var1->setpointCool=model.setpointCool*2.0;
    var1->setpointDry=model.setpointDry*2.0;
    send(msgVar1.set(value1));
    model.var1IsDirty = false;
  }
  if(model.var2IsDirty)
  {
    uint32_t value2 = 0;
    VAR2_DATA *var2 = (VAR2_DATA*)(&value2);
    var2->setpointMaintenance=model.setpointMaintenance*2.0;
    var2->fanAuto = model.fanAuto/10;
    var2->fanHeat = model.fanHeat/10;
    var2->fanCool = model.fanCool/10;
    var2->fanDry = model.fanDry/10;
    var2->fanFan = model.fanFan/10;
    send(msgVar2.set(value2));
    model.var2IsDirty = false; 
  }
}

// Handle incoming messages from the MySensors Gateway
void receive(const MyMessage &message) 
{
  bool triggerSend = false;
  if(message.sensor == msgStatus.sensor)
  {
    model.status = message.getBool();
    model.statusIsDirty = true;
    triggerSend = true;
  }
  else if(message.sensor == msgMode.sensor)
  {
    model.setMode(message.getInt());
    triggerSend = true;
  }
  else if(message.sensor == msgFan.sensor)
  {
    model.setFan(message.getInt());
    triggerSend = true;
  }
  else if(message.sensor == msgAirSwing.sensor)
  { 
    model.airSwing = message.getInt();
    model.airSwingIsDirty = true;
    triggerSend = true;
  }
  else if(message.sensor == msgProfile.sensor)
  {
    model.profile = message.getInt();
    model.profileIsDirty = true;
    triggerSend = true;
  }
  else if(message.sensor == msgSetpoint.sensor)
  {
    model.setSetpoint(message.getFloat());
    triggerSend = true;
  }
  // If we received changes - transmitt new codes 
  // TODO detect if something changed
  if(triggerSend)
  {
    lastCommandReceived = millis();
  }
  // We received saved values - update model
  if(message.sensor == msgVar1.sensor && message.type == msgVar1.type)
  {
    uint32_t value = message.getLong();
    if(message.getLong()>0)
    {
      VAR1_DATA *var1 = (VAR1_DATA*)(&value);
      //long value = message.getLong();
      float tmp = var1->setpointAuto/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        model.setpointAuto = tmp;
      tmp = var1->setpointHeat/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        model.setpointHeat = tmp;
      tmp = var1->setpointCool/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        model.setpointCool = tmp;
      tmp = var1->setpointDry/2.0;
      if(tmp >=16.0 && tmp <= 30.0)
        model.setpointDry = tmp;
      Serial.println(F("Received saved values:"));
      Serial.print(F("  setpointAuto: "));
      Serial.println(model.setpointAuto);
      Serial.print(F("  setpointHeat: "));
      Serial.println(model.setpointHeat);
      Serial.print(F("  setpointCool: "));
      Serial.println(model.setpointCool);
      Serial.print(F("  setpointDry: "));
      Serial.println(model.setpointDry);
      model.setpointIsDirty = true;
    }
  }
  // We received saved values - update model
  if(message.sensor == msgVar2.sensor && message.type == msgVar2.type)
  {
    uint32_t value = message.getLong();
    if(message.getLong()>0)
    {
      VAR2_DATA *var2 = (VAR2_DATA*)(&value);
      float tmp = var2->setpointMaintenance/2.0f;
      if(tmp >=8 && tmp < 16)
        model.setpointMaintenance = tmp;
      int fan = var2->fanAuto*10;
      model.fanAuto=fan;
      fan = var2->fanHeat*10;
      model.fanHeat=fan;
      fan = var2->fanCool*10;
      model.fanCool=fan;
      fan = var2->fanDry*10;
      model.fanDry=fan;
      fan = var2->fanFan*10;
      model.fanFan=fan;
      Serial.println(F("Received saved values:"));
      Serial.print(F("  setpointMaintenance: "));
      Serial.println(model.setpointMaintenance);
      Serial.print(F("  fanAuto: "));
      Serial.println(model.fanAuto);      
      Serial.print(F("  fanHeat: "));
      Serial.println(model.fanHeat);      
      Serial.print(F("  fanCool: "));
      Serial.println(model.fanCool);      
      Serial.print(F("  fanDry: "));
      Serial.println(model.fanDry);      
      Serial.print(F("  fanFan: "));
      Serial.println(model.fanFan);      
    }
  }
}

int receivePulses(void)
{
  // If ir-pin is low - don't try to read
  if((IRpin_PIN & (1 << IRpin))==0)
    return 0;
  
  uint16_t highpulse, lowpulse;  // temporary storage timing
  char symbolBuffer[2];
  memset(symbolBuffer, 0, sizeof(symbolBuffer));

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
  uint16_t currentpulse = 0; // index for pulses we're storing
  
  // Only Panasonic seems to use the pause
  space_pause_avg = 0;
  space_pause_cnt = 0;

  int currbyte = 0; 
  int currbit = 0;
  memset(irbytes, 0, sizeof(bufferLength));
  int rbytes=0;
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

// Send current model to heat pump
void sendHeatpumpIRCommand() 
{
  // IR template from NZ9SKE remote 
  static const uint8_t irtemplate[27] PROGMEM = {
    0x02, 0x20, 0xE0, 0x04, 0x00, 0x00, 0x00, 0x06, 0x02, 0x20, 0xE0, 0x04, 0x00, 0x08, 0x00, 0x80, 0x00, 0x00, 0x00, 0x0E, 0xE0, 0x00, 0x00, 0x89, 0x00, 0x00, 0x00
  //   0     1     2     3     4     5     6     7     8     9    10    11    12    13    14   15     16    17    18    19    20    21    22    23    24    25    26
  };
  // Create a copy 
  uint8_t buff[27];
  memcpy_P(buff, irtemplate, sizeof(irtemplate));
  // set values
  setStatus(buff, model.status);
  setMode(buff, model.getMode());
  setSetpoint(buff, model.getSetpoint());
  setAirSwingVertical(buff, model.airSwing);
  setFan(buff, model.getFan());
  setProfile(buff, model.profile);
  // Debug info - ir hex codes 
  Serial.println(F("Sending IR code to heatpump: "));
  sendPanasonic(irSender, buff);
  // Debug info - human readable
  printBuffer(buff);
  // Debug info - Verify that we can read the same values as we set from the buffer
  verifyBuffer( readStatus(buff), model.status, "status");
  verifyBuffer( readMode(buff), model.mode, "mode");
  verifyBuffer( readSetpoint(buff), model.getSetpoint(), "setpoint");
  verifyBuffer( readFan(buff), model.getFan(), "fan");
  verifyBuffer( readAirSwingVertical(buff), model.airSwing, "airSwing");
  verifyBuffer( readProfile(buff), model.profile, "profile");
}

// Print send buffer (ir codes) in human readable code
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

// Checks if this seems to be a valid ir code
bool decodePanasonicCS(byte *bytes, int byteCount)
{
  if(byteCount < 27)
  {
    Serial.println(F("Received less then 27 bytes over IR. Skipping parse."));
    return false;
  }
  // check if panasonic
  else if(byteCount > 27)
  {
    Serial.print(F("Received more then 27 bytes ("));
    Serial.print(byteCount);
    Serial.println(F("bytes) over IR. Parsing last 27 bytes as Panasonic CS:"));
    bytes += byteCount-27;
  }
  else
  {
    Serial.println(F("Received 27 bytes over IR. Parsing as Panasonic CS:"));
  }

  // Print the byte array
  Serial.print(F("  IR-byte array: "));
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

  // Check checksum
  // Assumption: bitwise sum of payload bytes (so ignore header, and last byte)
  byte checksum = 0x06; // found differce, so start with 0x06
  for (int i = 13; i < 26; i++){
    checksum = checksum + bytes[i];
  }
  checksum = (checksum & 0xFF); // mask out only first byte)
  Serial.print(F("  Calculated checksum: "));
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
  if (bytes[10] != 0xE0)
  { 
    Serial.println(F("Doesn't look like Panasonic CS protocol. Skipping parse."));
    return false;
  }
  // Print received values as human readable
  printBuffer(bytes);
  // Print extra info not displayed in above print

  // check horizontal swing
  Serial.print(F("  Horizontal swing: "));
  switch (bytes[17] & 0x0F)
  {
    case PANASONIC_AIRCON2_HS_AUTO:
      Serial.println(F("AUTO"));
      break;
    case PANASONIC_AIRCON2_HS_MIDDLE:
      Serial.println(F("Center"));
      break;
    case PANASONIC_AIRCON2_HS_LEFT:
      Serial.println(F("Left"));
      break;
    case PANASONIC_AIRCON2_HS_MLEFT:
      Serial.println(F("Left center"));
      break;    
    case PANASONIC_AIRCON2_HS_MRIGHT:
      Serial.println(F("Right center"));
      break;    
    case PANASONIC_AIRCON2_HS_RIGHT:
      Serial.println(F("Right"));
      break;
    default:
      Serial.print(F("UNDEFINED ("));
      Serial.print(bytes[17]&0x0F);
      Serial.println(F(")"));
      break;
  }

  // timer A (start)
  Serial.print(F("  Timer A active: "));
  if((bytes[13] & 0x02) == 0x02){ //check if second bit in byte is active
    Serial.println(F("Yes"));
  } else {
    Serial.println(F("No"));
  }        

  // timer B (end)
  Serial.print(F("  Timer B active: "));
  if((bytes[13] & 0x04) == 0x04){ //check if third bit in byte is active
    Serial.println(F("Yes"));
  } else {
    Serial.println(F("No"));
  }
  // Parsing as Panasonic CS ok
  return true;
}

// Read data from received buffer and set the correct values in the model
void transferRecivedDataToModel(byte *bytes)
{
  bool status = readStatus(bytes);
  if(model.status != status)
  {
      model.status=status;
      model.statusIsDirty=true;
  }
  int mode = readMode(bytes);
  if(model.getMode() != mode)
  {
    model.setMode(mode);
  }
  float setpoint = readSetpoint(bytes);
  if(model.getSetpoint() != setpoint)
    model.setSetpoint(setpoint);
  int fan = readFan(bytes);
  if(model.getFan() != fan)
  {
    model.setFan(fan);
  }
  int profile = readProfile(bytes);
  if(model.profile != profile)
  {
    model.profile = profile;
    model.profileIsDirty = true;  
  }
  int airSwing = readAirSwingVertical(bytes);
  if(model.airSwing != airSwing)
  {
    model.airSwing = airSwing;
    model.airSwingIsDirty = true;
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
  return ((int)bytes[14])/2.0f;
}

void setSetpoint(byte *bytes, float setpoint)
{
  byte data =((int)(setpoint*2.0f));
  bytes[14]=data;
}

int readFan(byte *bytes)
{
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
  switch(fan){
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
  switch (bytes[16] & 0x0F){
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

void sendPanasonic(IRSender& IR, byte *buffer)
{
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
}

void updateDisplay()
{
  lcd.clear();
  lcd.home();
  switch(model.getMode())
  {
    case DOMO_MODE_AUTO:
      lcd.print(F("Auto"));
      break;
    case DOMO_MODE_HEAT:
      lcd.print(F("Heat"));
      break;
    case DOMO_MODE_COOL:
      lcd.print(F("Cool"));
      break;
    case DOMO_MODE_DRY:
      lcd.print(F("Dry"));
      break;
    case DOMO_MODE_FAN:
      lcd.print(F("Fan "));
      break;
    case DOMO_MODE_MAINTENANCE:
      lcd.print(F("Maintenance"));
      break;
    }
    if(model.getMode()!=DOMO_MODE_FAN)
    {
      lcd.setCursor(12,0);
      lcd.print(model.getSetpoint(),1);
    }
    if(model.getMode()!=DOMO_MODE_MAINTENANCE)
    {
      lcd.setCursor(6,0);
      if(model.getProfile()==DOMO_PROFILE_POWERFUL)
      {
        lcd.print(F("Power"));
      }
      else if(model.getProfile()==DOMO_PROFILE_QUIET)
      {
        lcd.print(F("Quiet"));
      }
    }
    lcd.setCursor(0,1);
    lcd.print(F("Fan "));
    bool printedAuto = false;
    switch(model.airSwing)
    {
      case DOMO_VERTICAL_AIR_AUTO:
        lcd.print(F("Auto"));
        printedAuto = true;
        break;
      case DOMO_VERTICAL_AIR_STRAIGHT:
        lcd.print(F("15"));
        lcd.print((char)223);
        break;
      case DOMO_VERTICAL_AIR_DOWN1:
        lcd.print(F("22"));
        lcd.print((char)223);
        break;
      case DOMO_VERTICAL_AIR_DOWN2:
        lcd.print(F("30"));
        lcd.print((char)223);
        break;
      case DOMO_VERTICAL_AIR_DOWN3:
        lcd.print(F("37"));
        lcd.print((char)223);
        break;
      case DOMO_VERTICAL_AIR_DOWN4:
        lcd.print(F("45"));
        lcd.print((char)223);
        break;
      default:
        lcd.print(F("UNDEFINED"));
    }
    lcd.print(F(" "));
    if(model.getFan()==DOMO_FAN_AUTO)
    {
      if(printedAuto == false)
        lcd.print(F("Auto"));
    }
    else
    {
      lcd.print(model.getFan()/10-1);
      lcd.print(F("/5"));
    }
}

