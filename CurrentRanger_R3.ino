// *************************************************************************************************************
// CurrentRanger(TM) stock firmware
// https://lowpowerlab.com/CurrentRanger
// CurrentRanger is a *high-side* precision current meter featuring:
//   - fast autoranging
//   - uni/bi-directional modes (ie. DC/AC measurements)
//   - ultra low burden voltage
//   - 1mV per nA/uA/mA measurements with DMM/scope
//   - OLED standalone readings
//   - serial data logging option via 3.3v/RX/TX header or USB (must use isolation, read guide!)
//   - full digital control for power/switching
//   - LiPo powered with auto power-off feature (0.6uA quiescent current)
// *************************************************************************************************************
#ifndef CURRENT_RANGER
  #error CurrentRanger target board required, see guide on how to add it to the IDE: lowpowerlab.com/currentranger
#endif
//***********************************************************************************************************
#include <FlashStorage.h>          //for emulated EEPROM - https://github.com/cmaglie/FlashStorage
#include <Adafruit_FreeTouch.h>    //https://github.com/adafruit/Adafruit_FreeTouch
#include <U8g2lib.h>               //https://github.com/olikraus/u8g2/wiki/u8g2reference fonts:https://github.com/olikraus/u8g2/wiki/fntlistall
//#include <ATSAMD21_ADC.h>
//***********************************************************************************************************
#define BIAS_LED       11
#define LPFPIN         4
#define LPFLED         LED_BUILTIN
#define AUTOFF         PIN_AUTO_OFF
//***********************************************************************************************************
#define MA_PIN PIN_PA13  //#define MA  38
#define UA_PIN PIN_PA14  //#define UA  2
#define NA_PIN PIN_PA15  //#define NA  5
#define MA_GPIO_PIN PIN_PB11
#define UA_GPIO_PIN PIN_PA12
#define NA_GPIO_PIN PIN_PB10
#define PINOP(pin, OP) (PORT->Group[(pin) / 32].OP.reg = (1 << ((pin) % 32)))
#define PIN_OFF(THE_PIN) PINOP(THE_PIN, OUTCLR)
#define PIN_ON(THE_PIN) PINOP(THE_PIN, OUTSET)
#define PIN_TGL(THE_PIN) PINOP(THE_PIN, OUTTGL)
//***********************************************************************************************************
#define SENSE_OUTPUT           A3
#define SENSE_GNDISO           A2
#define SENSE_VIN              A5
#define ADC_PRESCALER          ADC_CTRLB_PRESCALER_DIV16
//#define ADC_AVGCTRL            ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4ul)
                               //ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x00ul);  // take 1 sample, adjusting result by 0
                               //ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); //take 16 samples adjust by 4
                               //ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4ul); //take 256 samples adjust by 4
                               //ADC_AVGCTRL_SAMPLENUM_512 | ADC_AVGCTRL_ADJRES(0x4ul); //take 512 samples adjust by 4
                               //ADC_AVGCTRL_SAMPLENUM_1024 | ADC_AVGCTRL_ADJRES(0x4ul); //take 1024 samples adjust by 4
#define ADC_SAMPCTRL           0b111 //sample timing [fast 0..0b111 slow]
#define ADCFULLRANGE           4095.0
#define VBATREADLOOPS          100  //read vbat every this many OLED_REFRESH_INTERVAL loops
#define LOBAT_THRESHOLD        3.40 //volts
#define DAC_GND_ISO_OFFSET     10
#define DAC_HALF_SUPPLY_OFFSET 512
#define OUTPUT_CALIB_FACTOR    1.00  //calibrate final VOUT value
#define ADC_OVERLOAD           3900  //assuming GNDISO DAC output is very close to 0, this is max value less ground offset (varies from unit to unit, 3900 is a safe value)
//***********************************************************************************************************
//#define ADC_CALIBRATE_FORCED
#define ADC_CALIBRATE_FORCED_OFFSET 0
#define ADC_CALIBRATE_FORCED_GAIN   2048
#define LDO_DEFAULT                 3.300 //volts, change to actual LDO output (measure GND-3V on OLED header)
//***********************************************************************************************************
#define BUZZER    1   // BUZZER pin
#define NOTE_C5   523
#define NOTE_D5   587
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_G5   784
#define NOTE_B5   988
#define NOTE_C6   1047
#define TONE_BEEP 4200
//***********************************************************************************************************
#define MODE_MANUAL                 0
#define MODE_AUTORANGE              1
#define STARTUP_MODE                MODE_MANUAL //or: MODE_AUTORANGE
#define SWITCHDELAY_UP              8 //ms
#define SWITCHDELAY_DOWN            8 //ms
#define RANGE_SWITCH_THRESHOLD_HIGH ADC_OVERLOAD //ADC's 12bit value
#define RANGE_SWITCH_THRESHOLD_LOW  0
//***********************************************************************************************************
#include <Wire.h>                   //i2c scanner: https://playground.arduino.cc/Main/I2cScanner
#define OLED_BAUD                   1600000 //fast i2c clock
#define OLED_ADDRESS                0x3C    //i2c address on most small OLEDs
#define OLED_REFRESH_INTERVAL       180     //ms
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//***********************************************************************************************************
#define TOUCH_N        8
#define TOUCH_U        9
#define TOUCH_M        A4
Adafruit_FreeTouch qt[3] = {
  Adafruit_FreeTouch( TOUCH_N, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE ),
  Adafruit_FreeTouch( TOUCH_U, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE ),
  Adafruit_FreeTouch( TOUCH_M, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE ),
};
#define TOUCH_HIGH_THRESHOLD  400 //range is 0..1023
#define TOUCH_SAMPLE_INTERVAL 50 //ms
//***********************************************************************************************************
#define SERIAL_UART_BAUD        230400      //Serial baud for HC-06/bluetooth output
#define BT_SERIAL_EN
//#define LOGGER_FORMAT_EXPONENT  //ex: 123E-3 = 123mA
//#define LOGGER_FORMAT_NANOS     //ex: 123456 = 123456nA = 123.456uA
//#define LOGGER_FORMAT_ADC       //raw ADC output - note: automatic ADC_REF change
#define BT_REFRESH_INTERVAL     200 //ms
//***********************************************************************************************************
#define AUTOFFBUZZDELAY         500 //ms
#define AUTOFF_DEFAULT          600 //seconds, turn unit off after 10min of inactivity
//***********************************************************************************************************
#define LOGGING_FORMAT_EXPONENT 0 //ex: 123E-3 = 123mA
#define LOGGING_FORMAT_NANOS    1 //ex: 1234 = 1.234uA = 0.001234mA
#define LOGGING_FORMAT_MICROS   2 //ex: 1234 = 1.234mA = 1234000nA
#define LOGGING_FORMAT_MILLIS   3 //ex: 1234 = 1.234A = 1234000uA = 1234000000nA
#define LOGGING_FORMAT_ADC      4 //raw output for each range (0..4095)
//***********************************************************************************************************
#define ADC_SAMPLING_SPEED_AVG   0
#define ADC_SAMPLING_SPEED_FAST  1
#define ADC_SAMPLING_SPEED_SLOW  2
//***********************************************************************************************************
int offsetCorrectionValue = 0;
uint16_t gainCorrectionValue = 0;
float ldoValue = 0, ldoOptimized=0;
uint16_t AUTOFF_INTERVAL = 0;
uint8_t USB_LOGGING_ENABLED = false;
uint8_t TOUCH_DEBUG_ENABLED = false;
uint8_t GPIO_HEADER_RANGING = false;
uint8_t BT_LOGGING_ENABLED = true;
uint8_t LOGGING_FORMAT = LOGGING_FORMAT_EXPONENT;
uint16_t ADC_SAMPLING_SPEED = ADC_SAMPLING_SPEED_AVG;
uint32_t ADC_AVGCTRL;
uint8_t calibrationPerformed=false;
uint8_t analog_ref_half=true;
char rangeUnit = 'm';
uint8_t OLED_found=false;
uint8_t autoffWarning=false;
uint8_t autoffBuzz=0;
#ifdef BT_SERIAL_EN
  uint8_t BT_found=false;
#endif
FlashStorage(eeprom_ADCoffset, int);
FlashStorage(eeprom_ADCgain, uint16_t);
FlashStorage(eeprom_LDO, float);
FlashStorage(eeprom_AUTOFF, uint16_t);
FlashStorage(eeprom_LOGGINGFORMAT, uint8_t);
FlashStorage(eeprom_ADCSAMPLINGSPEED, uint8_t);
//***********************************************************************************************************

void setup() {
  Serial.begin(1); //USB speed
/*
  //some buzz
  tone(BUZZER, NOTE_C5); delay(100);
  tone(BUZZER, NOTE_E5); delay(100);
  tone(BUZZER, NOTE_G5); delay(100);
  tone(BUZZER, NOTE_C6); delay(200);
  noTone(BUZZER);        delay(50);
  tone(BUZZER, NOTE_G5); delay(100);
  tone(BUZZER, NOTE_C6); delay(400);
  noTone(BUZZER);
*/

  delay(50); //Wire apparently needs this
  Wire.begin();
  Wire.beginTransmission(OLED_ADDRESS);
  byte error = Wire.endTransmission();
  if (error == 0)
  {
    Serial.print("OLED FOUND at 0x"); Serial.println(OLED_ADDRESS);
    u8g2.begin();
    //u8g2.setDisplayRotation(U8G2_R2); //if required (inside/custom mount?)
    u8g2.setBusClock(OLED_BAUD);
    OLED_found = true;
  }
  else Serial.println("NO OLED found...");

  pinMode(A0, OUTPUT); //DAC/GNDISO
  //DAC->CTRLB.bit.EOEN = 0x00; //enable high drive strength - already done in wiring.c

  pinMode(SENSE_OUTPUT, INPUT);
  pinMode(SENSE_GNDISO, INPUT); //GND-ISO
  pinMode(SENSE_VIN, INPUT); //VIN > 1MEG > SENSE_VIN > 2MEG > GND
  pinMode(AUTOFF, INPUT_PULLUP);
  pinMode(BIAS_LED, OUTPUT);
  pinMode(LPFLED, OUTPUT); //STATUS/LPF-LED
  pinMode(LPFPIN, OUTPUT); //LPF control pin
  pinMode(BUZZER, OUTPUT);
  PINOP(MA_PIN, DIRSET);
  PINOP(UA_PIN, DIRSET);
  PINOP(NA_PIN, DIRSET);
  PINOP(MA_GPIO_PIN, DIRSET);
  PINOP(UA_GPIO_PIN, DIRSET);
  PINOP(NA_GPIO_PIN, DIRSET);

  qt[0].begin(); qt[1].begin(); qt[2].begin(); //touch pads
  analogWriteResolution(10);  //DAC resolution
  analogReferenceHalf(true);

  //DAC->CTRLA.bit.RUNSTDBY = 0x01;delay(1);
  //DAC->CTRLB.bit.REFSEL=0;//pick internal reference, skip SYNCDAC (done by analogWrite)
  analogWrite(A0, DAC_GND_ISO_OFFSET);  // Initialize Dac to OFFSET

  AUTOFF_INTERVAL = eeprom_AUTOFF.read();
  if (AUTOFF_INTERVAL==0) {
    AUTOFF_INTERVAL = AUTOFF_DEFAULT;
    eeprom_AUTOFF.write(AUTOFF_INTERVAL);
  }

  LOGGING_FORMAT = eeprom_LOGGINGFORMAT.read();

  offsetCorrectionValue = eeprom_ADCoffset.read();
  gainCorrectionValue = eeprom_ADCgain.read();
  ldoValue = eeprom_LDO.read();

  if(ldoValue==0)
    saveLDO(LDO_DEFAULT);
  else ldoOptimizeRefresh();

  ADC_SAMPLING_SPEED = eeprom_ADCSAMPLINGSPEED.read();
  refreshADCSamplingSpeed(); //load correct value into ADC_AVGCTRL

  if (gainCorrectionValue!=0) //check if anything saved in EEPROM (gain changed via SerialUSB +/-)
    analogReadCorrectionForced(offsetCorrectionValue, gainCorrectionValue);
  else {
    analogReadCorrectionForced(ADC_CALIBRATE_FORCED_OFFSET, ADC_CALIBRATE_FORCED_GAIN);
    eeprom_ADCoffset.write(offsetCorrectionValue);
    eeprom_ADCgain.write(gainCorrectionValue);
    //(offset, gain) - gain is 12 bit number (1 bit integer + 11bit fractional, see DS p895)
    //               - offset is 12bit 2s complement format (DS p896)
  }

  if (OLED_found)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tf);
    u8g2.setCursor(15,14); u8g2.print("CurrentRanger");
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(0,28); u8g2.print("Offset:");
    u8g2.setCursor(64,28); u8g2.print(offsetCorrectionValue);
    u8g2.setCursor(0,42); u8g2.print("Gain  :");
    u8g2.setCursor(64,42); u8g2.print(gainCorrectionValue);
    u8g2.setCursor(0,56); u8g2.print("LDO   :");
    u8g2.setCursor(64,56); u8g2.print(ldoValue,3);
    u8g2.sendBuffer();
    delay(2000);
  }

#ifdef BT_SERIAL_EN
  //BT check
  Serial.print("Bluetooth AT check @");Serial.print(SERIAL_UART_BAUD);Serial.print("baud...");
  SerialBT.begin(SERIAL_UART_BAUD);
  SerialBT.print("AT"); //assuming HC-06, no line ending required
  uint32_t timer=millis();
  while(millis()-timer<1000) //about 1s to respond
  {
    if (SerialBT.available()==2 && SerialBT.read()=='O' && SerialBT.read()=='K')
    {
      BT_found=true;
      break;
    }
  }

  Serial.print(BT_found?"OK!":"No HC-06 response.\r\nChecking for BT v3.0...");

  if (!BT_found)
  {
    SerialBT.print("\r\n"); //assuming HC-06 version 3.0 that requires line ending
    uint32_t timer=millis();
    while(millis()-timer<50) //about 50ms to respond
    {
      if (SerialBT.available()==4 && SerialBT.read()=='O' && SerialBT.read()=='K' && SerialBT.read()=='\r' && SerialBT.read() == '\n')
      {
        BT_found=true;
        break;
      }
    }
  
    Serial.println(BT_found?"OK!":"No response.");
  }
#endif

  printCalibInfo();
  printSerialMenu();
  WDTset();
  if (STARTUP_MODE == MODE_AUTORANGE) toggleAutoranging();
}

uint32_t oledInterval=0, lpfInterval=0, offsetInterval=0, autorangeInterval=0, btInterval=0,
         autoOffBuzzInterval=0, touchSampleInterval=0, lastRangeChange=0;
byte LPF=0, BIAS=0, AUTORANGE=0;
byte readVbatLoop=0;
float vbat=0, VOUT=0;
float read1=0,read2=0,readDiff=0;
bool rangeSwitched=false;
#define RANGE_MA rangeUnit=='m'
#define RANGE_UA rangeUnit=='u'
#define RANGE_NA rangeUnit=='n'

void rangeBeep(uint16_t switch_delay=0)
{
  uint16_t freq = NOTE_C5;
  if (RANGE_UA) freq = NOTE_D5;
  if (RANGE_MA) freq = NOTE_E5;
  if (switch_delay==0)
    tone(BUZZER, freq, 20);
  else {
    tone(BUZZER, freq);
    delay(switch_delay);
    noTone(BUZZER);
  }
}

void loop()
{
  uint32_t timestamp=micros();
  if (Serial.available()>0)
  {
    #define MODE_DISABLED 0
    #define MODE_GAIN 1
    #define MODE_LDO 2
    static char mode;
    
    char inByte = Serial.read();
    switch (inByte)
    {
      case '!':
        if (mode) {
          mode = 0;
          Serial.println("exit mode");
        }
        break;
      case 'G':
        if (mode == MODE_DISABLED) {
          mode = MODE_GAIN;
          Serial.println("gain adj mode");
        }
        break;
      case 'L':
        if (mode == MODE_DISABLED) {
          mode = MODE_LDO;
          Serial.println("ldo adj mode");
        }
        break;
      case '+':
        switch (mode) {
          case MODE_GAIN:
            eeprom_ADCgain.write(++gainCorrectionValue);
            analogReadCorrection(offsetCorrectionValue,gainCorrectionValue);
            Serial.print("new gainCorrectionValue = ");
            Serial.println(gainCorrectionValue);
            break;
          case MODE_LDO:          
            saveLDO(ldoValue+0.001);
            Serial.print("new LDO_Value = ");
            Serial.println(ldoValue, 3);
            break;
          default:
            Serial.println("no mode selected");
            break;
        }
        break;
      case '-':
        switch (mode) {
          case MODE_GAIN:
            eeprom_ADCgain.write(--gainCorrectionValue);
            analogReadCorrection(offsetCorrectionValue,gainCorrectionValue);
            Serial.print("new gainCorrectionValue = ");
            Serial.println(gainCorrectionValue);
            break;
          case MODE_LDO:          
            saveLDO(ldoValue-0.001);
            Serial.print("new LDO_Value = ");
            Serial.println(ldoValue, 3);
            break;
          default:
            Serial.println("no mode selected");
            break;
        }
        break;
      case 'u': //toggle USB logging
        USB_LOGGING_ENABLED =! USB_LOGGING_ENABLED;
        Serial.println(USB_LOGGING_ENABLED ? "USB_LOGGING_ENABLED" : "USB_LOGGING_DISABLED");
        break;
      case 't': //toggle touchpad serial output debug info
        TOUCH_DEBUG_ENABLED =! TOUCH_DEBUG_ENABLED;
        Serial.println(TOUCH_DEBUG_ENABLED ? "TOUCH_DEBUG_ENABLED" : "TOUCH_DEBUG_DISABLED");
        break;
      case 'g': //toggle GPIOs indicating ranging
        GPIO_HEADER_RANGING =! GPIO_HEADER_RANGING;
        if (GPIO_HEADER_RANGING) {
          if (rangeUnit=='m') PIN_ON(MA_GPIO_PIN); else PIN_OFF(MA_GPIO_PIN);
          if (rangeUnit=='u') PIN_ON(UA_GPIO_PIN); else PIN_OFF(UA_GPIO_PIN);
          if (rangeUnit=='n') PIN_ON(NA_GPIO_PIN); else PIN_OFF(NA_GPIO_PIN);
        }
        Serial.println(GPIO_HEADER_RANGING ? "GPIO_HEADER_RANGING_ENABLED" : "GPIO_HEADER_RANGING_DISABLED");
        break;
      case 'b': //toggle BT/serial logging
        BT_LOGGING_ENABLED =! BT_LOGGING_ENABLED;
        Serial.println(BT_LOGGING_ENABLED ? "BT_LOGGING_ENABLED" : "BT_LOGGING_DISABLED");
        break;
      case 'f': //cycle through output logging formats
        if (++LOGGING_FORMAT>LOGGING_FORMAT_ADC) LOGGING_FORMAT=LOGGING_FORMAT_EXPONENT;
        eeprom_LOGGINGFORMAT.write(LOGGING_FORMAT);
        if (LOGGING_FORMAT==LOGGING_FORMAT_EXPONENT) Serial.println("LOGGING_FORMAT_EXPONENT"); else
        if (LOGGING_FORMAT==LOGGING_FORMAT_NANOS) Serial.println("LOGGING_FORMAT_NANOS"); else
        if (LOGGING_FORMAT==LOGGING_FORMAT_MICROS) Serial.println("LOGGING_FORMAT_MICROS"); else
        if (LOGGING_FORMAT==LOGGING_FORMAT_MILLIS) Serial.println("LOGGING_FORMAT_MILLIS"); else
        if (LOGGING_FORMAT==LOGGING_FORMAT_ADC) Serial.println("LOGGING_FORMAT_ADC");
        break;
      case 's':
        if (++ADC_SAMPLING_SPEED>ADC_SAMPLING_SPEED_SLOW) ADC_SAMPLING_SPEED=ADC_SAMPLING_SPEED_AVG;
        if (ADC_SAMPLING_SPEED==ADC_SAMPLING_SPEED_AVG) Serial.println("ADC_SAMPLING_SPEED_AVG"); else
        if (ADC_SAMPLING_SPEED==ADC_SAMPLING_SPEED_FAST) Serial.println("ADC_SAMPLING_SPEED_FAST"); else
        if (ADC_SAMPLING_SPEED==ADC_SAMPLING_SPEED_SLOW) Serial.println("ADC_SAMPLING_SPEED_SLOW");
        eeprom_ADCSAMPLINGSPEED.write(ADC_SAMPLING_SPEED);
        refreshADCSamplingSpeed();
        break;
      case 'a': //toggle autoOff function
        if (AUTOFF_INTERVAL == AUTOFF_DEFAULT)
        {
          Serial.println("AUTO_OFF_DISABLED");
          AUTOFF_INTERVAL = 0xFFFF;
        }
        else
        {
          Serial.println("AUTO_OFF_ENABLED");
          AUTOFF_INTERVAL = AUTOFF_DEFAULT;
          lastRangeChange = millis();
        }
        eeprom_AUTOFF.write(AUTOFF_INTERVAL);
        break;
      case '?':
        printCalibInfo();
        printSerialMenu();
        break;
      default: break;
    }
  }

  if (AUTORANGE)
  {
    readVOUT();
    //assumes we only auto-range in DC mode (no bias)
    if (readDiff <= RANGE_SWITCH_THRESHOLD_LOW)
    {
      if      (RANGE_MA) { rangeUA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_DOWN); }
      else if (RANGE_UA) { rangeNA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_DOWN); }
    }
    else if (readDiff >= RANGE_SWITCH_THRESHOLD_HIGH)
    {
      if      (RANGE_NA) { rangeUA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_UP); }
      else if (RANGE_UA) { rangeMA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_UP); }
    }
    if (rangeSwitched) {
      lastRangeChange=millis();
      rangeSwitched=false;
      return; //!!!
    }
  }

  uint8_t VOUTCalculated=false;
  if (USB_LOGGING_ENABLED)
  {//TODO: refactor
    if (!AUTORANGE) readVOUT();
    VOUT = readDiff*ldoOptimized*(BIAS?1:OUTPUT_CALIB_FACTOR);
    VOUTCalculated=true;
    if(LOGGING_FORMAT == LOGGING_FORMAT_EXPONENT) { Serial.print(VOUT); Serial.print("e"); Serial.println(RANGE_NA ? -9 : RANGE_UA ? -6 : -3); } else
    if(LOGGING_FORMAT == LOGGING_FORMAT_NANOS) Serial.println(VOUT * (RANGE_NA ? 1 : RANGE_UA ? 1000 : 1000000)); else
    if(LOGGING_FORMAT == LOGGING_FORMAT_MICROS) Serial.println(VOUT * (RANGE_NA ? 0.001 : RANGE_UA ? 1 : 1000)); else
    if(LOGGING_FORMAT == LOGGING_FORMAT_MILLIS) Serial.println(VOUT * (RANGE_NA ? 0.000001 : RANGE_UA ? 0.001 : 1)); else
    if(LOGGING_FORMAT == LOGGING_FORMAT_ADC) Serial.println(readDiff,0);
  }

#ifdef BT_SERIAL_EN
  if (BT_found && BT_LOGGING_ENABLED && millis() - btInterval > BT_REFRESH_INTERVAL) //refresh rate (ms)
  {
    if (OLED_found) {
      u8g2.setFont(u8g2_font_siji_t_6x10); //https://github.com/olikraus/u8g2/wiki/fntgrpsiji
      u8g2.drawGlyph(104, 10, 0xE00B); //BT icon
    }

    btInterval = millis();
    if (!AUTORANGE) readVOUT();
    if (!VOUTCalculated) {
      VOUT = readDiff*ldoOptimized*(BIAS?1:OUTPUT_CALIB_FACTOR);
      VOUTCalculated=true;
    }
    if(LOGGING_FORMAT == LOGGING_FORMAT_EXPONENT) { SerialBT.print(VOUT); SerialBT.print("e"); SerialBT.println(RANGE_NA ? -9 : RANGE_UA ? -6 : -3); } else
    if(LOGGING_FORMAT == LOGGING_FORMAT_NANOS) SerialBT.println(VOUT * (RANGE_NA ? 1 : RANGE_UA ? 1000 : 1000000)); else
    if(LOGGING_FORMAT == LOGGING_FORMAT_MICROS) SerialBT.println(VOUT * (RANGE_NA ? 0.001 : RANGE_UA ? 1 : 1000)); else
    if(LOGGING_FORMAT == LOGGING_FORMAT_MILLIS) SerialBT.println(VOUT * (RANGE_NA ? 0.000001 : RANGE_UA ? 0.001 : 1)); else
    if(LOGGING_FORMAT == LOGGING_FORMAT_ADC) SerialBT.println(readDiff,0);
  }
#endif

  //OLED refresh: ~22ms (SCK:1.6mhz, ADC:64samples/DIV16/b111)
  if (OLED_found && millis() - oledInterval > OLED_REFRESH_INTERVAL) //refresh rate (ms)
  {
    oledInterval = millis();
    if (!AUTORANGE) readVOUT();
    if (!VOUTCalculated) VOUT = readDiff*ldoOptimized*(BIAS?1:OUTPUT_CALIB_FACTOR);
    u8g2.clearBuffer(); //175us
    u8g2.setFont(u8g2_font_6x12_tf); //7us

    //limit how often we read the battery since it's not expected to change a lot
    if (readVbatLoop==0) {
      if (analog_ref_half)
      {
        analogReferenceHalf(false);
        vbat=adcRead(SENSE_VIN);
        analogReferenceHalf(true);
      }
      else vbat=adcRead(SENSE_VIN);
      vbat=((vbat/ADCFULLRANGE) * ldoValue) * 1.5; //1.5 given by vbat->A5 resistor ratio (1 / (2M * 1/(1M+2M)))
    }
    if (readVbatLoop == VBATREADLOOPS) readVbatLoop=0;
    else readVbatLoop++;

    u8g2.setFont(u8g2_font_siji_t_6x10);
    if (vbat>4.3)
      u8g2.drawGlyph(115, 10, 0xE23A); //charging!
    else if(vbat>4.1)
      u8g2.drawGlyph(115, 10, 0xE24B); //100%
    else if(vbat>3.95)
      u8g2.drawGlyph(115, 10, 0xE249); //80%
    else if(vbat>3.85)
      u8g2.drawGlyph(115, 10, 0xE247); //60%
    else if(vbat>3.75)
      u8g2.drawGlyph(115, 10, 0xE245); //40%
    else if(vbat>3.65)
      u8g2.drawGlyph(115, 10, 0xE244); //20%
    else if(vbat>LOBAT_THRESHOLD)
      u8g2.drawGlyph(115, 10, 0xE243); //5%!
    else u8g2.drawGlyph(115, 10, 0xE242); //u8g2.drawStr(88,12,"LoBat!");

    u8g2.setFont(u8g2_font_6x12_tf); //7us
    if (AUTORANGE)
    {
      u8g2.drawStr(0,12, analog_ref_half ? "AUTO\xb7\xbd" : "AUTO");
      u8g2.setCursor(42,12); u8g2.print(readDiff,0);
    }
    else
    {
      if (analog_ref_half) u8g2.drawStr(0,12,"\xbd");
      u8g2.setCursor(12,12); u8g2.print(readDiff,0);
    }

    if (autoffBuzz) u8g2.drawStr(5,26,"* AUTO OFF! *"); //autoffWarning
    u8g2.setFont(u8g2_font_helvB24_te);
    u8g2.setCursor(106,64); u8g2.print('A');
    u8g2.setCursor(RANGE_MA?102:106,38); u8g2.print(RANGE_UA?char('µ'):rangeUnit);
    u8g2.setFont(u8g2_font_logisoso32_tr);
    u8g2.setCursor(0,64); u8g2.print((BIAS&&abs(VOUT)>=0.4||!BIAS&&VOUT>=0.4)?VOUT:0, abs(VOUT)>=1000?0:1);
    if (!BIAS && readDiff>ADC_OVERLOAD || BIAS && abs(readDiff)>ADC_OVERLOAD/2)
    {
      u8g2.setFont(u8g2_font_9x15B_tf);
      u8g2.drawStr(0,28, "OVERLOAD!");
    }
    u8g2.sendBuffer();
  }

  WDTclear();
  handleTouchPads();
  handleAutoOff();
  //Serial.println(micros()-timestamp);
} //loop()

uint32_t buttonLastChange_range;
uint16_t valM=0, valU=0, valN=0;
void handleTouchPads() {
  if ((millis() - buttonLastChange_range < 200) ||
      (millis() - touchSampleInterval < TOUCH_SAMPLE_INTERVAL))
    return;

  if (TOUCH_DEBUG_ENABLED) {
    Serial.print(qt[2].measure());Serial.print('\t');
    Serial.print(qt[1].measure());Serial.print('\t');
    Serial.println(qt[0].measure());
  }

  bool MA_PRESSED = qt[2].measure()>TOUCH_HIGH_THRESHOLD;
  bool UA_PRESSED = qt[1].measure()>TOUCH_HIGH_THRESHOLD;
  bool NA_PRESSED = qt[0].measure()>TOUCH_HIGH_THRESHOLD;

  touchSampleInterval = millis();

  if (MA_PRESSED || UA_PRESSED || NA_PRESSED) lastRangeChange=millis();

  //range switching
  if (!AUTORANGE)
  {
    if (MA_PRESSED && !UA_PRESSED && !NA_PRESSED && rangeUnit!='m') { rangeMA(); rangeBeep(); }
    if (UA_PRESSED && !MA_PRESSED && !NA_PRESSED && rangeUnit!='u') { rangeUA(); rangeBeep(); }
    if (NA_PRESSED && !UA_PRESSED && !MA_PRESSED && rangeUnit!='n') { rangeNA(); rangeBeep(); }
  }

  //LPF activation --- [NA+UA]
  if (UA_PRESSED && NA_PRESSED && !MA_PRESSED && millis()-lpfInterval>1000) { toggleLPF(); Beep(3, false); }

  //offset toggling (GNDISO to half supply) --- [MA+UA]
  if (MA_PRESSED && UA_PRESSED && !NA_PRESSED && millis()-offsetInterval>1000) { toggleOffset(); Beep(3, false); }

  //AUTORANGE toggling
  if (MA_PRESSED && NA_PRESSED && !UA_PRESSED && millis()-autorangeInterval>1000) { toggleAutoranging(); Beep(20, false); delay(50); Beep(20, false); }
}

void rangeMA() {
  rangeUnit='m';
  PIN_ON(MA_PIN);
  PIN_OFF(UA_PIN);
  PIN_OFF(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_ON(MA_GPIO_PIN);
    PIN_OFF(UA_GPIO_PIN);
    PIN_OFF(NA_GPIO_PIN);
  }
  analogReferenceHalf(true);
#ifdef BT_OUTPUT_ADC
  if (BT_found) SerialBT.println("RANGE: MA");
#endif
}

void rangeUA() {
  rangeUnit='u';
  PIN_OFF(MA_PIN);
  PIN_ON(UA_PIN);
  PIN_OFF(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_OFF(MA_GPIO_PIN);
    PIN_ON(UA_GPIO_PIN);
    PIN_OFF(NA_GPIO_PIN);
  }
  analogReferenceHalf(true);
#ifdef BT_OUTPUT_ADC
  if (BT_found) SerialBT.println("RANGE: UA");
#endif
}

void rangeNA() {
  rangeUnit='n';
  PIN_OFF(MA_PIN);
  PIN_OFF(UA_PIN);
  PIN_ON(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_OFF(MA_GPIO_PIN);
    PIN_OFF(UA_GPIO_PIN);
    PIN_ON(NA_GPIO_PIN);
  }
  analogReferenceHalf(true);
#ifdef BT_OUTPUT_ADC
  if (BT_found) SerialBT.println("RANGE: NA");
#endif
}

void handleAutoOff() {
  if (millis() - lastRangeChange > uint32_t(AUTOFF_INTERVAL)*1000-5000)
  {
    autoffWarning = true;

    if (millis()-autoOffBuzzInterval> AUTOFFBUZZDELAY)
    {
      autoOffBuzzInterval = millis();
      autoffBuzz=!autoffBuzz;

      if (autoffBuzz)
        tone(BUZZER, NOTE_B5);
      else
        noTone(BUZZER);
    }

    if (millis() - lastRangeChange > uint32_t(AUTOFF_INTERVAL)*1000)
    {
      pinMode(AUTOFF, OUTPUT);
      digitalWrite(AUTOFF, LOW);
    }
  }
  else if (autoffWarning) { autoffWarning=autoffBuzz=false; digitalWrite(AUTOFF, HIGH); noTone(BUZZER); }
}

void toggleLPF() {
  LPF=!LPF;
  lpfInterval = millis();
  digitalWrite(LPFPIN, LPF);
  digitalWrite(LPFLED, LPF);
  if (AUTORANGE && !LPF) toggleAutoranging(); //turn off AUTORANGE
}

void toggleOffset() {
  BIAS=!BIAS;
  offsetInterval = millis();
  analogWrite(A0, (BIAS ? DAC_HALF_SUPPLY_OFFSET : DAC_GND_ISO_OFFSET));
  digitalWrite(BIAS_LED, BIAS);
  if (AUTORANGE && BIAS) toggleAutoranging(); //turn off AUTORANGE
}

void toggleAutoranging() {
  autorangeInterval = millis();
  AUTORANGE=!AUTORANGE;
  if (AUTORANGE && BIAS) toggleOffset(); //turn off BIAS
  if (AUTORANGE && !LPF) toggleLPF(); //turn on BIAS
}

void Beep(byte theDelay, boolean twoSounds) {
  tone(BUZZER, TONE_BEEP, theDelay);
  if (twoSounds)
  {
    delay(10);
    tone(BUZZER, 4500, theDelay);
  }
}

static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while(ADC->STATUS.bit.SYNCBUSY == 1);
}

void setupADC() {
  ADC->CTRLA.bit.ENABLE = 0;              // disable ADC
  syncADC();
  ADC->REFCTRL.bit.REFCOMP = 1;
  ADC->CTRLB.reg = ADC_PRESCALER | ADC_CTRLB_RESSEL_12BIT;
  ADC->AVGCTRL.reg = ADC_AVGCTRL;
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL;
  ADC->CTRLA.bit.ENABLE = 1;  // enable ADC
  syncADC();
//  // ADC Linearity/Bias Calibration from NVM (should already be done done in core)
//  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
//  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
//  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
//  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
}

int adcRead(byte ADCpin) { return (int)analogRead(ADCpin); }
void readVOUT() {
  readDiff = adcRead(SENSE_OUTPUT) - adcRead(SENSE_GNDISO);

  if (!analog_ref_half && readDiff > RANGE_SWITCH_THRESHOLD_LOW && readDiff < RANGE_SWITCH_THRESHOLD_HIGH/3)
  {
    analogReferenceHalf(true);
    readVOUT();
  }
  else if (analog_ref_half && readDiff >= RANGE_SWITCH_THRESHOLD_HIGH)
  {
    analogReferenceHalf(false);
    readVOUT();
  }
}

void analogReadCorrectionForced(int offset, uint16_t gain) {
  offsetCorrectionValue=offset;
  gainCorrectionValue=gain;
  analogReadCorrection(offset,gain);
}

void WDTset() {
  // Generic clock generator 2, divisor = 32 (2^(DIV+1))
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  // Enable clock generator 2 using low-power 32KHz oscillator. With /32 divisor above, this yields 1024Hz(ish) clock.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while(GCLK->STATUS.bit.SYNCBUSY);
  // WDT clock = clock gen 2
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;

  WDT->CTRL.reg = 0; //disable WDT
  while(WDT->STATUS.bit.SYNCBUSY);
  WDT->INTENCLR.bit.EW   = 1;      //disable early warning
  WDT->CONFIG.bit.PER    = 0xA;    //period ~8s
  WDT->CTRL.bit.WEN      = 0;      //disable window mode
  while(WDT->STATUS.bit.SYNCBUSY);
  WDTclear();
  WDT->CTRL.bit.ENABLE = 1;        //enable WDT
  while(WDT->STATUS.bit.SYNCBUSY);
}

uint32_t WDTInterval=0;
void WDTclear() {
  if (millis() - WDTInterval > 6999) //pet the dog every 7s
  {
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    //while(WDT->STATUS.bit.SYNCBUSY);
    WDTInterval=millis();
  }
}

void ldoOptimizeRefresh() {
  if (analog_ref_half)
    ldoOptimized = (ldoValue*500)/ADCFULLRANGE;
  else
    ldoOptimized = (ldoValue*1000)/ADCFULLRANGE;
}

void saveLDO(float newLdoValue) {
  ldoValue = newLdoValue;
  eeprom_LDO.write(newLdoValue);
  ldoOptimizeRefresh();
}

void refreshADCSamplingSpeed() {
  if (ADC_SAMPLING_SPEED==ADC_SAMPLING_SPEED_AVG)
    ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_64 | ADC_AVGCTRL_ADJRES(0x4ul);
  else if (ADC_SAMPLING_SPEED==ADC_SAMPLING_SPEED_FAST)
    ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); //take 16 samples adjust by 4
  else if (ADC_SAMPLING_SPEED==ADC_SAMPLING_SPEED_SLOW)
    ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4ul); //take 512 samples adjust by 4
  //other combinations:
  //ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4ul)
  //ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x00ul);  // take 1 sample, adjusting result by 0
  //ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); //take 16 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4ul); //take 256 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_512 | ADC_AVGCTRL_ADJRES(0x4ul); //take 512 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_1024 | ADC_AVGCTRL_ADJRES(0x4ul); //take 1024 samples adjust by 4
  setupADC();
}

void printCalibInfo() {
  Serial.println("\r\nADC calib. values:");
  Serial.print("Offset="); Serial.println(offsetCorrectionValue);
  Serial.print("Gain="); Serial.println(gainCorrectionValue);
  Serial.print("LDO="); Serial.println(ldoValue,3);
}
void printSerialMenu() {
  Serial.println("\r\nUSB serial commands:");
  Serial.println("a = toggle Auto-Off function");
  Serial.print  ("b = toggle BT/serial logging (");Serial.print(SERIAL_UART_BAUD);Serial.println("baud)");
  Serial.println("f = cycle serial logging formats (exponent,nA,uA,mA/raw-ADC)");
  Serial.println("g = toggle GPIO range indication (SCK=mA,MISO=uA,MOSI=nA)");
  Serial.println("s = cycle ADC sampling speeds (average,faster,slower)");
  Serial.println("t = toggle touchpad serial output debug info");
  Serial.println("u = toggle USB/serial logging");
  Serial.println("+ = increase mode value");
  Serial.println("- = decrease mode value");
  Serial.println("? = Print this menu and calib info");
  Serial.println("\r\nMode commands:");
  Serial.println("G = gain calibration mode (1)");
  Serial.println("L = ldo calibration mode (1 mV)");
  Serial.println("! = exit mode");
  Serial.println();
}

void analogReferenceHalf(uint8_t half) {
  analog_ref_half = half;
  analogReference(half ? AR_INTERNAL1V65 : AR_DEFAULT);
  ldoOptimizeRefresh();
}

void analogReadCorrection(int offset, uint16_t gain)
{
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain);
  ADC->CTRLB.bit.CORREN = 1;
  while(ADC->STATUS.bit.SYNCBUSY);
}
