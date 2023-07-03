/**
 * @file        TLI4971.cpp
 * @brief       TLI4971 C++ Class
 *
 *              Infineon TLI4971 Current sensor C++ class:
 *               -
 *
 * @date        August 2019
 * @copyright   Copyright (c) 2019 Infineon Technologies AG
 */

#include <stdlib.h>
#include <Arduino.h>
#include "TLI4971.h"
#include "util/SICI.h"



#define mapThrS1(thr) (thr<8)?0x16+thr*3+(int)(thr*0.25):0x10+(thr-8)*5-(int)((thr-8)*0.25)
#define mapThrS2(thr) (thr<8)?0x11+thr*3-(int)(thr/3):0x0D+(thr-8)*4-(int)((thr-8)/6)
#define mapThrS3(thr) (thr<8)?0xB+thr*2:0x08+(thr-8)*3
#define mapThrS4(thr) (thr<8)?0x17+thr*3+(int)((thr+1)/3):0x11+(thr-8)*5
#define mapThrS5(thr) (thr<8)?0x10+thr*2+(int)(thr*0.5):0xB+(thr-8)*4-(int)((thr-8)*0.25)
#define mapThrS6(thr) (thr<8)?0x9+thr+(int)(thr*0.5)+(int)(thr>2 && thr%2):0x06+(thr-8)*2+(int)((thr-8)*0.5)

#define NO_HYST_THR -1
#define mapHystThrS1(thr) (thr<8)?0x5+thr-(int)((thr+2)/3):0x4+(thr-8)-(int)((thr-8)*0.25)
#define mapHystThrS2(thr) (thr<8)?0x4+thr-(int)(thr/3)-(int)((thr+7)/8):0x3+(thr-8)-(int)((thr-7)/4)
#define mapHystThrS3(thr) (thr<8)?0x3+(thr>=3)+(thr>=5):0x2+(thr-8)-(int)((thr-8)/2)+int((thr-8)/6)-((thr-8)>6)
#define mapHystThrS4(thr) (thr<8)?0x5+thr-(int)((thr+1)/3):0x4+(thr-8)
#define mapHystThrS5(thr) (thr<8)?0x4+thr-(int)((thr+1)/2):0x3+(thr-8)-(int)((thr-5)/4)
#define mapHystThrS6(thr) (thr<8)?0x2+(int)((thr+1)/3):0x2+(int)((thr-8)/2)

#define NUM_CONFIG_REGS (sizeof(configRegs) / sizeof(configRegs[0]))


/**
 * @brief           Current sensor instance constructor
 *
 * @param[in]       aout  analog channel for reading sensors analog output
 * @param[in]       vref  analog channel for sensors reference voltage output
 * @param[in]       pwr   pin for switching sensors VDD
 * @param[in]       sici  pin for SICI communication
 * @param[in]       ocd1  pin for over-current detection 1 signal of sensor
 * @param[in]       ocd2  pin for over-current detection 2 signal of sensor
 * @param[in]       mux   pin connected to analog multiplexer on Shield2Go
 * @param[in]       mc5V states whether microcontroller is a 5V or 3V3 device (needed for calculation)
 * @param[in]       progPwr pin used to enable programming supply to OCD2 (active high)
 *
 * @return          void
 */
TLI4971::TLI4971(int aout, int vref, int pwr, int sici, int ocd1, int ocd2, int mux, bool mc5V, int progPwr)
{
  ocd1Pin = ocd1;
  ocd2Pin = ocd2;
  pwrPin = pwr;
  vrefPin = vref;
  aoutPin = aout;
  siciPin = sici;
  muxPin = mux;
  progPowerPin = progPwr;

  bus = tli4971::Sici(siciPin, pwrPin);

  pinMode(ocd1Pin, INPUT);
  pinMode(ocd2Pin, INPUT);
  pinMode(pwrPin, OUTPUT);
  pinMode(muxPin, OUTPUT);
  digitalWrite(muxPin, HIGH);
  pinMode(progPowerPin, OUTPUT);

  configAdc(mc5V, adcResol);
}

/**
 * @brief     Current sensor instance destructor  .
 *            Reconfigurates pins to INPUT and ADC resolution to 8 Bit
 *
 * @return    void
 */
TLI4971::~TLI4971(void)
{
  end();
  pinMode(pwrPin, INPUT);
  pinMode(muxPin, INPUT);
  configAdc(ll5V, 8);
}

/**
 * @brief     Turn on sensor supply, initialy reads sensor configuration and checks if sensor can be configurated
 *
 * @param[in]   leaveCommsActive    If true, the sensor's SICI bus will remain active after this function completes.
 *                                  This will prevent analog usage of the sensor, but will avoid the need to cycle power
 *                                  in the future, thereby avoiding resetting other sensors on the same power bus.
 *
 * @param[in]   noPowerCycle        If true, the power to the sensor will not be cycled.  You must turn off power and drive the analog pin low
 *                                  manually before calling this function.  This is useful for sensors on a shared power bus.
 *
 * @return    bool
 * @retval    true if sensor can be configurated
 * @retval    false if sensor not available for configuration (SICI communication failed)
 */
bool TLI4971::begin(bool leaveCommsActive, bool noPowerCycle, bool skipEnterIF)
{
  bus.begin();
  if(!skipEnterIF)
  {
    if(!bus.enterSensorIF(noPowerCycle))
    {
      bus.end();
      return false;
    }
  }

  //power down ISM
  bus.transfer16(0x8250);
  bus.transfer16(0x8000);

  //Disable failure indication
  bus.transfer16(0x8010);
  bus.transfer16(0x0000);

  //read out configuration

  bus.transfer16(0x0400);
  for(int i = 0; i < (NUM_CONFIG_REGS - 1); i += 1)
  {
    uint16_t reg = (0x40 + i + 1) << 4;
    configRegs[i] = bus.transfer16(reg);
  }
  //last register is read by sending a dummy command
  configRegs[NUM_CONFIG_REGS - 1] = bus.transfer16(0xFFFF);
  //Write + Read to check if interface is working? Or CRC Check?

  bus.transfer16(0x8400);
  bus.transfer16(0x1234);

  bus.transfer16(0x0400);
  if(bus.transfer16(0xFFFF) != 0x1234)
  {
    bus.end();
    return false;
  }

  if(!leaveCommsActive)
  {
    //power on ISM
    bus.transfer16(0x8250);
    bus.transfer16(0x0000);

    bus.end();
    siciActive = false;
  }
  else
  {
    siciActive = true;
  }

  //Get Sensor settings from registers
  measRange = (configRegs[0]&0x001F);
  opMode = (configRegs[0]&0x0060)>>5;
  //more?

  return true;
}

bool TLI4971::TLI4971ParallelBegin(TLI4971 *sensors[], int numSensors, bool leaveCommsActive, bool noPowerCycle)
{
  if (numSensors > 0)
  {
    tli4971::Sici *busses[numSensors];
    for (int i = 0; i < numSensors; i += 1)
    {
      busses[i] = &sensors[i]->bus;
    }
    return tli4971::Sici::parallelEnterSensorIF(busses, numSensors, noPowerCycle);
  }
  return false;
}

/**
 * @brief     Resets sensor to factory settings.
 *
 * @return    bool
 * @retval    true if restart succeeded
 * @retval    false if restart failed (SICI communication not possible)
 */
bool TLI4971::reset()
{
  end();
  return begin();
}

/**
 * @brief   Ends sensor library
 *          Turn off sensor supply
 *
 * @return  void
 */
void TLI4971::end()
{
  digitalWrite(pwrPin, HIGH);
  //return true;
}

/**
 * @brief   Reads sensor outputs and calculates current value. If Software OCD is used, this function can trigger the handler.
 *
 * @return  double
 * @retval  current value in Ampere
 */
double TLI4971::read()
{
  double val = ((double)analogRead(aoutPin)-(double)analogRead(vrefPin))/(pow(2.0,adcResol));  //read analog pins and divide by ADC resolution
  val *= ll5V ? 5000.0 : 3300.0;  //multiply with 5000 mV or 3300 mV depending on supply voltage of microcontroller
  val /= (double)measRange*2.0; //setting of measRange*2 is equal to mv/A of the sensor

  if(opMode == FD)  //in fully differential mode: sensitivity is doubled
    val /= 2.0;
  if(swOcdThreshold > 0 && !swOcdTriggered && abs(val) > swOcdThreshold)
  {
    swOcdTriggered = true;
    lastSwOcdValue = val;
    _swOcdFunction();
  }
  else if(swOcdTriggered && abs(val) < swOcdThreshold - swOcdCompHyst)
  {
    swOcdTriggered = false;
  }
  return val;
}

/**
 * @brief   Reads state of ocd1-pin from sensor.
 *
 * @return  bool
 * @retval  true if OCD1 is triggered
 * @retval  false if OCD1 is not triggered
 */
bool TLI4971::getOcd1State()
{
  return !digitalRead(ocd1Pin);
}

/**
 * @brief   Reads state of ocd2-pin from sensor.
 *
 * @return  bool
 * @retval  true if OCD2 is triggered
 * @retval  false if OCD2 is not triggered
 */
bool TLI4971::getOcd2State()
{
  return !digitalRead(ocd2Pin);
}

/**
 * @brief   Get state of Software OCD.
 *
 * @return  bool
 * @retval  true if Software-OCD is triggered
 * @retval  false if Software-OCD is not triggered
 */
bool TLI4971::getSwOcdState()
{
  return swOcdTriggered;
}

/**
 * @brief       Configurate ADC.
 *
 * @param[in]   logicLevel5V    true for 5V microcontrollers, false for 3.3V microcontrollers
 * @param[in]   adcResolution   number of Bits the ADC returns
 *
 * @return      void
 */
void TLI4971::configAdc(bool logicLevel5V, int adcResolution)
{
  ll5V = logicLevel5V;
#if defined(ADC_RESOLUTION) || defined(ADC_MAX_READ_RESOLUTION)
  if(adcResolution > 0)
  {
      adcResol = adcResolution;
      analogReadResolution(adcResol);
  }
#endif
}

/**
 * @brief   If polling is used as OCD mode this function will perform the polling.
 *          Needs to be called repeatedly, e.g. in loop. Handler functions for OCD-signals will be called by this method.
 *          SW-OCD will be detected as well, if in use.
 *
 * @return  void
 */
void TLI4971::ocdPolling()
{
  if(ocd1Mode == POLLING)
  {
    bool pinState = digitalRead(ocd1Pin);
    if(pinState == LOW && lastOcd1PinState == HIGH) //Falling edge at OCD1
    {
      _ocd1Function();
    }
    lastOcd1PinState = pinState;
  }
  if(ocd1Mode == POLLING)
  {
    bool pinState = digitalRead(ocd2Pin);
    if(pinState == LOW && lastOcd2PinState == HIGH) //Falling edge at OCD2
    {
      _ocd1Function();
    }
    lastOcd2PinState = pinState;
  }
  if(swOcdThreshold > 0)
  {
    read();
  }
}

/**
 * @brief       Set Hysteresis value for SW-OCD. Signal will be reset when current < threshold - hysteresis
 *
 * @param[in]   hysteresis  value in Ampere to be set as Hysteresis.
 *
 * @return      void
 * @retval      true if setting hysteresis was successful
 * @retval      false if setting hysteresis was not successful
 */
bool TLI4971::setSwOcdCompHyst(double hysteresis)
{
  swOcdCompHyst = hysteresis;
  /*
   * ToDo: check if hysteresis is valid.
   */
  return true;
}

/**
 * @brief   As soon as a Software OCD is triggered, the current value (> threshold) is stored. This function returns it.
 *
 * @return  double
 * @retval  last current value which triggered the Software-OCD
 */
double TLI4971::getLastSwOcdCurrent()
{
  return lastSwOcdValue;
}

/**
 * @brief       Register a handler, which shall be called when the sensor triggeres OCD1.
 *
 * @param[in]   mode    INTERRUPT if hardware interrupt is available at the microcontroller for this pin.
 *                      POLLING if no hardware interrupt is available for this pin. See as well ocdPolling() in this case.
 * @param[in]   *func   pointer to handler function
 *
 * @return      bool
 * @retval      true if registering was possible
 * @retval      false if registering failed
 */
bool TLI4971::registerOcd1Function(int mode, void (*func)(void))
{
  ocd1Mode = mode;
  if(mode == INTERRUPT)
  {
    if(digitalPinToInterrupt(ocd1Pin) == NOT_AN_INTERRUPT)
      return false;

    attachInterrupt(digitalPinToInterrupt(ocd1Pin), func, FALLING);
    return true;
  }
  else if(mode == POLLING)
  {
    _ocd1Function = func;
  }
  return true;
}

/**
 * @brief       Register a handler which shall be called when the sensor triggeres OCD2.
 *
 * @param[in]   mode    INTERRUPT if hardware interrupt is available at the microcontroller for this pin.
 *                      POLLING if no hardware interrupt is available for this pin. Use as well ocdPolling() in this case.
 * @param[in]   *func   pointer to handler function
 *
 * @return      bool
 * @retval      true if registering was possible
 * @retval      false if registering failed
 */
bool TLI4971::registerOcd2Function(int mode, void (*func)(void))
{
  ocd2Mode = mode;
  if(mode == INTERRUPT)
  {
    if(digitalPinToInterrupt(ocd2Pin) == NOT_AN_INTERRUPT)
      return false;

    attachInterrupt(digitalPinToInterrupt(ocd2Pin), func, FALLING);
    return true;
  }
  else if(mode == POLLING)
  {
    _ocd2Function = func;
  }
  return true;
}

/**
 * @brief       Register a handler, which shall be called when the sensor value exceeds a certain threshold.
 *
 * @param[in]   currentLevel    threshold in Ampere
 * @param[in]   *func           pointer to handler function
 *
 * @return      bool
 * @retval      true if registering was possible
 * @retval      false if registering failed
 */
bool TLI4971::registerSwOcdFunction(double currentLevel, void (*func)(void))
{
  swOcdThreshold = currentLevel;
  _swOcdFunction = func;
  return true;
}

/**
 * @brief       Sets the measurement Range of the sensor (refer to Programming Guide).
 *
 * @param[in]   measuringRange    Range of Current measurement. Possible values are:
 *                                FSR120 = +/- 120A
 *                                FSR100 = +/- 100A
 *                                FSR75 = +/- 75A
 *                                FSR50 = +/- 50A
 *                                FSR37_5 = +/-37.5A
 *                                FSR25 = +/- 25A
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::setMeasRange(int measuringRange)
{
  uint16_t configBackup = configRegs[0];
  configRegs[0] &= 0xFFE0;
  configRegs[0] |= (measuringRange&0x001F);
  if(sendConfig())
  {
    measRange = measuringRange;
    return true;
  }
  configRegs[0] = configBackup;
  return false;
}

/**
 * @brief       Set operating mode of the sensor (refer to Programming guide).
 *
 * @param[in]   operatingMode   Mode of sensor operation. Changes the behaviour of AOut and Vref dependent of the measured current. Possible modes are:
 *                              SD_BID = Semi-differential Bi-directional: Sensor provides reference voltage. Current flow in both directions are measured.
 *                              FD = fully differential: vref is used as differential output signal from sensor. Increases resolution by factor 2.
 *                              SD_UNI = Semi-differential Uni-directional: Sensor provides reference voltage. Direction of current flow is known by the application. See setVrefExt() as well.
 *                              SE = Single Ended: Reference Voltage is provided by the µC or an external circuit. See setVrefExt() as well.
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::setOpMode(int operatingMode)
{
  uint16_t configBackup = configRegs[0];
  configRegs[0] &= 0xFF9F;
  configRegs[0] |= (operatingMode&0x0060);
  if(sendConfig())
  {
    opMode = operatingMode;
    if(opMode == S_ENDED)
      digitalWrite(muxPin, LOW); //if opMode == S_ENDED, set muxPin Low in order to activate ext. VDD/2 at sensor's Vref
    return true;
  }
  configRegs[0] = configBackup;
  return false;
}

/**
 * @brief       Configuration of OCD1.
 *
 * @param[in]   enable          true to enable OCD1 signal, false to disable.
 * @param[in]   threshold       set a Threshold level for OCD1. Range: [THR1_1 - THR1_8] according to Programming guide.
 * @param[in]   deglitchTime    set a time for deglitching. Range: [D0 - D7] according to Programming guide.
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::configOcd1(bool enable, int threshold, int deglitchTime)
{
  if(enable)
  {
    if(threshold > THR1_8 || threshold < THR1_1 || deglitchTime < D0 || deglitchTime > D7)
      return false;
    uint16_t setting;
    switch(measRange)
    {
      case FSR120: setting = mapThrS1(threshold); break;
      case FSR100: setting = mapThrS2(threshold); break;
      case FSR75: setting = mapThrS3(threshold); break;
      case FSR50: setting = mapThrS4(threshold); break;
      case FSR37_5: setting = mapThrS5(threshold); break;
      case FSR25: setting = mapThrS6(threshold); break;
    }
    uint16_t configBackup0 = configRegs[0];
    uint16_t configBackup1 = configRegs[1];
    configRegs[0] &= 0xBC7F;
    configRegs[1] &= 0xFC0F;
    configRegs[0] |= 0x4000 | (deglitchTime<<7);  //enable OCD1 and set deglitch-time
    configRegs[1] |= (setting<<4);   //set Threshold level
    if(sendConfig())
      return true;
    configRegs[0] = configBackup0;
    configRegs[1] = configBackup1;
  }
  else
  {
    uint16_t configBackup = configRegs[0];
    configRegs[0] &= 0xBFFF;  //Force OCD1En Bit to 0
    if(sendConfig())
    {
      ocd1Mode = NONE;
      return true;
    }
    configRegs[0] = configBackup;
  }
  return false;
}

/**
 * @brief       Configuration of OCD2.
 *
 * @param[in]   enable        true to enable OCD2 signal, false to disable.
 * @param[in]   threshold     set a Threshold level for OCD2. Range: [THR2_1 - THR2_8] according to Programming guide.
 * @param[in]   deglitchTime  set a time for deglitching. Range: [D0 - D15] according to Programming guide.
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::configOcd2(bool enable, int threshold, int deglitchTime)
{
  if(enable)
  {
    if(threshold > THR2_8 || threshold < THR2_1 || deglitchTime < D0 || deglitchTime > D15)
      return false;
    uint16_t setting;
    switch(measRange)
    {
      case FSR120: setting = mapThrS1(threshold); break;
      case FSR100: setting = mapThrS2(threshold); break;
      case FSR75: setting = mapThrS3(threshold); break;
      case FSR50: setting = mapThrS4(threshold); break;
      case FSR37_5: setting = mapThrS5(threshold); break;
      case FSR25: setting = mapThrS6(threshold); break;
    }
    uint16_t configBackup0 = configRegs[0];
    uint16_t configBackup1 = configRegs[1];
    configRegs[0] &= 0x43FF;
    configRegs[1] &= 0x03FF;
    configRegs[0] |= 0x8000 | (deglitchTime<<10);  //enable OCD1 and set deglitch-time
    configRegs[1] |= (setting<<10);   //set Threshold level
    if(sendConfig())
      return true;
    configRegs[0] = configBackup0;
    configRegs[1] = configBackup1;
  }
  else
  {
    uint16_t configBackup = configRegs[0];
    configRegs[0] &= 0xBFFF;  //Force OCD1En Bit to 0
    if(sendConfig())
    {
      ocd1Mode = NONE;
      return true;
    }
    configRegs[0] = configBackup;
  }
  return false;
}

/**
 * @brief       Sets the OCD hysteresis. OCD signals will reset if current < 20% of the provided threshold.
 *
 * @param[in]   threshold   20% of this Threshold are set as hysteresis for both OCD signals. Range: [THR1_1 - THR2_8]
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::setOcdCompHyst(int threshold)
{
  if(threshold < THR1_1 || threshold > THR2_8)
  {
    uint16_t configBackup = configRegs[1];
    configRegs[1] &= 0xFFF0;  //Force Comp_Hyst Bits to 0
    if(sendConfig())
      return true;
    configRegs[1] = configBackup;
  }
  else
  {
    uint16_t configBackup = configRegs[1];
    uint16_t setting = 0;
    switch(measRange)
    {
      case FSR120: setting = mapHystThrS1(threshold); break;
      case FSR100: setting = mapHystThrS2(threshold); break;
      case FSR75: setting = mapHystThrS3(threshold); break;
      case FSR50: setting = mapHystThrS4(threshold); break;
      case FSR37_5: setting = mapHystThrS5(threshold); break;
      case FSR25: setting = mapHystThrS6(threshold); break;
    }
    configRegs[1] &= 0xFFF0;
    configRegs[1] |= (setting&0x000F);
    if(sendConfig())
      return true;
    configRegs[1] = configBackup;
  }
  return false;
}

/**
 * @brief       Sets the reference voltage level. This can be used for Single Ended mode (setting needs to match external reference voltage), as well as Semi-differential Uni-directional mode (sensor will output this voltage at vRef).
 *
 * @param[in]   vrefExtVoltage    Voltage level to set at vRef. Possible levels:
 *                                V1_65 = 1.65V
 *                                V1_2 = 1.2V
 *                                V1_5 = 1.5V
 *                                V1_8 = 1.8V
 *                                V2_5 = 2.5V
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::setVrefExt(int vrefExtVoltage)
{
  if(vrefExtVoltage < 0 || vrefExtVoltage > 4)
    return false;
  uint16_t configBackup = configRegs[2];
  configRegs[2] &= 0xF8FF;
  configRegs[2] |= (vrefExt&0x0007)<<4;
  if(sendConfig())
  {
    vrefExt = vrefExtVoltage;
    return true;
  }
  configRegs[2] = configBackup;
  return false;
}

/**
 * @brief       If this is enabled the sensitivity is ratio-metric to VDD respective to VREF in single-ended mode. Default is disabled.
 *
 * @param[in]   enable  true: ratio-metric gain is enabled
                false: ratio-metric gain is disabled
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::setRatioGain(bool enable)
{
  uint16_t configBackup = configRegs[2];
  if(enable)
    configRegs[2] |= 0x4000;
  else
    configRegs[2] &= 0xBFFF;
  if(sendConfig())
  {
    return true;
  }
  configRegs[2] = configBackup;
  return false;
}

/**
 * @brief       If this is enabled the ratio-metric offset behavior of the quiescent voltage is activated. Default is disabled.
 *
 * @param[in]   enable  true: ratio-metric offset is enabled
                false: ratio-metric offset is disabled
 *
 * @return      bool
 * @retval      true if configuration succeeded
 * @retval      false if configuration failed
 */
bool TLI4971::setRatioOff(bool enable)
{
  uint16_t configBackup = configRegs[2];
  if(enable)
    configRegs[2] |= 0x8000;
  else
    configRegs[2] &= 0x7FFF;
  if(sendConfig())
  {
    return true;
  }
  configRegs[2] = configBackup;
  return false;
}

/**
 * @brief     Sends the configuration to temporary registers on the sensor.
 *
 * @return    bool
 * @retval    true if configuration succeeded
 * @retval    false if configuration failed
 */
bool TLI4971::sendConfig()
{
  bool result;
  if (!prepareBus())
  {
    result = false;
    goto cleanup;
  }


  if(!transferConfig(false))
  {
    result = false;
    goto cleanup;
  }

  result = true;

cleanup:
  closeBus();

  return result;
}

bool TLI4971::transferConfig(bool sendConfigToEEPROM)
{
  updateCRC();
  if(sendConfigToEEPROM)
  {
    //send configuration to EEPROM registers 0x0400 - 0x0420

    bus.transfer16(0x8400);
    bus.transfer16(configRegs[0]);
    bus.transfer16(0x8410);
    bus.transfer16(configRegs[1]);
    bus.transfer16(0x8420);
    bus.transfer16(configRegs[2]);

    //read data again and check for correct configuration
    bus.transfer16(0x0400);
    if(bus.transfer16(0x0410) != configRegs[0])
      return false;
    if(bus.transfer16(0x0420) != configRegs[1])
      return false;
    if(bus.transfer16(0xFFFF) != configRegs[2])
      return false;
  }
  else
  {
    //send configuration to volatile registers 0x0110 - 0x0130

    bus.transfer16(0x8110);
    bus.transfer16(configRegs[0]);
    bus.transfer16(0x8120);
    bus.transfer16(configRegs[1]);
    bus.transfer16(0x8130);
    bus.transfer16(configRegs[2]);

    //read data again and check for correct configuration
    bus.transfer16(0x0110);
    if(bus.transfer16(0x0120) != configRegs[0])
      return false;
    if(bus.transfer16(0x0130) != configRegs[1])
      return false;
    if(bus.transfer16(0xFFFF) != configRegs[2])
      return false;
  }
  return true;
}

bool TLI4971::prepareBus()
{
  if (!siciActive)
  {
    bus.begin();
    if(!bus.enterSensorIF())
      return false;
  }
    //power down ISM
    bus.transfer16(0x8250);
    bus.transfer16(0x8000);

    //Disable failure indication
    bus.transfer16(0x8010);
    bus.transfer16(0x0000);
  return true;
}

void TLI4971::closeBus()
{
  //power on ISM
  bus.transfer16(0x8250);
  bus.transfer16(0x0000);

  bus.end();
}

void TLI4971::EEPROMprogramZeros()
{
  //EEPROM command
  bus.transfer16(0x83E0);
  //program zeros
  bus.transfer16(0x024E);
}

void TLI4971::EEPROMprogramOnes()
{
  //EEPROM command
  bus.transfer16(0x83E0);
  //program ones
  bus.transfer16(0x024F);
}

void TLI4971::EEPROMrefresh()
{
  //EEPROM command
  bus.transfer16(0x83E0);
  //EEPROM refresh
  bus.transfer16(0x024C);
}

bool TLI4971::programConfigToEEPROM(bool leaveCommsActive)
{
  bool result;

  if(progPowerPin == PNUM_NOT_DEFINED)
  {
    result = false;
    goto cleanup;
  }

  //Force the OCD2 pin low to protect it from high voltage.
  //This assumes a series resistor is installed between the part and the MCU
  //as described in the documentation.
  pinMode(ocd2Pin, OUTPUT);
  digitalWrite(ocd2Pin, LOW);

  if(!prepareBus())
  {
    result = false;
    goto cleanup;
  }

  if(!transferConfig(true))
  {
    result = false;
    goto cleanup;
  }

  EEPROMprogramZeros();

  digitalWrite(progPowerPin, HIGH);

  delay(30);

  digitalWrite(progPowerPin, LOW);

  EEPROMrefresh();

  delayMicroseconds(100);

  if(!transferConfig(true))
  {
    result = false;
    goto cleanup;
  }

  EEPROMprogramOnes();

  digitalWrite(progPowerPin, HIGH);

  delay(30);

  digitalWrite(progPowerPin, LOW);

  EEPROMrefresh();
  delayMicroseconds(100);

  result = true;

cleanup:
  if (!leaveCommsActive)
  {
    closeBus();
  }

  pinMode(ocd2Pin, INPUT);

  return result;
}

//CRC calculation example for Infineon TLI4971 current sensor
//CRC 8 (SAE - J1850) CRC polynomial: x^8 + x^4 + x^3 + x^2 + 1
// len = 18 (EEPROM line 0-17)
#define CRC_POLYNOMIAL 0x1D
#define CRC_SEED 0xAA
//check CRC
bool TLI4971::checkCRC(uint16_t* data, int len) {
  uint8_t checkSum = data[2]&0xFF;//CRC lower byte in EEPROM line2
  return checkSum == crcCalc(data, len);
}

//read data beginning in EEPROM line 3 to line 17, append line 0 to line 2
uint8_t TLI4971::crcCalc(uint16_t* data, int len) {
  uint8_t crcData8[len*2];
  for(int i = 0; i < len; i++)
  {
    crcData8[i*2] = ( data[(i+3)%18] >>8 ) & 0xFF;//read upper 8 bit
    crcData8[i*2+1] = ( data[(i+3)%18] ) & 0xFF;//read lower 8 bit
  }
  return crc8(crcData8, len*2-1); //do not include last byte (line 2 lower byte)
}

// CRC calculation
uint8_t TLI4971::crc8(uint8_t *data, uint8_t length)
{
  uint32_t crc;
  int16_t i, bit;
  crc = CRC_SEED;
  for(i = 0; i < length; i++)
  {
    crc ^= data[i];
    for(bit = 0; bit < 8; bit++)
    {
      if((crc & 0x80) != 0)
      {
        crc <<= 1;
        crc ^= CRC_POLYNOMIAL;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
return ~crc; // ~crc = crc^0xFF;
}

void TLI4971::updateCRC(void)
{
  uint8 crc = crcCalc(configRegs, NUM_CONFIG_REGS);
  configRegs[2] &= 0xFF00;
  configRegs[2] |= crc;
}

bool TLI4971::getIsCRCValid(void)
{
  return checkCRC(configRegs, NUM_CONFIG_REGS);
}