/**
 * SICI.cpp - Part of the library for Arduino to control the tli4971-D050T4 current sensor.
 *
 * tli4971 is a high-precision current sensor based on Infineon´s proven Hall technology. 
 * The coreless concept significantly reduces footprint compared with existing solutions. 
 * tli4971 is an easy-to-use, fully digital solution that does not require external calibration 
 * or additional parts such as A/D converters, 0 pAmps or reference voltage. 
 * 
 * Have a look at the application note/reference manual for more information.
 * 
 * This file uses the OneWire library, originally developed by Jim Studt and many contributors since then.
 * The read_bit and write_bit functions are also based heavily on functions of the same name from that library.
 * Please have a look at the OneWire library for additional information about the license, copyright, and contributors.
 * 
 * Copyright (c) 2018 Infineon Technologies AG
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
 * following conditions are met:   
 *                                                                              
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
 * disclaimer.                        
 * 
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following 
 * disclaimer in the documentation and/or other materials provided with the distribution.                       
 * 
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote 
 * products derived from this software without specific prior written permission.                                           
 *                                                                              
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR  
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   
 */

#include "SICI.h"


#define T_EN 150
#define T_EN_MAX 400
#define T_LOW 20
#define ENTER_IF_COMMAND 0xDCBA

tli4971::Sici::Sici(uint8_t pin, uint8_t pwrPin)
{
	mActive = false;
	mPin = pin;
	mPwrPin = pwrPin;
	bitmask = PIN_TO_BITMASK(pin);
	baseReg = PIN_TO_BASEREG(pin);
}

void tli4971::Sici::begin(void)
{
	mActive = true;
}

void tli4971::Sici::end(void)
{
	digitalWrite(mPin, LOW);
	delay(6);	//End SICI communication
	pinMode(mPin, INPUT);
	mActive = false;
}

bool tli4971::Sici::enterSensorIF(bool noPowerCycle)
{
  uint16_t rec;
  if (!noPowerCycle) {
    //restart Sensor by switching VDD off and on again
    digitalWrite(mPwrPin, HIGH);		//For green Shield: digitalWrite(mPwrPin, LOW);
    digitalWrite(mPin, LOW);
    delay(100);
    digitalWrite(mPwrPin, LOW);		//For green Shield: digitalWrite(mPwrPin, HIGH);
  }
  //+IFX_ONEWIRE_PINOUTHIGH;
  //send low pulse to activate Interface
  delayMicroseconds(T_EN + T_LOW);
  digitalWrite(mPin, HIGH);
  delayMicroseconds(T_EN_MAX-T_EN-T_LOW);
  //send enter-interface-command
  rec = transfer16(ENTER_IF_COMMAND);
  
  return rec == 0;
}

static void tli4971::Sici::parallelTransfer16(Sici busses[], uint16_t dataIn[], uint16_t dataOut[], int numBusses)
{
  for (int b = 0; b < numBusses; b += 1)
  {
    dataOut[b] = 0;
  }

  //transfer and read data with LSB first
  for(int i = 0; i < 16; i++)
  {
    for (int b = 0; b < numBusses; b += 1)
    {
      busses[b]->write_bit((dataIn[b]>>(i))&0x1);
      dataOut[b] |= (busses[b]->read_bit()&0x1)<<(i);
    }
  }
  return dataOut;
}

static bool tli4971::Sici::parallelEnterSensorIF(Sici busses[], int numBusses, bool noPowerCycle = false)
{
  bool result = true;
  if (numBusses > 0) {
    uint16_t commands[numBusses];
    uint16_t results[numBusses];
    for (int b = 0; b < numBusses; b += 1)
    {
      commands[b] = ENTER_IF_COMMAND;
    }
    parallelTransfer16(busses, commands, results, numBusses);
    for (int b = 0; b < numBusses; b += 1)
    {
     if (results[b] != 0) {
       result = false;
     }
    }
    return result;
  }
  return false;
}

uint16_t tli4971::Sici::transfer16(uint16_t dataIn)
{
	uint16_t dataOut = 0;
  //transfer and read data with LSB first
  for(int i = 0; i < 16; i++)
  {
    write_bit((dataIn>>(i))&0x1);
    dataOut |= (read_bit()&0x1)<<(i);
  }
  return dataOut;
}

uint8_t tli4971::Sici::read_bit(void)
{
	// 1 is a long low pulse, 0 is a short low pulse
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(10);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(31);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(45);
	return !r;
}

void tli4971::Sici::write_bit(uint8_t value)
{
	// 1 is a long low pulse, 0 is a short low pulse
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	if (value & 1) {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(47);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(23);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(23);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(47);
	}
}
