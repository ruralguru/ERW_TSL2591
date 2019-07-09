/**
 * @file ERW_TSL2591.cpp
 * @author Earl R. Watkins II
 * @date 07/01/2019
 * @brief Library for control and reading of the TSL2591
 **/

 /*
    The following is included as some details leveraged from Adafruit_TSL2591.cpp such as the I2C read / write

     This is a library for the Adafruit TSL2591 breakout board
     This library works with the Adafruit TSL2591 breakout
     ----> https://www.adafruit.com/products/1980

     Check out the links above for our tutorials and wiring diagrams
     These chips use I2C to communicate

     Adafruit invests time and resources providing this open source code,
     please support Adafruit and open-source hardware by purchasing
     products from Adafruit!

     Software License Agreement (BSD License)

     Copyright (c) 2014 Adafruit Industries
     All rights reserved.

     Redistribution and use in source and binary forms, with or without
     modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
     3. Neither the name of the copyright holders nor the
     names of its contributors may be used to endorse or promote products
     derived from this software without specific prior written permission.

     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
     EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
     DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
     ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 #include "ERW_TSL2591.h"

/**
 * ERW_TSL2591 is the constructor
 * @param interrupt_pin tells the class where to read for the interrupt. if -1 then hw interrupt disabled.
 */
 ERW_TSL2591::ERW_TSL2591(int interrupt_pin)
{
  TSL2591_I2C = &Wire;
  if( interrupt_pin == -1 )
  {
    interrupt_hw_sw = 0;
  }
  else
  {
    TSL2591_int_pin = interrupt_pin;
    interrupt_hw_sw = 1;
    pinMode(TSL2591_int_pin, INPUT_PULLUP); /* Sets interrupt pin as an input for use inside class */
  }
  initial_persistence = TSL2591_ANY_VALUES_MASK;
  initial_enable = TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK;
  initial_INT_l = ADC_MIN_COUNT;
  initial_INT_h = ADC_MAX_COUNT_100;
  initial_timer_offset = 100;
}

/**
 * begin verifies the device is accessable and sets up the initial settings.
 * @return  is 0 for OK, -1 for wrong PID, and -2 for wrong device ID,
 *          -4 for set int fault, -8 for set persist fault,
 *          -16 for set set_sensitivity fault, and -32 for enable fault,
 *          -64 for test_INT fault.
 */
int8_t ERW_TSL2591::begin(void)
{
  int8_t returnVal = 0;
  uint8_t readVal1 = 0;
  uint8_t readVal2 = 0;
  TSL2591_I2C->begin();
  readVal1 = I2C_read(TSL2591_CMD_MASK | TSL2591_PID);
  readVal2 = I2C_read(TSL2591_CMD_MASK | TSL2591_ID);
  readVal1 &= TSL2591_PID_MASK;
  if( readVal1 != TSL2591_PID_VAL )
  {
    returnVal -= 1;
  }
  else if( readVal2 != TSL2591_ID_VAL )
  {
    returnVal -= 2;
  }
  else
  {
    returnVal = initial_setup();
  }
  last_time = millis();
  #ifdef SERIAL_DEBUG
    Serial.println("Begin");
    Serial.print("\tPID: 0x");
    Serial.print(readVal1, HEX);
    Serial.print("\tID: 0x");
    Serial.println(readVal2, HEX);
    Serial.print("\texit: ");
    Serial.println(returnVal);
  #endif
  return returnVal;
}

/**
 * initial_setup
 * @return  is 0 for OK, -4 for set int fault, -8 for set persist fault,
 *          -16 for set set_sensitivity fault, and -32 for enable fault,
 *          -64 for test_INT fault.
 */
int8_t ERW_TSL2591::initial_setup(void)
{
  int8_t returnVal = 0;
  if( enable(initial_enable) != 0 )
  {
    returnVal -= 32;
  }
  if( set_interrupts(initial_INT_l, initial_INT_h) != 0)
  {
    returnVal = -4;
  }
  if( set_persistence(initial_persistence) != 0 )
  {
    returnVal -= 8;
  }
  if( set_sensitivity(TSL2591_AGAIN_LOW, TSL2591_ATIME_100) != 0 )
  {
    returnVal -= 16;
  }
  if( test_INT() != 0 )
  {
    returnVal -= 64;
  }
  return returnVal;
}

/**
 * host_process checks the LUX sensor for valid data and returns the LUX or an error code.
 *              Only runs once every 3 samples to allow for INT.
 *              Impliments auto gain control.
 * @return  LUX reading if all ok. Returns 0 for counts under 20 at minimum gain,
 *          -1.0 for auto gain reduction, -2.0 for auto gain increase,
 *          -3.0 for ADC_MIN_COUNT, -4.0 for ADC_MAX_COUNT,
 *          -10.0 auto gain set fault, -11.0 interrupt gain fault,
 *          -20.0 data not ready.
 */
float ERW_TSL2591::host_process(void)
{
  float returnVal = 0.0;
  uint8_t get_LUX_flag = 1;
  uint32_t current_millis = millis();
  if( current_millis >= ( last_time + current_timer))
  {
    get_values();
    if(get_interrupts ())
    {
      if(( current_full >= current_INT_h) || ( current_IR >= current_INT_h ))
      {
        switch (current_gain_mask) {
          case TSL2591_AGAIN_LOW:
            returnVal = -4.0;
            break;
          default:
            returnVal = -1.0;
            current_gain_mask -= 1;
            break;
        }
      }
      else if(( current_full <= initial_INT_l) || ( current_IR <= initial_INT_l ))
      {
        switch (current_gain_mask) {
          case TSL2591_AGAIN_MAX:
            returnVal = -3.0;
            break;
          default:
            returnVal = -2.0;
            current_gain_mask += 1;
            break;
          }
      }
      else
      {
        returnVal = -11.0;
      }

      if(( (int32_t) returnVal == -1 ) || ( (int32_t) returnVal == -2 ))
      {
        get_LUX_flag = 0;
        current_time_mask = current_gain_mask * 3 / 2;
        if( set_sensitivity(current_gain_mask, current_time_mask) != 0 )
        {
          returnVal = -10.0;
        }
        if( current_gain_mask == TSL2591_AGAIN_LOW)
        {
          initial_INT_h = ADC_MAX_COUNT_100;
        }
        else
        {
          initial_INT_h = ADC_MAX_COUNT;
        }
        set_interrupts(initial_INT_l, initial_INT_h);
      }
      clear_interrupt(TSL2591_CMD_CLR_ALL);
    }

    if(( get_status() & TSL2591_AVALID_MASK ))
    {
      if( get_LUX_flag == 1)
      {
        returnVal = get_LUX();
      }
    }
    else
    {
      reset();
      initial_setup();
      returnVal = -20.0;
    }
    last_time = current_millis;
    #ifdef SERIAL_DEBUG
      Serial.print("FULL Count: ");
      Serial.print(current_full);
      Serial.print("\tIR Count: ");
      Serial.println(current_IR);

      switch ((int32_t) returnVal) {
        case -1:
          Serial.println("Autogain decrease.");
          break;
        case -2:
          Serial.print("Autogain increase.");
          Serial.print("Gain: ");
          Serial.println(current_gain_mask);
          break;
        case -3:
          Serial.println("ADC_MIN_COUNT");
          break;
        case -4:
          Serial.println("ADC_MAX_COUNT");
          break;
        case -10:
          Serial.println("Autogain set fault.");
          break;
        case -11:
          Serial.println("Interrupt set,but reading OK.");
          break;
        case -20:
          Serial.println("Data not ready.");
          break;
        default:
          Serial.print("LUX: ");
          Serial.println(returnVal);
          break;
      }
    #endif
  }
  return returnVal;
}

/**
 * TSL2591_reset issues a software reset to the IC.
 */
void ERW_TSL2591::reset(void)
{
  I2C_write(TSL2591_CMD_MASK | TSL2591_CONFIG, TSL2591_SRESET_MASK);
}

/**
 * test_INT test the hardware and software interrupt interractions.
 * @return  0 OK, -1 for clear_interrupt fault, -2 for Hardware read fault,
 *          -4 for npint fault, -8 for aint fault, -16 for second clear fault.
 */
int8_t ERW_TSL2591::test_INT(void)
{
  int8_t returnVal = 0;
  uint8_t read_holder1 = 0;
  uint8_t read_holder2 = 0;
  uint8_t read_holder3 = 0;
  if( clear_interrupt(TSL2591_CMD_CLR_ALL) != 0 )
  {
    returnVal = -1;
  }
  I2C_write(TSL2591_CMD_INT_NOW);

  read_holder1 = get_interrupts();
  read_holder2 = read_holder1 & TSL2591_NPINTR_MASK;
  read_holder3 = read_holder1 & TSL2591_AINT_MASK;
  read_holder1 &= 0xF0;

  if( interrupt_hw_sw )
  {
    if( read_holder1 !=  0xF0 )
    {
      returnVal -= 2;
    }
  }
  if( read_holder2 !=  TSL2591_NPINTR_MASK )
  {
    returnVal -= 4;
  }
  if( read_holder3 !=  TSL2591_AINT_MASK )
  {
    returnVal -= 8;
  }
  if( clear_interrupt(TSL2591_CMD_CLR_ALL) != 0 )
  {
    returnVal -= 16;
  }

  #ifdef SERIAL_DEBUG
    Serial.print("Interrupt test: ");
    Serial.println(returnVal);
  #endif

  return returnVal;
}

/**
 * enable enables the device acording to the input.
 * @param[in] desired_enable  is the mask for the enable register.
 * @return  is 0 for OK, and -1 for read != write.
 */
int8_t ERW_TSL2591::enable(uint8_t desired_enable)
{
  int8_t returnVal = 0;
  I2C_write(TSL2591_CMD_MASK | TSL2591_ENABLE, desired_enable);
  current_enable = I2C_read(TSL2591_CMD_MASK | TSL2591_ENABLE);
  current_enable &= (TSL2591_NPIEN_MASK | TSL2591_SAI_MASK | TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK);
  if( current_enable != desired_enable)
  {
    returnVal = -1;
  }
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("Desired enable: 0x");
    Serial.print(desired_enable, HEX);
    Serial.print("\tRead enable: 0x");
    Serial.print(current_enable, HEX);
    Serial.print("\treturnValue: ");
    Serial.println(returnVal);
  #endif
  return returnVal;
}

/**
 * disable disables the device acording to the input.
 * @param[in] desired_disable  is the mask for the enable register.
 * @return  is 0 for OK, and -1 for read != write.
 */
int8_t ERW_TSL2591::disable(uint8_t desired_disable)
{
  int8_t returnVal = 0;
  desired_disable = ~ desired_disable;
  desired_disable &= current_enable;
  I2C_write(TSL2591_CMD_MASK | TSL2591_ENABLE, desired_disable);
  current_enable = I2C_read(TSL2591_CMD_MASK | TSL2591_ENABLE);
  desired_disable &= (TSL2591_NPIEN_MASK | TSL2591_SAI_MASK | TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK);
  current_enable &= (TSL2591_NPIEN_MASK | TSL2591_SAI_MASK | TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK);
  if( current_enable != desired_disable)
  {
    returnVal = -1;
  }
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("Desired disable: 0x");
    Serial.print(desired_disable, HEX);
    Serial.print("\tRead enable: 0x");
    Serial.print(current_enable, HEX);
    Serial.print("\treturnValue: ");
    Serial.println(returnVal);
  #endif
  return returnVal;
}

/**
 * set_sensitivity is used to set and verify the gain of the IC as well as that used for math.
 * @param[in]  desired_gain is the gain step of the ADCs.
 * @param[in]  desired_integration_time is the
 * @return 0 good, else returns read with bit 3 set.
 */
uint8_t ERW_TSL2591::set_sensitivity(uint8_t desired_gain_mask, uint8_t desired_integration_time_mask)
{
  uint8_t desired_command = 0;
  uint8_t read_command = 0;
  uint8_t returnVal = 0;
  desired_gain_mask <<= 4;
  desired_command =  desired_gain_mask | desired_integration_time_mask;

  desired_gain_mask >>= 4;

  I2C_write(TSL2591_CMD_MASK | TSL2591_CONFIG, desired_command);
  read_command = I2C_read(TSL2591_CMD_MASK | TSL2591_CONFIG);

  current_gain_mask = read_command & TSL2591_AGAIN_MASK;
  current_gain_mask >>= 4;
  current_time_mask = read_command & TSL2591_ATIME_MASK;

  switch(current_gain_mask){
    case TSL2591_AGAIN_MAX:
      current_gain = TSL2591_AGAIN_MAX_VAL;
      break;
    case TSL2591_AGAIN_HIGH:
      current_gain = TSL2591_AGAIN_HIGH_VAL;
      break;
    case TSL2591_AGAIN_MED:
      current_gain = TSL2591_AGAIN_MED_VAL;
      break;
    default:
      current_gain = TSL2591_AGAIN_LOW_VAL;
      break;
  }

  switch(current_time_mask){
    case TSL2591_ATIME_600:
      current_integration_time = TSL2591_ATIME_600_VAL;
      break;
    case TSL2591_ATIME_500:
      current_integration_time = TSL2591_ATIME_500_VAL;
      break;
    case TSL2591_ATIME_400:
      current_integration_time = TSL2591_ATIME_400_VAL;
      break;
    case TSL2591_ATIME_300:
      current_integration_time = TSL2591_ATIME_300_VAL;
      break;
    case TSL2591_ATIME_200:
      current_integration_time = TSL2591_ATIME_200_VAL;
      break;
    default:
      current_integration_time = TSL2591_ATIME_100_VAL;
      break;
  }

  current_timer = (uint32_t) current_integration_time + initial_timer_offset;

  if( desired_command != read_command)
  {
    returnVal = read_command | 0x08; //set reserved bit to allow for all 0 read as fault.
  }
  #ifdef SERIAL_DEBUG
    Serial.println("Sensitivity:");
    Serial.print("\tDesired gain: 0x");
    Serial.print(desired_gain_mask, HEX);
    Serial.print("\tRead gain: 0x");
    Serial.println(current_gain_mask, HEX);
    Serial.print("\tDesired time: 0x");
    Serial.print(desired_integration_time_mask, HEX);
    Serial.print("\tRead time: 0x");
    Serial.println(current_time_mask, HEX);
    Serial.print("\treturnValue: ");
    Serial.println(returnVal);
  #endif
  return returnVal;

}

/**
 * set_persistence
 * @param[in]  desired_persistence is the desired persistence for interrupts.
 * @return  0 OK, -1 if read != write.
 */
int8_t ERW_TSL2591::set_persistence(uint8_t desired_persistence)
{
  int8_t returnVal = 0;
  I2C_write(TSL2591_CMD_MASK | TSL2591_PERSIST, desired_persistence);
  current_persistence = I2C_read(TSL2591_CMD_MASK | TSL2591_PERSIST);
  if( current_persistence != desired_persistence)
  {
    returnVal = -1;
  }
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("Desired persist: 0x");
    Serial.print(desired_persistence, HEX);
    Serial.print("\tRead persist: 0x");
    Serial.print(current_persistence, HEX);
    Serial.print("\treturnValue: ");
    Serial.println(returnVal);
  #endif
  return returnVal;
}

/**
 * set_interrupts sets the high and low thresholds to issue an interrupt.
 * @param[in] desired_int_high set upper limit to issue an interrupt.
 * @param[in] desired_int_low  set lower limit to issue an interrupt.
 * @returns   0 OK, -1 if int_low read != write, -2 if int_high read != write.
 */
int8_t ERW_TSL2591::set_interrupts(uint16_t desired_int_low, uint16_t desired_int_high)
{
  int8_t returnVal = 0;
  uint8_t read_holder = 0;
  I2C_write(TSL2591_CMD_MASK | TSL2591_AILTL, desired_int_low );
  I2C_write(TSL2591_CMD_MASK | TSL2591_AILTH, desired_int_low >> 8);
  I2C_write(TSL2591_CMD_MASK | TSL2591_AIHTL, desired_int_high );
  I2C_write(TSL2591_CMD_MASK | TSL2591_AIHTH, desired_int_high >> 8);

  I2C_write(TSL2591_CMD_MASK | TSL2591_NPAILTL, desired_int_low );
  I2C_write(TSL2591_CMD_MASK | TSL2591_NPAILTH, desired_int_low >> 8);
  I2C_write(TSL2591_CMD_MASK | TSL2591_NPAIHTL, desired_int_high );
  I2C_write(TSL2591_CMD_MASK | TSL2591_NPAIHTH, desired_int_high >> 8);

  read_holder = I2C_read(TSL2591_CMD_MASK | TSL2591_AILTL);
  current_INT_l = I2C_read(TSL2591_CMD_MASK | TSL2591_AILTH);
  current_INT_l <<= 8;
  current_INT_l |= read_holder;
  read_holder = 0;

  read_holder = I2C_read(TSL2591_CMD_MASK | TSL2591_AIHTL);
  current_INT_h = I2C_read(TSL2591_CMD_MASK | TSL2591_AIHTH);
  current_INT_h <<= 8;
  current_INT_h |= read_holder;
  if( current_INT_l != desired_int_low )
  {
    returnVal = -1;
  }
  if( current_INT_h != desired_int_high )
  {
    returnVal -= 2;
  }
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.println("Interrupt:");
    Serial.print("\tDesired INT l: ");
    Serial.print(desired_int_low);
    Serial.print("\tRead INT l: ");
    Serial.println(current_INT_l);
    Serial.print("\tDesired INT h: ");
    Serial.print(desired_int_high);
    Serial.print("\tRead INT h: ");
    Serial.println(current_INT_h);
    Serial.print("\treturnValue: ");
    Serial.println(returnVal);
  #endif
  return returnVal;
}

/**
 * clear_interrupt clears the desired interrupts.
 * @param[in]  desired_int_to_clear is a mask for clearing specific interrupt(s).
 * @return  0 OK, -1 for desired interrupt(s) not cleared.
 */
int8_t ERW_TSL2591::clear_interrupt(uint8_t desired_int_to_clear)
{
  int8_t returnVal = 0;
  uint8_t clear_holder = 0;
  I2C_write(desired_int_to_clear);
  clear_holder = get_status();
  if( desired_int_to_clear == TSL2591_CMD_CLR_ALL)
  {
    clear_holder &= ( TSL2591_AINT_MASK | TSL2591_NPINTR_MASK);
  }
  else if( desired_int_to_clear == TSL2591_CMD_CLR_ALS)
  {
  clear_holder &= TSL2591_AINT_MASK;
  }
  else if( desired_int_to_clear == TSL2591_CMD_CLR_NOP)
  {
    clear_holder &= TSL2591_NPINTR_MASK;
  }

  if( clear_holder != 0 )
  {
    returnVal = -1;
  }
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("Desired clear: 0x");
    Serial.print(desired_int_to_clear, HEX);
    Serial.print("\tRead INT: 0x");
    Serial.print(clear_holder, HEX);
    Serial.print("\treturnValue: ");
    Serial.println(returnVal);
  #endif
  return returnVal;
}

/**
 * get_LUX returns the current visible light reading in LUX. Also stores in Current_LUX.
 * @return  is a float of the current lux reading.
 */
float ERW_TSL2591::get_LUX(void)
{
  float LUX_numerator = 0;
  float LUX_denomenator = (current_gain * current_integration_time) / TSL2591_LUX_DF;
  LUX_numerator = ( (float)current_full - (float)current_IR ) * ( 1.0 - ( (float)current_IR / (float)current_full ));
  current_LUX = LUX_numerator / LUX_denomenator;
  #ifdef SERIAL_DEBUG
    Serial.println("LUX reading:");
    Serial.print("\tLUX:");
    Serial.println(current_LUX);

    Serial.print("\tNumerator:\t");
    Serial.print(LUX_numerator);
    Serial.print(" = ( ");
    Serial.print(current_full);
    Serial.print(" - ");
    Serial.print(current_IR);
    Serial.print(" ) * ( 1 - ( ");
    Serial.print(current_IR);
    Serial.print(" / ");
    Serial.print(current_full);
    Serial.print(" )");
    Serial.println();

    Serial.print("\tDenoninator:\t");
    Serial.print(LUX_denomenator);
    Serial.print(" = ( ");
    Serial.print(current_gain);
    Serial.print(" * ");
    Serial.print(current_integration_time);
    Serial.print(" ) / ");
    Serial.println(TSL2591_LUX_DF);
  #endif
  return current_LUX;
}

/**
 * get_status reads the status register.
 * @return  current_status
 */
uint8_t ERW_TSL2591::get_status(void)
{
  current_status = I2C_read(TSL2591_CMD_MASK | TSL2591_STATUS);
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("Status: 0x");
    Serial.println(current_status, HEX);
  #endif

  return current_status;
}

/**
 * get_values is used to return the digital reads from the channel values.
 *            stores values in public variables current_full and current_IR.
 */
void ERW_TSL2591::get_values(void)
{
  uint8_t read_holder = 0;
  TSL2591_I2C->beginTransmission(TSL2591_ADDR);
  TSL2591_I2C->write(TSL2591_CMD_MASK | TSL2591_FULL_DATAL);
  TSL2591_I2C->endTransmission();
  TSL2591_I2C->requestFrom((uint8_t)TSL2591_ADDR, (uint8_t)2);
  read_holder = TSL2591_I2C->read();
  current_full = TSL2591_I2C->read();
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.println("Channel Values:");
    Serial.print("\tFull high: 0x");
    Serial.print(current_full, HEX);
    Serial.print("\tFull low: 0x");
    Serial.print(read_holder, HEX);
  #endif
  current_full <<= 8;
  current_full |= read_holder;
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("\tFull complete: 0x");
    Serial.println(current_full, HEX);
  #endif
  TSL2591_I2C->beginTransmission(TSL2591_ADDR);
  TSL2591_I2C->write(TSL2591_CMD_MASK | TSL2591_IR_DATAL);
  TSL2591_I2C->endTransmission();
  TSL2591_I2C->requestFrom((uint8_t)TSL2591_ADDR, (uint8_t)2);
  read_holder = TSL2591_I2C->read();
  current_IR = TSL2591_I2C->read();
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("\tIR high: 0x");
    Serial.print(current_IR, HEX);
    Serial.print("\tIR low: 0x");
    Serial.println(read_holder, HEX);
  #endif
  current_IR <<= 8;
  current_IR |= read_holder;
  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("\tIR complete: 0x");
    Serial.println(current_IR, HEX);
  #endif
}

/**
 * get_interrupts checks the hardware pin for an interrupt. If set it then checks the status register.
 * @return  returns the status registor bits that handle interrupts.
 */
uint8_t ERW_TSL2591::get_interrupts(void)
{
  uint8_t returnVal = 0;
  if( interrupt_hw_sw == 0 )
  {
    returnVal = get_status();
    returnVal &= (TSL2591_NPINTR_MASK |  TSL2591_AINT_MASK);
    returnVal |= 0xF0;
  }
  else if( digitalRead(TSL2591_int_pin) == 0 )
  {
    returnVal = get_status();
    returnVal &= (TSL2591_NPINTR_MASK |  TSL2591_AINT_MASK);
    returnVal |= 0xF0;
  }

  #ifdef SERIAL_DEBUG_PRIVATE
    Serial.print("Interrupts: 0x");
    Serial.println(returnVal, HEX);
  #endif

  return returnVal;
}

/**
 * @brief This function is used to read a register.
 * @param[in]  I2C_reg The register to be read from
 * @return         Value read from the register
 */
uint8_t ERW_TSL2591::I2C_read(uint8_t I2C_reg)
{
  uint8_t returnVal = 0;
  TSL2591_I2C->beginTransmission(TSL2591_ADDR);
  TSL2591_I2C->write(I2C_reg);
  TSL2591_I2C->endTransmission();

  TSL2591_I2C->requestFrom((uint8_t)TSL2591_ADDR, (uint8_t)1);
  returnVal = TSL2591_I2C->read();
  return returnVal;
}

/**
 * @brief This function is used to write a register.
 * @param[in] I2C_reg The register desired to be written to.
 * @param[in] data    The byte of data to be written to the register.
 */
void ERW_TSL2591::I2C_write(uint8_t I2C_reg, uint8_t data)
{
  TSL2591_I2C->beginTransmission(TSL2591_ADDR);
  TSL2591_I2C->write(I2C_reg);
  TSL2591_I2C->write(data);
  TSL2591_I2C->endTransmission();
}

/**
 * @brief This function is used to write a register.
 * @param I2C_reg The register desired to be written to.
 */
void ERW_TSL2591::I2C_write(uint8_t I2C_reg)
{
  TSL2591_I2C->beginTransmission(TSL2591_ADDR);
  TSL2591_I2C->write(I2C_reg);
  TSL2591_I2C->endTransmission();
}
