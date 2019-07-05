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
 * @param interrupt_pin tells the class where to read for the interrupt.
 */
 ERW_TSL2591::ERW_TSL2591(int interrupt_pin)
{
  TSL2591_I2C = &Wire;
  TSL2591_int_pin = interrupt_pin;
  initial_persistence = TSL2591_3_VALUES_MASK;
  initial_enable = TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK;
  initial_INT_l = ADC_MIN_COUNT;
  initial_INT_h = ADC_MAX_COUNT_100;
  initial_timer_offset = 10;
  pinMode(TSL2591_int_pin, INPUT); /* Sets interrupt pin as an input for use inside class */
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
    if( set_interrupts(ALS_INT_l, ALS_INT_h) != 0)
    {
      returnVal = -4;
    }
    if( set_persistence(TSL2591_persistence) != 0 )
    {
      returnVal -= 8;
    }
    if( set_sensitivity(TSL2591_AGAIN_LOW, TSL2591_ATIME_100) != 0 )
    {
      returnVal -= 16;
    }
    if( enable(initial_enable) != 0 )
    {
      returnVal -= 32;
    }
    if( test_INT() != 0 )
    {
      returnVal -= 64;
    }

  }
  return returnVal;
}

/**
 * host_process checks the LUX sensor for valid data and returns the LUX or an error code.
 *              Only runs once every 3 samples to allow for INT.
 *              Impliments auto gain control.
 * @return  LUX reading if all ok. Returns 0 for counts under 20 at minimum gain,
 *          returns -1 for underflow, returns -2 for overflow.
 */
float ERW_TSL2591::host_process(void)
{
  float returnVal = 0;
  // always allow 3 * the atime for an interrupt
  // check for interrupts.
  // if interrupt then read val for auto gain control.
  // if no interrupt return lux.
  
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
  if( clearInterrupt(TSL2591_CMD_CLR_ALL) != 0 )
  {
    returnVal = -1;
  }
  I2C_write(TSL2591_CMD_INT_NOW);
  read_holder1 = get_interrupts();
  read_holder2 = read_holder1 & TSL2591_NPINTR_MASK;
  read_holder3 = read_holder1 & TSL2591_AINT_MASK;
  read_holder1 &= 0xF0;
  if( read_holder1 !=  0xF0 )
  {
    returnVal -= 2;
  }
  if( read_holder2 !=  TSL2591_NPINTR_MASK )
  {
    returnVal -= 4;
  }
  if( read_holder3 !=  TSL2591_AINT_MASK )
  {
    returnVal -= 8;
  }
  if( clearInterrupt(TSL2591_CMD_CLR_ALL) != 0 )
  {
    returnVal -= 16;
  }
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
  if( read_enable ! = desired_enable)
  {
    returnVal = -1;
  }
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
  I2C_write(TSL2591_CMD_MASK | TSL2591_ENABLE, desired_disable);
  current_enable = I2C_read(TSL2591_CMD_MASK | TSL2591_ENABLE);
  desired_disable ~= desired_disable;
  desired_disable &= (TSL2591_NPIEN_MASK | TSL2591_SAI_MASK | TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK);
  current_enable &= (TSL2591_NPIEN_MASK | TSL2591_SAI_MASK | TSL2591_AIEN_MASK | TSL2591_AEN_MASK | TSL2591_PON_MASK);
  if( read_disable ! = desired_disable)
  {
    returnVal = -1;
  }
  return returnVal;
}

/**
 * set_sensitivity is used to set and verify the gain of the IC as well as that used for math.
 * @param[in]  desired_gain is the gain step of the ADCs.
 * @param[in]  desired_integration_time is the
 * @return 0 good, else returns read with bit 3 set.
 */
uint8_t ERW_TSL2591::set_sensitivity(uint8_t desired_gain, uint8_t desired_integration_time)
{
  uint8_t desired_command = 0;
  uint8_t read_command = 0;
  unt8_t returnVal = 0;

  desired_command = desired_gain | desired_integration_time;

  I2C_write(TSL2591_CMD_MASK | TSL2591_CONFIG, desired_command);
  read_command = I2C_read(TSL2591_CMD_MASK | TSL2591_CONFIG);

  desired_gain =  read_command | TSL2591_AGAIN_MASK;
  desired_integration_time = read_command | TSL2591_ATIME_MASK;

  switch(desired_gain){
    case TSL2591_AGAIN_MAX:
      current_gain = 9900.0F;
      break;
    case TSL2591_AGAIN_HIGH:
      current_gain = 400.0F;
      break;
    case TSL2591_AGAIN_MED:
      current_gain = 24.5F;
      break;
    default:
      current_gain = 1.0F;
      break;
  }

  switch(desired_integration_time){
    case TSL2591_ATIME_600:
      current_integration_time = 600.0;
      break;
    case TSL2591_ATIME_500:
      current_integration_time = 500.0;
      break;
    case TSL2591_ATIME_400:
      current_integration_time = 400.0;
      break;
    case TSL2591_ATIME_300:
      current_integration_time = 300.0;
      break;
    case TSL2591_ATIME_200:
      current_integration_time = 200.0;
      break;
    default:
      current_integration_time = 100.0;
      break;
  }
  current_timer = (uint32_t) current_integration_time + initial_timer_offset;

  if( desired_command != read_command)
  {
    returnVal = read_command | 0x08; //set reserved bit to allow for all 0 read as fault.
  }

  return returnVal

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
  if( read_persistence != desired_persistence)
  {
    returnVal = -1;
  }
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
  I2C_write(TSL2591_CMD_MASK | TSL2591_AILTL, desired_int_low);
  I2C_write(TSL2591_CMD_MASK | TSL2591_AILTH, desired_int_low >> 8);
  I2C_write(TSL2591_CMD_MASK | TSL2591_AIHTL, desired_int_high);
  I2C_write(TSL2591_CMD_MASK | TSL2591_AIHTH, desired_int_high >> 8);
  read_holder = I2C_read(TSL2591_CMD_MASK | TSL2591_AILTL);
  current_INT_l = I2C_read(TSL2591_CMD_MASK | TSL2591_AILTH);
  current_INT_l << 8;
  current_INT_l |= read_holder;
  read_holder = 0;
  read_holder = I2C_read(TSL2591_CMD_MASK | TSL2591_AIHTL);
  current_INT_h = I2C_read(TSL2591_CMD_MASK | TSL2591_AIHTH);
  current_INT_h << 8;
  current_INT_h |= read_holder;
  if( current_INT_l != desired_int_low )
  {
    returnVal = -1;
  }
  if( current_INT_h != desired_int_high )
  {
    returnVal -= 2;
  }
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
  if( desired_int_to_clear == TSL2591_CMD_CLR_ALS)
  {
    clear_holder &= TSL2591_AINT_MASK;
  }
  else if( desired_int_to_clear == TSL2591_CMD_CLR_ALL)
  {
    clear_holder &= ( TSL2591_AINT_MASK | TSL2591_NPINTR_MASK);
  }
  else if( desired_int_to_clear == TSL2591_CMD_CLR_NOP)
  {
    clear_holder &= TSL2591_NPINTR_MASK;
  }

  if( clear_holder != 0 )
  {
    returnVal = -1;
  }
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
  LUX_numerator = ( current_full - current_IR ) * ( 1 - ( current_IR / current_full ));
  current_LUX = LUX_numerator / LUX_denomenator;
  return current_LUX;
}

/**
 * get_status reads the status register.
 * @return  current_status
 */
uint8_t ERW_TSL2591::get_status(void)
{
  current_status = read(TSL2591_CMD_MASK | TSL2591_STATUS);
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
  TSL2591_I2C->write(TSL2591_FULL_DATAL);
  TSL2591_I2C->endTransmission();
  TSL2591_I2C->requestFrom((uint8_t)TSL2591_ADDR, (uint8_t)4);
  int blah = TSL2591_I2C->available();
  Serial.print("Bytes Available: ");
  Serial.println(blah);
  read_holder = TSL2591_I2C->read();
  current_full = TSL2591_I2C->read();
  current_full <<= 8;
  current_full |= read_holder;
  read_holder = TSL2591_I2C->read();
  current_IR = TSL2591_I2C->read();
  current_IR <<= 8;
  current_IR |= read_holder;
}

/**
 * get_interrupts checks the hardware pin for an interrupt. If set it then checks the status register.
 * @return  returns the status registor bits that handle interrupts.
 */
uint8_t ERW_TSL2591::get_interrupts(void)
{
  uint8_t retunVal = 0;
  if( digitalRead(TSL2591_int_pin) == 1 )
  {
    returnVal = get_status();
    returnVal &= (TSL2591_NPINTR_MASK |  TSL2591_AINT_MASK);
    returnVal |= 0xF0;
  }
  return returnVal;
}

/**
 * @brief This function is used to read a register.
 * @param[in]  I2C_reg The register to be read from
 * @return         Value read from the register
 */
uint8_t ERW_TSL2591::I2C_read(uint8_t I2C_reg)
{
  TSL2591_I2C->beginTransmission(TSL2591_ADDR);
  TSL2591_I2C->write(I2C_reg);
  TSL2591_I2C->endTransmission();
  TSL2591_I2C->requestFrom((uint8_t)TSL2591_ADDR, (uint8_t)1);
  return TSL2591_I2C->read();
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
