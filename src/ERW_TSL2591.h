/**
 * @file ERW_TSL2591.h
 * @author Earl R. Watkins II
 * @date 07/01/2019
 * @brief Library for control and reading of the TSL2591
 **/
 /*
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

 #ifndef ERW_TSL2591_h
 #define ERW_TSL2591_h

 #include <Arduino.h>
 #include <Wire.h>

 // Channel 0 is full light spectrum. Channel 1 is IR spectrum.

 #define TSL2591_ADDR               0x29  //!<I2C address for device

 #define TSL2591_ENABLE             0x00  //!<Enable register address [r/w].
 #define TSL2591_CONFIG             0x01  //!<Configuration register address [r/w].
 #define TSL2591_AILTL              0x04  //!<ALS interrupt lower threshold lower byte register address [r/w].
 #define TSL2591_AILTH              0x05  //!<ALS interrupt lower threshold upper byte register address [r/w].
 #define TSL2591_AIHTL              0x06  //!<ALS interrupt upper threshold lower byte register address [r/w].
 #define TSL2591_AIHTH              0x07  //!<ALS interrupt upper threshold upper byte register address [r/w].
 #define TSL2591_NPAILTL            0x08  //!<No Persist ALS interrupt lower threshold lower byte register address [r/w].
 #define TSL2591_NPAILTH            0x09  //!<No Persist ALS interrupt lower threshold upper byte register address [r/w].
 #define TSL2591_NPAIHTL            0x0A  //!<No Persist ALS interrupt upper threshold lower byte register address [r/w].
 #define TSL2591_NPAIHTH            0x0B  //!<No Persist ALS interrupt upper threshold upper byte register address [r/w].
 #define TSL2591_PERSIST            0x0C  //!<Interrupt persistence filter register address [r/w].
 #define TSL2591_PID                0x11  //!<Package ID register address [r].
 #define TSL2591_ID                 0x12  //!<Device ID register address [r].
 #define TSL2591_STATUS             0x13  //!<Device Status register address [r].
 #define TSL2591_FULL_DATAL         0x14  //!<Channel 0 low data byte register address [r].
 #define TSL2591_FULL_DATAH         0x15  //!<Channel 0 upper data byte register address [r].
 #define TSL2591_IR_DATAL           0x16  //!<Channel 1 low data byte register address [r].
 #define TSL2591_IR_DATAH           0x17  //!<Channel 1 upper data byte register address [r].

 #define TSL2591_RESET_VALUE        0x00  //!< Reset Value for all registers accept IDs.

 #define TSL2591_CMD_MASK           0xA0  //!< Mask for addressing command register.
 #define TSL2591_CMD_INT_NOW        0xE4  //!< Command register force interrupt.
 #define TSL2591_CMD_CLR_ALS        0xE6  //!< Command register clear ALS interrupt.
 #define TSL2591_CMD_CLR_ALL        0xE7  //!< Command register clear all interrupts.
 #define TSL2591_CMD_CLR_NOP        0xEA  //!< Command register clear no persist interrupt.

 #define TSL2591_NPIEN_MASK         0x80  //!< No persist interrupt enable mask.
 #define TSL2591_SAI_MASK           0x40  //!< Sleep after interrupt
 #define TSL2591_AIEN_MASK          0x10  //!< ALS interrupt enable mask.
 #define TSL2591_AEN_MASK           0x02  //!< ALS enable mask.
 #define TSL2591_PON_MASK           0x01  //!< Power on mask.

 #define TSL2591_SRESET_MASK        0x80  //!< Reset device mask.
 #define TSL2591_AGAIN_MASK         0x30  //!< ALS gain mask.
 #define TSL2591_ATIME_MASK         0x07  //!< ALS integration time mask.

 #define TSL2591_AGAIN_LOW          0x00  //!<ALS gain of x1
 #define TSL2591_AGAIN_MED          0x10  //!<ALS gain of x24.5
 #define TSL2591_AGAIN_HIGH         0x20  //!<ALS gain of x400
 #define TSL2591_AGAIN_MAX          0x30  //!<ALS gain of x 9200 / 9900.

 #define TSL2591_ATIME_100          0x00  //!<ALS time of 100ms.
 #define TSL2591_ATIME_200          0x01  //!<ALS time of 200ms.
 #define TSL2591_ATIME_300          0x02  //!<ALS time of 300ms.
 #define TSL2591_ATIME_400          0x03  //!<ALS time of 400ms.
 #define TSL2591_ATIME_500          0x04  //!<ALS time of 500ms.
 #define TSL2591_ATIME_600          0x05  //!<ALS time of 600ms.

 #define TSL2591_60_VALUES_MASK     0x0F  //!< 60 for interrupt mask.
 #define TSL2591_55_VALUES_MASK     0x0E  //!< 55 for interrupt  mask.
 #define TSL2591_50_VALUES_MASK     0x0D  //!< 50 for interrupt  mask.
 #define TSL2591_45_VALUES_MASK     0x0C  //!< 45 for interrupt  mask.
 #define TSL2591_40_VALUES_MASK     0x0B  //!< 40 for interrupt  mask.
 #define TSL2591_35_VALUES_MASK     0x0A  //!< 35 for interrupt  mask.
 #define TSL2591_30_VALUES_MASK     0x09  //!< 30 for interrupt  mask.
 #define TSL2591_25_VALUES_MASK     0x08  //!< 25 for interrupt  mask.
 #define TSL2591_20_VALUES_MASK     0x07  //!< 20 for interrupt  mask.
 #define TSL2591_15_VALUES_MASK     0x06  //!< 15 for interrupt  mask.
 #define TSL2591_10_VALUES_MASK     0x05  //!< 10 for interrupt  mask.
 #define TSL2591_5_VALUES_MASK      0x04  //!< 5 for interrupt  mask.
 #define TSL2591_3_VALUES_MASK      0x03  //!< 3 for interrupt  mask.
 #define TSL2591_ANY_VALUES_MASK    0x02  //!< Any outside value for interrupt  mask.
 #define TSL2591_ALWAYS_VALUES_MASK 0x01  //!< ALS cycle generates interrupt  mask.

 #define TSL2591_PID_MASK           0x30  //!< Product ID mask.
 #define TSL2591_PID_VAL            0x00  //!< Expected PID.
 #define TSL2591_ID_VAL             0x50  //!< Expected Device ID.


 #define TSL2591_NPINTR_MASK        0x20  //!< No persist interrupt indicator mask.
 #define TSL2591_AINT_MASK          0x10  //!< ALS interrupt indicator mask.
 #define TSL2591_AVALID_MASK        0x01  //!< ALS data valid mask.

 #define TSL2591_LUX_DF             408.0F//!< LUX coefficient from Adafruit.
 #define ADC_MAX_COUNT              65530 //!< MAX ADC cound for all but 100 ms atime.
 #define ADC_MAX_COUNT_100          35860 //!< MAX ADC cound when using 100 ms atime.
 #define ADC_MIN_COUNT              20    //!< MIN ADC cound for dark is 20.

 class ERW_TSL2591
 {
 	public: //Can be called upon.

 	//Public Functions

    /**
     * ERW_TSL2591 is the constructor
     * @param interrupt_pin tells the class where to read for the interrupt.
     */
 		ERW_TSL2591(int interrupt_pin);

    /**
     * begin verifies the device is accessable and sets up the initial settings.
     * @return  is 0 for OK, -1 for wrong PID, and -2 for wrong device ID,
     *          -4 for set int fault, -8 for set persist fault,
     *          -16 for set set_sensitivity fault, and -32 for enable fault,
     *          -64 for test_INT fault.
     */
 		int8_t begin(void);

    /**
     * host_process checks the LUX sensor for valid data and returns the LUX or an error code.
     *              Only runs once every 2 samples to allow for INT.
     *              Impliments auto gain control.
     * @return  LUX reading if all ok. Returns 0 for counts under 20 at minimum gain,
     *          returns -1 for underflow, returns -2 for overflow.
     */
    float host_process(void);

    /**
     * TSL2591_reset issues a software reset to the IC.
     */
    void reset(void);

    /**
     * test_INT test the hardware and software interrupt interractions.
     * @return  0 OK, -1 for clear_interrupt fault, -2 for Hardware read fault,
     *          -4 for npint fault, -8 for aint fault, -16 for second clear fault.
     */
    int8_t test_INT(void);


 	//Public Variables

    uint16_t current_full;
    uint16_t current_IR;
    float current_LUX;

 	private:
 	//Private Functions

    /**
     * enable enables the device acording to the input.
     * @param[in] desired_enable  is the mask for the enable register.
     * @return  is 0 for OK, and -1 for read != write.
     */
    int8_t enable(uint8_t desired_enable);

    /**
     * disable disables the device acording to the input.
     * @param[in] desired_disable  is the mask for the enable register.
     * @return  is 0 for OK, and -1 for read != write.
     */
    int8_t disable(uint8_t desired_disable);

    /**
     * set_sensitivity is used to set and verify the gain of the IC as well as that used for math.
     * @param[in]  desired_gain is the gain step of the ADCs.
     * @param[in]  desired_integration_time is the
     * @return 0 good, else returns read with bit 3 set.
     */
    int8_t set_sensitivity(uint8_t desired_gain, uint8_t desired_integration_time);

    /**
     * set_persistence
     * @param[in]  desired_persistence is the desired persistence for interrupts.
     * @return  0 OK, -1 if read != write.
     */
    int8_t set_persistence(uint8_t desired_persistence);

    /**
     * set_interrupts sets the high and low thresholds to issue an interrupt.
     * @param[in] desired_int_high set upper limit to issue an interrupt.
     * @param[in] desired_int_low  set lower limit to issue an interrupt.
     * @returns   0 OK, -1 if int_low read != write, -2 if int_high read != write.
     */
    int8_t set_interrupts(uint16_t int_low, uint16_t int_high);

    /**
     * clear_interrupt clears the desired interrupts.
     * @param[in]  desired_int_to_clear is a mask for clearing specific interrupt(s).
     * @return  0 OK, -1 for desired interrupt(s) not cleared.
     */
    int8_t clear_interrupt(uint8_t desired_int_to_clear);

    /**
     * get_LUX returns the current visible light reading in LUX. Also stores in Current_LUX.
     * @return  is a float of the current lux reading.
     */
    float get_LUX(void);

    /**
     * get_status reads the status register.
     * @return  current_status
     */
    uint8_t get_status(void);

    /**
     * get_values is used to return the digital reads from the channel values.
     *            stores values in public variables current_full and current_IR.
     */
    void get_values(void);

    /**
     * get_interrupts checks the hardware pin for an interrupt. If set it then checks the status register.
     * @return  returns the status registor bits that handle interrupts.
     */
    uint8_t get_interrupts(void);

    /**
   * @brief This function is used to read a register.
   * @param[in]  I2C_reg The register to be read from
   * @return         Value read from the register
   */
  	uint8_t I2C_read(uint8_t I2C_reg);

    /**
     * @brief This function is used to write a register.
     * @param[in] I2C_reg The register desired to be written to.
     * @param[in] data    The byte of data to be written to the register.
     */
  	void I2C_write(uint8_t I2C_reg, uint8_t data);

    /**
     * @brief This function is used to write a register.
     * @param[in] I2C_reg The register desired to be written to.
     */
    void I2C_write(uint8_t I2C_reg);

 	//Private Variables

    int TSL2591_int_pin;
    uint8_t initial_persistence;
    uint8_t initial_enable;
    uint16_t initial_INT_h;
    uint16_t initial_INT_l;
    uint32_t initial_timer_offset;

    TwoWire *TSL2591_I2C;

    float current_integration_time;
    float current_gain;

    uint8_t current_status;
    uint8_t current_persistence;
    uint8_t current_enable;
    uint16_t current_INT_h;
    uint16_t current_INT_l;
    uint32_t current_timer;

    uint32_t last_time;



 };
 #endif
