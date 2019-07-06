/**
 * @file ERW_TSL2591.ino
 * @author Earl R. Watkins II
 * @date 07/05/19
 * @brief minimal working example of auto gain library for TSL2591.
 */

#include <ERW_TSL2591.h>

#define LUX_INT_PIN 11

ERW_TSL2591 LUX_sensor(LUX_INT_PIN);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT); //Enable Built in LED.
  digitalWrite(LED_BUILTIN, HIGH); //Turn LED on.
  Wire.setClock(400000); /*Send I2C data at 10x the rate. */
  Serial.begin(9600); /* this can be changed for faster serial communications */

  LUX_sensor.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  LUX_sensor.host_process();

}
