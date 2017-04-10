/******************************************************************************/
/*macro definitions of Rotary angle sensor and output pin*/
#define ROTARY_ANGLE_SENSOR A0
#define ADC_REF 5//reference voltage of ADC is 5v
#define GROVE_VCC 5//VCC of the grove interface is normally 5v
#define FULL_ANGLE 300//full value of the rotary angle is 300 degrees
/*header*/
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2, 3);

void setup()
{
  Serial.begin(115200);
  bluetooth.begin(115200);
}

void loop()
{
  int degrees;
  degrees = getDegree();

  byte maxoutput;
  /*The degrees is 0~300, should be converted to be 0~255 to control the*/
  /*limitation of motor output                                          */
  maxoutput = map(degrees, 0, FULL_ANGLE, 0, 255);
  delay(500);
  /*transmit signal to Slave HC-06                                      */
  bluetooth.println(maxoutput);
}

/*PWM control motor output                         */
/*If maxoutput is 0,the motor is off.              */
/*The range of limitation is 0~255                 */
/************************************************************************/
/*Function: Get the angle between the mark and the starting position    */
/*Parameter:-void                                                       */
/*Return:   -int,the range of degrees is 0~300                          */
int getDegree()
{
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
  float voltage;
  voltage = (float)sensor_value * ADC_REF / 1023;
  float degrees = (voltage * FULL_ANGLE) / GROVE_VCC;
  return degrees;
}
