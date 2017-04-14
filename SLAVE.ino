/************************************************************************/
/*macro definitions of Gyro sensor and output pin*/
#define ADC_REF 3.3//reference voltage of ADC is 3.3v for MPU-6050
#define VCC 5//VCC of the grove interface is normally 5v
/*header*/
#include <SoftwareSerial.h>
#include <Wire.h>


SoftwareSerial bluetooth(4, 3);

void setup()
{
  Serial.begin(115200);
  bluetooth.begin(115200);
}

void loop()
{
  char temp[5];
  if (bluetooth.available())
  {
    if (bluetooth.read() == 'a')
    {
      byte leng = bluetooth.readBytesUntil('z', temp, 5);

      Serial.print("Input data Length : ");
      Serial.println(leng);
      char buffer[leng+1];
      for (int i = 0 ; i < leng ; i++)
      {
        buffer[i] = temp[i];
      }
      int integer = 0;
      integer = atoi(buffer);
      int dgt = integer ; //temp
      int chk = 1;
      // calc digit of PWM
      if (dgt > 10)
      {
        for (dgt ; dgt >10;)
        {
           dgt = dgt/10;
           chk++; 
        }
      }
      if (chk == leng)
      {
        PWM = integer;
        /*print data that converted into int         */
        Serial.print("received data : ");
        Serial.println(PWM);
        /*confirming value by execute add operation  */
        long plus = PWM + 50;
        Serial.print("plus 50 : ");
        Serial.println(plus);
      }
      /*initialize buffer & index                    */
      for (int a = 0; a < leng+2; a++)
      {
        buffer[a] = NULL;
      }
    }
  }
}
/************************************************************************/

