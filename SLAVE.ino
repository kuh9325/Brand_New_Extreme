/******************************************************************************/
/*macro definitions of Gyro sensor and output pin*/
#define SDA A4
#define SCL A5
#define ADC_REF 3.3//reference voltage of ADC is 3.3v for MPU-6050
#define VCC 5//VCC of the grove interface is normally 5v
/*header*/
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial bluetooth(2,3);

void setup() 
{
    Serial.begin(9600);
    bluetooth.begin(9600);
}

void loop() 
{   
  /*receiving signal from master HC-06           */
  char buffer[3];                 //input data into array
  char i = 0;                     //index
  
  while(bluetooth.available())
  {
    buffer[i] = bluetooth.read(); //receiving data via bluetooth
    i++;                          //counting
  }
  
  int PWM = atoi(buffer);         //char to int conversion
  if(PWM != 0)
  {
    /*print data that converted into int         */
    Serial.print("received data : ");
    Serial.println(PWM);

    /*confirming value by execute add operation  */
    int plus = PWM+50;    
    Serial.print("plus 50 : ");
    Serial.println(plus);   
  }
  /*initialize buffer & index                    */
  for(int a=0;a<4;a++)
  {
    buffer[a] = NULL;
    i = 0;
  }
  delay(500);
}
/************************************************************************/

