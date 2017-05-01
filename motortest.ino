#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13)
//#define FREQ 200.0

char buffer[20];               //통신을 할때 buffer배열에 전송받은 데이터 입력
char bufferIndex = 0; 
Servo left;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  left.attach(6, 0, 10000);              //pin 5, range 0~2000
  Serial.begin(115200);
  Wire.begin();

  pinMode(INTERRUPT_PIN, INPUT);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  PWMtransmit();                   //transmit PWM to ESC
}

// ================================================================
// ===                       SUBROUTINES                        ===
// ================================================================

void PWMtransmit()
{
  while(Serial.available())
  {
    buffer[bufferIndex]  = Serial.read();   //시리얼 통신으로 버퍼배열에 데이터 수신
    bufferIndex++;                          //데이터 수신 후 버퍼 인덱스 1 증가
  }         
  int pos = atoi(buffer);                   //atoi()함수로 char값을 int값으로 변환
  if(pos != 0)
  {
    Serial.print("Input data : ");
    Serial.println(pos);                    //int값으로 변환된 데이터 출력
    
    int plus = pos+50;    
    Serial.print("plus 50 : ");
    Serial.println(plus);                   //출력된 데이터에 50을 더하여 출력
  }
  delay(100); 
   
  //버퍼 초기화
  for(int a=0;a<21;a++) {
    buffer[a] = NULL;
  }
  bufferIndex = 0;
  left.writeMicroseconds(pos);
  delay(5000);
}

