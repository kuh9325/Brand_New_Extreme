#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
const int MPU_addr = 0x68;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13)


double dt = 0.01;
double pitch = 0;
double roll = 0;
float filterKP = 20;
float filterKI = 3*dt;

class PIDController
{
    double kP;
    double kI;
    double kD;
    double xError;
    double xDesired;
    double xErrorPrev;
    double xErrorIntegral;
    boolean feed;
  public:
    PIDController(double myKP, double myKI, double myKD, boolean isFeed)
    {
      kP = myKP;
      kI = myKI;
      kD = myKD;
      xError = 0;
      xDesired = 0;
      xErrorPrev = 0;
      xErrorIntegral = 0;
      feed = isFeed;
    }

    void setDesired(double myXDesired)
    {
      xDesired = myXDesired;
    }

    double update(double x, double dx)
    {
      double command = 0.0;
      xError = (xDesired - x);
      xErrorIntegral += xError;
      if (feed)
        command = kP * xError + kI * xErrorIntegral + kD * dx;
      else
        command = kP * xError + kI * xErrorIntegral + kD * (xError - xErrorPrev);
      xErrorPrev = xError;
      return command;
    }
};

PIDController rollFilter(filterKP, filterKI, 0, 0);
PIDController pitchFilter(filterKP, filterKI, 0, 0);

// MPU control/status vars
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //acc, tmp, gyro data
// PWM vars
int minv = 1203;                           //stop PWM signal
int PWML = minv;                           //left PWM signal
int PWMR = minv;                           //right PWM signal

SoftwareSerial bluetooth(4, 3);
Servo left;
Servo right;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  mpuinit();
  calibAccelGyro();
  left.attach(5, 1000, 2000);              //pin 5, range 0~2000
  right.attach(6, 1000, 2000);             //pin 6, range 0~2000
  Serial.begin(115200);
  bluetooth.begin(115200);
  Wire.begin();
  // init ESC
  left.writeMicroseconds(PWML);
  right.writeMicroseconds(PWMR);
  pinMode(INTERRUPT_PIN, INPUT);
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  BTreceive();                   //receive maxoutput
  readAccelGyro();
  calcAccelYPR();                  //calc accel
  calcGyro();                      //calc gyro
  calccompliYPR();                 //using complimentary filter
  PWMtransmit();                   //transmit PWM to ESC
}

// ================================================================
// ===                       SUBROUTINES                        ===
// ================================================================
void mpuinit()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr); //comm I2C that having addr 0x68
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readAccelGyro()
{
  Wire.beginTransmission(MPU_addr); //I2C comm start
  Wire.write(0x3B);                 //save 0x3B
  Wire.endTransmission(false);
  //transmit restart msg (remains connection)
  Wire.requestFrom(MPU_addr, 14, true);
  //0x68 -> 0x3B ~ 48, total 14byte save
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

int SIG;
char temp[5];
char buffer[5];
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float compli_a_y;
float baseAcX, baseAcY, baseAcZ;            //accel avg
float baseGyX, baseGyY, baseGyZ;            //gyro avg

void BTreceive()
{
  if (bluetooth.available())
  {
    // data starts a and it ends z
    if (bluetooth.read() == 'a')
    {
      byte leng = bluetooth.readBytesUntil('z', temp, 5);
      for (int i = 0 ; i < leng ; i++)
      {
        buffer[i] = temp[i];
      }
      int integer = atoi(buffer);
      int dgt = integer ;                   //temp value
      int chk = 1;
      /*calc digit of integer                       */
      if (dgt > 10)
      {
        for (dgt ; dgt >10;)
        {
           dgt = dgt/10;
           chk++; 
        }
      }
      if (chk == leng) // check lost value
      {
        SIG = integer;
      }
      /*initialize buffer & index                    */
      for (int a = 0; a < leng+2; a++)
      {
        buffer[a] = NULL;
      }
    }
  }
}

void calibAccelGyro()
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  readAccelGyro();

  //calc avg
  for(int i=0; i<10; i++)
  {
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }
  baseAcX = sumAcX / 10;
  baseAcY = sumAcY / 10;
  baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10;
  baseGyY = sumGyY / 10;
  baseGyZ = sumGyZ / 10;
}

void calcAccelYPR()
{
  float accel_x, accel_y, accel_z;     //final compesated value
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/PI;

  accel_x = AcX - baseAcX;                 // ac_x now - avg
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);       // make value 9.81m/s^2

  //calc angle by tilting
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;

  accel_angle_z = 0;
}

float gyro_x, gyro_y, gyro_z; //global var that saves gyro

void calcGyro()
{
  const float GYROXYZ_TO_DEGREES_PER_SED = 131;
 
  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;
}

void calccompliYPR()
{
  pitchFilter.setDesired(accel_angle_x);
  pitch = pitch + (gyro_x + pitchFilter.update(pitch, 0)) * dt;

  rollFilter.setDesired(accel_angle_y);
  roll = roll + (gyro_z + rollFilter.update(roll, 0)) * dt;
}

int yaw_gyro;

void PWMtransmit()
{
  int turnover = 100;                        //stop gyro signal
  int maxoutput, biasoutput;
  
  yaw_gyro = abs(gyro_z);
  maxoutput = map(SIG, 0, 255, minv, 1800);
  biasoutput = map(maxoutput, minv, 1800, minv, 1600);

  if (yaw_gyro < turnover)
  {
    if (roll < 0)
    {
      PWML = maxoutput;
      PWMR = biasoutput;
    }
    else if (roll > 0)
    {
      PWMR = maxoutput;
      PWML = biasoutput;
    }
  }
  else
  {
    PWML = minv;
    PWMR = minv;
    delay(2000);
  }
}

