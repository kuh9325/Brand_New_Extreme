// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
const int MPU_addr = 0x68;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13)

// MPU control/status vars
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //acc, tmp, gyro data

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
  initDT();
  left.attach(5, 0, 2000);              //pin 5, range 0~2000
  right.attach(6, 0, 2000);             //pin 6, range 0~2000
  //Serial.setTimeout(50);                //50ms delayed
  Serial.begin(115200);
  bluetooth.begin(115200);
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
  BTreceive(); 
  readAccelGyro();
  calcDT();                        //calc peroid time
  calcAccelYPR();                  //calc accel
  calcGyro();                      //calc gyro
  calccompliYPR();                 //using complimentary filter
  Validate();                      //validate data
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
  Wire.endTransmission(false); //transmit restart msg (remains connection)
  Wire.requestFrom(MPU_addr, 14, true); //0x68 -> 0x3B ~ 48, total 14byte save
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

int PWM;
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;            //accel avg
float baseGyX, baseGyY, baseGyZ;            //gyro avg

void BTreceive()
{
  /*receiving signal from master HC-06           */
  char buffer[3];                 //input data into array
  char i = 0;                     //index

  while (bluetooth.available())
  {
    buffer[i] = bluetooth.read(); //receiving data via bluetooth
    i++;                          //counting
  }

  PWM = atoi(buffer);         //char to int conversion
  /*initialize buffer & index                    */
  for (int a = 0; a < 4; a++)
  {
    buffer[a] = NULL;
    i = 0;
  }
}

void calibAccelGyro()
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  readAccelGyro();

  //calc avg
  for(int i=0; i<10; i++){
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }
  baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}

unsigned long t_now;             //current measuring period time
unsigned long t_prev;            //prev measuring period time

void initDT()
{
  t_prev = millis();
}

void calcDT(){
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0; //millis()로 얻은 값은 밀리초 단위이니까!!!!
  t_prev = t_now;
}

void calcAccelYPR()
{
  float accel_x, accel_y, accel_z;     //final compesated value
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/PI;

  accel_x = AcX - baseAcX;                 // ac_x now - avg
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);

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
  const float ALPHA = 0.96;
  float tmp_angle_y;  //prev filtered value

  tmp_angle_y = filtered_angle_y + gyro_y * dt;

  filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
}

int roll_angle, yaw_gyro;

void Validate()
{
  yaw_gyro = gyro_z;
  roll_angle = filtered_angle_y;

  Serial.print("roll angle:\t");
  Serial.print(roll_angle);
  Serial.print("\tyaw gyro:\t");
  Serial.println(yaw_gyro);
  delay(5);
}

void PWMtransmit()
{
  int turnover = 100;                        //stop standard gyro
  int minv = 1500;                           //stop signal
  int maxoutput, biasoutput;
  int chkl, chkr;

  maxoutput = map(PWM, 0, 235, minv, 2000);
  biasoutput = map(maxoutput, minv, 2000, minv, 1900);

  if (yaw_gyro < turnover)
  {
    if (roll_angle > 5.)
    {
      left.writeMicroseconds(maxoutput);
      right.writeMicroseconds(biasoutput);
      chkl = maxoutput;
      chkr = biasoutput;
    }
    else if (roll_angle < -5.)
    {
      left.writeMicroseconds(biasoutput);
      right.writeMicroseconds(maxoutput);
      chkl = biasoutput;
      chkr = maxoutput;
    }
    else
    {
      left.writeMicroseconds(biasoutput);
      right.writeMicroseconds(biasoutput);
      chkl = biasoutput;
      chkr = biasoutput;
    }
  }
  else
  {
    left.writeMicroseconds(minv);
    right.writeMicroseconds(minv);
    chkl = biasoutput;
    chkr = biasoutput;
    Serial.print("drift detected! stop for a 2s.");
    delay(2000);
  }
    Serial.print("PWM : ");
    Serial.println(PWM);
    Serial.print("bias throttle : ");
    Serial.println(biasoutput);
    Serial.print("max throttle : ");
    Serial.println(maxoutput);
    Serial.print("left throttle : ");
    Serial.println(chkl);
    Serial.print("right throttle : ");
    Serial.println(chkr);
}

