#include <SoftwareSerial.h>
SoftwareSerial bluetooth(2, 3); // RX, TX
char serial_data;
char bluetooth_data;
void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  bluetooth.begin(9600);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  Serial.println("Ready");
  digitalWrite(13, HIGH);
}
void loop() // run over and over
{
  if (Serial.available()){
    serial_data = Serial.read();
     if(serial_data == '0'){
       bluetooth.print("AT");}
     else if(serial_data == 'N'){
       bluetooth.print("AT+NAMEBNEboard");}
     else if(serial_data == 'V'){
       bluetooth.print("AT+VERSION");}
     else if(serial_data == 'M'){
       bluetooth.print("AT+ROLE=M");} //Master
     else if(serial_data == 'S'){
       bluetooth.print("AT+ROLE=S");} //Slave
     else if(serial_data == 'P'){
       bluetooth.print("AT+PIN1234");}
     else if(serial_data == 'B'){
       bluetooth.print("AT+BAUD8");}
     else {bluetooth.write(Serial.read());}
  }
  if (bluetooth.available()){
    bluetooth_data=bluetooth.read();
    Serial.print(bluetooth_data);
  }
}
