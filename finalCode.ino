#include <Encoder.h>
#include <Wire.h>
Encoder knobLeft(2, 6);
#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
double voltage = 30;
int samplingTime = 1000; //milliseconds
int delayVal = 20;
int motorEnable = 4;
int motor1Direction = 7;
int motor1Speed = 9;
int currentPosition;
int desiredPosition = 3200;
double diffVal;
double newDiff;
double kP = 0.518;
double kI = 12.419;



void setup() {
pinMode(motorEnable, OUTPUT);
pinMode(motor1Direction, OUTPUT);
pinMode(motor1Speed, OUTPUT);
digitalWrite(motorEnable, HIGH);
digitalWrite(motor1Direction, LOW);
Serial.begin(9600);
Serial.print("Time(ms):  Voltage(V):  Velocity(rad/s)");
Serial.println();
pinMode(13, OUTPUT);
// initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);
// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendData);
Serial.println("Ready!");

}

void loop() {
  controlMotor(desiredPosition);
}

void controlMotor(double desired){
currentPosition = knobLeft.read();
diffVal = desired - currentPosition;
  while(diffVal != 0){
    newDiff = (diffVal*kP) + (diffVal*.02*kI);
    analogWrite(9, newDiff);
    currentPosition = knobLeft.read();
    diffVal = desired - currentPosition;
  }
}

// callback for received data
void receiveData(int byteCount){
  while(Wire.available()) {
    number = Wire.read();
    Serial.print("data received: ");
    //Serial.println(number);
  }
}

// callback for sending data
void sendData(){
  Wire.write(number + 5);
}



  
