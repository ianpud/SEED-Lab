#include <Wire.h>
#define SLAVE_ADDRESS 0x04
#include <Encoder.h>

Encoder leftWheel(2,5); //Encoder Pins for Left Wheel
Encoder rightWheel(3,6);//Encoder Pins for Right Wheel

int motorEnable = 4;

int wheelRightDirection = 8;
int wheelLeftDirection = 7;

int wheelRightSpeed = 10;
int wheelLeftSpeed = 9;

int rightPinA = 3;
int leftPinA = 2;

int rightPinB = 6;
int leftPinB = 5;

long positionLeft  = -999;
long positionRight = -999;


int currentPosition;
int currentError;
double valueVolts;
double inputVolts;
double kP = 0.518; //Kp value for PID Controller


double desired_distance = 0;
double desired_angle = 0;
bool make_circle = false;
bool ready_bool = false;
bool new_command = false;
bool sweep_complete = true;

int state = 0;
double read_value;

void setup() {
  pinMode(motorEnable, OUTPUT);
  pinMode(wheelRightDirection, OUTPUT);
  pinMode(wheelLeftDirection, OUTPUT);
  pinMode(wheelRightSpeed, OUTPUT);
  pinMode(wheelLeftSpeed, OUTPUT);
  digitalWrite(motorEnable, HIGH);
  digitalWrite(wheelLeftDirection, LOW);
  digitalWrite(wheelRightDirection, HIGH);
  analogWrite(wheelRightSpeed, HIGH);
  analogWrite(wheelLeftSpeed, HIGH);

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(9600);
  Serial.println("Ready!");
  ready_bool = true;
}

void loop() {
  //while(true){
    if(new_command){
      new_command = false;
      Serial.print("\nAngle: ");
      if(desired_angle >= 232){
        desired_angle = (-1 * (255 - desired_angle));
      }else{
        desired_angle = desired_angle;
      }
      Serial.println(desired_angle);
      Serial.print("Distance: ");
      Serial.println(desired_distance);
      turnAngle(desired_angle);
      delay(1000);
      drive(desired_distance);
      if(make_circle){
        delay(500);
        drive(-0.5);
        delay(100);
        turnAngle(-80);
        delay(100);
        drive(-0.5);
        delay(100);
        doCircle();
        //doSquare();
        Serial.println("Made a circle!");
        make_circle = false;
      }
    }
  //}
}

//void loop() {
//  turnAngle(90);
//  delay(1000);
//  drive(1);
//}

// callback for received data
void receiveData(int byteCount){
  while(Wire.available()) {
    read_value = Wire.read();
  }
  if(state == 0 && read_value == 254){
    state = 1;
  }else if(state == 0 && read_value == 255){
    state = 2;
  }else if(state == 0 && read_value == 253){
    make_circle = true;
  }else if(state == 0 && read_value == 251){
    analogWrite(9, 0); //Send PWM value to motors
    analogWrite(10, 0);
  }else if(state == 0 && read_value == 250){
    digitalWrite(wheelRightDirection, LOW);
    digitalWrite(wheelLeftDirection, LOW);
    analogWrite(9, 50); //Send PWM value to motors
    analogWrite(10, 50);
  }else if(read_value == 252){
    new_command = true;
    state = 0;
  }else if(state == 1){
    desired_angle = read_value;
    state = 0;
  }else if(state == 2){
    desired_distance = read_value / 12;
    state = 0;
  }
}

// callback for sending data
void sendData(){
  Wire.write(ready_bool);
}

//Function to turn the robot
void turnAngle(double angle){
  Serial.println("Started turning!");
    double anglein = angle;
    int countout = anglein * 18.5; //Converts from degrees to counts
    int temp = 0;
    leftWheel.write(0); //Reset encoder values to 0
    rightWheel.write(0);

    analogWrite(9, 60); //Send PWM value to motors
    analogWrite(10, 60);
    //Serial.println("1");
    
    while(1){
      if(new_command){
        break;
      }
      currentPosition = rightWheel.read();
      currentError = countout - currentPosition;
      if(currentError > 2){ //If error is more than 0 correct by turning clockwise
        digitalWrite(wheelRightDirection, HIGH);
        digitalWrite(wheelLeftDirection, HIGH);
        //Serial.println("loop1");
       }
      else if(currentError < 2){ //If error is less than 0 keep turning counter clockwise
        digitalWrite(wheelRightDirection, LOW);
        digitalWrite(wheelLeftDirection, LOW);
        //Serial.println("loop2");
      }
      else{ //Break loop if error is 0
        temp += 1;
        if(temp == 5){
          analogWrite(9, 0);
          analogWrite(10, 0);
          break;
       //Serial.println("loop3");
        }
      }

    valueVolts = abs(currentError*kP); //Runs error through PID controller
    inputVolts = (((valueVolts)/8)*255); //Convert from volts to PWM
    if(inputVolts < 40){
      inputVolts = 40;
    }
    //inputVolts = inputVolts * 0.75;
    analogWrite(9, inputVolts); //Send PWM value to motors
    analogWrite(10, inputVolts); 
  }
  Serial.println("Done turning!");
}

//Function to move the robot forwards and backwards
void drive(double feet){
  Serial.println("Started driving!");
  leftWheel.write(0); //Reset encoder values to 0
  rightWheel.write(0);
  int temp = 0;
  double feetin = feet;
  double inchin = feetin*12; //Converts desired feet to inches
  int countout = inchin*170; //Converts inches to encoder counts

  analogWrite(9, 60); //Send PWM value to motors
  analogWrite(10, 60);

  while(1){ //Infinite loop
    if(new_command){
        break;
      }
    currentPosition = leftWheel.read();
    currentError = countout - currentPosition;
    if(currentError < 3){ //If the error is less than 0 keep going straight
      digitalWrite(wheelRightDirection, HIGH);
      digitalWrite(wheelLeftDirection, LOW);
      Serial.println("<");
    }else if(currentError > 3){ //If error is more that 0 correct by going backwards
      digitalWrite(wheelRightDirection, LOW);
      digitalWrite(wheelLeftDirection, HIGH);
    }else{ //Break loop if error is 0
      analogWrite(9, 0);
      analogWrite(10, 0);
      break;
    }

    valueVolts = abs(currentError*kP); //Sends error value through PID controller
    inputVolts = (((valueVolts)/8)*255); //Convert from volts to PWM
    if(inputVolts < 50){
      inputVolts = 50;
    }
    analogWrite(9, inputVolts);
    analogWrite(10, inputVolts);
  }
  Serial.println("Done driving!");
}

//Circle function
void doCircle(){
  leftWheel.write(0); //Reset encoder values to 0
  rightWheel.write(0);
  double feetin = 13;
  double inchin = feetin*12; //Converts desired feet to inches
  int countout = inchin*170; //Converts inches to encoder counts
  
  currentPosition = leftWheel.read();
  currentError = countout - currentPosition;

  while(1){ //Infinite loop
    if(new_command){
        break;
      }
    currentPosition = leftWheel.read();
    currentError = countout - currentPosition;
    if(currentError < 3){ //If the error is less than 0 keep going straight
      digitalWrite(wheelRightDirection, LOW);
      digitalWrite(wheelLeftDirection, HIGH);
      analogWrite(9, 200); //Send PWM value to motors
      analogWrite(10, 100);
    }else if(currentError > 3){ //If error is more that 0 correct by going backwards
      digitalWrite(wheelRightDirection, LOW);
      digitalWrite(wheelLeftDirection, HIGH);
      analogWrite(9, 200); //Send PWM value to motors
      analogWrite(10, 105);
    }else{ //Break loop if error is 0
      analogWrite(9, 0);
      analogWrite(10, 0);
      break;
    }
  }
}

//Unused square function
void doSquare(){
  turnAngle(90);
  delay(100);
  drive(1);
  delay(100);
  turnAngle(-90);
  delay(100);
  drive(2);
  delay(100);
  turnAngle(-90);
  delay(100);
  drive(2);
  delay(100);
  turnAngle(-90);
  delay(100);
  drive(2);
  delay(100);
  turnAngle(-90);
  delay(100);
  drive(1);
  delay(100);
}
