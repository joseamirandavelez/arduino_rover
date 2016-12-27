/****************************************************************************************************************************************
 Simple Arduino robot controller
 Version: 1.0
 Author: José Miranda
 License: Creative COmmons (CC) Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) (https://creativecommons.org/licenses/by-sa/4.0/)
 Dependencies: Adafruit PMWServoDriver.h
 Requirements: Arduino
               Adafruit Servo Shield
               Regular servo at channel 0
               Continuous rotation servos at channels 1 and 2
               Sunfounder SF-SR02 ultrasonic distance finder at analog input 0
*****************************************************************************************************************************************/

//#include <Wire.h>     //This seems to be unnecesary, so its commented out
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();      //Sets the shield to address 0x40

#define sensorServo 0             //Sensor Servo PWM channel
#define leftServo 1               //Left motion servo
#define rightServo 2              //Right motion servo
#define moveSensorServo true      //Set to false to stop the sensor servo
#define SIG A0                    //Sensor signal write/read at Analog 0
#define debug false               //Set the debug flag
#define debugDist true            //Set the debug distance flag
#define servoLowLimit 100         //Servo's low
#define servoHighLimit 300        //Servo's high 
#define stopSpeed 200             //Servo's stop speed
#define sensorDelay 250
#define rotationDelay 250
#define reverseDelay 500
#define stopDelay 100

int degree = 0;
float distance;

void setup() {
  if(debug==true || debugDist==true){
    Serial.begin(9600);
    Serial.println("Running Setup routine...");
    Serial.flush();
  }
  pwm.begin();
  pwm.setPWMFreq(60);  //Sets the PWM frequency to 60 Hz. Analog servos run at ~60 Hz updates.
  yield();
}

void loop() {  
  int distance = 0;
  distance = ping();                                    //Measure distance
  if (distance > 50) {
    moveFwd(100);                                       //If the distance is longer than 50cm, keep moving forward.
  }/* else if (distance <= 50) {                          //If distance is less than 50 cm
    moveFwd(3.4*(distance-20));                         //Move forward at a speed calculated with the formula: speed%=3.4*(distance-20).
                                                        //This slows down as the robot approaches an obstacle. In the future, I am going
                                                        //to take advantage of this and will start determining the best route before reaching the obstacle.
                                                        //For now it just slows down until it stops. The formula may have to be tweaked in order to get a
                                                        //smoother stop.
  }*/
  if (distance <= 10) {
    stopMoving();
    moveAft(100);
    rotateToBestAngle();
  } else if (distance <= 20) {
    stopMoving();
    rotateToBestAngle();
  }    
}

//Moves the sensor servo to the desired angular position between -45 and 45 deg
void sensorAngle(int degree) {
  int servomin = 220;
  int servomax = 370;
  uint16_t pulselength;
  pulselength = map(degree, -45, 45, servomin, servomax);
  if(debug==true){
    Serial.println(degree);
    Serial.flush();
    Serial.println("");
    Serial.flush();
  }
  pwm.setPWM(sensorServo, 0, pulselength);
  delay(sensorDelay);
}

void stopMoving() {
  int degree =0;
  uint16_t pulselength = map(degree, -100, 100, servoLowLimit, servoHighLimit);
  pwm.setPWM(leftServo, 0, pulselength);
  pwm.setPWM(rightServo, 0, pulselength);
    if(debug==true){
      Serial.println("Stop");
      Serial.flush();
  }
  delay(stopDelay);
}

//Moves the robot forward
void moveFwd(int speedValue) {
  uint16_t pulselengthLeft = map(speedValue, 0, 100, stopSpeed, servoHighLimit);
  uint16_t pulselengthRight = map(speedValue, 0, 100, stopSpeed, servoLowLimit);
  pwm.setPWM(leftServo, 0, pulselengthLeft);
  pwm.setPWM(rightServo, 0, pulselengthRight);
  
  if(debug==true){
    Serial.print("Speed:");
    Serial.println(speedValue);
    Serial.print(" Left: ");
    Serial.println(pulselengthLeft);
    Serial.print("Right: ");
    Serial.println(pulselengthRight);
    Serial.println("");
    Serial.flush();
  }

}


//Moves the robot aft
void moveAft(int speedValue) {
  uint16_t pulselengthLeft = map(speedValue, 0, 100, stopSpeed, servoLowLimit);
  uint16_t pulselengthRight = map(speedValue, 0, 100, stopSpeed, servoHighLimit);
  pwm.setPWM(leftServo, 0, pulselengthLeft);
  pwm.setPWM(rightServo, 0, pulselengthRight);
  
  if(debug==true){
    Serial.print("Speed:");
    Serial.println(speedValue);
    Serial.print(" Left: ");
    Serial.println(pulselengthLeft);
    Serial.print("Right: ");
    Serial.println(pulselengthRight);
    Serial.println("");
    Serial.flush();
  }
  delay(reverseDelay);
}


//Rotates the robot left
void rotateLeft(int speedValue) {
  uint16_t pulselengthLeft = map(speedValue, 0, 100, stopSpeed, servoLowLimit);
  uint16_t pulselengthRight = map(speedValue, 0, 100, stopSpeed, servoLowLimit);
  pwm.setPWM(leftServo, 0, pulselengthLeft);
  pwm.setPWM(rightServo, 0, pulselengthRight);
  
  if(debug==true){
    Serial.print("Speed:");
    Serial.println(speedValue);
    Serial.print(" Left: ");
    Serial.println(pulselengthLeft);
    Serial.print("Right: ");
    Serial.println(pulselengthRight);
    Serial.println("");
    Serial.flush();
  }
  delay(rotationDelay);
}


//Rotates the robot right
void rotateRight(int speed) {
  uint16_t pulselengthLeft = map(speed, 0, 100, stopSpeed, servoHighLimit);
  uint16_t pulselengthRight = map(speed, 0, 100, stopSpeed, servoHighLimit);
  pwm.setPWM(leftServo, 0, pulselengthLeft);
  pwm.setPWM(rightServo, 0, pulselengthRight);
  
  if(debug==true){
    Serial.print("Speed:");
    Serial.println(speed);
    Serial.print(" Left: ");
    Serial.println(pulselengthLeft);
    Serial.print("Right: ");
    Serial.println(pulselengthRight);
    Serial.println("");
    Serial.flush();
  }
  delay(rotationDelay);
}


//Measure the distance using the ultrasonic sensor
int ping() {
  int dist;
  unsigned long rxTime;
  
  pinMode(SIG, OUTPUT);                     //Set SIG as Output to send ping
  digitalWrite(SIG, HIGH);                  //Generate a pulse of 10us
  delayMicroseconds(30);      
  digitalWrite(SIG, LOW);
  pinMode(SIG, INPUT);                      //Set SIG as Input to read ping
  rxTime = pulseIn(SIG, HIGH);              //Wait for the ping to be heard
  //Serial.println(rxTime);
  dist = (float)rxTime * 34029 / 1000000;       //Conver the time to distance
  if (dist < 2) {
    dist = 0;
  }
  if(debugDist==true){
    Serial.print(dist);
    Serial.println("");
    Serial.flush();
  }
  return dist;
}

int getAngle() {
  int pingResult;
  int bestAngle;
  for (int angle =-45; angle <= 45; angle +=90) {       //Check left and right (can be adjusted to iterate on more angles)
      sensorAngle(angle);                               //Rotate sensor
      pingResult = ping();                              //Get distance
      if (pingResult >= distance) {                     //For each of the angles, store the one with the longest distance
        distance = pingResult;
        bestAngle = angle;
      }
    }
    sensorAngle(0);                                     //Move sensor back to home position
    return bestAngle;
}

void rotateToBestAngle() {
  int bestAngle;
  bestAngle=getAngle();                                 //Get angle with longest clearance
  if (bestAngle > 0) {                                  //If best angle is less than zero (to the left)
    rotateLeft(25);                                       //Rotate left
  } else {                                              //If not
    rotateRight(25);                                      //Rotate right                            
  }
}



