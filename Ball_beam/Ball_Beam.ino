
#include<Servo.h>
#include<PID_v1.h>


const int servoPin = 9;                                              //Servo Pin
  
float Kp_low = 2.5;                                                  //Set PID gains
float Ki_low = 0.2;                                                      
float Kd_low = 0.5; 

float Kp_low = 5;                                                   
float Ki_low = 0;                                                      
float Kd_low = 2.5; 

double Setpoint, Input, Output, ServoOutput;                                       


PID myPID(&Input, &Output, &Setpoint, Kp_low, Ki_low, Kd_low, DIRECT);    //Initialize PID object, which is in the class PID.                                                                                                            
                                                                     
Servo myServo;                                                       //Initialize Servo.

void setup() {

  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //position as the input to the PID algorithm

 
  myPID.SetMode(AUTOMATIC);                                          
  myPID.SetOutputLimits(-40,40);                                     //Set Output limits to protect hardware
}

void loop()
{
 
  Setpoint = 20;                                                     // Can be changed to any desired position
  Input = readPosition();                                            
 
  double gap = abs(Setpoint - Input);
  
  if (gap < 2) {
    myPID.SetTunings(Kp_low, Ki_low, Kd_low);
  } else {
    myPID.SetTunings(Kp_high, Ki_high, Kd_high);
  }
  
  myPID.Compute();                                                   //Get output
  
  ServoOutput = 75 + Output;                                         //CALIBRATE HERE !!!! (set number to anything for horizontal)
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
  
  
}
      
      
      

float readPosition() {
  const int trigPin = 5;
  const int echoPin = 6;
  long duration, distance;

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  digitalWrite(trigPin, LOW);                                         //Clear trig pin
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);                                        //Set trigpin high for 10 ms
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); 
  
  duration = pulseIn(echoPin, HIGH);                                  //Get ultrasound wave travel time


  distance = duration * 0.034/2;
    
  if(distance > 30)                                                   //Set max distance
  {distance = 30;}
  
  
  Serial.println(distance);
  
  return distance;                                                    //Returns distance value.
}
