
#include<Servo.h>
#include<PID_v1.h>


const int servoPin = 9;                                              //Set servo Pin
  
float Kp = 2.5;                                                      //Set PID gains
float Ki = 0;                                                      
float Kd = 1.1;                                                  
double Setpoint, Input, Output, ServoOutput;                                       


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.                                                                                                            
                                                                     
Servo myServo;                                                       //Initialize Servo.

void setup() {

  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //position as the input to the PID algorithm

 
  myPID.SetMode(AUTOMATIC);                                          
  myPID.SetOutputLimits(-80,80);                                     //Set Output limits to protect hardware
}

void loop()
{
 
  Setpoint = 15;
  Input = readPosition();                                            
 
  myPID.Compute();                                                   //Get output
  
  ServoOutput = 102+Output;                                          //CALIBRATE HERE !!!! (set number to anything for horizontal)
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
  
  
}
      
      
      

float readPosition() {
  delay(40);                                                            //Prevent echo affecting output 
  
  
const int pingPin = 7;

long duration, cm;
unsigned long now = millis();
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  cm = duration/(29*2);
  
  if(cm > 30)     // 30 cm is the maximum position for the ball
  {cm=30;}
  
  
  Serial.println(cm);
  
  return cm;                                          //Returns distance value.
}
