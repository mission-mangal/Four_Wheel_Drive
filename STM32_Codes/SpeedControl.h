#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#include <Arduino.h>
#include <AutoPID.h>

class SpeedControl
{

private:
  // Motor Pin Configuration
  uint8_t pwmPin;
  uint8_t dirPin;
  uint8_t encA;
  uint8_t encB;
  bool polarity_reverse = false;

  //Motor Related Parameters
  
  float motorRadius = 1; // in metres
  float maxSpeed = 0.5; // Rotation Per Second

  // PID Control Related Parameter
  double outputMin  = -255;
  double  outputMax = 255;


public: 
  //Velocity control related parameters
  volatile long currentPosition;
  volatile long previousPosition;
  volatile double previousTime;
  volatile double currentTime;
  double currentVelocity =  0;
  double setPoint;
  double outputPWM;
  long CPR = 36124;
  float kP  = 1;
  float kI  = 10.0;
  float kD  = 20.0;
  // double * position;
  AutoPID pid{&currentVelocity, &setPoint, &outputPWM, outputMin, outputMax, kP, kI, kD};
  // Encoder * encoder;

  void  initialSetup()
  {
  // Setting the pin configurations
  pinMode(PC13, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  digitalWrite(dirPin , LOW);
  analogWrite(pwmPin , 0);
  // delay(2000);
  //variables for velocity calculation
  setPoint = 0;
  currentPosition = 0;
  previousPosition = 0;
  previousTime = micros();
  currentVelocity = 0;
  outputPWM;
  }

  SpeedControl()
  {
    // params:  direction pin , pwm pin , encoder A , encoder B
   
  }

  void setPin(uint8_t pinDir , uint8_t pinPWM ,uint8_t aEnc , uint8_t bEnc )
  {
  dirPin = pinDir;
  pwmPin = pinPWM;
  encA  = aEnc;
  encB = bEnc;
  // encoder  =  new Encoder(encA , encB);
  initialSetup();

  }

  ~SpeedControl()
  {
    // destructor for the class
    // delete encoder;
  }

  void setReversePolarity(bool polarity)
  {
    polarity_reverse = polarity;
  }

  void setCount(long count)
  {
    currentPosition = count;
  }

  void setMotorPWM(int PWM)
  {
  if (polarity_reverse)
    PWM = PWM * -1;
    // Changing the motor direction based on the PWM value
  if (PWM <= 0)
      digitalWrite(dirPin , LOW);
  else 
      digitalWrite(dirPin , HIGH);

  // generating the PWM pulses for changing the motor velocity
    analogWrite(pwmPin , abs(PWM));
  }

  void setPIDValue(float p , float i , float d )
  {
    kP = p;
    kI = i;
    kD = d;
    pid.setGains(kP, kI , kD);
  }

  void setPIDOutput()
  {
    // if (setPoint > 0){
      outputMin = -(setPoint*255.0)/maxSpeed;
      outputMax = (setPoint*255.0)/maxSpeed;
      if(outputMin > outputMax)
      {
        pid.setOutputRange(outputMax, outputMin);
      }
      else{
        pid.setOutputRange(outputMin , outputMax);
      }
    // }
    // else{
    //   outputMin = (setPoint*255.0)/2.0;
    //   outputMax = 0;
    //   pid.setOutputRange(outputMin , outputMax);
    // }
  }

  void setCPR(long cpr)
  {
    CPR = cpr;
  }

  void setMotorMaxSpeed(float speed)
  {
    maxSpeed = speed;
  }

  void setSpeed(float rps)
  {
    // input: motor speed in rotation per second
    if (rps < maxSpeed)
    setPoint = rps;
    else
    setPoint = maxSpeed;
  }

  void controlLoop()
  { 

    
  // if (setPoint==0.5)
  // {
  //   digitalWrite(PC13, LOW);
  // }
  // else{
  //   digitalWrite(PC13, HIGH);
  // }
  // Calculating the motor velocity
  // currentPosition = encoder->read();
  currentTime = micros();
  currentVelocity = (currentPosition - previousPosition)/((CPR / 1000000.00) * (currentTime - previousTime));
  previousPosition = currentPosition;
  previousTime = currentTime;
 // Running PID and changing motor PWM signal accordingly
  pid.run();
  setMotorPWM(outputPWM);
  // if (currentVelocity + 0.02 > setPoint ){
  //   setMotorPWM(outputPWM);
  //   pid.stop();
  // }
  
  }


};

#endif
