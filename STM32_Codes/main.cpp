#include "SpeedControl.h"
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

void encoder1();
void encoder2();
void encoder3();
void encoder4();
void encoder5();
void encoder6();

long count1=0, count2=0, count3=0, count4=0, count5=0, count6=0;

SpeedControl FRONTLEFT;
SpeedControl FRONTRIGHT;

SpeedControl REARRIGHT;
SpeedControl REARLEFT;

volatile float LEFT=0;
volatile float RIGHT=0;

ros::NodeHandle nh2;

void resetCallback1(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    NVIC_SystemReset();
}

// void remoteCb(const std_msgs::Float32 &msg)
// {
//     delay(100);
//     analogWrite(PB0, 0);
//     analogWrite(PB8, 0);
//     analogWrite(PA5, 0);
//     analogWrite(PB4, 0);
//     analogWrite(PA1, 0);
//     analogWrite(PB13, 0);
// }

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> resetService("reset_stm32", &resetCallback1);

std_msgs::Float32 FRONTLEFT_currentVelocity;
std_msgs::Float32 FRONTRIGHT_currentVelocity;

std_msgs::Float32 REARLEFT_currentVelocity;
std_msgs::Float32 REARRIGHT_currentVelocity;

std_msgs::Int32 FRONTLEFT_currentPosition;
std_msgs::Int32 FRONTRIGHT_currentPosition;

std_msgs::Int32 REARLEFT_currentPosition;
std_msgs::Int32 REARRIGHT_currentPosition;


double *setpoint1=&FRONTLEFT.setPoint;
double *setpoint2=&FRONTRIGHT.setPoint;

double *setpoint5=&REARLEFT.setPoint;
double *setpoint6=&REARRIGHT.setPoint;


void messageCb(const geometry_msgs::Twist &vel) {

  LEFT = (float)vel.linear.x - ((float)vel.angular.z);
  RIGHT = (float)vel.linear.x + (float)vel.angular.z;
  
  *setpoint1 = LEFT;
  *setpoint2 = RIGHT;

  *setpoint5 = LEFT;
  *setpoint6 = RIGHT;
}

ros::Publisher FRONTLEFT_currentVelocity_pub("FRONTLEFT_currentVelocity", &FRONTLEFT_currentVelocity);
ros::Publisher FRONTRIGHT_currentVelocity_pub("FRONTRIGHT_currentVelocity", &FRONTRIGHT_currentVelocity);

ros::Publisher REARLEFT_currentVelocity_pub("REARLEFT_currentVelocity", &REARLEFT_currentVelocity);
ros::Publisher REARRIGHT_currentVelocity_pub("REARRIGHT_currentVelocity", &REARRIGHT_currentVelocity);

ros::Publisher FRONTLEFT_currentPosition_pub("FRONTLEFT_currentPosition", &FRONTLEFT_currentPosition);
ros::Publisher FRONTRIGHT_currentPosition_pub("FRONTRIGHT_currentPosition", &FRONTRIGHT_currentPosition);

ros::Publisher REARLEFT_currentPosition_pub("REARLEFT_currentPosition", &REARLEFT_currentPosition);
ros::Publisher REARRIGHT_currentPosition_pub("REARRIGHT_currentPosition", &REARRIGHT_currentPosition);

ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel", &messageCb);
// ros::Subscriber<std_msgs::Float32> remote("/remote_kill_switch", &remoteCb);

void setup() {
 
  pinMode(PC13, OUTPUT);

  // Pins for black pill
  // FRONTLEFT.setPin(PA8, PB13, PB15, PB14);  // DONE
  // FRONTRIGHT.setPin(PA7, PB0, PB1, PB2);  // DONE
  // MIDDLELEFT.setPin(PB9, PB8, PB7, PB6);  // DONE
  // MIDDLERIGHT.setPin(PB5, PB4, PA9, PA10); // DONE
  // REARLEFT.setPin(PA2, PA1, PA0, PC14);  // DONE
  // REARRIGHT.setPin(PA6, PA5, PA4, PA3);  // DONE


  // New pins according to PCB
  // FRONTLEFT.setPin(PA8, PB15, PB13, PB14);  // Done
  FRONTLEFT.setPin(PA8, PA0, PB13, PB14);  // Done
  FRONTRIGHT.setPin(PA7, PB0, PB1, PB2);    // Done
  // MIDDLELEFT.setPin(PB9, PB5, PB7, PB6);  // Done  (Connections are reversed)
  // MIDDLERIGHT.setPin(PB5, PB4, PA10, PA9);
  REARLEFT.setPin(PB6, PB7, PB9, PB8);
  REARRIGHT.setPin(PB5, PB4, PB3, PA15);

  
  FRONTLEFT.setCPR(38500);
  FRONTRIGHT.setCPR(38500);
  REARLEFT.setCPR(38500);
  REARRIGHT.setCPR(38500);

  FRONTLEFT.setReversePolarity(false);
  FRONTRIGHT.setReversePolarity(false);
  REARLEFT.setReversePolarity(false);
  REARRIGHT.setReversePolarity(false);

  attachInterrupt(digitalPinToInterrupt(PB13),encoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(PB1),encoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(PB9),encoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(PB3),encoder4,RISING);
  attachInterrupt(digitalPinToInterrupt(PB14),encoder5,RISING);
  attachInterrupt(digitalPinToInterrupt(PB8),encoder6,RISING);
  

  FRONTLEFT.setPIDValue(60, 80, 20);
  FRONTRIGHT.setPIDValue(60, 80, 20); 
  REARLEFT.setPIDValue(60, 80, 20);
  REARRIGHT.setPIDValue(60, 80, 20);

  nh2.initNode();
  nh2.subscribe(velocity_sub);
  // nh2.subscribe(remote);
  nh2.advertise(FRONTLEFT_currentVelocity_pub);
  nh2.advertise(FRONTRIGHT_currentVelocity_pub);
  nh2.advertise(REARLEFT_currentVelocity_pub);
  nh2.advertise(REARRIGHT_currentVelocity_pub);

  nh2.advertise(FRONTLEFT_currentPosition_pub);
  nh2.advertise(FRONTRIGHT_currentPosition_pub);
  nh2.advertise(REARLEFT_currentPosition_pub);
  nh2.advertise(REARRIGHT_currentPosition_pub);

  nh2.advertiseService(resetService);

}

void loop() {

  nh2.spinOnce();
  if (nh2.connected())
  {
     digitalWrite(PC13, LOW);

  FRONTLEFT.setSpeed(*setpoint1);
  FRONTRIGHT.setSpeed(*setpoint2);
  
  REARLEFT.setSpeed(*setpoint5);
  REARRIGHT.setSpeed(*setpoint6);

  if (*setpoint1>0){
    FRONTLEFT.setCount(count1);
  FRONTRIGHT.setCount(count2);
  
  REARLEFT.setCount(count3);
  REARRIGHT.setCount(count4);
  }

  else{
    FRONTLEFT.setCount(count5);
  FRONTRIGHT.setCount(count2);

  REARLEFT.setCount(count6);
  REARRIGHT.setCount(count4);
  }

  FRONTLEFT.controlLoop();
  FRONTRIGHT.controlLoop();
  REARLEFT.controlLoop();
  REARRIGHT.controlLoop();

  FRONTLEFT_currentVelocity.data = FRONTLEFT.currentVelocity;
  FRONTLEFT_currentVelocity_pub.publish(&FRONTLEFT_currentVelocity);



  REARLEFT_currentVelocity.data = REARLEFT.currentVelocity;
  REARLEFT_currentVelocity_pub.publish(&REARLEFT_currentVelocity);

  FRONTRIGHT_currentVelocity.data = FRONTRIGHT.currentVelocity;
  FRONTRIGHT_currentVelocity_pub.publish(&FRONTRIGHT_currentVelocity);


  REARRIGHT_currentVelocity.data = REARRIGHT.currentVelocity;
  REARRIGHT_currentVelocity_pub.publish(&REARRIGHT_currentVelocity);


  FRONTLEFT_currentPosition.data = FRONTLEFT.currentPosition;
  FRONTLEFT_currentPosition_pub.publish(&FRONTLEFT_currentPosition);


  REARLEFT_currentPosition.data = REARLEFT.currentPosition;
  REARLEFT_currentPosition_pub.publish(&REARLEFT_currentPosition);

  FRONTRIGHT_currentPosition.data = FRONTRIGHT.currentPosition;
  FRONTRIGHT_currentPosition_pub.publish(&FRONTRIGHT_currentPosition);


  REARRIGHT_currentPosition.data = REARRIGHT.currentPosition;
  REARRIGHT_currentPosition_pub.publish(&REARRIGHT_currentPosition);

  
  }
  else
  {
    digitalWrite(PC13, HIGH);
  }
  
  delay(100);
  }


void encoder1()
{
  int b = digitalRead(PB14);
  if (b==0)
  {
    count1--;
  }
  else{
    count1++;
  }
}
void encoder2()
{
  int b = digitalRead(PB2);
  if (b==0)
  {
    count2--;
  }
  else{
    count2++;
  }
}

void encoder3()
{
  int b = digitalRead(PB8);
  if (b==0)
  {
    count3--;
  }
  else{
    count3++;
  }
}

void encoder4()
{
  int b = digitalRead(PA15);
  if (b==0)
  {
    count4--;
  }
  else{
    count4++;
  }
}

void encoder5()
{
  int b = digitalRead(PB13);
  if (b==0)
  {
    count5++;
  }
  else{
    count5--;
  }
}

void encoder6()
{
  int b = digitalRead(PB9);
  if (b==0)
  {
    count6++;
  }
  else{
    count6--;
  }
}
