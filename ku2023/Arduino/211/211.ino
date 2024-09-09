#define ip1 3 //1->3pin
#define ip2 5 // 2->5pin
#define ip3 6 // 3->6pin
//#define ip4 9 // 4->9pin
#define ip5 10 // 5->10pin
#define ip6 11 // 6->10pin
#define led1 22// siganl_light_r
#define led2 24// signal_light_g
#define led3 26// signal_light_b

//4,6-> 10pin

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

int ch1; //
int ch2; //
int ch3;
int ch5; //LED
int ch6; //
int val1;
int val2;
int relaypin = 10;


Servo thruster1;
Servo thruster2;


void thruster_l( const std_msgs::UInt16& cmd_msg) {
  thruster1.writeMicroseconds(cmd_msg.data);//1100-1900
}

void thruster_r( const std_msgs::UInt16& cmd_msg) {
  thruster2.writeMicroseconds(cmd_msg.data);//1100-1900
}

ros::Subscriber<std_msgs::UInt16> sub1("thrusterL", thruster_l);
ros::Subscriber<std_msgs::UInt16> sub2("thrusterR", thruster_r);

void setup()
{
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  //pinMode (ip6,OUTPUT);
  pinMode (ip1, INPUT);
  pinMode (ip2, INPUT);
  pinMode (ip3, INPUT);
  pinMode (ip5, INPUT);
  pinMode (9, INPUT);
  pinMode (8, INPUT);
  pinMode (led1, INPUT);
  pinMode (led2, INPUT);
  pinMode (led3, INPUT);

  thruster1.attach(13); // 13pin //left L
  thruster2.attach(12);// 12pin //right R

  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);

  delay(7000);
  //  Serial.begin(9600);

}

void loop()
{
//  if(Serial.available() < 0){
//    thruster1.writeMicroseconds(1500);
//    thruster2.writeMicroseconds(1500);
//  }
//  
  //  int signal_v = 1700;
  //  thruster1.writeMicroseconds(signal_v);//1100-1900
  //  thruster2.writeMicroseconds(1600);//1100-1900
//  while (Serial.available() == 0);
  //  int val = Serial.parseInt();
    Serial.println(ch1);
  ch5 = pulseIn(ip5, HIGH);
//  Serial.println(ch5);
  {
    if (ch5 < 1450)
    {
      digitalWrite(8,HIGH);
      digitalWrite(9,LOW);
      autonomous();
      digitalWrite(led1,LOW);
      digitalWrite(led2,HIGH);
      digitalWrite(led3,LOW);
    }
    else if (ch5 > 1450)
    {
      digitalWrite(8,LOW);
      digitalWrite(9,HIGH);
      rc();
      digitalWrite(led1,LOW);
      digitalWrite(led2,LOW);
      digitalWrite(led3,HIGH);
    }
  }



}
