//Slightly modified ros example button press code

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

const int button_pin = 3;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  pinMode(button_pin, INPUT);
  
  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = ! digitalRead(button_pin);
 
}

void loop()
{
  
  bool reading = digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    pushed_msg.data = reading;
    published = true;
  }
  pub_button.publish(&pushed_msg);
  last_reading = reading;
  
  nh.spinOnce();
}
