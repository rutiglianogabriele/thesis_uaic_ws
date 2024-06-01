#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;
std_msgs::String str_msg;

//Publishing to the topic "Gripper_status"
ros::Publisher chatter("gripper_status", &str_msg);

void open_gripper(){
  char gripper_open[14] = "Gripper Open!";
  Serial.println("Gripper is Open");
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);

  //publishing the state of the gripper
  str_msg.data = gripper_open;
  chatter.publish( &str_msg);
  
}

void close_gripper(){
  char gripper_closed[16] = "Gripper Closed!";
  Serial.println("Gripper is Closed");
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);

  //publishing the state of the gripper
  str_msg.data = gripper_closed;
  chatter.publish( &str_msg);
}

void gripperAction( const std_msgs::Bool& action_msg){
  if(action_msg.data == false){
    open_gripper();
  }
  if(action_msg.data == true){
    close_gripper();
  }
}


//Subscribing to the topic "Gripper_command"
ros::Subscriber<std_msgs::Bool> sub("gripper_close", gripperAction);

void setup() {
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  Serial.begin(57600);
}

void loop() {
//  open_gripper();
//  delay(3000);
//  close_gripper();
  nh.spinOnce();
  delay(3000);
}
