/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

int poscase(int charin) {
  int pos;
  str_msg.data = (int)charin;
  chatter.publish( &str_msg );
  charin = 'a';
  switch (charin) {
    case 'a':
      pos = 1;
      break;
    case 'b':
      pos = 2;
      break;
    case 'c':
      pos = 3;
      break;
    case 'd':
      pos = 4;
      break;
    case 'e':
      pos = 5;
      break;
    default:
      pos = 3;
      break;
  }
  return pos;  
}

void messageCb( const std_msgs::String& toggle_msg){
  char* input = (const char*) toggle_msg.data;
//    input[input_size] = 0;
    char* command = strtok(input, "&");
    while(command != 0)
    {
      char* separator = strchr(command, ':');
      if (separator != 0) {
        *separator = 0;
        char x[2];// = command;
        int xpos = poscase(command);
        String foo;
        foo = String(xpos);
        foo.toCharArray(x,2);
        str_msg.data = x;
        chatter.publish( &str_msg );
//        int xpos = atoi(command);         
//        inX = constrain(xpos, EYE_MIN, EYE_MAX);
        ++separator;
        char y[2];// = separator;
        int ypos = poscase(separator);
        foo = String(ypos);
        foo.toCharArray(y,2);
        str_msg.data = y;
        chatter.publish( &str_msg );
//        int ypos = atoi(separator);
//        inY = constrain(ypos, EYE_MIN, EYE_MAX);
//        Serial.print("xpos/ypos: ");
//        Serial.print(inX);
//        Serial.print("/");
//        Serial.println(inY);
      }
      command = strtok(0, "&");
//      Serial.println("tick");
    }
}

ros::Subscriber<std_msgs::String> sub("toggle_led", messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
//  str_msg.data = hello;
//  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
