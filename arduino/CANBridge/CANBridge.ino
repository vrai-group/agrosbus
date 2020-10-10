// Licensed under the Apache License, Version 2.0  license. See LICENSE file in the project root for full license information.
// Luigi Di Marcantonio AGROSBUS Project UNIVPM

#define USE_USBCON
//libraries for ROS communication
#include <ros.h>
#include <agrosbuspkg/CANFrameData.h>
#include <std_msgs/UInt8.h>

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <CAN.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//for sync with actual system time
#include <Time.h> 

#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message
#define BUFFER_LENGHT 64

int l=0;

ros::NodeHandle nh;
agrosbuspkg::CANFrameData ros_pkt;
std_msgs::UInt8 int_value;

long long int epoch_at_start = 0; //time to set epoch and millis


//define where to publish
ros::Publisher fromarduino("fromarduino", &ros_pkt);
ros::Publisher uint8test("uint8test", &int_value);



//info of the two circulare queue
//from arduino to ros is buff_toROS
//from ros to arduino is buff_toCAN

typedef struct raw_data{
   unsigned char * value;
   unsigned int id;
   unsigned char len;
   unsigned long long timestamp;
} raw_data;

unsigned char headR = 0;
unsigned char tailR = 0;
unsigned int countR = 0;
raw_data buff_toROS[BUFFER_LENGHT];

unsigned char headC = 0;
unsigned char tailC = 0;
unsigned int countC = 0;
raw_data buff_toCAN[BUFFER_LENGHT];

void processSyncMessage(){
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if(pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
       epoch_at_start = pctime;
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST); 
  return 0; // the time will be sent later in response to serial mesg
}


//SET CAN SPEED
//Function to call when can't start CAN communication at 500Kbps
void set_can_speed()
{
  Serial.println("Trying 1000Kbps..");
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN 1000 failed!");
    
    Serial.println("Trying 500Kbps..");
    if(!CAN.begin(500E3)){
      Serial.println("Starting CAN 500 failed!");
      
      Serial.println("Trying 250Kbps..");
      if(!CAN.begin(250E3)){
        Serial.println("Starting CAN 250Kbps failed!");
        
        Serial.println("Trying 125Kbps..");
        if(!CAN.begin(125E3)){
          Serial.println("Starting CAN 125Kbps failed!");
          Serial.println("Error...");
          while (1);
        }
        else{
          Serial.println("CAN speed 125Kbps OK");
          return;
        }
      }
      else{
        Serial.println("CAN speed 250Kbps OK");
        return;
      }
    }
    else{
      Serial.println("CAN speed 500Kbps OK");
      return;
    }
  }
  else{
    Serial.println("CAN speed 1000Kbps OK");
    return;
  }
}


//FUNCTIONS TO ADD packets to the tails


//ARDUINO -> ROS
void AddOneToROStail(int packetSize){
  
  int i = 0;

  if(countR == BUFFER_LENGHT){
    Serial.print("Overrun from CAN..losing packets");
    return;
  }
  
  buff_toROS[headR].id = CAN.packetId();
  buff_toROS[headR].len = packetSize;
  buff_toROS[headR].timestamp = epoch_at_start + millis();

  //get the raw bytes
  buff_toROS[headR].value = new unsigned char[packetSize];
  i=0;
  while (CAN.available()) {
        buff_toROS[headR].value[i] = CAN.read();
        i++;
  }
  
  headR = headR + 1;
  headR = headR%BUFFER_LENGHT; //make sure head [0 % BUFFER_LENGHT-1]
  countR = countR + 1; //new alement in the queue  
}

//ROS -> ARDUINO
void AddOneToCANtail(const agrosbuspkg::CANFrameData& can_pkt){
  
  int i = 0;
  
  //before adding check if is full
  if(countC == BUFFER_LENGHT){
    Serial.print("Overrun from ROS..losing packets");
    return;
  }
  
  buff_toCAN[headC].id = can_pkt.id;
  buff_toCAN[headC].len = can_pkt.lenght;
  buff_toCAN[headC].timestamp = can_pkt.timestamp;

  //getting the raw bytes
  buff_toCAN[headC].value = new unsigned char[can_pkt.lenght];
  for(i=0; i<can_pkt.lenght; i++){
    buff_toCAN[headC].value[i] = can_pkt.value[i];
  }

  headC = headC + 1;
  headC = headC%BUFFER_LENGHT; //make sure head [0 % BUFFER_LENGHT-1]
  countC = countC + 1; //new alement in the queue
  
}

//FUNCTIONS TO send packets to ros or can network (Sender Board)

//TO ROS TOPIC /fromarduino
void PublishOneToROS(){
  
    ros_pkt.id = buff_toROS[headR].id;
    ros_pkt.timestamp = buff_toROS[headR].timestamp;
    l = buff_toROS[headR].len;
    ros_pkt.lenght = l;

    //define lenght and assign pointer
    ros_pkt.value_length = l;
    ros_pkt.value = buff_toROS[headR].value;

    tailR = tailR + 1;
    tailR = tailR%BUFFER_LENGHT;
    countR = countR - 1;

    fromarduino.publish(&ros_pkt);
    
    free(buff_toROS[headR].value);
    free(ros_pkt.value);
  
}

//TO Sender board

void PublishOneToCAN(){

  CAN.beginPacket(buff_toCAN[tailC].id);
  int l = buff_toCAN[tailC].len;
  int i=0;
  
  for(i=0; i<l; i++){
    CAN.write(buff_toCAN[tailC].value[i]);
  }
  CAN.endPacket();
  
  free(buff_toCAN[tailC].value);
  
  tailC = tailC + 1;
  tailC = tailC%BUFFER_LENGHT;
  countC = countC-1;

}

//SET UP AND LOOP SECTION

//setting subscription to fromros topic
ros::Subscriber<agrosbuspkg::CANFrameData> fromros("fromros", &AddOneToCANtail);

void setup() {
  
  //start the serial communication
  Serial.begin(115200);

  
  while (!Serial); //wait until ready


  //Setting the CAN connection
  Serial.print("CAN Receiver...");
  CAN.begin(500E3);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)){
      Serial.println("Starting CAN 500Kbps failed!");
      Serial.println("Trying other CAN speed:");
      set_can_speed(); //if 500 Kbps not work
  }

  
  setSyncProvider(requestSync);  //set function to callback when sync required
  Serial.println("Waiting for time sync message");
  Serial.println("Send T+epoch");

  //wait until user not send the time info
  while(!Serial.available());
  //set the Arduino time based on message received
  //wait for info about epoch
  processSyncMessage();

  //initNodeHandle
  nh.initNode();
  nh.advertise(fromarduino);
  //nh.advertise(uint8test); //test if we loose packets..
  nh.subscribe(fromros);

  epoch_at_start = epoch_at_start * pow(10,3);

  //Set call back function for each CAN packet received
  //Each time a CAN packet comes add_one() is called
  CAN.onReceive(AddOneToROStail);
}

void loop() {
   nh.spinOnce();
   
   //cleaning the toROS queue
   while(headR != tailR){
     PublishOneToROS();
   }
   //cleaning the toCAN queue
   
   while(headC != tailC){
     PublishOneToCAN();
   }
}
