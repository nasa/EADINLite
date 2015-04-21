#include "eadin.h" // custom libary included later!


// DEFINITIONS
#define RTS 4 // the switch used to set read / transmit
#define my_ID 0x00 // the ID of this node

// ALLOCATE MESSAGE MEMORY AND STRUCTURES
uint8_t data[DATA_L];
uint8_t message[MESSAGE_L];
struct missive_essentials content;
uint8_t request_type;
uint8_t unique_id; // unique message identifier
uint8_t flag=0x00; // used to indicate message success/failure
uint16_t sensorData=0x0000;

// DEBUGGING VARIABLES
float start_time;
float stop_time;

void setup() {  
  delay(5000);
  
  Serial.begin(115200);
  Serial.println("Sketch : EADIN_Master");
  
  // Start the RS-485 Connection
  eadin_configure(my_ID,RTS,&Serial1,4000000);
}

void loop() {
    start_time=micros();
    
    // set what message number we are on  
    unique_id=random(2,255); // unique message id between 2 and 255, a unique id of zero means accept any message 
    
    // contact a remote node
    content.request_type=ENQ;
    content.message_no=unique_id;
    content.node_ds=0x01;
    write_missive(&content); // write the new message
    flag = read_missive(&content); // listen for a response
    stop_time=micros();
    if (flag==1){ // successful message read
      sensorData = word(content._data[1],content._data[0]); 
      Serial.print("Message Read : Success, ");
      Serial.print(sensorData);
      Serial.print(", Round Trip Time (micros) : ");
      Serial.println(stop_time-start_time);
    }
    else {
      Serial.print("Message Read : Failure, ");
      Serial.print(flag); 
      Serial.print(", Round Trip Time (micros) : ");
      Serial.println(stop_time-start_time);
      // for failure types definition see eadin.cpp
      // quick reference:
      // #2 - didn't find a start sequence 0x00 0x01 after 50 tries
      // #3 - didn't find a sync string 0x55
      // #4 - did not receive the remainding bytes of the message
      // #5 - request type was not 0x05 / 0x06 / 0x15 (ENQ / ACK / NAK)
      // #6 - the message was not addressed to this node
      // #7 - the message was not sent from the node we are expecting a message from
      // #8 - the unique message ID was incorrect
      // #9 - the CRC check failed
      clear_serialbuffer();
    } 
    flag=0;
}
