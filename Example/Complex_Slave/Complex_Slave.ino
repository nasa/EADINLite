// This code demonstrates how to use the EADIN library on a slave node that is constantly polling for messages
//   from the master node. However, the slave node must also update it's sensed state at least every X milliseconds,
//   even if it doesn't receive a message from the master node.

// INCLUDES
#include "eadin.h" // custom libary included later!

// CLASSES
EADIN rs485(0x01);

// SCHEDULING
unsigned int hampsterWheel = 15; // (ms) the maximum time between updating sensor information
unsigned long timeout = millis() + hampsterWheel;

// STORAGE
uint8_t data_in[8] = {0x00};
uint8_t data_out[8] = {0x00};

void setup() {
  // Start the RS-485 Connection
  rs485.begin(&Serial1,4000000,4);
}

void loop() {
  
  // service RS485 network
  int flag = 0;
  flag = rs485.read(data_in); // -> read data from master
  if (flag == 1){ // we have detected a message from the master
    rs485.write(data_out); // < - send data to master
  } 
  
  // While Loop - put processes that are only run occasionally in here  
  if (flag == 1 || millis() > timeout)
    { // deleting the flag == 1 will cause the code to execute independeltly of the timing of the master read
    // this may cause sincronization problems because when the code inside this if statement is executing
    // messages from the master controller are being ignored
    
    // ADD SOME CODE TO RUN A SENSOR / ACTUATOR HERE
    // It will execute every 15 milliseconds or after a successful read from the master
    timeout = millis() + hampsterWheel;
    } 
}

