#include "eadin.h" // custom libary included later!

// DEFINITIONS
#define RTS 4 // the switch used to set read / transmit
#define my_ID 0x01 // the ID of this node

// ALLOCATE MESSAGE MEMORY AND STRUCTURES
struct missive_essentials content;
uint8_t flag;

// POTENTIOMETER SETUP
const uint8_t sensorPin = A4; //Sensor connected to Analog 4
int sensorValue = 0;
float sensorVoltage = 0;
unsigned int sensorData = 0;


void setup() {
  delay(5000);
  
  Serial.begin(115200);
  Serial.println("Sketch Name : EADIN_Slave");
  
  // Start the RS-485 Connection
  eadin_configure(my_ID,RTS,&Serial1,4000000);
}

void loop() {
  // read the latest potentiometer reading
  sensorData = read_pot(sensorPin);

  // Read data sent to the device
  content.node_ds=0;
  content.message_no=0;
  flag = read_missive(&content);
  if (flag==1){ // messsage was successfully ready & was for this node 
    if (content.request_type==ENQ || content.request_type==NAK){ // reply is requested
      content.request_type=ACK;
      content.node_ds=0;
      content._data[0]=sensorData & 0xff; // take the right 8 bits
      content._data[1]=sensorData >> 8; // take the left 8 bits
      write_missive(&content);
    }
  }
  else
  {
    clear_serialbuffer();
  }
  flag=0;
}

// +++++++++++++++++ READ POTENTIOMETER +++++++++++++++++++++++++
unsigned int read_pot(uint8_t sensor_pin){
  int sensorValue = analogRead(sensor_pin);
  float sensorVoltage= sensorValue*(5.0/1023.0); //Sets sensor value 5v=1023.0
  unsigned int out = sensorVoltage*100; // Defines out- Voltage*100
  // setDecimalsI2C(0b0000010); // sets the decimal point on the display up by 2 decimals
  // above function is included as part of the function that sets the 7segdisplay properties
  return out;
}

