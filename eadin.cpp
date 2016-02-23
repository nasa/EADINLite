# include "eadin.h"


//********************* BUILD OPTIONS ***********************
/* DEBUG: Uncomment this line if you want to see debugging output
 please note that debugging requires allot of memory and 
 it may not load on microprocessors with less than 2.5KB of dynamic memory */

//#define DEBUG

/* MEMORY: Comment out this line if you want to decrease the memory footprint 
of the program by ~ 500B . This will limit the program to using only CRCSlow, 
which will increase the time for message creation, transmission, and reply 
receipt.*/

#define enable_CRCFast

//******************END BUILD OPTIONS ***********************


// Setup local variables
uint32_t baud;
HardwareSerial* rw_port;
uint8_t RTS;
uint8_t my_ID;
float waitTime; // used in message_start() function
float timeOutFactor;
bool wwFlag = false; // used exclusively by the master to help space out timing between multiple write requests (write write flag)
                    // false means we have not attempted multiple writs in a row without reading first
                    // the write command has a built in delay and will reset the wwFlag to false

// setup message specific bytes and their meaning
// from here you can change the hard coded values of the start byte information
// or the ENQ/ACK/NACK values if you desire. Additionally, the polynomial
// used in the CRC calculations can be changed here. Changing the polynomial
// will also cause the recaluclation of CRCtables1 and CRCtables2 below only if 
// the qCRC flag is set to true in the constructor
static const uint8_t BREAK0 = 0x00;
static const uint8_t BREAK1 = 0x01;
static const uint8_t SYNC = 0x55;
static const uint8_t ENQ = 0x05;
static const uint8_t ACK = 0x06;
static const uint8_t NAK = 0x15;
uint16_t polynomial = 0xC599;

// setup message length information
// this can be used to dynamically change the side of the data payload part 
// of the message. Increased message length will cause increases in message
// transport delay from the master to the slave nodes. Additionally, the
// delay functions built into the code (waitTime and timeOutFactor) may need 
// to be retuned if overall message size exceeds 18 bytes (which is the default size)
static const uint8_t PREAMBLE_L = 3;
static const uint8_t HEADDER_L = 5;
static const uint8_t DATA_L = 8;
static const uint8_t FOOTER_L = 2;
static const uint8_t MESSAGE_L = PREAMBLE_L+HEADDER_L+DATA_L+FOOTER_L;

uint8_t message[MESSAGE_L];
uint8_t preamble[PREAMBLE_L] = {BREAK0,BREAK1,SYNC};
uint8_t headder[HEADDER_L];
uint8_t _data[DATA_L];
uint8_t footer[FOOTER_L];

uint8_t target = 0x00; // destination address
uint8_t message_no = 0x00;

bool slave = false; // is this a slave node
bool debug = false; // show error messages on Serial. port
bool quickCRC = false; // which CRC method to use (fast or slow)

/* 
The following CRCtables1 and CRCtables2 were originally a single uint16_t
table, however, the arduino platform was unable to handle a uint16_t
table that had 256 elements in it. By breaking it up into two uint8_t 
tables, and then recombining the individual lookup elements into a single 
uint16_t value at the time of function execution, we successfully worked 
around the problem. 
The root of the problem is most likely due to a memory paging issue, where 
the arudino microcontroller has several sections of memory that is 256
bytes large, and anything that goes over that length can't be referenced
properly. 
*/

#ifdef enable_CRCFast
uint8_t CRCtable1[256] = 
   {0x00, 0xFF, 0xFE, 0x01, 0xFC, 0x03, 0x02, 0xFD, 0xF8, 
    0x07, 0x06, 0xF9, 0x04, 0xFB, 0xFA, 0x05, 0xF0, 0x0F, 
    0x0E, 0xF1, 0x0C, 0xF3, 0xF2, 0x0D, 0x08, 0xF7, 0xF6, 
    0x09, 0xF4, 0x0B, 0x0A, 0xF5, 0xE0, 0x1F, 0x1E, 0xE1, 
    0x1C, 0xE3, 0xE2, 0x1D, 0x18, 0xE7, 0xE6, 0x19, 0xE4, 
    0x1B, 0x1A, 0xE5, 0x10, 0xEF, 0xEE, 0x11, 0xEC, 0x13, 
    0x12, 0xED, 0xE8, 0x17, 0x16, 0xE9, 0x14, 0xEB, 0xEA, 
    0x15, 0xC0, 0x3F, 0x3E, 0xC1, 0x3C, 0xC3, 0xC2, 0x3D, 
    0x38, 0xC7, 0xC6, 0x39, 0xC4, 0x3B, 0x3A, 0xC5, 0x30, 
    0xCF, 0xCE, 0x31, 0xCC, 0x33, 0x32, 0xCD, 0xC8, 0x37, 
    0x36, 0xC9, 0x34, 0xCB, 0xCA, 0x35, 0x20, 0xDF, 0xDE, 
    0x21, 0xDC, 0x23, 0x22, 0xDD, 0xD8, 0x27, 0x26, 0xD9, 
    0x24, 0xDB, 0xDA, 0x25, 0xD0, 0x2F, 0x2E, 0xD1, 0x2C, 
    0xD3, 0xD2, 0x2D, 0x28, 0xD7, 0xD6, 0x29, 0xD4, 0x2B, 
    0x2A, 0xD5, 0x80, 0x7F, 0x7E, 0x81, 0x7C, 0x83, 0x82, 
    0x7D, 0x78, 0x87, 0x86, 0x79, 0x84, 0x7B, 0x7A, 0x85, 
    0x70, 0x8F, 0x8E, 0x71, 0x8C, 0x73, 0x72, 0x8D, 0x88, 
    0x77, 0x76, 0x89, 0x74, 0x8B, 0x8A, 0x75, 0x60, 0x9F, 
    0x9E, 0x61, 0x9C, 0x63, 0x62, 0x9D, 0x98, 0x67, 0x66, 
    0x99, 0x64, 0x9B, 0x9A, 0x65, 0x90, 0x6F, 0x6E, 0x91, 
    0x6C, 0x93, 0x92, 0x6D, 0x68, 0x97, 0x96, 0x69, 0x94, 
    0x6B, 0x6A, 0x95, 0x40, 0xBF, 0xBE, 0x41, 0xBC, 0x43, 
    0x42, 0xBD, 0xB8, 0x47, 0x46, 0xB9, 0x44, 0xBB, 0xBA, 
    0x45, 0xB0, 0x4F, 0x4E, 0xB1, 0x4C, 0xB3, 0xB2, 0x4D, 
    0x48, 0xB7, 0xB6, 0x49, 0xB4, 0x4B, 0x4A, 0xB5, 0xA0, 
    0x5F, 0x5E, 0xA1, 0x5C, 0xA3, 0xA2, 0x5D, 0x58, 0xA7, 
    0xA6, 0x59, 0xA4, 0x5B, 0x5A, 0xA5, 0x50, 0xAF, 0xAE, 
    0x51, 0xAC, 0x53, 0x52, 0xAD, 0xA8, 0x57, 0x56, 0xA9, 
    0x54, 0xAB, 0xAA, 0x6B}; // the table to store the CRCFAST() shortcut information in

uint8_t CRCtable2[256] = 
   {0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x02, 0x02, 0x07, 
    0x07, 0x06, 0x06, 0x04, 0x04, 0x05, 0x05, 0x0F, 0x0F, 
    0x0E, 0x0E, 0x0C, 0x0C, 0x0D, 0x0D, 0x08, 0x08, 0x09, 
    0x09, 0x0B, 0x0B, 0x0A, 0x0A, 0x1F, 0x1F, 0x1E, 0x1E, 
    0x1C, 0x1C, 0x1D, 0x1D, 0x18, 0x18, 0x19, 0x19, 0x1B, 
    0x1B, 0x1A, 0x1A, 0x10, 0x10, 0x11, 0x11, 0x13, 0x13, 
    0x12, 0x12, 0x17, 0x17, 0x16, 0x16, 0x14, 0x14, 0x15, 
    0x15, 0x3F, 0x3F, 0x3E, 0x3E, 0x3C, 0x3C, 0x3D, 0x3D, 
    0x38, 0x38, 0x39, 0x39, 0x3B, 0x3B, 0x3A, 0x3A, 0x30, 
    0x30, 0x31, 0x31, 0x33, 0x33, 0x32, 0x32, 0x37, 0x37, 
    0x36, 0x36, 0x34, 0x34, 0x35, 0x35, 0x20, 0x20, 0x21, 
    0x21, 0x23, 0x23, 0x22, 0x22, 0x27, 0x27, 0x26, 0x26, 
    0x24, 0x24, 0x25, 0x25, 0x2F, 0x2F, 0x2E, 0x2E, 0x2C, 
    0x2C, 0x2D, 0x2D, 0x28, 0x28, 0x29, 0x29, 0x2B, 0x2B, 
    0x2A, 0x2A, 0x7F, 0x7F, 0x7E, 0x7E, 0x7C, 0x7C, 0x7D, 
    0x7D, 0x78, 0x78, 0x79, 0x79, 0x7B, 0x7B, 0x7A, 0x7A, 
    0x70, 0x70, 0x71, 0x71, 0x73, 0x73, 0x72, 0x72, 0x77, 
    0x77, 0x76, 0x76, 0x74, 0x74, 0x75, 0x75, 0x60, 0x60, 
    0x61, 0x61, 0x63, 0x63, 0x62, 0x62, 0x67, 0x67, 0x66, 
    0x66, 0x64, 0x64, 0x65, 0x65, 0x6F, 0x6F, 0x6E, 0x6E, 
    0x6C, 0x6C, 0x6D, 0x6D, 0x68, 0x68, 0x69, 0x69, 0x6B, 
    0x6B, 0x6A, 0x6A, 0x40, 0x40, 0x41, 0x41, 0x43, 0x43, 
    0x42, 0x42, 0x47, 0x47, 0x46, 0x46, 0x44, 0x44, 0x45, 
    0x45, 0x4F, 0x4F, 0x4E, 0x4E, 0x4C, 0x4C, 0x4D, 0x4D, 
    0x48, 0x48, 0x49, 0x49, 0x4B, 0x4B, 0x4A, 0x4A, 0x5F, 
    0x5F, 0x5E, 0x5E, 0x5C, 0x5C, 0x5D, 0x5D, 0x58, 0x58, 
    0x59, 0x59, 0x5B, 0x5B, 0x5A, 0x5A, 0x50, 0x50, 0x51, 
    0x51, 0x53, 0x53, 0x52, 0x52, 0x57, 0x57, 0x56, 0x56, 
    0x54, 0x54, 0x55, 0x87}; // the table to store the CRCFAST() shortcut information in
#endif

//+++++++++++++++++++++++++
//    PUBLIC FUNCTIONS
//+++++++++++++++++++++++++

// Default Constructor
EADIN::EADIN(uint8_t T_my_ID /*= random(1,255)*/, bool qCRC /*= true*/, uint16_t poly /*= 0xC599*/)
{
    my_ID = T_my_ID;// node ID (0x00 = master, all others are slaves) 
    // if my_ID is left blank, a random value will be assigned and printed 
    // to the serial port if debug is true
    quickCRC = qCRC; // set to true if you want to use the fast method or false if you want to use the slow method    
    // error handling if quick method is selected but data array supporting it is disabled to conserve memory
    #ifndef enable_CRCFast 
        #ifdef DEBUG
            if(qCRC)
            {
                Serial.println("ERROR: CRCFast selected but enable_CRCFast is commented out in eadin.cpp to save memory!");  
                Serial.println("Switching to CRCSlow");
            }
        #endif
        qCRC = false;
    #endif
    polynomial = poly; // the polynomial to use for the CRC calculations (default = 0xC599)
    if (my_ID!= 0x00){slave=true;} // this is a slave node
    // don't put any delays or Serial.prints here because delays brick the arduino and Serial.prints show up before you can physically open up the terminal
}

// (see website for general creation: http://www.learncpp.com/cpp-tutorial/85-constructors/)
void EADIN::begin(HardwareSerial *T_rw_port /*= &Serial1 */, uint32_t T_baud /* = 115200 */, uint8_t T_RTS /* = 4 */) // constructor with parameters
{
    delay(3000); // we always want to have a short delay before the start of the program to enable
                // access to the boot loader incase we screw up loading a program
    #ifdef DEBUG
        Serial.begin(115200);
        Serial.print("Node ID (HEX): ");
        Serial.println(my_ID,HEX);
        Serial.println("To start node with specific ID, pass ID argument to EADIN() at class assignment.");
        if (slave){Serial.println("This node is a Slave node.");}
        else {Serial.println("This node is the Master node.");}
        Serial.println("Starting hardware serial port.");
    #endif
    // do some checks to make sure input values are valid using the assert() function?
    // set the values based on default / input
    rw_port = T_rw_port; // the port used for control communication
    baud = T_baud; // baud rate of communication
    (*rw_port).begin(baud);
    (*rw_port).setTimeout(0.5);// (milliseconds) any amount of time below 1 
    // will cause readBytes & readBytesUntil to run as fast as possible without 
    // a timeout. Thus 0.5 is essentially = zero timeout
    RTS = T_RTS; // the read transmit switch, a must with RS-485 networks
    // pull the RTS to low incase it was high for some reason
    digitalWrite(RTS,LOW); // this is an arduino IDE specific function

    // ending delay for read function
    // these may need to be retuned if overall message size increases past 18 bytes
    waitTime = 1000*115200/baud; //(micros)
    timeOutFactor = 575+75*4000000/baud; // (micros)
    
    // Recalculate CRCFast table if required if the table is enabled
    #ifdef enable_CRCFast
        if (quickCRC && polynomial != 0xC599)
        {
            CRC16Fast_table();
        }
    #endif
}

void EADIN::end(){
    // this call closes the port
    #ifdef DEBUG
        Serial.println("Closing hardware serial port.");
    #endif
    (*rw_port).end();        
}

void EADIN::write(uint8_t _data[], uint8_t dest /* = 0x00 */, bool cmd_only /* false */)
{
    // this call writes a command to the port. If the node is the master node,
    // then a destination node ID must be specified. If it is not, the master
    // will communicate with the last node that data was sent to. Thus, if 
    // the master previously communicated with node 0x02, and you call OBJECT.write(data)
    // without specifying a destination, the master will again write information 
    // to node 0x02. If the node is not the master node, you can leave 'dest' 
    // blank, as slave nodes can only communicate with the master by design.
    // This is because we are basing the control scheme on a distributed rather
    // than a decentralized architecture.
    // cmd_only can be set to true only if the node is the master node. This
    // will set the 4th byte of the message to NAK as opposed to ENQ. This
    // indicates to a slave node that no response to the message is expected 
    // Please note that the handling of NAK vs. ENQ vs. ACK messages must be 
    // programmed by the user after reading the whole message using OBJECT.read(data).
    // Thus, if you wanted a slave to not respond to NAK messages, you would
    // need to execute the following code on the micro:
    // OBJECT.read(data); if (data[3]!=NAK){OBJECT.write(response_data);}

    // error handling if user enters incorrect input based on the node ID of
    // this microcontroller and it's message target
    if (slave && target != 0x00) // illegal user input (slave targeting slave)
    {
        target = 0x00;        
        #ifdef DEBUG
            Serial.println("Illegal operation: Slave nodes must write to master and cannot write to other slave nodes.");
            Serial.println("WARNING: Correcting illegal usage by redirecting slave to talk to master.");
        #endif        
    }
    else if (!slave && dest == 0x00)// illegal user input (master targeting master)
    {
        #ifdef DEBUG
            Serial.println("Illegal operation: Master cannot write to itself.");
            Serial.println("WARNING: Correcting illegal usage by attempting to use last known good target.");
        #endif
        if (target == 0x00){
            #ifdef DEBUG
                Serial.println("WRITE FAILURE: Unable to redirect. Last known good target is self!");
            #endif
            return;
            // we can't reassign target = dest because both target from previous
            // time and dest are invalid
        }
        #ifdef DEBUG
            else {
                    Serial.print("SUCCESS: Last known good target is node (HEX) : ");
                    Serial.println(target,HEX);                        
                // don't have to take any action here because target is already valid      
            }
        #endif  
    }
    else{target = dest;} // assign target as destination from command input

    #ifdef DEBUG
        Serial.print("Writing to hardware serial port. Target Node (HEX) : ");
        Serial.println(target,HEX);
    #endif

    // setup the message information
    // this information is stored for the duration of OBJECT.write(data) in the
    // 'uint8_t message[MESSAGE_L]' data array
    setPreamble(); // set the first 3 bytes of the message

    // increment the unique message number
    // each message sent by the master has a new random key. This helps the 
    // master node identify if the slave is responding to the master's latest 
    // request or if the slave is responding to an old request. i.e. 
    // master sets message_no to 0x07 and asks slave node for pressure reading
    // slave does not respond within timeout
    // master sets message_no to 0xA1 and asks same slave node for pressure reading
    // slave responds with pressure reading and message_no of 0xA1
    // in this scenario, the master knows that it has received the most updated
    // information from the slave node. If the slave node responded back with
    // message_no of 0x07, the master would know that the data is 'stale' 
    // in hard-real-time systems, 'stale' data is thrown out rather than used 
    // for control information. This is due to the fact that stale data can 
    // cause unexpected oscillations in a control system.
    if (!slave){message_no = random(2,255);} 

    // set the ENQ/ACK/NAK as well as the rest of the headder information
    if (!slave) {setHeadder(cmd_only);} // only masters can send command-only (NAK) commands
    else {setHeadder();}

    // load the desired data into the message        
    setData(_data); 

    // create the CRC of the message
    // the CRC information is usually limited to message less the preamble
    // see the OBJECT.setCRC() command for details. 
    setCRC();

    // Inject delay if we are writing multiple times without reading. 
    // If you are the master and you've written a message without reading 
    // a message, wait for a few microseconds to help remote nodes clear 
    // their buffers from the message you just sent.
    // see Rest wwFlag below
    if (!slave & wwFlag)
    {
        delayMicroseconds(500);
        #ifdef DEBUG
            Serial.println("CAUTION: Multiple message writes without reading has been detected.");
            Serial.println("NOTE: New message write speed delayed by 1/2 milisecond.");
            Serial.println("      To avoid this delay in the future, set cmd_only to true, e.g. OBJ.write(mess,dest,true); ");
        #endif
    }
    else if (!slave){wwFlag = true;} // set the read-read flag)
    else {}
    // we don't want the master to continuously write messages without giving slaves the chance to clear out thier buffers of messages not meant for them
    // note: this was moved from the bottom of the code to ensure that this message 
    // would have the best chance of being read as opposed to trying to ensure that 
    // the next message has a good chance of being read

    // send message[MESSAGE_L]
    serialBroadcast();
    #ifdef DEBUG
        Serial.print("Sent Message (HEX): ");
        for (uint8_t i=0; i<MESSAGE_L; i++)
        {
            Serial.print(message[i],HEX);Serial.print(",");
        }
        Serial.println("");
    #endif

    if (slave){message_no = 0x00;} // clear the message number as it is only good once

    // Reset wwFlag
    if (!slave & cmd_only){delayMicroseconds(500);wwFlag = false;} // give everyone time to wipe their buffers after sending this message
    // this command makes it faster to write the next time, but penalizes you this time
    // we did this to ensure that multiple write requests, or time sensitive write requests
    // such as actuator commands, could be carried out quickly after the control sequence was calculated
    // e.g. master calculates next control value and then immediatly commands actuator with NAK
    // then master has to wait 500micros after sending that command.
    // if the master writes too fast, a node may not have time to finish
    // processing the last message before receiving a new message, this will
    // then cause the node to time out before it can respond to the masters 
    // inquiry. This section of code was inserted after extensive testing and 
    // debugging.
}

uint8_t EADIN::read(uint8_t rx_data[]){ // returns a read success / failure flag
    // the OBJECT.read(data) function does not require a target. If the master
    // node is executing this function call, it is implied that the master will
    // attempt to read from node it last tried to write to.
    // If the node is a slave node, the slave node can only talk to the master
    // node by design. 
    if (target == 0x00 && my_ID == 0x00) // illegal operation, master trying to read from itself
    {
        #ifdef DEBUG
            Serial.println("Illegal operation: Master is attempting to read message from itself.");
            Serial.println("READ FAILURE: Target node set incorrectly by OBJ.write() command.");            
            Serial.println("SOLUTION: Master must write a message to a valid target before it can read a message from that target.");
            Serial.println("Please use OBJ.write() to write to a valid target before attempting to use OBJ.read() command.");            
        #endif
        return 0;
    }

    #ifdef DEBUG
        Serial.print("Reading from hardware serial port. Target Node (HEX) : ");
        Serial.println(target,HEX);
    #endif  

    if (!slave){wwFlag = false;} // clear the write-write flag which is only used by the master

    uint8_t rx_message[MESSAGE_L]; // assign a local var to hold message info
    uint8_t Bdex =0; // the byte index that we are writing to, if you don't

    // READ PREAMBLE
    // PREAMBLE - FIND START
    // find the first couple bytes of the message, typically a 0x00 & 0x01
    if (message_start()==0)
    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 2 - Could not find start of message");
        #endif
        return 2;
    }
    else {rx_message[0]=BREAK0;
         rx_message[1]=BREAK1;}
    //delayMicroseconds(500); // give the buffer time to build up the rest of the message

    // PREAMBLE - CHECK SYNC
    // read the next byte. This sync byte is not necessary in the arduino
    // implementation but is specified by the EADIN spec that has been released so far
    uint8_t sync = 0x00;
    (*rw_port).readBytes(&sync,1);
    if (sync!=SYNC)    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 3 - incorrect sync string received or no sync string received");
        #endif
        // cleanSerial(1); probably don't need this here
        return 3;
    }
    else {rx_message[2]=sync;}

    // READ HEADDERS
    Bdex=PREAMBLE_L; // we are now ready to write the Xth byte of data
    // we've already read a few bytes of the message (START and SYNC bytes)
    // now we need to read the rest of the message which is MESSAGE_Length - 3 bytes
    if ((*rw_port).readBytes(&rx_message[Bdex],HEADDER_L) != HEADDER_L)    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 4 - we were unable to read all the headder data");
        #endif
        cleanSerial(DATA_L+FOOTER_L); // clear out alteast this many bits
        return 4;
    }

    // DATA INTEGRITY CHECKS
    // CHECK - REQUEST TYPE
    if ((slave && (rx_message[Bdex]==ENQ || rx_message[Bdex]==NAK)) || (!slave && rx_message[Bdex]==ACK)){} // correct request type 
        // note master will not accept NAK from slave, only slaves accept NAK from master
    else
    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 5 - incorrect request type");
        #endif
        cleanSerial(DATA_L+FOOTER_L); // clear out alteast this many bits
        return 5;
    }
    Bdex++; // we are looking at the next byte in the message

    // CHECK - DESTINATION
    // Check who the message is addressed to
    if (rx_message[Bdex] != my_ID)    
    {
        #ifdef DEBUG
            Serial.println("READ WARNING: Flag 6 - data is not for this node");
        #endif
        cleanSerial(DATA_L+FOOTER_L); // clear out alteast this many bits
        return 6;
    } 
    Bdex++; // we are looking at the next byte in the message

    // CHECK - SOURCE
    // check who sent the message
    if (rx_message[Bdex] != target)
    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 7 - data is not from expected source node");
        #endif
        cleanSerial(DATA_L+FOOTER_L); // clear out alteast this many bits
        return 7;
    } 
    Bdex++; // we are looking at the next byte in the message

    // CHECK - MESSAGE #
    // Check for the message number
    // to read any message, set message_no to 0x00 when calling read_message
    if (!slave && rx_message[Bdex] != message_no)
    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 8 - message# received did not match with expected message#");
        #endif
        cleanSerial(DATA_L+FOOTER_L); // clear out alteast this many bits
        return 8;
    } 
    else {message_no=rx_message[Bdex];} // assign the current message# to the output letter info    
    Bdex++; // we are looking at the next byte in the message

    // CHECK - TBD
    Bdex++; // we are looking at the next byte in the message

    // READ DATA & FOOTER
    Bdex=PREAMBLE_L+HEADDER_L; // we are now ready to read the Xth byte of data
    if ((*rw_port).readBytes(&rx_message[Bdex],DATA_L+FOOTER_L) != DATA_L+FOOTER_L)    
    {
        #ifdef DEBUG
            Serial.println("READ FAILURE: Flag 9 - we were unable to read all the message data & footer");
        #endif
        cleanSerial(FOOTER_L); // clear out alteast this many bits
        return 9;
    } 

    // CHECK - CRC
    // Verify the CRC matches with what was sent in the message
    // note: indexed at zero so to get the last two elements we have to use -2 and -1  
    uint16_t crc = checkCRC(rx_message);
    if (crc == word(rx_message[MESSAGE_L-2],rx_message[MESSAGE_L-1]))
    {
        // CRC was successful, 
        // copy the received message to the output
        for (uint8_t i=0; i<DATA_L; i++){
            rx_data[i]=rx_message[PREAMBLE_L+HEADDER_L+i];}
        #ifdef DEBUG
            Serial.println("READ SUCCESS: Flag 1 - all checks passed");
            Serial.print("Read Message (HEX): ");
            for (uint8_t i=0; i<MESSAGE_L; i++){Serial.print(message[i],HEX);Serial.print(",");}
            Serial.println("");
        #endif
        return 1;
        } 
    else 
    {
        // crc failed
        // clear the message field for good measure
        for (uint8_t i=0; i<DATA_L; i++){
            rx_data[i]=0x00;}
            #ifdef DEBUG
                Serial.println("READ FAILURE: Flag 10 - crc check failed");
            #endif
            return 10;
   } 
}  


//+++++++++++++++++++++++++
//    PRIVATE FUNCTIONS
//+++++++++++++++++++++++++

void EADIN::setPreamble()
{
    // creates the preamble which is typically the first 3 bytes of a message
    // it assigns this information to the 'message[MESSAGE_L]' array
    #ifdef DEBUG
        Serial.println("fcall: setPreamble()");
    #endif
    for (uint8_t i=0;i<PREAMBLE_L;i++){
        message[i]=preamble[i];}
}
            
// Creates the headder part of the message based on who the message is for
// only master controller sends messages directly to other nodes, only other nodes will send messages to master
void EADIN::setHeadder(bool T_cmd_only /* = false */)
{
    #ifdef DEBUG
        Serial.println("fcall: setHeadder()");
    #endif
    // default 0x00 message_no means accept any message

    // if the message is for the master
    if (slave){message[PREAMBLE_L] = ACK;}
    // message is from the master
    else if (T_cmd_only){message[PREAMBLE_L] = NAK;} // no acknowledgement needed, command only
    else{message[PREAMBLE_L] = ENQ;}

    message[PREAMBLE_L+1] = target;
    message[PREAMBLE_L+2] = my_ID;
    message[PREAMBLE_L+3] = message_no;
    message[PREAMBLE_L+4] = 0xFA; // place holder 
}

void EADIN::setData(uint8_t _data[])
{
    // sets the data portion of the message in the 'message[MESSAGE_L]' array
    #ifdef DEBUG
        Serial.println("fcall: setData()");
    #endif      
    for (uint8_t i=0; i<DATA_L; i++){
        message[PREAMBLE_L+HEADDER_L+i]=_data[i];}
}

// ++++++++++++++++++ CRC SLOW ++++++++++++++++++++++++++++++++++++++++++
uint16_t EADIN::CRCSlow(uint8_t T_message[]){
    #ifdef DEBUG
        Serial.println("fcall: CRCSlow()");
    #endif
    // a simple way to implment a CRC16 without a table lookup
    // the method has been expanded to include allowing the user to skip preamble
    // and footer bytes and thus not include them in the CRC calculation

    // INPUTS
    // message[] - the data array to be used for the CRC
    // PREAMBLE_L - the number of bytes to skip before starting the CRC
    // FOOTER_L - the number of bytes to skip at the end of the message
    // MESSAGE_L - constant that describes message length in bytes
    
    // for reference see:
    // A painless guide to CRC Error Detection Algorithms
    // Ross N. WIlliams
    // http://www.ross.net/crc/download/crc_v3.txt

    uint16_t crc = 0x0000; // initial CRC value to 2 bytes of zeros per EADIN spec
    for (int j=PREAMBLE_L; j<MESSAGE_L-FOOTER_L; j++){ // for every byte in the message
        crc ^= T_message[j]; // XOR is addition and subtraction for bits. This adds 0x0000 onto the message.
        for (int i = 0; i < 8; ++i){
            int 
            flag = crc & 0x0001; 
            crc >>= 0x0001; // shift one bit to the right
            if (flag) // if a 1 pops out of the buffer
                crc ^= polynomial; // XOR the result with the EADIN polynomial
        }
    }
    crc |= 1 << 15; // writes a 1 to the most significant bit, this is a C++ generic implementation
    //bitSet(remainder,15); // writes a 1 to the most significant bit, this is an arduino specific implementation
    return crc;
}

#ifdef enable_CRCFast
// +++++++++++++++++ CRC FAST +++++++++++++++++++++++++++++++++++++++++++++
uint16_t EADIN::CRCFast(uint8_t T_message[]){
    // this is a faster CRC method but provides less strength than the slow method
    // this method allows the user to skip a given number of bytes in the 
    // PREAMBLE and footer of the data and thus not include them in the CRC calcs

    #ifdef DEBUG
        Serial.println("fcall: CRCFast()");
    #endif
    // INPUTS
    // message[] - the data array to be used for the CRC
    // PREAMBLE_L - the number of bytes to skip before starting the CRC
    // FOOTER_L - the number of bytes to skip at the end of the message
    // MESSAGE_L - constant that describes message length in bytes

    // set this value to zero if you want to CRC the whole message
    uint8_t data_tmp = 0xAA; // temporary holding place
    uint16_t remainder = 0x0000; // initial remainder
    for (int m = PREAMBLE_L; m < MESSAGE_L-FOOTER_L; m++){
        data_tmp = T_message[m] ^ (remainder >> 8); // take just one byte of the remainder 
        remainder =  word(CRCtable1[data_tmp],CRCtable2[data_tmp]) ^ (remainder << 8); // using the data calculated previously, XOR it with the polynomial, then XOR that with the other byte of the remainter, then repeat
    // we had to split the tables up because the arduino mega2560 didn't like a single array of size uint16_t [256]    
    }
    remainder |= 1 << 15; // writes a 1 to the most significant bit, this is a C++ generic implementation
    //bitSet(remainder,15); // writes a 1 to the most significant bit, this is an arduino specific implementation
    return remainder;
}

// +++++++++++++++ CRC FAST TABLE +++++++++++++++++++++++++++++++++++++
void EADIN::CRC16Fast_table(){
    #ifdef DEBUG
        Serial.println("fcall: CRC16Fast_table()");
    #endif
    // create the table of all possible XOR combinations of a given 2byte CRC   
    // typical computation time is 61 miliseconds
    #ifdef DEBUG
        Serial.println("Clearing CRCFast tables 1 & 2...");
    #endif
    for (uint16_t i=0; i<0xFF; i++){CRCtable1[i]=0x0000;CRCtable2[i]=0x0000;}
    #ifdef DEBUG
        Serial.print("Rebuilding CRCFast table with polynomial (HEX): ");
        Serial.println(polynomial,HEX);
    #endif
    for (uint16_t i=0x00; i<=0xFF; i++){
        // per page 14 of A painless guide to crc error detection algorithms by Williams,
        // we can XOR the i value 8 times into the polynomial and we will get a unique value out
        // this is my interpretation of how we can make a unique table
        // example of how moving the bits works
        //  Serial.println("");
        //  Serial.println("Bit Moving");
        //  uint16_t tester = 0xFF;
        //  Serial.println(tester,BIN);
        //  Serial.println(tester<<=1,BIN);
      
        uint16_t divisor = i;
        uint16_t temp_table[8];
      
        // XOR the 0-255# with the polynomial 8 times
        for (int j=0;j<8;j++){
            temp_table[j] = divisor ^ polynomial;
            divisor <<= 1; // shift I over to the left by one bit (adds a zero)
        }
        
        // XOR those 8 results with eachother 
        //CRCtable[i]=temp_table[0]^temp_table[1]^temp_table[2]^temp_table[3]^temp_table[4]^temp_table[5]^temp_table[6]^temp_table[7];
        uint16_t data_split=temp_table[0]^temp_table[1]^temp_table[2]^temp_table[3]^temp_table[4]^temp_table[5]^temp_table[6]^temp_table[7];
        CRCtable1[i] = data_split & 0x00FF;
        CRCtable2[i] = data_split >> 8;
    }
    #ifdef DEBUG
        Serial.println("Rebuilding CRCFast table complete.");
        Serial.println("New CRC Table : CRCtabel1[256]: (HEX)");
        for (uint8_t i=0; i<0xFF; i++){Serial.print(CRCtable1[i],HEX);Serial.print(",");}
        Serial.println("");
        Serial.println("New CRC Table : CRCtabel2[256]: (HEX)");
        for (uint8_t i=0; i<0xFF; i++){Serial.print(CRCtable2[i],HEX);Serial.print(",");}
        Serial.println("");
    #endif
}
#endif

// +++++++++++++++ SETCRC +++++++++++++++++++++++++++++++++++++++++++++
void EADIN::setCRC()
{
    // sets the CRC portion of the message in the 'message[MESSAGE_l]' array
    #ifdef DEBUG
        Serial.println("fcall: setCRC()");
    #endif
    // only contains the CRC check
    uint16_t crc;
    #ifdef enable_CRCFast
        if (quickCRC){crc = CRCFast(message);} // determine which CRC Method to use
        else {crc = CRCSlow(message);}
    #else
        crc = CRCSlow(message);
    #endif
    // perform the CRC on the whole message less the first three bytes (headders) 
    // and the last two bytes which are reserved for the CRC
    // assign the CRC info to the message
    message[MESSAGE_L-2] = crc >> 8; // take the left 8 bits by shifting the 16 bit value over 8 bits to the right
    message[MESSAGE_L-1] = crc & 0x00FF; // take the right 8 bits by &ing the crc with 0x00FF
    // message is now complete
}

// +++++++++++++++ CHECK CRC ++++++++++++++++++++++++++++++++++++++++
uint16_t EADIN::checkCRC(uint8_t T_message[])
{
    // verifies that the CRC contained inside of the 'message[MESSAGE_L]' array
    // matches that calculated by this program
    #ifdef DEBUG
        Serial.println("fcall: checkCRC()");
    #endif
    // only contains the CRC check
    uint16_t crc;
    #ifdef enable_CRCFast
        if (quickCRC){crc = CRCFast(T_message);} // determine which CRC Method to use
        else {crc = CRCSlow(T_message);}
    #else
        crc = CRCSlow(T_message);
    #endif

    // perform the CRC on the whole message less the first three bytes (headders) 
    // and the last two bytes which are reserved for the CRC
    // assign the CRC info to the message
    return crc;
}

// +++++++++++++++ SERIAL BROADCAST++++++++++++++++++++++++++++++++++++
void EADIN::serialBroadcast()
{            
    // SEND MESSAGE contained in the 'message[MESSAGE_L]' array
    #ifdef DEBUG
        Serial.println("fcall: serialBroadcast()");
    #endif
    digitalWrite(RTS, HIGH); // RTS - the read / transmit switch turn it on to write mode
    (*rw_port).write(message,MESSAGE_L);
    #ifdef DEBUG
        for (uint8_t i=0; i<MESSAGE_L; i++)
            {            
                Serial.print(message[i],HEX);Serial.print(",");
            } // write the message
        Serial.println("");
    #endif
    (*rw_port).flush(); // wait for all the message elements to be sent & the buffer to be clear before you stop transmitting
    digitalWrite(RTS, LOW); // switch to listen mode
}

//++++++++++++++++ Find Start of Message ++++++++++++++++++++++++
bool EADIN::message_start(){
    // find the start bytes of a message. By default this looks for 0x00 0x01 
    // in the message buffer. 
    #ifdef DEBUG
        Serial.println("fcall: message_start()");
    #endif
    uint8_t temp0, temp1 = 0xAB;   
    uint8_t bytes_arrived;
    bool flag = true; // first time 
    float timeOut = micros() + timeOutFactor;
    while (timeOut > micros())
    {
        #ifdef DEBUG
            Serial.println("While Loop");
        #endif
        bytes_arrived = (*rw_port).available();
        if (bytes_arrived>MESSAGE_L*1.5) // we are a message behind, wipe the buffer
        {
            #ifdef DEBUG
                Serial.println("Buffer > Message_L*1.5");
            #endif
            // note, originally we tried if (bytes_arrived>MESSAGE_L*2), but what would happen is that
            // when a message wasn't read in time (flag 2 on master), then master would be perpetually 1 message behind
            // we decreased this to be less than MESSAGE_L*2 because that way you couldn't store two complete messages in the buffer
            // thus you couldn't read the old message first, you'd have to clean the buffer of both messages, and then get the third message
            cleanSerial(bytes_arrived);
        }
        if (bytes_arrived>MESSAGE_L/2) // we might have a message, try to read it     
        {
            #ifdef DEBUG
                Serial.println("Buffer > Message_L/2");
            #endif
            if (flag){(*rw_port).readBytes(&temp0,1);flag = false;} // grab the first byte and store for this session
            for (int i =0; i<MESSAGE_L*2; i++)
            {
                (*rw_port).readBytes(&temp1,1); // read one byte at a time
                if (temp0 == BREAK0 && temp1 == BREAK1)
                {
                    delayMicroseconds(waitTime); // give the rest of the message some time to buffer
                    return 1;
                } 
                else {temp0=temp1;} 
            }
        }    
    }
    return 0; // we have been unable able to find a message within j iterations
}  

// +++++++++++++++ CLEAR THE SERIAL BUFFER ++++++++++++++++++++++++++++
void EADIN::cleanSerial(uint8_t bytes /* = 0 */){
    // clear X bytes from the serial buffer, if X is unassigned, will clear all 
    // bytes. Note that we assume a maximum serial buffer size of 64 bytes. 
    // Platforms with different buffer size may want to increase or decrease
    // the maximum serial buffer size 'max_buf'.

    uint8_t max_buf = 64; // maximum serial buffer size of the hardware
    uint8_t bytes_arrived;
    #ifdef DEBUG
        Serial.println("fcall: cleanSerial()");
    #endif
    if (bytes==0){bytes_arrived= (*rw_port).available();}
    else {bytes_arrived = bytes;}
    // clean out the serial buffer so we're ready for any new messages
    // we want to prevent old message stack up and partial message stackup incase we error out of reading something
    uint8_t black_hole [max_buf]; // trash bin array
    // the buffer should only be max_buf Bytes wide, but in future versions it 
    // might be larger.
    // limit the number of bytes we'll destroy to that of our black hole size
    if (bytes_arrived>max_buf){bytes_arrived=max_buf;}
    #ifdef DEBUG
        Serial.println("Check Arrived");
    #endif
    (*rw_port).readBytes(&black_hole[0],bytes_arrived); // place a bunch of bits in the garbage
    #ifdef DEBUG
        Serial.println("Delete");
    #endif
}