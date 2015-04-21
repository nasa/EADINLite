#include "eadin.h"
// required for all read/write/setup functions for EADIN

//uint16_t CRCtable[256]; // only used for CRCFAST()
const uint16_t CRCtable[256] = 
   {0x0000, 0x00FF, 0x01FE, 0x0101, 0x03FC, 0x0303, 0x0202, 0x02FD, 0x07F8, 
    0x0707, 0x0606, 0x06F9, 0x0404, 0x04FB, 0x05FA, 0x0505, 0x0FF0, 0x0F0F, 
    0x0E0E, 0xEF1,  0x0C0C, 0x0CF3, 0x0DF2, 0x0D0D, 0x0808, 0x08F7, 0x09F6, 
    0x0909, 0x0BF4, 0xB0B,  0x0A0A, 0x0AF5, 0x1FE0, 0x1F1F, 0x1E1E, 0x1EE1, 
    0x1C1C, 0x1CE3, 0x1DE2, 0x1D1D, 0x1818, 0x18E7, 0x19E6, 0x1919, 0x1BE4, 
    0x1B1B, 0x1A1A, 0x1AE5, 0x1010, 0x10EF, 0x11EE, 0x1111, 0x13EC, 0x1313, 
    0x1212, 0x12ED, 0x17E8, 0x1717, 0x1616, 0x16E9, 0x1414, 0x14EB, 0x15EA, 
    0x1515, 0x3FC0, 0x3F3F, 0x3E3E, 0x3EC1, 0x3C3C, 0x3CC3, 0x3DC2, 0x3D3D, 
    0x3838, 0x38C7, 0x39C6, 0x3939, 0x3BC4, 0x3B3B, 0x3A3A, 0x3AC5, 0x3030, 
    0x30CF, 0x31CE, 0x3131, 0x33CC, 0x3333, 0x3232, 0x32CD, 0x37C8, 0x3737, 
    0x3636, 0x36C9, 0x3434, 0x34CB, 0x35CA, 0x3535, 0x2020, 0x20DF, 0x21DE, 
    0x2121, 0x23DC, 0x2323, 0x2222, 0x22DD, 0x27D8, 0x2727, 0x2626, 0x26D9, 
    0x2424, 0x24DB, 0x25DA, 0x2525, 0x2FD0, 0x2F2F, 0x2E2E, 0x2ED1, 0x2C2C, 
    0x2CD3, 0x2DD2, 0x2D2D, 0x2828, 0x28D7, 0x29D6, 0x2929, 0x2BD4, 0x2B2B, 
    0x2A2A, 0x2AD5, 0x7F80, 0x7F7F, 0x7E7E, 0x7E81, 0x7C7C, 0x7C83, 0x7D82, 
    0x7D7D, 0x7878, 0x7887, 0x7986, 0x7979, 0x7B84, 0x7B7B, 0x7A7A, 0x7A85, 
    0x7070, 0x708F, 0x718E, 0x7171, 0x738C, 0x7373, 0x7272, 0x728D, 0x7788, 
    0x7777, 0x7676, 0x7689, 0x7474, 0x748B, 0x758A, 0x7575, 0x6060, 0x609F, 
    0x619E, 0x6161, 0x639C, 0x6363, 0x6262, 0x629D, 0x6798, 0x6767, 0x6666, 
    0x6699, 0x6464, 0x649B, 0x659A, 0x6565, 0x6F90, 0x6F6F, 0x6E6E, 0x6E91, 
    0x6C6C, 0x6C93, 0x6D92, 0x6D6D, 0x6868, 0x6897, 0x6996, 0x6969, 0x6B94, 
    0x6B6B, 0x6A6A, 0x6A95, 0x4040, 0x40BF, 0x41BE, 0x4141, 0x43BC, 0x4343, 
    0x4242, 0x42BD, 0x47B8, 0x4747, 0x4646, 0x46B9, 0x4444, 0x44BB, 0x45BA, 
    0x4545, 0x4FB0, 0x4F4F, 0x4E4E, 0x4EB1, 0x4C4C, 0x4CB3, 0x4DB2, 0x4D4D, 
    0x4848, 0x48B7, 0x49B6, 0x4949, 0x4BB4, 0x4B4B, 0x4A4A, 0x4AB5, 0x5FA0, 
    0x5F5F, 0x5E5E, 0x5EA1, 0x5C5C, 0x5CA3, 0x5DA2, 0x5D5D, 0x5858, 0x58A7, 
    0x59A6, 0x5959, 0x5BA4, 0x5B5B, 0x5A5A, 0x5AA5, 0x5050, 0x50AF, 0x51AE, 
    0x5151, 0x53AC, 0x5353, 0x5252, 0x52AD, 0x57A8, 0x5757, 0x5656, 0x56A9, 
    0x5454, 0x54AB, 0x55AA, 0x876B}; // the table to store the CRCFAST() shortcut information in


uint8_t my_ID; // the ID of this node
uint8_t RTS; // the read & transmit switch
HardwareSerial* rw_port;
unsigned long rw_speed; // (baudrate) the speed at which the above port operates
uint8_t trash[64];

//+++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++
// these are subfunctions that only need to be called by the write_message and read_message programs
bool message_start();
uint16_t CRCFast(uint8_t message[]);
void CRC16Fast_tabel(uint16_t CRCtable[]);
uint16_t CRCSlow(uint8_t message[]);

//+++++++++++++++++++ CONFIGURE EADIN +++++++++++++++++++++++++++++++
void eadin_configure(
    uint8_t node_id, uint8_t read_transmit_switch,HardwareSerial *serialport,
    unsigned long serial_speed){
    // setup the ID of the node and the READ / TRANSMIT switch
    RTS = read_transmit_switch;
    my_ID = node_id;
    rw_port = serialport;
    rw_speed = serial_speed;

    // initialize the CRCTable which is ONLY used for CRCFAST()
    // only activate this if the CONST declaration above is commented out
    //CRC16Fast_tabel(CRCtable); // only use if you are rebuilding the table

    (*rw_port).begin(rw_speed);
    (*rw_port).setTimeout(0.5); // (milliseconds) any amount of time below 1 
    // will cause readBytes & readBytesUntil to run as fast as possible without 
    // a timeout. Thus 0.5 is essentially = zero timeout
}

// +++++++++++++++++ CLEAR THE SERIAL BUFFER +++++++++++++++++++++++++
void clear_serialbuffer(){
    // turns off and on the serial port. This allows old messages to be discarded
    // which is useful when we are listening for a message from node B, 
    // we don't want to get a message from node A
    // this is not a guarantee we won't get an old message, but it is a measure we can take
    (*rw_port).end();
    (*rw_port).begin(rw_speed);
}

//// +++++++++++++++++ CLEAR THE SERIAL BUFFER +++++++++++++++++++++++++
//void clear_serialbuffer(){
//    // trashes all the data in the serial buffer. This allows old messages to be discarded
//    // which is useful when we are listening for a message from node B, 
//    // we don't want to get a message from node A
//    // this is not a guarantee we won't get an old message, but it is a measure we can take
//    (*rw_port).readBytes(&trash[0],Serial.available());
//    //(*rw_port).flush(); // wait for all the message elements to be read?
//}

//++++++++++++++++++++ WRITE MESSAGE ++++++++++++++++++++++++++++++++
void write_missive(missive_essentials *content){
    // writes a message to the RS-485 buss
    // INPUTS
    // request_type (NAK,ACK,ENQ) - the type of reply required
    // node_to - which node would you like to send the info to
    // message_no - the unique message reply identifier to allow the master to 
    //  identify old info and discard it
    // _data[] - what bytes of data to send. This is a constant size array 
    //  whos size is specified by data_L

    // Blank Message
    uint8_t message[MESSAGE_L]; 
    
    // CONSTRUCT PREAMBLE
    uint8_t preamble[PREAMBLE_L] = {BREAK0,BREAK1,SYNC};
    // preamble contains the break and sync fields

    // HEADDERS
    uint8_t headder[HEADDER_L] = {content -> request_type,content -> node_ds,my_ID,content -> message_no,0xFB};
    // request_type: ENQ-ACK-NAK
    //node_to: which node to send the request to
    //my_ID: the id of the node SENDING the message
    //message_no: the message number (used to uniquely identify a message reply)
    //0xFB: reserved for future buildout

    // DATA
    // already built

    // MESSAGE ASSEMBLY
    // put the different part of the message together
    for (uint8_t i=0;i<PREAMBLE_L;i++){
        message[i]=preamble[i];}
    for (uint8_t i=0; i<HEADDER_L; i++){
        message[PREAMBLE_L+i]=headder[i];}
    for (uint8_t i=0; i<DATA_L; i++){
        message[PREAMBLE_L+HEADDER_L+i]=content -> _data[i];}

    // FOOTERS 
    // only contains the CRC check
    uint16_t crc = CRCMethod(message); 
    // perform the CRC on the whole message less the first three bytes (headders) 
    // and the last two bytes which are reserved for the CRC
    // assign the CRC info to the message
    message[MESSAGE_L-2] = crc & 0xff; // take the right 8 bits
    message[MESSAGE_L-1] = crc >> 8; // take the left 8 bits by shifting the 16 bit value over 8 bits to the left
    // message is now complete

    // SEND MESSAGE
    digitalWrite(RTS, HIGH); // RTS - the read / transmit switch turn it on to write mode
    for (uint8_t i=0; i<MESSAGE_L; i++){(*rw_port).write(message[i]);} // write the message
    (*rw_port).flush(); // wait for all the message elements to be sent & the buffer to be clear before you stop transmitting
    digitalWrite(RTS, LOW); // switch to listen mode

}

// +++++++++++++++++ READ MESSAGE +++++++++++++++++++++++++++++++++++
uint8_t read_missive(missive_essentials *content){
    // this is the mean message function that reads a whole message
    // request_type returns ENQ/ACK/NACK to receiving node so they can properly respond to re
    // node_from - the node that is sending the message
    // to read any message, set message_no to 0x00 when calling read_message
    // _data[] - the location to store final message information
    // output - 1 = no errors, 2+ different types of errors

     // Blank Message
    uint8_t message[MESSAGE_L]; 
    uint8_t Bdex =0; // the byte index that we are writing to, if you don't
    // handle this properly you will overwrite your data!
 
    // READ PREAMBLE
    // PREAMBLE - FIND START
    // find the first couple bytes of the message, typically a 0x00 & 0x01
    if (message_start()==0){return 2;}
    else {message[0]=BREAK0;
        message[1]=BREAK1;}
        //message[2]=SYNC;}
    delayMicroseconds(30); // give the buffer time to build up the rest of the message

    // PREAMBLE - CHECK SYNC
    // read the next byte. This sync byte is not necessary in the arduino
    // implementation but is specified by the EADIN spec that has been released so far
    uint8_t sync = 0x00;
    (*rw_port).readBytes(&sync,1);
    if (sync!=SYNC){return 3;} // incorrect sync string received or no sync string received
    else {message[2]=sync;}
    
    // READ HEADDERS
    Bdex=PREAMBLE_L; // we are now ready to write the Xth byte of data
    // we've already read a few bytes of the message (START and SYNC bytes)
    // now we need to read the rest of the message which is MESSAGE_Length - 3 bytes
    if ((*rw_port).readBytes(&message[Bdex],HEADDER_L) != HEADDER_L){return 4;} // we were unable to read all the data

    // DATA INTEGRITY CHECKS
    // CHECK - REQUEST TYPE
    if (message[Bdex]==ENQ || message[Bdex]==ACK || message[Bdex]==NAK){ // correct request type 
        content -> request_type=message[Bdex];}  // set the flag on message type
    else {return 5;} // incorrect request type
    Bdex++; // we are looking at the next byte in the message

    // CHECK - DESTINATION
    // Check who the message is addressed to
    if (message[Bdex] != my_ID){return 6;} // data is not for me
    Bdex++; // we are looking at the next byte in the message

    // CHECK - SOURCE
    // check who sent the message
    if (message[Bdex] != content -> node_ds){return 7;}
    Bdex++; // we are looking at the next byte in the message

    // CHECK - MESSAGE #
    // Check for the message number
    // to read any message, set message_no to 0x00 when calling read_message
    if (content -> message_no != 0x00 && message[Bdex] != content -> message_no){return 8;} // message received did not match with expected message
    else {content -> message_no=message[Bdex];} // assign the current message# to the output letter info    
    Bdex++; // we are looking at the next byte in the message

    // CHECK - TBD
    Bdex++; // we are looking at the next byte in the message

    // READ DATA & FOOTER
    Bdex=PREAMBLE_L+HEADDER_L; // we are now ready to read the Xth byte of data
    if ((*rw_port).readBytes(&message[Bdex],DATA_L+FOOTER_L) != DATA_L+FOOTER_L){return 4;} // we were unable to read all the data

    // CHECK - CRC
    // Verify the CRC matches with what was sent in the message
    // note: indexed at zero so to get the last two elements we have to use -2 and -1  
    uint16_t crc = CRCMethod(message);
    if (crc == word(message[MESSAGE_L-1],message[MESSAGE_L-2])){
        // CRC was successful, 
        // copy the received message to the output
        for (uint8_t i=0; i<DATA_L; i++){
            content -> _data[i]=message[PREAMBLE_L+HEADDER_L+i];}
        return 1;
        } 
    else {
        // crc failed
        // clear the message field for good measure
        for (uint8_t i=0; i<DATA_L; i++){
            content -> _data[i]=0x00;}
        return 9;
        } 
}

//++++++++++++++++ Find Start of Message ++++++++++++++++++++++++
bool message_start(){
    // function verifies that we have received the correct start sequence and the message is beginning
    int j = 50*MESSAGE_L; // the number of microseconds that we should wait before assuming we can't find a message start
    // this depends on the average time it takes for a sent message to receive a complete reply 
    // see 'timeout and RTT testing.xls' in the eadin library for the calculation of the 31 number    
    j=ceil(j/14); // it takes 14 microseconds to run a .readBytes command
    uint8_t temp0, temp1 = 0xAB; // temporary values used to check start_transmission blocks
    (*rw_port).readBytes(&temp0,1);
    for (int i =0; i<j; i++){  // attempt to read the serial buffer 
        (*rw_port).readBytes(&temp1,1);
        if (temp0 == BREAK0 && temp1 == BREAK1){return 1;} 
        else {temp0=temp1;} 
    }
    return 0; // we have been able to find a message within j iterations
}

////++++++++++++++++ Find Start of Message ++++++++++++++++++++++++
//bool message_start(){
//    // function verifies that we have received the correct start sequence and the message is beginning  
//    int j = 35*MESSAGE_L; // the number of microseconds that we should wait before assuming we can't find a message start
//    // this depends on the average time it takes for a sent message to receive a complete reply 
//    // see 'timeout and RTT testing.xls' in the eadin library for the calculation of the 31 number    
//    j=ceil(j/14); // it takes 14 microseconds to run a .readBytes command
//    uint8_t temp0, temp1, temp2 = 0xAB; // temporary values used to check start_transmission blocks
//    (*rw_port).readBytes(&temp0,1);
//    for (int i =0; i<j; i++){  // attempt to read the serial buffer 
//        (*rw_port).readBytes(&temp1,1);
//        if (temp0 == BREAK0 && temp1 == BREAK1){
//            (*rw_port).readBytes(&temp2,1);
//            if (temp2 == SYNC) {return 1;}
//            else {temp0=temp2;}
//        }
//        else {temp0=temp1;}
//    }
//    return 0; // we have been able to find a message within j iterations
//}

// +++++++++++++++++ CRC FAST +++++++++++++++++++++++++++++++++++++++++++++
uint16_t CRCFast(uint8_t message[]){
    // this is a faster CRC method but provides less strength than the slow method
    // this method allows the user to skip a given number of bytes in the 
    // PREAMBLE and footer of the data and thus not include them in the CRC calcs

    // INPUTS
    // message[] - the data array to be used for the CRC
    // PREAMBLE_L - the number of bytes to skip before starting the CRC
    // FOOTER_L - the number of bytes to skip at the end of the message
    // MESSAGE_L - constant that describes message length in bytes

    // set this value to zero if you want to CRC the whole message
    uint8_t data_tmp = 0xAA; // temporary holding place
    uint16_t remainder = 0x0000; // initial remainder
    for (int m = PREAMBLE_L; m < MESSAGE_L-FOOTER_L; m++){
        data_tmp = message[m] ^ (remainder >> 8); // take just one byte of the remainder
        remainder =  CRCtable[data_tmp] ^ (remainder << 8); // using the data calculated previously, XOR it with the polynomial, then XOR that with the other byte of the remainter, then repeat
    }
    bitSet(remainder,15); // writes a 1 to the most significant didget  
    return remainder;
}

// +++++++++++++++ CRC FAST TABLE +++++++++++++++++++++++++++++++++++++
void CRC16Fast_tabel(uint16_t CRCtable[]){
    // create the table of all possible XOR combinations of a given 2byte CRC   
    // typical computation time is 61 miliseconds
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
            temp_table[j] = divisor ^ POLYNOMIAL;
            divisor <<= 1; // shift I over to the left by one bit (adds a zero)
        }
        
        // XOR those 8 results with eachother 
        CRCtable[i]=temp_table[0]^temp_table[1]^temp_table[2]^temp_table[3]^temp_table[4]^temp_table[5]^temp_table[6]^temp_table[7];
    }
}

// ++++++++++++++++++ CRC SLOW ++++++++++++++++++++++++++++++++++++++++++
uint16_t CRCSlow(uint8_t message[]){
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
    // ftp.adelaide.edu.au/pub/rocksoft/crc_v3.txt 

    uint16_t crc = 0x0000; // initial CRC value to 2 bytes of zeros per EADIN spec
    for (int j=PREAMBLE_L; j<MESSAGE_L-FOOTER_L; j++){ // for every byte in the message
        crc ^= message[j]; // XOR is addition and subtraction for bits. This adds 0x0000 onto the message.
        for (int i = 0; i < 8; ++i){
            int 
            flag = crc & 0x0001; 
            crc >>= 0x0001; // shift one bit to the right
            if (flag) // if a 1 pops out of the buffer
                crc ^= POLYNOMIAL; // XOR the result with the EADIN polynomial
        }
    }
    bitSet(crc,15); // flip the most significant bit to a 1
    return crc;
}

// ++++++++++++++++++ blabbering_id10t ++++++++++++++++++++++++++++++++++++++++++
void blabbering_id10t(){
    // this program simulates what happens when a node starts spewing random 
    // information, disregarding the traditional call/response protocol
    digitalWrite(RTS, HIGH);
    (*rw_port).write(random(0,255));
    (*rw_port).flush();
    digitalWrite(RTS, LOW);    
}

