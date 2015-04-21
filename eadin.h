/* EADIN.h & EADIN.cpp
 * 
 * Authored by Eliot Aretskin-Hariton
 * NASA Glenn Research Center, Cleveland, Ohio, 44135
 * earetski@mail.nasa.gov
 *
 * This code was created to support the Distributed Engine Control task
 * as part of the Fixed Wing Aeronautics project. The purpose of this research 
 * was to enable multiple microcontrollers to speak with eacho ther per the
 * protocol specified in the preliminary release of EADIN BUS.
 * EADIN BUS is a candidate for distributed control systems on aircraft engines.
 * The primary use of this code was to assist in the modelling of small local 
 * networks which contain 16 or fewer nodes.
 *
 * This communication protocol uses
 * a master node which distributes information between nodes through a call and
 * response system. The RS-485 network is simplex and thus does not allow
 * multiple nodes to talk at the same time. No time synchronization between 
 * nodes is required for this network. These factors enable the master to
 * request information from sensors and command actuators, one at a time. 
 * In the current implementation, no information is passed from individual
 * nodes without first going through the master node. 
 *
 * While other communication protocols do exist like ModbusMaster and simple-modbus,
 * the speed of these communication protocols on the RS-485 network was not 
 * sufficient for our needs which required message send to reply receipt times
 * of 1 millisecond. Additionally, the other protocols did not implement the 
 * same message system as specified by the preliminary documents regarding
 * the EADIN protocol.
 *
 * The EADIN protocal as implemented by this code has the following structure:
 * Total Size: 18 bytes
 * Preamble: 3 Bytes
 *  2 byte start byte (0x00, 0x01)
 *  1 byte sync  byte (0x55)
 * Headder: 5 Bytes
 *  1 byte request type (ENQ/ACK/NAK = 0x05/0x06/0x15)
 *  1 byte node_ID of desination
 *  1 byte node_ID of sender 
 *  1 byte unique message number
 *  1 byte extra space to be used in future development
 * Data: Variable (8 bytes Default)
 *  8 Bytes DATA_L (can be modified)
 * Footer: 2 Bytes
 *  2 bytes CRCFast (a 16 bit CRC, Default)
 *
 * Note: In testing CRCFast had more validation failures (undetected crc errors)
 * on single bit flips than CRCSlow. However, CRCSlow takes 
 * ~16 micro seconds per byte whereas CRCFast takes half the time. This makes
 * a considerable difference when an 18 byte message with CRCSlow takes up 
 * around 300 micro seconds to verify and you have to perform this operation 
 * four times for every ENQ / ACK (message roundtrip). Thus the CRC alone could
 * take up 1.2 miliseconds when we are trying to have a total message time 
 * of less than 1 milisecond. Thus, for fast communication speeds, it is
 * advantageous to use the CRCFast instead. You can always use internal controls
 * and limiters to discard validated data if the data does not past 'sanity check'
 * even though it passes CRC. This type of advanced functionality would have to
 * be programmed into the controller.
*/

#ifndef EADIN_H
#define EADIN_H

#include "Arduino.h"

#define ENQ 0x05
#define ACK 0x06
#define NAK 0x15


// MESSAGE SIZE SETTINGS
/* All messages sent between nodes are of constant size (even requests containing)
 * no information. A message has four parts Preamble+Headder+Data+Footer
 * The preamble is made up of start byte info and a syncronization byte
 * The preamble is not included in the CRC calculations
 * The Headder contains general message information like who the message
 * is from, who it is being sent to, and a message# to indicate to the master
 * if the message is old data. Since the network is implemented without
 * hardware interrupts, there is the possibility that a node responds 
 * with dated information and we must protect against this possibility.
 * the data part of the message contains any tybe of data the nodes wish to 
 * send to / from the master.
 * The footer contains CRC information, which is a hash of the Headder+Data
 * parts of the message.
 *
 * Note: Any changes to PREAMBLE_L, HEADDER_L, or FOOTER_L will require the user
 * to edit the code in eadin.cpp to ensure that these extra bytes are being
 * handeled properly.
 * Increasing / Decreasing the size of DATA_L requires no changes to eadin.cpp
*/ 
#define PREAMBLE_L 3
// the number of bytes that happens before the headder
// the primary purpose of the preamble section is to separate out parts of 
// the message that will not be used in the CRC calculations
// preamble contains the break and sync fields

# define HEADDER_L 5
// the number of bytes in the headder 

#define DATA_L 8
// the maximum amount of data bytes (not headder or footer)
// that can be sent in a message
// this value must be manually resized if any of the headder, footer, or 
// message length byte sizes change. For the size of the headder and footers,
// read the void write_message() function

#define FOOTER_L 2
// the number of bytes at the end of the message

const uint8_t MESSAGE_L = PREAMBLE_L+HEADDER_L+DATA_L+FOOTER_L; 
// maximum length of message to be sent in bytes

#define DEBUG false 
// set to true to get more outputs on how the protocol is working

#define CRCMethod CRCFast
// the CRC implementation we will use CRCFast | CRCSlow are the options
#define POLYNOMIAL 0xC599 
// polynomial for the CRC

#define BREAK0 0x00
#define BREAK1 0x01
#define SYNC 0x55

typedef struct missive_essentials{
    uint8_t request_type;
    uint8_t node_ds; // destination / source of message 
    uint8_t message_no;
    uint8_t _data[DATA_L];}; // define the message structure type

// ++++++++++++++ function definitions +++++++++++++++++++++++
// these do not include the subfunctions called by write_messsage and read_message
// those are defined separately in eadin.cpp

void eadin_configure(
    uint8_t node_id, 
    uint8_t read_transmit_switch,
    HardwareSerial *serialport,
    unsigned long serial_speed);

void write_missive(missive_essentials *content);

uint8_t read_missive(missive_essentials *content);

void clear_serialbuffer();

void blabbering_id10t();
#endif