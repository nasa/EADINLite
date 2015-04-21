# EADINLite
EADIN_Lite Network Protocol

Authored by Eliot Aretskin-Hariton
NASA Glenn Research Center, Cleveland, Ohio, 44135
earetski@mail.nasa.gov

This code was created to support the Distributed Engine Control task
as part of the Fixed Wing Aeronautics project. The purpose of this research 
was to enable multiple microcontrollers to speak with eacho ther per the
protocol specified in the preliminary release of EADIN BUS. EADIN BUS is a 
candidate for distributed control systems on aircraft engines and is being
worked on by the Distributed Engine Control Working Group (DECWG) 
http://www.decwg.org/. The primary use of this code was to assist in the 
modelling of small local networks which contain 16 or fewer nodes. 
Ultimately, we expect this network to be implemented in an FPGA or ASIC 
as opposed to it's current implementation on a microcontroller. 

This communication protocol uses a master node which distributes 
information between nodes through a call and response system. The RS-485 
network is simplex and thus does not allow multiple nodes to talk at 
the same time. No time synchronization between nodes is required for 
this network. These factors enable the master to request information 
from sensors and command actuators, one at a time. In the current 
implementation, no information is passed from individual nodes without 
first going through the master node. 

While other communication protocols do exist like ModbusMaster and simple-modbus,
the speed of these communication protocols on the RS-485 network was not 
sufficient for our needs which required message send to reply receipt times
of 1 millisecond. Additionally, the other protocols did not implement the 
same message system as specified by the preliminary documents regarding
the EADIN protocol.

The EADIN protocal as implemented by this code has the following structure:
Total Size: 18 bytes
* Preamble: 3 Bytes
** byte start byte (0x00, 0x01)
** byte sync  byte (0x55)
* Headder: 5 Bytes 
** 1 byte request type (ENQ/ACK/NAK = 0x05/0x06/0x15)
** 1 byte node_ID of desination
** 1 byte node_ID of sender
** 1 byte unique message number
** 1 byte extra space to be used in future development
* Data: Variable (8 bytes Default)
** 8 Bytes DATA_L (can be modified)
* Footer: 2 Bytes 
** 2 bytes CRCFast (a 16 bit CRC, Default)

References:
	Ross N. Williams, A painless guide to CRC Error Detection Algorithms
	availalbe for download at: http://www.ross.net/crc/download/crc_v3.txt