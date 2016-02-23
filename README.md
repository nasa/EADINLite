# EADINLite
EADIN_Lite Network Protocol 
 
Authored by Eliot Aretskin-Hariton
NASA Glenn Research Center, Cleveland, Ohio, 44135
earetski@mail.nasa.gov

**Network Protocol Summary Stats:**
* Half-Duplex works with wired or wireless networks
* Command / Response protocol using 1 Master / Multiple Slave Architecture
* 8 Byte payload
* RTT Performance (See Table and Note below)  
      
|  Speed       |  TYPICAL        | WORSE CASE      |
|:------------:|:---------------:|:---------------:|
| 4000000 baud |     943 +/- 13  |   981 +/- 13    |
|  921600 baud |   1,197 +/- 15  |  1,280 +/-  7   |
|  115200 baud |   4,467 +/- 12  |  4,907 +/-  7   |
|    9600 baud |  45,798 +/- 12  | 50,750 +/- 20   |
|     units    | (micros 1-sigma)|(micros 1-sigma) |

Note: Performance based on message Round Trip Time (RTT), which includes
formulation of the message by the master, receipt of message by slave
and recept of respons from slave by master. master -> slave -> master. 
Time is expressed in microseconds.

* Memory Requirements

| Memeory Req. | Program Storage (Bytes) | Dynamic Memory (Bytes) |
|:------------:|:-----------------------:|:----------------------:|
| Default      | 9,510                   |    908                 |
| Minimum      | 8,618                   | 396                    |

**Overview:**
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

**Details:**
The EADIN protocal as implemented by this code has the following structure:
Total Size: 18 bytes
* Preamble: 3 bytes
	* 2 bytes start of message (0x00, 0x01)
	* 1 byte a sync  byte (0x55)
* Headder: 5 bytes 
	* 1 byte request type (ENQ/ACK/NAK = 0x05/0x06/0x15)
	* 1 byte node_ID of desination
	* 1 byte node_ID of sender
	* 1 byte unique message number
	* 1 byte extra space to be used in future development
* Data: X bytes (8 bytes Default)
	* 8 bytes DATA_L (can be modified)
* Footer: 2 bytes 
	* 2 bytes CRCFast (a 16 bit CRC, Default)


**Updates:**
Version 2 is incompatible with previous versions as it is constructed with 
different function calls using an object oriented programming approach for
easier use. The code now contains built in timing functions which should
enable the user to simply call OBJ.read() OBJ.write() functions without worrying
about inserting delays between the write and read operations. These delays
should scale with network speed selected from 9600 - 4000000 baud. 

**References:**
EADIN Lite Communication Network
Read More: http://www.techbriefs.com/component/content/article/ntb/tech-briefs/electronics-and-computers/23450

Eliot Aretskin-Hariton, Benchmarking Variants of a Hardware-in-the-Loop Simulation System
Read More: http://arc.aiaa.org/doi/abs/10.2514/6.2016-1425

Eliot Aretskin-Hariton, A Modular Framework for Modeling Hardware Elements in Distributed Engine Control Systems
Read More: http://arc.aiaa.org/doi/abs/10.2514/6.2014-3530

Dennis Culley, Developing an Integration Infrastructure for Distributed Engine Control Technologies
Read More: http://arc.aiaa.org/doi/abs/10.2514/6.2014-3532

Ross N. Williams, A painless guide to CRC Error Detection Algorithms
Read More: http://www.ross.net/crc/download/crc_v3.txt
