#include <stdint.h>

#ifndef EADIN_H
#define EADIN_H

#include "Arduino.h"
// set to true to get more outputs on how the protocol is working
// NOTES:
// The header file should have the default parameters, the cpp should not.

class EADIN
{
    private: // these variables and functions are not accessible to subclasse

        void setPreamble();

        void setHeadder(bool T_cmd_only = false);

        void setData(uint8_t _data[]);

        uint16_t CRCSlow(uint8_t T_message[]);

        uint16_t CRCFast(uint8_t T_message[]);

        void CRC16Fast_table();

        void setCRC();

        uint16_t checkCRC(uint8_t T_message[]);

        void serialBroadcast();

        bool message_start();
        
        void cleanSerial(uint8_t bytes = 0);

    public: // visible to all outside the scope
        // Default Constructor
        EADIN(uint8_t T_my_ID = random(1,255), bool qCRC = true, uint16_t poly = 0xC599);
    
        // (see website for general creation: http://www.learncpp.com/cpp-tutorial/85-constructors/)
        void begin(HardwareSerial *T_rw_port = &Serial1, uint32_t T_baud=115200, uint8_t T_RTS=4); // constructor with parameters

        void write(uint8_t _data[], uint8_t dest = 0x00, bool cmd_only = false );

        uint8_t read(uint8_t rx_data[]); // returns a read success / failure flag

        void end();
};

#endif

