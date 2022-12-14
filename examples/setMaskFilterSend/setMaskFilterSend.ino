/*!
 * @file setMaskFilterSend.ino
 * @brief CAN-BUS Shield, set_mask_filter_send
 * @n     this demo will show you how to use mask and filter
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2022-9-03
 * @url https://github.com/DFRobot/DFRobot_CANBUS
 */
#include <DFRobot_CANBUS.h>
#include <SPI.h>
const int SPI_CS_PIN = 10;
DFRobot_CANBUS CAN(SPI_CS_PIN); // Set CS pin
void setup()
{
    Serial.begin(115200);
    int count = 50; // the max numbers of initializint the CAN-BUS, if initialize failed first!.  
    do {
        CAN.init();   //must initialize the Can interface here! 
        if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
        {
            Serial.println("DFROBOT's CAN BUS Shield init ok!");
            break;
        }
        else
        {
            Serial.println("DFROBOT's CAN BUS Shield init fail");
            Serial.println("Please Init CAN BUS Shield again");

            delay(100);
            if (count <= 1)
                Serial.println("Please give up trying!, trying is useless!");
        }

    }while(count--);
}

unsigned char data[8] = {'D', 'F', 'R', 'O', 'B', 'O', 'T', '!'};

void loop()
{
    for(int id=0; id<10; id++)
    {
        memset(data, id, sizeof(data));                 // set id to send data buff, id is arranged form 0x00 to 0x09.
        CAN.sendMsgBuf(id, 0, sizeof(data), data);
        delay(100);
    }
}

