/*!
 * @file DFRobot_CANBUS.h
 * @brief Define the basic structure of class DFRobot_CANBUS 
 * @details By a simple mechanical structure with the sensor, that can be read to the mass of the body
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2022-05-05
 * @https://github.com/DFRobot/DFRobot_CANBUS
 */

#ifndef _DFROBOT_CANBUS_
#define _DFROBOT_CANBUS_
#include <Arduino.h>
#include <SPI.h>


#define TIMEOUTVALUE    50
#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3

#define MCP_TXB_EXIDE_M     0x08                                       
#define MCP_DLC_MASK        0x0F                                      
#define MCP_RTR_MASK        0x40                                        

#define MCP_RXB_RX_ANY      0x60
#define MCP_RXB_RX_EXT      0x40
#define MCP_RXB_RX_STD      0x20
#define MCP_RXB_RX_STDEXT   0x00
#define MCP_RXB_RX_MASK     0x60
#define MCP_RXB_BUKT_MASK   (1<<2)


#define MCP_TXB_TXBUFE_M    0x80
#define MCP_TXB_ABTF_M      0x40
#define MCP_TXB_MLOA_M      0x20
#define MCP_TXB_TXERR_M     0x10
#define MCP_TXB_TXREQ_M     0x08
#define MCP_TXB_TXIE_M      0x04
#define MCP_TXB_TXP10_M     0x03

#define MCP_TXB_RTR_M       0x40                                        
#define MCP_RXB_IDE_M       0x08                                       
#define MCP_RXB_RTR_M       0x40                                       

#define MCP_STAT_RXIF_MASK   (0x03)
#define MCP_STAT_RX0IF (1<<0)
#define MCP_STAT_RX1IF (1<<1)

#define MCP_EFLG_RX1OVR (1<<7)
#define MCP_EFLG_RX0OVR (1<<6)
#define MCP_EFLG_TXBO   (1<<5)
#define MCP_EFLG_TXEP   (1<<4)
#define MCP_EFLG_RXEP   (1<<3)
#define MCP_EFLG_TXWAR  (1<<2)
#define MCP_EFLG_RXWAR  (1<<1)
#define MCP_EFLG_EWARN  (1<<0)
#define MCP_EFLG_ERRORMASK  (0xF8)                                      



#define MCP_RXF0SIDH    0x00
#define MCP_RXF0SIDL    0x01
#define MCP_RXF0EID8    0x02
#define MCP_RXF0EID0    0x03
#define MCP_RXF1SIDH    0x04
#define MCP_RXF1SIDL    0x05
#define MCP_RXF1EID8    0x06
#define MCP_RXF1EID0    0x07
#define MCP_RXF2SIDH    0x08
#define MCP_RXF2SIDL    0x09
#define MCP_RXF2EID8    0x0A
#define MCP_RXF2EID0    0x0B
#define MCPCANSTAT     0x0E
#define MCPCANCTRL     0x0F
#define MCP_RXF3SIDH    0x10
#define MCP_RXF3SIDL    0x11
#define MCP_RXF3EID8    0x12
#define MCP_RXF3EID0    0x13
#define MCP_RXF4SIDH    0x14
#define MCP_RXF4SIDL    0x15
#define MCP_RXF4EID8    0x16
#define MCP_RXF4EID0    0x17
#define MCP_RXF5SIDH    0x18
#define MCP_RXF5SIDL    0x19
#define MCP_RXF5EID8    0x1A
#define MCP_RXF5EID0    0x1B
#define MCP_TEC         0x1C
#define MCP_REC         0x1D
#define MCP_RXM0SIDH    0x20
#define MCP_RXM0SIDL    0x21
#define MCP_RXM0EID8    0x22
#define MCP_RXM0EID0    0x23
#define MCP_RXM1SIDH    0x24
#define MCP_RXM1SIDL    0x25
#define MCP_RXM1EID8    0x26
#define MCP_RXM1EID0    0x27
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCPCANINTE     0x2B
#define MCPCANINTF     0x2C
#define MCP_EFLG        0x2D
#define MCP_TXB0CTRL    0x30
#define MCP_TXB1CTRL    0x40
#define MCP_TXB2CTRL    0x50
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71


#define MCP_TX_INT          0x1C                                  
#define MCP_TX01_INT        0x0C                                   
#define MCP_RX_INT          0x03                                    
#define MCP_NO_INT          0x00                                   

#define MCP_TX01_MASK       0x14
#define MCP_TX_MASK         0x54


#define MCP_WRITE           0x02

#define MCP_READ            0x03

#define MCP_BITMOD          0x05

#define MCP_LOAD_TX0        0x40
#define MCP_LOAD_TX1        0x42
#define MCP_LOAD_TX2        0x44

#define MCP_RTS_TX0         0x81
#define MCP_RTS_TX1         0x82
#define MCP_RTS_TX2         0x84
#define MCP_RTS_ALL         0x87

#define MCP_READ_RX0        0x90
#define MCP_READ_RX1        0x94

#define MCP_READ_STATUS     0xA0

#define MCP_RX_STATUS       0xB0

//Rx STATUS INSTRUCTION
//bit 7:6
#define MCP_MESGE_NO_MSK    (3 << 6)        
#define MCP_MESGE_RXB0_MSK  (1 << 6)        
#define MCP_MESGE_RXB1_MSK  (2 << 6)         
#define MCP_MESGE_RXB01_MSK (3 << 6)         
//bit 4:3
#define MCP_FRAME_STDDF_MASK    (0 << 3)        
#define MCP_FRAME_REMOTERF_MASK (1 << 3)     
#define MCP_FRAME_EXTDF_MASK    (2 << 3)        
#define MCP_FRAME_EXTRF_MASK    (3 << 3)       
//bit 2:0
#define MCP_FILMATCH_STD_MSK   (0x7FF < 0)       
#define MCP_FILMATCH_EXT_MSK   (0x7FFFF < 0)        




#define MCP_RESET           0xC0

#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP    0xE0
#define MODE_MASK       0xE0
#define ABORT_TX        0x10
#define MODE_ONESHOT    0x08
#define CLKOUT_ENABLE   0x04
#define CLKOUT_DISABLE  0x00
#define CLKOUT_PS1      0x00
#define CLKOUT_PS2      0x01
#define CLKOUT_PS4      0x02
#define CLKOUT_PS8      0x03



#define SJW1            0x00
#define SJW2            0x40
#define SJW3            0x80
#define SJW4            0xC0



#define BTLMODE         0x80
#define SAMPLE_1X       0x00
#define SAMPLE_3X       0x40



#define SOF_ENABLE      0x80
#define SOF_DISABLE     0x00
#define WAKFIL_ENABLE   0x40
#define WAKFIL_DISABLE  0x00



#define MCP_RX0IF       0x01
#define MCP_RX1IF       0x02
#define MCP_TX0IF       0x04
#define MCP_TX1IF       0x08
#define MCP_TX2IF       0x10
#define MCP_ERRIF       0x20
#define MCP_WAKIF       0x40
#define MCP_MERRF       0x80

#define MCP_16MHz_1000kBPS_CFG1 (0x00)
#define MCP_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP_16MHz_1000kBPS_CFG3 (0x82)

#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xF0)
#define MCP_16MHz_500kBPS_CFG3 (0x86)

#define MCP_16MHz_250kBPS_CFG1 (0x41)
#define MCP_16MHz_250kBPS_CFG2 (0xF1)
#define MCP_16MHz_250kBPS_CFG3 (0x85)

#define MCP_16MHz_200kBPS_CFG1 (0x01)
#define MCP_16MHz_200kBPS_CFG2 (0xFA)
#define MCP_16MHz_200kBPS_CFG3 (0x87)

#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xFA)
#define MCP_16MHz_100kBPS_CFG3 (0x87)


#define MCP_16MHz_95kBPS_CFG1 (0x03)
#define MCP_16MHz_95kBPS_CFG2 (0xAD)
#define MCP_16MHz_95kBPS_CFG3 (0x07)

#define MCP_16MHz_83k3BPS_CFG1 (0x03)
#define MCP_16MHz_83k3BPS_CFG2 (0xBE)
#define MCP_16MHz_83k3BPS_CFG3 (0x07)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xFF)
#define MCP_16MHz_80kBPS_CFG3 (0x87)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xFA)
#define MCP_16MHz_50kBPS_CFG3 (0x87)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xFF)
#define MCP_16MHz_40kBPS_CFG3 (0x87)

#define MCP_16MHz_33kBPS_CFG1 (0x09)
#define MCP_16MHz_33kBPS_CFG2 (0xBE)
#define MCP_16MHz_33kBPS_CFG3 (0x07)

#define MCP_16MHz_31k25BPS_CFG1 (0x0F)
#define MCP_16MHz_31k25BPS_CFG2 (0xF1)
#define MCP_16MHz_31k25BPS_CFG3 (0x85)

#define MCP_16MHz_20kBPS_CFG1 (0x0F)
#define MCP_16MHz_20kBPS_CFG2 (0xFF)
#define MCP_16MHz_20kBPS_CFG3 (0x87)

#define MCP_16MHz_10kBPS_CFG1 (0x1F)
#define MCP_16MHz_10kBPS_CFG2 (0xFF)
#define MCP_16MHz_10kBPS_CFG3 (0x87)

#define MCP_16MHz_5kBPS_CFG1 (0x3F)
#define MCP_16MHz_5kBPS_CFG2 (0xFF)
#define MCP_16MHz_5kBPS_CFG3 (0x87)



#define MCPDEBUG        (0)
#define MCPDEBUG_TXBUF  (0)
#define MCP_N_TXBUFFERS (3)

#define MCP_RXBUF_0 (MCP_RXB0SIDH)
#define MCP_RXBUF_1 (MCP_RXB1SIDH)

#define MCPSPI_SELECT()   digitalWrite(SPI_CS, LOW)
#define MCPSPI_UNSELECT() digitalWrite(SPI_CS, HIGH)

#define MCP_OK         (0)
#define MCP_FAIL       (1)
#define MCP_ALLTXBUSY      (2)

#define CANDEBUG   1

#define CANUSELOOP 0

#define CANSENDTIMEOUT (200)                                           

#define CANAUTOPROCESS (1)
#define CANAUTOON  (1)
#define CANAUTOOFF (0)

#define CAN_STDID (0)
#define CAN_EXTID (1)

#define CANDEFAULTIDENT    (0x55CC)
#define CANDEFAULTIDENTEXT (CAN_EXTID)

#define CAN_5KBPS    1
#define CAN_10KBPS   2
#define CAN_20KBPS   3
#define CAN_31K25BPS 4
#define CAN_33KBPS   5
#define CAN_40KBPS   6
#define CAN_50KBPS   7
#define CAN_80KBPS   8
#define CAN_83K3BPS  9
#define CAN_95KBPS   10
#define CAN_100KBPS  11
#define CAN_125KBPS  12
#define CAN_200KBPS  13
#define CAN_250KBPS  14
#define CAN_500KBPS  15
#define CAN_1000KBPS 16

#define CAN_OK                  (0)
#define CAN_FAILINIT            (1)
#define CAN_FAILTX              (2)
#define CAN_MSGAVAIL            (3)
#define CAN_NOMSG               (4)
#define CAN_CTRLERROR           (5)
#define CAN_GETTXBFTIMEOUT      (6)
#define CAN_SENDMSGTIMEOUT      (7)
#define CAN_FAIL                (0xff)

#define CAN_MAX_MESSAGE_LENGTH (8)







#define MAX_MESSAGE_LENGTH 8

class DFRobot_CANBUS
{
public:
  /*!
   * @enum eFilter_t
   * @brief 滤波器 
   */
  typedef enum {
  
     MCP_RXF0 = 0,    
     MCP_RXF1,
     MCP_RXF2,
     MCP_RXF3,
     MCP_RXF4,
     MCP_RXF5,
  
  
  }eFilter_t;

  /*!
   * @enum eMasker_t
   * @brief 屏蔽寄存器选择 
   */
  typedef enum{

     MCP_RXM0 = 0,  
     MCP_RXM1,   
  }eMasker_t;

public:
  /*!
   * @fn DFRobot_CANBUS
   * @brief Constructor, specify the CAN-BUS Shield V1.0 SPI chip select pin.
   * @param _CS  Chip select
   */
  DFRobot_CANBUS(uint8_t _CS);
  
  /*!
   * @fn init
   * @brief Initialize SPI module; reset MCP2515. Parameter
   * @return None
   */
  void  init(void);

  /*!
   * @fn begin
   * @brief Initialize setting CAN-BUS buadrate, follow the init() function.
   * @param speedset  Baudrate: CAN_5KBPS, CAN_10KBPS, CAN_20KBPS, CAN_31K25BPS, 
   * @n CAN_33KBPS, CAN_40KBPS, CAN_50KBPS, CAN_80KBPS, CAN_83K3BPS, CAN_95KBPS, 
   * @n CAN_100KBPS, CAN_125KBPS, CAN_200KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS.
   * @return If the initialization is successful, return "CAN_OK"; if initialization fails, return "CAN_FAILINIT".
   */
  uint8_t begin(uint8_t speedset);    
  
  /*!
   * @fn sendMsgBuf
   * @brief Send a set of data frame.
   * @param id Data frame ID
   * @param ext "ext = 0", it is a standard frame; "ext = 1", it is an extended frame.
   * @param len data length, len < 8
   * @param buf  Data buffer point
   * @return If success, returns "CAN_OK"; if timeout, returns "CAN_SENDMSGTIMEOUT"; If you fail to get the next free buffer, it returns "CAN_GETTXBFTIMEOUT".
   */
  uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);   

  /*!
   * @fn isRemoteRequest
   * @brief Check whether it is a remote frame
   * @return "1", Yes; "0", No
   */
  uint8_t isRemoteRequest(void);
  
  /*!
   * @fn initMask
   * @brief Initialize the mask register
   * @param Masker_num mask register name: MCP_RXM0、MCP_RXM1; If Masker_num = MCP_RXM0,
   * @n initialize the mask register 0 (mask register 0 receives data from buffer0); if Masker_num = MCP_RXM1,
   * @n initialize the mask register 1 (mask register 1 receives data from buffer1).
   * @param ext configure standard frame with mask register setting; "ext = 1", configure extended frame with mask register setting.
   * @param Data Write this data into mask register, to configure which register will be blocked.
   * @return if success, returns "MCP_OK"; if fail, returns "MCP_FAIL"
   */
  uint8_t initMask(eMasker_t Masker_num, uint8_t ext, uint32_t Data);
  
  /*!
   * @fn checkReceive
   * @brief check the validity of received data frame.
   * @return If the shield receives the valid data frame, return "CAN_MSGAVAIL"; if no, return "CAN_NOMSG";
   */
  uint8_t checkReceive(void);

  /*!
   * @fn initFilter
   * @brief  Initialize the message acceptance filter register.
   * @param filterNum Message acceptance filter number, could be MCP_RXF0, MCP_RXF1, MCP_RXF2, MCP_RXF3, MCP_RXF4, MCP_RXF5.
   * @param ext if "ext=0", it means the message acceptance filter receive standard data frame message only; 
   * @n     if "ext=1", it means the message acceptance filter receive extended data frame message only.
   * @param data Filtered message ID. Only the data frame with filtered id can be received by CAN controller. 
   * @n     So an upcoming data frame whether can be received depends the value in MCP_RXM0/MCP_RXM1 in 
   * @n     init_Mask() function, the value in MCP_RXF0 registers in init_Filter() function and upcoming message identifier
   * @n     ID. These three values can be looked up on the Table below. If every true result is received, then the message
   * @n     will be received by the CAN controller. Otherwise, it will be discarded.
   * @return "CAN_OK" means reading successfully; Contrarily, return "MCP_FA".
   */
  uint8_t initFilter(eFilter_t filterNum, uint8_t ext, uint32_t data);
  
  /*!
   * @fn sendMsgBuf
   * @brief Send remote sending request message..
   * @param id   Data frame ID
   * @param ext "ext = 0", it is a standard frame; "ext = 1", it is an extended frame.
   * @param rtr data length, len < 8
   * @param len "rtr = 1", it is a Remote sending request frame; "rtr = 0", it is a data frame.
   * @param buf   Data buffer point
   * @return If success, returns "CAN_OK"; if timeout, returns "CAN_SENDMSGTIMEOUT"; If you fail to get the next free buffer, it returns "CAN_GETTXBFTIMEOUT".
   */
  uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf);  

  /*!
   * @fn readMsgBuf
   * @brief Read data from MCP2515 receiving buffer.
   * @param len   Save the receiving data length
   * @param buf  Save the receiving data
   * @return  "CAN_OK" means reading successfully; Contrarily, return "CAN_NOMSG".
   */
  uint8_t readMsgBuf(uint8_t *len, uint8_t *buf);
  
  /*!
   * @fn readMsgBufID
   * @brief read data from MCP2515 receiving buffer and read this data frame ID.
   * @param ID  Save the data frame ID
   * @param len   Save the receiving data length
   * @param buf  Save the receiving data
   * @return  "CAN_OK" means reading successfully; Contrarily, return "CAN_NOMSG".
   */
  uint8_t readMsgBufID(uint32_t *ID, uint8_t *len, uint8_t *buf);
  
  /*!
   * @fn checkError
   * @brief MCP2515 control error inquiry
   * @return "CAN_CTRLERROR" means it sends the control error; Contrarily, return "CAN_OK".
   */
  uint8_t checkError(void);

  /*!
   * @fn getCanId
   * @brief  Get the current data frame's ID
   * @return Data frame ID
   */
  uint32_t getCanId(void);
  
  /*!
   * @fn isExtendedFrame
   * @brief Check whether it is a extended frame.
   * @return  "1", Yes; "0", No
   */
  uint8_t isExtendedFrame(void);

private:

  void mcpReset(void);                                           
  uint8_t mcpReadRegister(const uint8_t RegAddr);                   
  void mcpReadMulitiRegisters(const uint8_t RegAddr, 
                         uint8_t *buf, 
                              uint8_t len);
  void mcpSetRegister(const uint8_t RegAddr,                      
                           const uint8_t value);
  
  void mcpSetMulitRegisterS(const uint8_t RegAddr,                     
                            const uint8_t *buf,
                            const uint8_t len);
  
  void mcpInitBuffers(void);
  void mcpModifyRegister(const uint8_t RegAddr,                   
                              const uint8_t mask,
                              const uint8_t data);
  uint8_t mcpReadStatus(void);                                    
  uint8_t mcpSetMode(const uint8_t newMode);                
  uint8_t mcpConfigRate(const uint8_t canSpeed);                     
  uint8_t mcpInit(const uint8_t canSpeed);                           
  
  void mcpWriteid( const uint8_t mcp_addr,                       
                             const uint8_t ext,
                             const uint32_t id );
  
  void mcpReadid( const uint8_t mcp_addr,uint8_t* ext,uint32_t* id );
  
  void mcpWritecanMsg( const uint8_t sidh_addr );          
  void mcpReadcanMsg( const uint8_t sidh_addr);            
  void mcpStarttransmit(const uint8_t mcp_addr);                  
  uint8_t mcpGetNextFreeTXBuf(uint8_t *txbuf_n);                     
  uint8_t setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t rtr, uint8_t *pData);   
  uint8_t setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData);           
  uint8_t clearMsg();                                               
  uint8_t readMsg();                                               
  uint8_t sendMsg();                                               
private:
  
  uint8_t canExtFlg;                                                 
  uint32_t  canID;                                                      
  uint8_t   canDlc;                                                     
  uint8_t   canDta[MAX_MESSAGE_LENGTH];                            
  uint8_t   canRtr;                                                    
  uint8_t   canfilhit;
  uint8_t   SPI_CS;

};

#endif
