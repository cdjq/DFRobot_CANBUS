/*!
 * @file DFRobot_CANBUS.cpp
 * @brief Define the basic structure of class DFRobot_CANBUS 
 * @details By a simple mechanical structure with the sensor, that can be read to the mass of the body
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2022-05-05
 * @https://github.com/DFRobot/DFRobot_CANBUS
 */
#include "DFRobot_CANBUS.h"

#define ReadWriteOneByte SPI.transfer
#define Read() SPI.transfer(0x00)




void DFRobot_CANBUS::mcpReset(void)
{
    MCPSPI_SELECT();
    ReadWriteOneByte(MCP_RESET);
    MCPSPI_UNSELECT();
    delay(10);
}


uint8_t DFRobot_CANBUS::mcpReadRegister(const uint8_t RegAddr)                                                                     
{
    uint8_t ret;

    MCPSPI_SELECT();
    ReadWriteOneByte(MCP_READ);
    ReadWriteOneByte(RegAddr);
    ret = Read();
    MCPSPI_UNSELECT();

    return ret;
}

void DFRobot_CANBUS::mcpReadMulitiRegisters(const uint8_t RegAddr, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	if ( len > CAN_MAX_MESSAGE_LENGTH)
	{
		len = CAN_MAX_MESSAGE_LENGTH;
	}
	
	MCPSPI_SELECT();
	ReadWriteOneByte(MCP_READ);
	ReadWriteOneByte(RegAddr);

	for (i=0; i < len; i++) {
		buf[i] = Read();
	}
	MCPSPI_UNSELECT();
}

 
void DFRobot_CANBUS::mcpSetRegister(const uint8_t RegAddr, const uint8_t value)
{
    MCPSPI_SELECT();
    ReadWriteOneByte(MCP_WRITE);
    ReadWriteOneByte(RegAddr);
    ReadWriteOneByte(value);
    MCPSPI_UNSELECT();
}


void DFRobot_CANBUS::mcpSetMulitRegisterS(const uint8_t RegAddr, const uint8_t *buf, const uint8_t len)
{
    uint8_t i;
	
    MCPSPI_SELECT();
    ReadWriteOneByte(MCP_WRITE);
    ReadWriteOneByte(RegAddr);
       
    for (i=0; i < len; i++) 
    {
        ReadWriteOneByte(buf[i]);
    }
    MCPSPI_UNSELECT();
}



void DFRobot_CANBUS::mcpModifyRegister(const uint8_t RegAddr, const uint8_t mask, const uint8_t data)
{
    MCPSPI_SELECT();
    ReadWriteOneByte(MCP_BITMOD);
    ReadWriteOneByte(RegAddr);
    ReadWriteOneByte(mask);
    ReadWriteOneByte(data);
    MCPSPI_UNSELECT();
}

uint8_t DFRobot_CANBUS::mcpReadStatus(void)                             
{
	uint8_t ret;
	
	MCPSPI_SELECT();
	ReadWriteOneByte(MCP_RX_STATUS);
	ret = Read();
	MCPSPI_UNSELECT();
	
	return ret;
}


uint8_t DFRobot_CANBUS::mcpSetMode(const uint8_t newMode)
{
    uint8_t ret;

    mcpModifyRegister(MCPCANCTRL, MODE_MASK, newMode);
    ret = mcpReadRegister(MCPCANCTRL);
    ret &= MODE_MASK;

    if ( ret == newMode ) 
    {
        return MCP_OK;
    }

    return MCP_FAIL;

}

uint8_t DFRobot_CANBUS::mcpConfigRate(const uint8_t canSpeed)            
{
    uint8_t preSet, cfg1, cfg2, cfg3;
	
    preSet = 0;
    switch (canSpeed) 
    {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (CAN_33KBPS):
        cfg1 = MCP_16MHz_33kBPS_CFG1;
        cfg2 = MCP_16MHz_33kBPS_CFG2;
        cfg3 = MCP_16MHz_33kBPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_83K3BPS):
        cfg1 = MCP_16MHz_83k3BPS_CFG1;
        cfg2 = MCP_16MHz_83k3BPS_CFG2;
        cfg3 = MCP_16MHz_83k3BPS_CFG3;
        break;  

        case (CAN_95KBPS):
        cfg1 = MCP_16MHz_95kBPS_CFG1;
        cfg2 = MCP_16MHz_95kBPS_CFG2;
        cfg3 = MCP_16MHz_95kBPS_CFG3;
        break;

        case (CAN_100KBPS):                                           
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (CAN_1000KBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        preSet = 1;
        break;
    }

    if (!preSet) {
        mcpSetRegister(MCP_CNF1, cfg1);
        mcpSetRegister(MCP_CNF2, cfg2);
        mcpSetRegister(MCP_CNF3, cfg3);
        return MCP_OK;
    }
    else {
        return MCP_FAIL;
    }
}


void DFRobot_CANBUS::mcpInitBuffers(void)
{
    uint8_t i, a1, a2, a3;
 
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                        
        mcpSetRegister(a1, 0);
        mcpSetRegister(a2, 0);
        mcpSetRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcpSetRegister(MCP_RXB0CTRL, 0);
    mcpSetRegister(MCP_RXB1CTRL, 0);
}

uint8_t DFRobot_CANBUS::mcpInit(const uint8_t canSpeed)                    
{

  uint8_t res;

   
    res = mcpSetMode(MODE_CONFIG);
    if(res > 0)
    {
      delay(10);
      return res;
    }
	delay(10);


                                                                       
    if(mcpConfigRate(canSpeed))
    {
	  delay(10);
      return res;
    }
	delay(10);


    if ( res == MCP_OK ) {

                                                                       
        mcpInitBuffers();

                                                                       
        mcpSetRegister(MCPCANINTE, MCP_RX0IF | MCP_RX1IF);

                                                                 
        mcpModifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
        mcpModifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_STDEXT);
                                                                       
        res = mcpSetMode(MODE_NORMAL);                                                                
        if(res)
        {  
          delay(10);
          return res;
        }

        delay(10);

    }
    return res;

}


void DFRobot_CANBUS::mcpWriteid( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcpSetMulitRegisterS( mcp_addr, tbufdata, 4 );
}

void DFRobot_CANBUS::mcpReadid( const uint8_t mcp_addr, uint8_t* ext, uint32_t* id )
{
    uint8_t id_Buf[4];

    mcpReadMulitiRegisters( mcp_addr, (uint8_t *)id_Buf, 4 );

    *id = (id_Buf[MCP_SIDH]<< 3) + (id_Buf[MCP_SIDL]>>5);

    if ( (id_Buf[MCP_SIDL] & 1 << 3)) 
    {
                                                                       
        *id = (*id<<2) + (id_Buf[MCP_SIDL] & 0x03);
        *id = (*id<<8) + id_Buf[MCP_EID8];
        *id = (*id<<8) + id_Buf[MCP_EID0];
        *ext = 1;
    }
}

void DFRobot_CANBUS::mcpWritecanMsg( const uint8_t sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = sidh_addr;
    mcpSetMulitRegisterS(mcp_addr+5, canDta, canDlc );                 
    if ( canRtr == 1)                                                   
    {
        canDlc |= MCP_RTR_MASK;  
    }
    mcpSetRegister((mcp_addr+4), canDlc );                        
    mcpWriteid(mcp_addr, canExtFlg, canID );                     
}





void DFRobot_CANBUS::mcpReadcanMsg( const uint8_t idh_addr)        
{

    mcpReadid( idh_addr, &canExtFlg,&canID );   

    canRtr = mcpReadRegister( idh_addr - 1 );
    canDlc = mcpReadRegister( idh_addr + 4 );

    if ((canRtr & (1 << 3))) {
        canRtr = 1;
    }
    else {
        canRtr = 0;
    }

    canDlc &= MCP_DLC_MASK;
    mcpReadMulitiRegisters( idh_addr+5, (uint8_t *)canDta, canDlc);
}

void DFRobot_CANBUS::mcpStarttransmit(const uint8_t mcp_addr)              
{
    mcpModifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}


uint8_t DFRobot_CANBUS::mcpGetNextFreeTXBuf(uint8_t *txbuf_n)                 
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                       
    for (i=0; i < MCP_N_TXBUFFERS; i++) {
        ctrlval = mcpReadRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   
                                                                        
            res = MCP_OK;
            return res;                                                 
        }
    }
    return res;
}


DFRobot_CANBUS::DFRobot_CANBUS(uint8_t _CS)
{
   
    SPI_CS = _CS;
    pinMode(SPI_CS, OUTPUT);
	
    MCPSPI_UNSELECT();

}


 

void DFRobot_CANBUS::init(void)
{
	SPI.begin();
    mcpReset();

}

uint8_t DFRobot_CANBUS::begin(uint8_t speedset)
{
    uint8_t res;
	
    res = mcpInit(speedset);
    if (res == MCP_OK) return CAN_OK;
    else return CAN_FAILINIT;
}




uint8_t DFRobot_CANBUS::initMask(eMasker_t Masker_num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP_OK;  
	
	delay(10);
    res = mcpSetMode(MODE_CONFIG);
    if(res > 0){  
	delay(10);
    return res;
    }
    
    if (Masker_num == MCP_RXM0){
        mcpWriteid(MCP_RXM0SIDH, ext, ulData);

    }
    else if(Masker_num == MCP_RXM1){
        mcpWriteid(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP_FAIL;
    
    res = mcpSetMode(MODE_NORMAL); 
    if(res > 0){

	delay(10);
    return res;
  }  
    delay(10);
    return res;
}




uint8_t DFRobot_CANBUS::initFilter(eFilter_t filterNum, uint8_t ext, uint32_t data)
{
    uint8_t res = MCP_OK;  
	delay(10);
	
    res = mcpSetMode(MODE_CONFIG);
	
    if(res > 0)
    {  
      delay(10);
      return res;
    }
    
    switch( filterNum )
    {
        case MCP_RXF0:
	
        mcpWriteid(MCP_RXF0SIDH, ext, data);
        break;

        case MCP_RXF1:
        mcpWriteid(MCP_RXF1SIDH, ext, data);
        break;

        case MCP_RXF2:
        mcpWriteid(MCP_RXF2SIDH, ext, data);
        break;

        case MCP_RXF3:
        mcpWriteid(MCP_RXF3SIDH, ext, data);
        break;

        case MCP_RXF4:
        mcpWriteid(MCP_RXF4SIDH, ext, data);
        break;

        case MCP_RXF5:
        mcpWriteid(MCP_RXF5SIDH, ext, data);
        break;

        default:
        res = MCP_FAIL;
    }
    
    res = mcpSetMode(MODE_NORMAL);
    if(res > 0)
    {  
      delay(10);
      return res;
    }  
	
	delay(10);

    return res;
}





uint8_t DFRobot_CANBUS::setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t rtr, uint8_t *pData)
{
    int i = 0;
    canExtFlg = ext;
    canID     = id;
    canDlc    = len;
    canRtr    = rtr;
    for(i = 0; i < MAX_MESSAGE_LENGTH; i++)
    {
        canDta[i] = pData[i];
    }
    return MCP_OK;
}


uint8_t DFRobot_CANBUS::setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData)
{
    int i = 0;
    canExtFlg = ext;
    canID     = id;
    canDlc    = len;
    for(i = 0; i < MAX_MESSAGE_LENGTH; i++)
    {
        canDta[i] = *(pData+i);
    }
    return MCP_OK;
}

uint8_t DFRobot_CANBUS::clearMsg()
{

    canID       = 0;
    canRtr      = 0;
    canExtFlg   = 0;
    canfilhit   = 0;
    canDlc      = 0;
    for(int i = 0; i<canDlc; i++ )
      canDta[i] = 0x00;

    return MCP_OK;
}


uint8_t DFRobot_CANBUS::sendMsg()
{
    uint8_t res, txindex;
    uint16_t uiTimeOut = 0;

    do {
        res = mcpGetNextFreeTXBuf(&txindex);                       
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                     
    }
    uiTimeOut = 0;
    mcpWritecanMsg( txindex);
    mcpStarttransmit( txindex );
    do
    {
        uiTimeOut++;        
        res = mcpReadRegister(txindex);  			                
        res = res & 0x08;                               		
    }while(res && (uiTimeOut < TIMEOUTVALUE));   
    if(uiTimeOut == TIMEOUTVALUE)                                       
    {
        return CAN_SENDMSGTIMEOUT;
    }
    return CAN_OK;

}


uint8_t DFRobot_CANBUS::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf)
{
    setMsg(id, ext, len, rtr, buf);
    return sendMsg();
}





uint8_t DFRobot_CANBUS::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    setMsg(id, ext, len, buf);
    return sendMsg();
}


uint8_t DFRobot_CANBUS::readMsg()
{
    uint8_t status, res;
   
	
    status = mcpReadStatus();
    
    if ( status & MCP_MESGE_RXB0_MSK )                        
    {
        mcpReadcanMsg( MCP_RXBUF_0);             //0x61
        mcpModifyRegister(MCPCANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( status & MCP_MESGE_RXB1_MSK )               //message in rx buf1                   
    {
        mcpReadcanMsg( MCP_RXBUF_1);
        mcpModifyRegister(MCPCANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else if (status & MCP_MESGE_RXB01_MSK)
    {
        res = CAN_OK;
    }
	else
	{
		res = CAN_NOMSG;
	}
		
    return res;
}


uint8_t DFRobot_CANBUS::readMsgBuf(uint8_t *len, uint8_t *buf)
{
    uint8_t  res, i;
    
    res= readMsg();
    
    if (res == CAN_OK) {
       *len = canDlc;
       for( i = 0; i<canDlc; i++) {
         buf[i] = canDta[i];
       } 
    } else {
       	 *len = 0;
    }
    return res;
}

uint8_t DFRobot_CANBUS::readMsgBufID(uint32_t *ID, uint8_t *len, uint8_t *buf)
{
    uint8_t ret;
	
    ret = readMsg();
    if (ret == CAN_OK) {
       *len = canDlc;
       *ID  = canID;
       for(int i = 0; i<canDlc && i < MAX_MESSAGE_LENGTH; i++) {
          buf[i] = canDta[i];
       }
    } else {
       *len = 0;
    }
    return ret;
}


uint8_t DFRobot_CANBUS::checkReceive(void)
{
    uint8_t res;
    res = mcpReadStatus();                                         
    if ( res & MCP_MESGE_NO_MSK ) 
    {
        return CAN_MSGAVAIL;
    }
    else 
    {
        return CAN_NOMSG;
    }
}



uint8_t DFRobot_CANBUS::checkError(void)
{
    uint8_t flag = mcpReadRegister(MCP_EFLG);

    if ( flag & MCP_EFLG_ERRORMASK ) 
    {
        return CAN_CTRLERROR;
    }
    else 
    {
        return CAN_OK;
    }
}


uint32_t DFRobot_CANBUS::getCanId(void)
{
    return canID;
} 


uint8_t DFRobot_CANBUS::isRemoteRequest(void)
{
    return canRtr;
} 

uint8_t DFRobot_CANBUS::isExtendedFrame(void)
{
    return canExtFlg;
} 

