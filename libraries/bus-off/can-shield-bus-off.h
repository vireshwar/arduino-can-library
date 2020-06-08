
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin


char strBuf[80];


void canShieldSetup()
{
    while (CAN_OK != CAN.begin(CAN_500KBPS)){              // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
	
	CAN.mcp2515_modifyRegister(MCP_TXB0CTRL,3,3);
	CAN.mcp2515_modifyRegister(MCP_TXB1CTRL,3,2);
	CAN.mcp2515_modifyRegister(MCP_TXB2CTRL,3,1);
    delay(100);                   
}

inline byte readTEC(){
	return CAN.mcp2515_readRegister(MCP_TEC);
}

inline void printTEC(){
  byte tec = readTEC();
  sprintf(strBuf, "TEC:%d", tec);
  Serial.print(strBuf);
}

inline byte readREC(){
	return CAN.mcp2515_readRegister(MCP_REC);
}

inline void printREC(){
  byte rec = readREC();
  sprintf(strBuf, "REC:%d", rec);
  Serial.print(strBuf);
}

inline byte readErrorState(){
  return CAN.mcp2515_readRegister(MCP_EFLG);
}

inline void printErrorState(){
  byte errorState = readErrorState();
  sprintf(strBuf, "Error-State:%d", errorState);
  Serial.print(strBuf);
}

inline void enableOneShot(){
	CAN.mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
}

inline void disableOneShot(){
	CAN.mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
}

inline byte sendInAnyBuf(unsigned long id, int txMsgLen, unsigned char * txMsgPayload){
	return CAN.sendMsgBuf(id, 0, txMsgLen, txMsgPayload, false);
}

inline byte sendInAnyBufExt(unsigned long id, int txMsgLen, unsigned char * txMsgPayload){
	return CAN.sendMsgBuf(id, 1, txMsgLen, txMsgPayload, false);
}

inline byte sendInAnyBufRemote(unsigned long id, int txMsgLen, unsigned char * txMsgPayload){
	return CAN.sendMsgBuf(id, 0, 1, txMsgLen, txMsgPayload, false);
}

inline byte sendInParticularBuf(unsigned long id, int txMsgLen, unsigned char * txMsgPayload, byte bufNum){
	return CAN.trySendMsgBuf(id, 0, 0, txMsgLen, txMsgPayload, bufNum);
}

inline byte setMsgInParticularBuf(unsigned long id, byte txMsgLen, unsigned char * txMsgPayload, byte bufNum) {
    byte txbuf_n;

    if (bufNum < MCP_N_TXBUFFERS) { // Use specified buffer
        if (CAN.mcp2515_isTXBufFree(&txbuf_n, bufNum) != MCP2515_OK) {
            return CAN_FAILTX;
        }
    } else {
        if (CAN.mcp2515_getNextFreeTXBuf(&txbuf_n) != MCP2515_OK) {
            return CAN_FAILTX;
        }
    }

    CAN.mcp2515_buffer_canMsg(txbuf_n, id, 0, 0, txMsgLen, txMsgPayload);

    return CAN_OK;
}


inline void emptyBuffers(){
	unsigned char rxMsgLen = 0;
	unsigned char rxMsgPayload[8];
	while (CAN_MSGAVAIL == CAN.checkReceive()){
		CAN.readMsgBuf(&rxMsgLen, rxMsgPayload);
	}
}

inline long readMsgID(){
	unsigned char rxMsgLen = 0;
	unsigned char rxMsgPayload[8];
	CAN.readMsgBuf(&rxMsgLen, rxMsgPayload);
	return CAN.getCanId();
}

inline void waitNewMsg(long msgID){
  bool received = false;
  while (!received){
    while (CAN_MSGAVAIL == CAN.checkReceive()){
      if (readMsgID()==msgID) {
        received=true;
        break;
      }
    }
  }
}


inline void filterRxMsg(int rxMsgIDs[], int rxMsgIDsLen){
	CAN.mcp2515_setCANCTRL_Mode(MODE_CONFIG);
	CAN.mcp2515_modifyRegister(MCP_RXB0CTRL,
							 MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
							 MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
	CAN.mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
							 MCP_RXB_RX_STDEXT);
	CAN.mcp2515_write_id(MCP_RXM0SIDH, 0, 0x3ff);
	CAN.mcp2515_write_id(MCP_RXM1SIDH, 0, 0x3ff);
	for (int i=0; i<6; i++){
	int idx = i%rxMsgIDsLen;
	int registerAddr = 0;
	switch(i){
	  case 0: registerAddr=MCP_RXF0SIDH; break;
	  case 1: registerAddr=MCP_RXF1SIDH; break;
	  case 2: registerAddr=MCP_RXF2SIDH; break;
	  case 3: registerAddr=MCP_RXF3SIDH; break;
	  case 4: registerAddr=MCP_RXF4SIDH; break;
	  case 5: registerAddr=MCP_RXF5SIDH; break;
	}
	CAN.mcp2515_write_id(registerAddr, 0, rxMsgIDs[idx]);
	}
	CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);
}

inline void resetCANController(){
  CAN.mcp2515_fast_reset();
  CAN.mcp2515_configRate(CAN_500KBPS, MCP_16MHz);
  CAN.mcp2515_modifyRegister(MCP_TXB0CTRL,3,3);
  CAN.mcp2515_modifyRegister(MCP_TXB1CTRL,3,2);
  CAN.mcp2515_modifyRegister(MCP_TXB2CTRL,3,1);
  CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);
}

inline void resetErrorCounter(){
  CAN.mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  CAN.mcp2515_configRate(CAN_500KBPS, MCP_16MHz);	
  CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);
}

inline bool noTXPending(int bufsToCheck[], int numBuf){
  bool noTx = true;
  for (int bufIdx=0; bufIdx<numBuf; bufIdx++){
    byte txreq = CAN.mcp2515_readRegister(txCtrlReg(bufsToCheck[bufIdx])) & 0x08;
    if (txreq){
      noTx = false; break;
    }
  }
  return noTx;
}