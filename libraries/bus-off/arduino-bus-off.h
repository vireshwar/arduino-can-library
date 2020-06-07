

#define TimerCounterMax 65536 // 2^16
#define Freq 16 //MHz
#define OverflowTime (TimerCounterMax/Freq) //us, 2^16/freq

#define BIT_CYC 32 // 16 cycles/us * 2 us
#define ERROR_CYC 192 // 16 cycles/us * 2 us * 6 bit
#define DELAY_SAMPLE 8
#define WAIT_EOF_BIT 22 
#define WAIT_BIT_SAMPLE 26 // 30 for 1 sample, but 26 works best with this implementation

#define EOFdetectTime 16 // 2 us * 8 bit

#define PinRead 5 // RX Pin  
#define PinDebug 6 // Tx Pin for debugging 
#define PinWrite 7 // Tx Pin for sending bits

#define NerrorBusoff 32

volatile unsigned int timerOverflow = 0;  

const unsigned int NsampleBit=14;





inline unsigned int computeMsgID(bool *bitFrameLocal){
  unsigned int msgIDlocal=0;
  unsigned char numZero=1;
  unsigned char numOne=0;
  unsigned char numIDbit=0;
  unsigned char numFrameBit=1; 
  while (numIDbit<11) {
    if(numZero<5 && numOne<5) {
      msgIDlocal = (msgIDlocal<<1) | bitFrameLocal[numFrameBit];
      numIDbit++;
    }
    
    if(!bitFrameLocal[numFrameBit]){
      numZero++;
      numOne=0;
    } else {
      numZero=0;
      numOne++;
    }
    numFrameBit++;
  }
  return msgIDlocal;
}


inline unsigned int sampleFrameBits(){
	bool bitFrame[NsampleBit]; 
	
    bitFrame[0] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE); 
    bitFrame[1] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);   
    bitFrame[2] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);    
    bitFrame[3] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);    
    bitFrame[4] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);    
    bitFrame[5] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);  
    bitFrame[6] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);
    bitFrame[7] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);   
    bitFrame[8] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);     
    bitFrame[9] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);      
    bitFrame[10] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);     
    bitFrame[11] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);    
    bitFrame[12] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);       
    bitFrame[13] = ((PIND & _BV(PinRead)) >> PinRead); __builtin_avr_delay_cycles(WAIT_BIT_SAMPLE);
	
	unsigned int msgIDlocal=computeMsgID(bitFrame);
	return msgIDlocal;
}

inline void causeError() {
  PORTD = B01111111; // Port-7 - Set to 0
  __builtin_avr_delay_cycles(ERROR_CYC);  // 
  PORTD = B11111111; // Port-7 - Set to 1  
}


inline void detectEndFrame() {
  int numOne = 0;
  bool bitVal = false;
  while (numOne < 7) {
    bitVal = ((PIND & _BV(PinRead)) >> PinRead);
    if (bitVal) numOne++;
    else numOne = 0;
    __builtin_avr_delay_cycles(WAIT_EOF_BIT);
  } 
}


inline void detectStartFrame() {
  while ((PIND & _BV(PinRead)) >> PinRead);
  __builtin_avr_delay_cycles(DELAY_SAMPLE); 
}

inline void delayCycle(unsigned int numCycle){
	__builtin_avr_delay_cycles(numCycle); 
}	

inline void startCleanTimer(){
  timerOverflow = 0; TCNT1 = 0; TCCR1B = 1;
}


inline void stopTimer(){
  TCCR1B = 0;
}

inline unsigned int getTimerCounter(){
 return TCNT1;
}

inline unsigned int getTimerOverflow(){
 return timerOverflow;
}

inline unsigned long getTotalTimeCounter(){
  return timerOverflow*TimerCounterMax+TCNT1;
}

inline unsigned long getElapsedTime(){ //in us
  unsigned long currTimerOverflow = timerOverflow; 
  unsigned int currTimer = TCNT1; 
  return (currTimerOverflow*OverflowTime + currTimer/Freq);
}

ISR(TIMER1_OVF_vect){  
  timerOverflow++;  
}

inline void printSample(bool *bitFrameLocal) {
 int i = 0;
 while(i<NsampleBit) {   // print the data     
	Serial.print(bitFrameLocal[i]);
	i++;   
 }
 Serial.println();
}

void printTime(uint64_t value)
{
  if ( value >= 10 ) printTime(value / 10);
  Serial.print((int)(value % 10));
}

inline unsigned int computeStuffedID(bool *bitFramelocal){
 unsigned int stuffedIDlocal=0;
 unsigned char numFrameBit=0; 
 while (numFrameBit<NsampleBit) {
   stuffedIDlocal = (stuffedIDlocal<<1) | bitFramelocal[numFrameBit];
   numFrameBit++;
 }
 return stuffedIDlocal;
}

// Setup
void arduinoSetup() {
	Serial.begin(1000000);
	TCCR1A = 0;  
	TCCR1B = 0;  
	TCCR1C = 0;  
	TIMSK1 = _BV(TOIE1);
	delay(100);
	startCleanTimer();	
	// set the digital Pin 
	pinMode(PinRead, INPUT);  
	pinMode(PinDebug, OUTPUT);
	pinMode(PinWrite, OUTPUT);

	PORTD = B11111111; // Set all pins to 1
}