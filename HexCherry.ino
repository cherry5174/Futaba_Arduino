/*DevLog

2015.03.27 Futaba Sbus -> motor
           Reciver R6208SB -> 8 chanel                  
*/
#include <avr/io.h>
#include <avr/pgmspace.h>
#define  VERSION  220

#define IDLE         0
#define HEADER_START 1
#define HEADER_M     2
#define HEADER_ARROW 3
#define HEADER_SIZE  4
#define HEADER_CMD   5
#define HEADER_ERR   6

#define MSP_SET_RAW_RC_TINY      150   //in message          4 rc chan
#define RC_CHANS                 18

enum rc {
    ROLL,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
static uint16_t sbusIndex=0;

//static int rcValue[8];
static int auxChannels;

static int c_state;
static byte err_rcvd;
static byte checksum;
static byte cmd;
static int offset, dataSize;
static byte inBuf[64];
static int p;

/*********** RC alias *****************/



#define SERIAL_BUF_LEN 64

static byte serialBuf[SERIAL_BUF_LEN];

int motor1 = 10;      
int motor2 = 6;      
int motor3 = 5;      
int motor4 = 9;   
int motor5 = 13;     
int motor6 = 11;   

void init_motors(){
  pinMode(motor1, OUTPUT);   // sets the pin as output
  pinMode(motor2, OUTPUT);   // sets the pin as output
  pinMode(motor3, OUTPUT);   // sets the pin as output
  pinMode(motor4, OUTPUT);   // sets the pin as output
  pinMode(motor5, OUTPUT);   // sets the pin as output
  pinMode(motor6, OUTPUT);   // sets the pin as output
}

void write_motors(int thr){
  int refinedThr= (thr - 1000) / 4;
   
  analogWrite(motor1, refinedThr);
  analogWrite(motor2, refinedThr);
  analogWrite(motor3, refinedThr);
  analogWrite(motor4, refinedThr);
  analogWrite(motor5, refinedThr);
  analogWrite(motor6, refinedThr);
}

void init_rc_data(){  
  rcValue[0] = 1500;
  rcValue[1] = 1500;
  rcValue[2] = 1500;
  rcValue[3] = 1000;
  rcValue[4] = 1500;
  rcValue[5] = 1500;
  rcValue[6] = 1500;
  rcValue[7] = 1500;
}

void readSbus(){
  #define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
  static uint16_t sbus[25]={0};
  while(!Serial1){
    int val = Serial1.read();
    if(sbusIndex==0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+976; // Perhaps you may change the term "/2+976" -> center will be 1486
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+976; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+976; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+976; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+976; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+976;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+976; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+976; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+976; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+976; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+976; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+976; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+976; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+976; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+976; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+976; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      #if defined(FAILSAFE)
      if (!((sbus[23] >> 3) & 0x0001))
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
      #endif
    }
  }        
}

void setup()
{
    Serial1.begin(100000);  //2 futaba sbus receiver
    while(!Serial1){        };
    init_rc_data();
    
    delay(100);
}

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;  

// ******** Main Loop *********
void loop () {
    static uint32_t rcTime  = 0;
    int len = 0;

    if (currentTime > rcTime ) { // 50Hz
        rcTime = currentTime + 20000;
        
        //len = Serial1.readBytes((char *)serialBuf, SERIAL_BUF_LEN); 
      
        readSbus();   
    } 
    else { // not in rc loop  
   
    }

    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;
    
    write_motors(rcValue[3]);
}




