#include <avr/wdt.h> 
#define F_CPU 1000000UL
#define BAUD_PRESCALER 0


// === CONTROL IO DEFINATIONS ====
#define HBLED 4
#define INTLED 5
#define GPIO4 6
#define GPIO3 7
#define GPIO1 11
#define GPIO2 10
#define TXD 1 

// ==  STAGE IO DEFINATIONS == 
#define TRIGGER1 8
#define TRIGGER2 9
#define OUTN 13
#define OUTP 12
#define ICOIL 3


#define REG_ONDELAYN 1
#define REG_ONDELAYP 2
#define REG_OFFDELAY 3
#define REG_SAFETYTO 4

#define CMD_PING 0x0f
#define CK_CMD_PING(data) (CMD_PING==(data&0x0f) ? 1:0)
#define CMD_ACK 0x01
#define CK_CMD_ACK(data) (CMD_ACK==(data&0x0f) ? 1:0)
#define CMD_RST 0x02
#define CK_CMD_RST(data) (CMD_RST==(data&0x0f) ? 1:0)
#define CMD_ENUM 0x03
#define CK_CMD_ENUM(data) (CMD_ENUM==(data&0x0f) ? 1:0)
#define CMD_CONT 0x04// continue enum
#define CK_CMD_CONT(data) (CMD_CONT==(data&0x0f) ? 1:0)
#define CMD_ARM 0x05
#define CK_CMD_ARM(data) (CMD_ARM==(data&0x0f) ? 1:0)
#define CMD_DISARM 0x06
#define CK_CMD_DISARM(data) (CMD_DISARM==(data&0x0f) ? 1:0)
#define CMD_STREAM 0x07
#define CK_CMD_STREAM(data) (CMD_STREAM==(data&0x0f) ? 1:0)
#define CMD_METER 0x08
#define CK_CMD_METER(data) (CMD_METER==(data&0x0f) ? 1:0)
#define CMD_FIRE 0x09
#define CK_CMD_FIRE(data) (CMD_FIRE==(data&0x0f) ? 1:0)
#define CMD_REGR 0x0a
#define CK_CMD_REGR(data) (CMD_REGR==(data&0x0f) ? 1:0)
#define CMD_REGW 0x0b
#define CK_CMD_REGW(data) (CMD_REGW==(data&0x0f) ? 1:0)
#define CMD_OBSTACLE 0x0c
#define CK_CMD_OBSTACLE(data) (CMD_OBSTACLE==(data&0x0f) ? 1:0)


#define TxPack(TxADDR,TxCMD) ((char)(TxADDR<<4)+TxCMD)

#define SID(RxByte)  ((RxByte&0xf0)>>4)
#define ForMe(RxByte) (((SID(RxByte) == stage_id) | (SID(RxByte) == 0)) ? 1:0)

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

/// GLOBAL VARIABLES

char stage_id;
char recieved_data;
unsigned int heartrate;
int HBstate;
char lastPB;
unsigned char estopCtr;
char ADCBuff[128]={0x00};
unsigned int ADCCtr=0;
unsigned char skipByte=0;

unsigned int dtSpeed;
bool newDataAval;
// settings vars
struct config{ // number is num of 4us counts(256=1ms)
  unsigned char onDelayN=75;  // 75 = 300us
  unsigned char onDelayP=25;   //p is after N, how long?
  unsigned char offDelay=100;  //off >> onD + onP - expected meter PW
  unsigned char safetyTO=20;// in ms
};
struct config stageConfig;
enum T0Actions{
  NHIGH,
  PHIGH,
  PNLOW,
  NONE
};
enum T0Actions T0Action=NONE;

enum T1Actions{
  ESTOP,
  T1NONE
};
enum T1Actions T1Action=T1NONE;

enum stageStates{
  _IDLE,
  _ARMED,
};
enum stageStates stageState=_IDLE;

enum phyStates{
  _CMD,
  _ADDRR,
  _ADDRW,
  _DATAW,
  _STREAM,
};
enum phyStates phyState=_CMD;


// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    return;
}

void writeReg(unsigned char reg , unsigned char data){
  switch(reg){
    case REG_ONDELAYN: stageConfig.onDelayN=data;
      break;
    case REG_ONDELAYP: stageConfig.onDelayP=data;
      break;
    case REG_OFFDELAY: stageConfig.offDelay=data;
      break;
    case REG_SAFETYTO: stageConfig.safetyTO=data;
      break;
    default:
      break;
  }
}
unsigned char readReg(unsigned char reg ){
  switch(reg){
    case REG_ONDELAYN: return stageConfig.onDelayN;
      break;
    case REG_ONDELAYP: return stageConfig.onDelayP;
      break;
    case REG_OFFDELAY: return stageConfig.offDelay;
      break;
    case REG_SAFETYTO: return stageConfig.safetyTO;
      break;
    default:
      break;
  }
}
void initialize_ADC(void)
{
//  ADCSRA |= 0 << ADPS2 | 1 << ADPS1 | 0 << ADPS0;// /16 prescaler
//  ADMUX |= 1<<REFS1 | 1<<REFS0 | 1 << ADLAR | 1 << MUX1 | 1 << MUX0;//2.56 internal ref, PA3 pin as input
////  ADCSRA |= 1 << ADIE;
//  ADCSRB |= 0x00;
//  ADCSRA = 0xC0;//|= 1 << ADEN;
//  sei();
//  ADCSRA |= 1 << ADSC;
  ADCSRA |= 1 << ADPS2;
  
  ADMUX |= 1<<REFS0| 1 << ADLAR | 1 << MUX1 | 1 << MUX0;
  
  ADCSRA |= 1 << ADIE;
  
  ADCSRA |= 1 << ADEN;
  ADCSRB=0x00;

  ADCSRA |= 0 << ADPS2 | 0 << ADPS1 | 0 << ADPS0;// /16 prescaler
  
  sei();
  
}

void initialize_UART(void)
{
  LINCR = 0;
  LINBTR |= _BV(LDISR) | 1 << LBT4;
  LINBTR &= ~(1 << LBT5);
  LINBTR &= ~(1 << LBT3);
  LINBTR &= ~(1 << LBT2);
  LINBTR &= ~(1 << LBT1);
  LINBTR &= ~(1 << LBT0);
  
  LINBRRH = (BAUD_PRESCALER >> 8);
  LINBRRL = BAUD_PRESCALER;
  
  LINCR |= 1 << LENA  | 1 << LCMD1 | 1 << LCMD2;
  LINCR &= ~(1<< LCMD0);
  LINENIR=0x01;

}

void initialize_GPIO(void){
  pinMode(HBLED, OUTPUT);
  digitalWrite(HBLED, LOW);
  
  pinMode(INTLED, OUTPUT);
  digitalWrite(INTLED, HIGH);
  
  pinMode(GPIO1, INPUT);  
  pinMode(GPIO2, INPUT);

  pinMode(GPIO3, OUTPUT);
  digitalWrite(GPIO3, LOW);
  
  pinMode(GPIO4, OUTPUT);
  digitalWrite(GPIO4, LOW);

// configure stage interface 
  pinMode(OUTN, OUTPUT);
  digitalWrite(OUTN, HIGH);
  pinMode(OUTP, OUTPUT);
  digitalWrite(OUTP, LOW);
  
  pinMode(TRIGGER1, INPUT);
  pinMode(TRIGGER2, INPUT);
}
void txen(int en){
  if(en==1)
    LINCR |= 1 << LCMD0;
  else{
    initialize_UART();
    pinMode(TXD, INPUT);
  }
}
void sendByte(char c){
  txen(1);
  LINDAT=c;
  while (LINSIR & (1 << LBUSY)); 
  txen(0);
}

void initGlobalVars(){
  stage_id=0;
  recieved_data=0x00;
  heartrate=35535;
  HBstate=1;
  lastPB=0x00;
  estopCtr=0;
  stageState=_IDLE;
  phyState=_CMD;
  newDataAval=false;
  skipByte=0;

}

void initialize_PCINT(){
  
//  PCICR|= 1<<PCIE1; // pcint enable, bank 1 aka port b , comment for for default disarm
  PCMSK1|= 0x03;//1<<PCINT8 | 1<<PCINT9; // pcint active pins Pb0 Pb1
}
void initialize_TIMER0(){ // trigger delay timer
  TCCR0A= 0x00; //Normal port operation, OC0A disconnected. Normal
  TCCR0B= 0x04; //clkT0S/64 (from prescaler)8us tick
  //TCNT0 read from here;
  TIMSK0|= 1<<TOIE0; 
}
void initialize_TIMER1(){ // meter timer
  TCCR1A= 0x00; //Normal port operation, OC0A disconnected. Normal
  TCCR1B= 0x02; //clkT0S/8 (from prescaler)1us tick
  //TCNT1 read from here;
  TIMSK1|= 1<<TOIE1; 
}

void delay_us(char delay){
  for(char i=0;i<delay;i++){
    _NOP();
    }
}

void armStage(){
  PCICR = 1<<PCIE1; // pcint enable, bank 1 aka port b
  stageState=_ARMED;
}
void disarmStage(){
  PCICR = 0x00; 
  stageState=_IDLE;
}
int main(void)
{ 
  RSTLBL:initGlobalVars();
  CLKPR=0x80;// the tool writes fucked up prescaler, restoring clock to full speed
  CLKPR=0x00;

  sei();
  initialize_UART();
  initialize_ADC();
  initialize_GPIO();
  initialize_PCINT();
  initialize_TIMER0();
  initialize_TIMER1();
  unsigned int hrcnt=0;
  char rxbuf=0;
  int effHr,hrbeeps;
  char SWRESET=false;
  unsigned char regAddr,regData;
  while(digitalRead(GPIO1)==0);
  sendByte(TxPack(stage_id,CMD_PING));
  //wait untill I1 goes high

  wdt_enable(WDTO_60MS);
  digitalWrite(HBLED,!digitalRead(HBLED));  
  while(1)
  {
    for(hrbeeps=0;hrbeeps<5*stage_id+1;hrbeeps++){

      ////// ^HR control, letting LED indicate which stage it is
      
      for(hrcnt=0;hrcnt<heartrate;hrcnt++){
        //_NOP();
        if(~SWRESET)
          wdt_reset();//feeding WDT 
        if(newDataAval){
          newDataAval=false;
          rxbuf=recieved_data;

          if(phyState==_ADDRR){
            regAddr=rxbuf;
            sendByte(readReg(regAddr));
            phyState=_CMD;
          }
          else if(phyState==_ADDRW){
            regAddr=rxbuf;
            phyState=_DATAW;
          }
          else if(phyState==_DATAW){
            regData=rxbuf;
            writeReg(regAddr,regData);
            phyState=_CMD;
            sendByte(TxPack(stage_id,CMD_ACK)); 
          }
          else if(phyState==_CMD)
          {
            
            if(CK_CMD_ENUM(rxbuf) && (stage_id==0)){
           // if((recieved_data==0x13) && (stage_id==0)){
              digitalWrite(INTLED,LOW);
              stage_id=SID(rxbuf);
              sendByte(TxPack(stage_id,CMD_ACK));   
            }
            else if(CK_CMD_CONT(rxbuf) && ForMe(rxbuf)){// process cont command
              digitalWrite(GPIO3,HIGH);
              delay_us(50);
              digitalWrite(GPIO3,LOW);
            }
            else if(CK_CMD_RST(rxbuf) && ForMe(rxbuf)){
  //            rxbuf=0x00;
  //            recieved_data;
  //            LINENIR=0x00;
  //            while(1);//let code die here untill resets
                wdt_disable();
                initialize_GPIO();
                delay_us(50);
                goto RSTLBL;
            }
            else if(CK_CMD_PING(rxbuf) && ForMe(rxbuf)){
              sendByte(TxPack(stage_id,CMD_ACK));
            }
            else if(CK_CMD_ARM(rxbuf) && ForMe(rxbuf)){
              armStage();
              sendByte(TxPack(stage_id,CMD_ACK));
            }
            else if(CK_CMD_DISARM(rxbuf) && ForMe(rxbuf)){
              disarmStage();
              sendByte(TxPack(stage_id,CMD_ACK));
            }
            else if(CK_CMD_STREAM(rxbuf) && ForMe(rxbuf)){
              sendByte(TxPack(stage_id,CMD_ACK));
              sendByte(ADCCtr);
              txen(1);
              for (char i=0;i<ADCCtr;i++){
                while (LINSIR & (1 << LBUSY)); 
                LINDAT=ADCBuff[i];
              }
              txen(0);
            }
            else if(CK_CMD_METER(rxbuf) && ForMe(rxbuf)){
              sendByte(TxPack(stage_id,CMD_ACK));
              sendByte(dtSpeed>>8);
              sendByte(dtSpeed&0xff);
            }
            else if(CK_CMD_FIRE(rxbuf) && ForMe(rxbuf)){
              digitalWrite(GPIO4,!digitalRead(GPIO4));
              if(stageState==_ARMED){
                sendByte(TxPack(stage_id,CMD_ACK));
                startDischarge();
              }
            }
            else if(CK_CMD_REGR(rxbuf)){
              if(ForMe(rxbuf)){
                sendByte(TxPack(stage_id,CMD_ACK));
                phyState=_ADDRR;
              }
              else{
                skipByte=1;
              }     
            }
           else if(CK_CMD_REGW(rxbuf)){
              if(ForMe(rxbuf)){
                sendByte(TxPack(stage_id,CMD_ACK));
                phyState=_ADDRW;
              }
              else{
                skipByte=2;
              }     
            }
           else if(CK_CMD_OBSTACLE(rxbuf) && ForMe(rxbuf)){
              sendByte(TxPack(stage_id,CMD_ACK));
//              sendByte(digitalRead(TRIGGER1)<<1 | digitalRead(TRIGGER2));
              sendByte(PINB);
            }
            rxbuf=0x00;    
          }   
        }
      }
      if (hrbeeps<2*stage_id)
      {
        digitalWrite(HBLED,!digitalRead(HBLED));  
      }
        
    }
  }
}
void startDischarge(){
  TCNT0=255-stageConfig.onDelayN;
  T0Action=NHIGH;
  TCNT1=65535-1000;
  T1Action = ESTOP;
}
void endDischarge(){
  TCNT0=255-stageConfig.offDelay;
  T0Action=PNLOW;  
  disarmStage();
}
void estop(){
  digitalWrite(OUTP,LOW);//low active
  digitalWrite(OUTN,HIGH);//low active
  ADCSRA &= ~(1 << ADIE);// disable ADC Int here
  T0Action = NONE;
  T1Action = T1NONE;
  disarmStage();
  
}

ISR(TIMER1_OVF_vect) {
  
  if(T1Action == ESTOP){
    digitalWrite(GPIO4,!digitalRead(GPIO4));
    TCNT1=65535-1000;
    estopCtr++;
    if(estopCtr>=stageConfig.safetyTO){
      estop();//pull estop
      estopCtr=0;
    }
  }
}
ISR(TIMER0_OVF_vect) {
  TCNT0=0;
  if(T0Action == NHIGH){
    ADCCtr=0;
    ADCSRA |= 1 << ADIE;// enable ADC Int here
    ADCSRA |= 1 << ADSC;
    digitalWrite(OUTN,LOW);//low active
    TCNT0=255-stageConfig.onDelayP;
    T0Action = PHIGH;
   }
   else if(T0Action == PHIGH){
    digitalWrite(OUTP,HIGH);//low active
    T0Action = NONE;
   }
   else if(T0Action == PNLOW){
    digitalWrite(OUTP,LOW);//low active
    digitalWrite(OUTN,HIGH);//low active
    ADCSRA &= ~(1 << ADIE);// disable ADC Int here
    T0Action = NONE;
   }
   else{
   }
}

ISR(PCINT1_vect){
  
  // 7 us react time, not bad
  char newPB=PINB;
  if(((lastPB^newPB)&0x03) == 0x01){
    if((newPB&0x01) == 0x01){
      //Trig 1 high
      TCNT1=0;
    }
    else{
      //Trig 1 low
    }
  }
  else if(((lastPB^newPB)&0x03) == 0x02){
    if((newPB&0x02) == 0x02){
        //Trig 2 high
        dtSpeed=TCNT1;
        if (stage_id!=1)
        {
          startDischarge();
        }
        
    }
    else{
        //Trig 2 low
        endDischarge();
    }  
  }
  else{} // nothing changed
  
  lastPB=newPB; 
}

ISR (LIN_TC_vect)
{ 
  recieved_data = LINDAT;    
  if(skipByte){
    skipByte--;
    return;
  }  
  newDataAval=true;
}

ISR(ADC_vect)
{
  
  ADCCtr++;
  if(ADCCtr<=sizeof(ADCBuff)){
    ADCBuff[ADCCtr]=ADCH;
  }
  ADCSRA |= 1 << ADSC;
}
