#define F_CPU 1000000UL
#define BAUD_PRESCALER 0


// === IO DEFINATIONS ====
#define HBLED 4
#define INTLED 5
#define GPIO4 6
#define GPIO3 7
#define GPIO1 11
#define GPIO2 10
#define TXD 1 
#define CMD_PING 0x0f
#define CK_CMD_PING(data) (CMD_PING==(data&CMD_PING) ? 1:0)
#define CMD_ACK 0x01
#define CK_CMD_ACK(data) (CMD_ACK==(data&CMD_ACK) ? 1:0)
#define CMD_RST 0x02
#define CK_CMD_RST(data) (CMD_RST==(data&CMD_RST)) ? 1:0)
#define CMD_ENUM 0x03
#define CK_CMD_ENUM(data) (CMD_ENUM==(data&CMD_ENUM) ? 1:0)
#define CMD_ARM 0x03
#define CK_CMD_ARM(data) (CMD_ARM==(data&CMD_ARM) ? 1:0)
#define CMD_CONT 0x04// continue enum
#define CK_CMD_CONT(data) (CMD_CONT==(data&CMD_CONT) ? 1:0)

#define TxPack(TxADDR,TxCMD) ((char)(TxADDR<<4)+TxCMD)

#define SID(RxByte)  ((RxByte&0xf0)>>4)
#define ForMe(RxByte) (SID(RxByte) == stage_id ? 1:0)

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
static volatile char Eight_Bit_ADC_Value;
char stage_id=0;



void initialize_ADC(void)
{
  ADCSRA |= 1 << ADPS2;
  ADMUX |= 1 << ADLAR | 1 << MUX1 | 1 << MUX0;
  ADCSRA |= 1 << ADIE;
  ADCSRA |= 1 << ADEN;
  //  sei();
  ADCSRA |= 1 << ADSC;
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
  
  LINCR |= 1 << LENA | 0 << LCMD0 | 1 << LCMD1 | 1 << LCMD2;
}

void initialize_GPIO(void){
  pinMode(HBLED, OUTPUT);
  digitalWrite(HBLED, HIGH);
  
  pinMode(INTLED, OUTPUT);
  digitalWrite(INTLED, HIGH);
  
  pinMode(GPIO1, INPUT);  
  pinMode(GPIO2, INPUT);

    pinMode(GPIO3, OUTPUT);
  digitalWrite(GPIO3, LOW);
  
  pinMode(GPIO4, OUTPUT);
  digitalWrite(GPIO4, LOW);

  
}
void txen(int en){
  if(en==1)
    LINCR |= 1 << LCMD0;
  else{
    LINCR &= ~(1<< LCMD0);
    pinMode(TXD, INPUT);
  }
}
void sendByte(char c){
  txen(1);
  LINDAT=c;
  while (LINSIR & (1 << LBUSY)); 
  txen(0);
}

char recieved_data=0x00;
int heartrate=1000;
int HBstate=1;
int main(void)
{ 
  CLKPR=0x80;// the tool writes fucked out prescaler, restoring clock to full speed
  CLKPR=0x00;
  DDRA |= (1 << PINA4) | (1 << PINA5) | (1 << PINA6) | (1 << PINA7)  ;
  LINENIR=0x03;
  
//  PCICR=0x02; // pcint enable
//  PCMSK1=0x0c; // pcint enable
  sei();
  initialize_UART();
  //initialize_ADC();
  initialize_GPIO();
  int hrcnt=0;
  while(digitalRead(GPIO1)==0);
  sendByte(TxPack(stage_id,CMD_PING));
//  LINDAT=0x59;
  //wait untill I1 goes high
  char rxbuf=0;
  int effHr,hrbeeps;
  while(1)
  {
    for(hrbeeps=0;hrbeeps<=2*stage_id+1;hrbeeps++){
      if(hrbeeps>=2*stage_id+1)
          effHr=heartrate;
      else
          effHr=heartrate/stage_id;
      ////// ^HR control, letting LED indicate which stage it is
      for(hrcnt=0;hrcnt<effHr;hrcnt++){
        //_NOP();
        if(recieved_data!=0x00){
          rxbuf=recieved_data;
          recieved_data=0x00;
          if(CK_CMD_ENUM(rxbuf) && (stage_id==0)){
         // if((recieved_data==0x13) && (stage_id==0)){
            digitalWrite(INTLED,LOW);
            stage_id=SID(rxbuf);
            sendByte(TxPack(stage_id,CMD_ACK));
            
            
          }
          else if(CK_CMD_CONT(rxbuf) && ForMe(rxbuf)){// process cont command
            digitalWrite(GPIO3,HIGH);
          }
//          else if(CK_CMD_RST(rxbuf) && ForMe(rxbuf)){
//            
//          }
          else if(CK_CMD_PING(rxbuf) && ForMe(rxbuf)){
            sendByte(TxPack(stage_id,CMD_ACK));
          }
         
        }
      }
      digitalWrite(HBLED,!digitalRead(HBLED));
    }
//    digitalWrite(GPIO3,!digitalRead(GPIO3));
//    digitalWrite(GPIO4,!digitalRead(GPIO4));


  }

}

ISR (LIN_TC_vect)
{ 
  recieved_data = LINDAT;      
}

ISR(ADC_vect)
{
  Eight_Bit_ADC_Value = ADCH;
  ADCSRA |= 1 << ADSC;
}
