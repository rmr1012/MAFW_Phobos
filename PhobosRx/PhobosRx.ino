
#define SV1 0x18
#define SV2 0x81
//states
#define LISTEN 0
#define ADDR 1
#define CMDRX 2
#define CKSUM 3
#define TALK 4

#define RX_ACK 0x00

#define CMD_ADDRPRG 0x55

// === IO DEFINATIONS ====
#define HBLED 4
#define INTLED 5
#define GPIO3 6
#define GPIO4 7

int stage_id=0;

int rx_state=LISTEN;

int addr=0x00;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(HBLED, OUTPUT);
  Serial.begin(230400*4);//1Mhz UART
  LINBRRH=0x00;
  LINBRRL=0x00;
  CLKPR=0x80;// the tool writes fucked out prescaler, restoring clock to full speed
  CLKPR=0x00;
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(HBLED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(HBLED, LOW);    // turn the LED off by making the voltage LOW
  delay(250);                       // wait for a second
}

void serialEvent() {
  unsigned char command=0x00;
  unsigned char command_candidate=0x00;
  unsigned char addr_candidate=0x00;
  unsigned char stateReg[2] ={0x00,0x00};
  while (Serial.available()) {
    // get the new byte:
    unsigned char inChar = (unsigned char)Serial.read();
    if(rx_state==LISTEN){
      stateReg[1]=stateReg[0];
      stateReg[0]=inChar;
    }
    if (stateReg[1]==SV1 && stateReg[0]==SV2 && rx_state==LISTEN){
      rx_state=ADDR;
    }
    else if (rx_state==ADDR){
      rx_state=CMDRX;
      addr_candidate=inChar;
    }
    else if (rx_state==CMDRX){
      rx_state=CKSUM;
      command_candidate=inChar;
    }
    else if (rx_state==CKSUM){
      unsigned char sum=command_candidate+stateReg[1]+stateReg[0];
      if (sum==inChar){
        command=command_candidate;
        if(!digitalRead(1) && addr==0 && command==CMD_ADDRPRG){
          addr=addr_candidate;
        }
      }
      replyAck();
        
      } 
      rx_state==LISTEN;
    }
  }
 void replyAck(){
  Serial.print(addr);
  Serial.print(RX_ACK);
 }
