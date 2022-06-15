/*******************************************************************************************
 *  BIT_BANG_TEST   -UNO Demo for Bit Bang Serial  -->WORKING !!
 *                   This implementes a 250 Data Link, which can't be used with a UART
 *                    device because of the non-standard Buad Rate 
 *   Intended for very Small & Simple comm with ATTINY devices that do not have UART or
 *    enough space for SoftwareSerial options
 *    
 *  **NOTE: Remove anything with PRINT and Flags as there is no need for these in ATTINY
 *  
 *  Connections: Pin(3) on UNO A to (3) on UNO B,  GND to GND
 *  TEST:  On Serial Monitor of UNO A  Enter>M someMsg
 *         Observe Serial Monitor on UNO B ->Should show the msg
 *         Try same on UNO B to UNO A
 *         Works very well in testing ATTINY88 which has no Serial Port 
 *           >>This sends and Rcvs msgs - showing both, so can debug ATTINY that way
 *******************************************************************************************/ 
#define TITLE               "BIT BANG SERIAL "
#define APP_REV             "2.1 " 
#define PRINT               Serial.print
#define PRINTLN             Serial.println
#define ONBOARD_LED_PIN     13
#define NUM_TIMERS          2
#define HEARTBEAT_TMRX      0     // timer index
#define CMDSTR_LEN          11
#define COMMBFR_SIZ         40
#define COMM_PIN            3 
#define BIT_FRAME_TIME      4
#define NXT_BYTE_TIMEOUT    10    // ms to wait for nxt byte START_BIT
#define HEARTBEAT_RATE      250   // ms

#define SHOW_STS_FLAG       0x20
#define SHOW_HLP_FLAG       0x40
 
char cmdStr[CMDSTR_LEN+1], ch; 
byte cmdStrLen, commState, flags;
byte bitCnt, bitMask, bitTime, nxtSample, newCh, commNdx, commBfrLen, commBfr[COMMBFR_SIZ+2];
unsigned sftTmr[NUM_TIMERS];

void startTimer(byte tx,unsigned dly) { if(tx<NUM_TIMERS)sftTmr[tx]=dly; }
byte timerExpired(byte tx) { return (sftTmr[tx]) ? 0:1; }

void setFlag(byte flagV) { flags |= flagV; } 
void clrFlag(byte flagV) { flags &= ~flagV; } 
inline byte tstFlag(byte flagV) { return (flags & flagV); }
void toggleFlag(byte flagV) { if (flags & flagV) flags &= ~flagV;  else  flags |= flagV;  }
////////////////////////////////////////////////////////////////////
void setup() { 
  Serial.begin(115200); delay(800);
  flags = 0;
  setFlag(SHOW_HLP_FLAG); 
  pinMode(ONBOARD_LED_PIN, OUTPUT); 
  commState=0;              // INIT BIT BANG COMM = RCV MODE 
}

////////////////////////////////////////////////////////////////////
void loop() 
{ byte ioV; static byte nxtCycle, cmdRdy, cmdIdleCnt, heartBeat; static unsigned long sysTick;

                    ////////////////////////////////////////////////////////////////////////
                    // EVERY NEW ms DO BIT BANG SERIAL COMM
                    //  This App limits the tasks done on each pass, separating them in time
                    //  >>On new ms do Comm, Next Cycle (some micro sec later) do Debug Serial
                    //    On next (3rd cycle of this ms) do low priority task - Heartbeat
                    ////////////////////////////////////////////////////////////////////////
  if(sysTick != millis())     // NEW ms
  {
    sysTick = millis();
    for(byte tx=0; tx<NUM_TIMERS ;tx++) { if(sftTmr[tx]) sftTmr[tx]--; }    // DECR ALL ACTV TIMERS every ms

        // DO SERIAL COMM TASKS EVERY ms
    ++bitTime;                // INCR bitTime every ms    
    switch(commState)
    {
      case 0:                 // INIT RCV_MODE
        pinMode(COMM_PIN, INPUT_PULLUP);
        commState = 1;
        break;

      case 1:                 // WAITING FOR START OF NEW MSG ->START BIT of FIRST BYTE 
        if(!digitalRead(COMM_PIN))              // LOW ACTV ->START BIT RCVD
        {
          bitMask = 0x40;   newCh = 0;  bitCnt = 1;
          bitTime = 0;    nxtSample = BIT_FRAME_TIME + BIT_FRAME_TIME/2;  // Set for middle of Next Frame 
          commNdx = 0;                        // Reset msg bfr index
          commState = 3;                      
        }
        else if(commBfrLen && bitTime > 5)      // IDLE & NEW REPLY MSG TO SEND HAS ARRIVED
        {
          commState = 100;                      // Begin Sending Reply Msg
        }
        break;
        
      case 2:                 // WAITING FOR START OF NEXT BYTE IN THIS MSG 
        if(!digitalRead(COMM_PIN))              // LOW ACTV ->START BIT RCVD
        {
          bitMask = 0x40;   newCh = 0;  
          bitCnt = 1;
          ++commState;                      
          bitTime = 0;    nxtSample = BIT_FRAME_TIME + BIT_FRAME_TIME/2;  // Set for middle of Next Frame  
        }
        else if(commNdx)                // NO START RCVD for Next Byte ->Check for TIMEOUT
        {                               //   if commNdx then have rcvd some msg bytes
          if(bitTime >= nxtSample)            // TIMEOUT:  so this is END OF MSG
          {                                   //  Process Rcvd Msg then Reset
              PRINT(F("Rcvd Msg>")); PRINTLN((char *)commBfr);  
            commState = 0;                    // Begin wait for Next Msg to start 
            commNdx = 0;                      // Drain Rcvd Msg
          }
        } 
        break;

      case 3:                 // WAITING FOR NEXT (of 7) DATA BIT  
        if(bitTime >= nxtSample)        // Time to Sample
        { 
          if(!digitalRead(COMM_PIN)) newCh |= bitMask;        // If Input==ACTV Then Set this bit in newCh
          bitMask >>= 1;                    // RT Shift bitMask for next bit
          if(++bitCnt > 7)
            commState = 4;
          bitTime = 0;  nxtSample = BIT_FRAME_TIME;   // ReSet Time vars for next bit sample time - move 1 bit frame time
        }
        break;

      case 4:                 // WAITING FOR STOP  (bitCnt >= 8 now)
        if(bitTime >= nxtSample)        // Time to Sample
        { 
          if(digitalRead(COMM_PIN))                       // Rcvd STOP
          {                                   // Finished Rcving this byte
            if(commNdx < COMMBFR_SIZ)
            {
              commBfr[commNdx++] = newCh;     // append newCh to msg
              commBfr[commNdx] = 0;           // terminate msg string 
            }
            commState = 2;                    // Goto Next Byte 
            bitTime = 0;                      // reset bitTime - will use to test End Of Msg Timeout
            nxtSample = NXT_BYTE_TIMEOUT;     // Set to Timeout period  (waiting for Nxt Msg Byte)
          }
        }
        break;
                    ////////////////////////////////////////////////////////////////////////
                    //  SEND MSG ON SAME PIN
                    ////////////////////////////////////////////////////////////////////////
      case 100:                 // SETUP TO SEND REPLY MSG
      case 200:                 //   same in Send Only Mode
        pinMode(COMM_PIN, OUTPUT);  digitalWrite(COMM_PIN, HIGH);   // Set COMM_PIN as OUTPUT & Set HIGH == STOP
        ++commState;
        break;

      case 101:                 // WAITING TO BEGIN SEND WHEN MSG is Loaded 
      case 201:
        if(commBfrLen)
        { 
          ++commState; 
          commNdx = 0;
        }
        break;

      case 102:                 // START SEND OF NEXT BYTE
      case 202:
        if(commNdx < commBfrLen)
        {
          digitalWrite(COMM_PIN, LOW);                              // SET START BIT Actv
          bitTime = 0;  nxtSample = BIT_FRAME_TIME;
          bitMask = 0x40;   newCh = commBfr[commNdx++];
          bitCnt = 0;
          ++commState; 
        }
        else            // Msg Complete
        {
          PRINT(F("SENT>")); PRINTLN((char *)commBfr);   
          commBfrLen = 0;     // Drain Bfr 
          if(commState>=200)            // Ended Sending in Send Only Mode
            commState = 200;            // Go back to wait for Nxt Msg to Send
          else          
            commState = 0;              // Go back to start of RCV_MODE
        }
        break;

      case 103:                         // SENDING DATA BITS
      case 203:
        if(bitTime >= nxtSample)        // TIME TO SET NEXT BIT
        {
          ++bitCnt;
          ioV = 1;
          if(bitCnt <= 7)             // Sending Data Bit
          {
            ioV = (newCh & bitMask) ? 0:1;    // Data bit==1 is Set LOW ACTV
            bitMask >>= 1;              // RT Shift bit mask for next bit
          }
          else                        // STOP_BIT  == 1
          { 
            if(bitCnt >= 9)           // Done go back to Start Nxt Byte state 
              --commState; 
          }
          digitalWrite(COMM_PIN, ioV);
          bitTime = 0;  nxtSample = BIT_FRAME_TIME;
        }
        break;

      default:
        PRINT(F("ERROR: commState="));PRINT(commState);PRINTLN(F("  ->Back To RcvMode[0]"));
        commState = 0;
        break; 
    }
    ++nxtCycle;
  }
        ///////////////////////////////////////////////////////////////////
        // HANDLE DEBUG SERIAL at 1 ms pace, But not on a New ms cycle
        //  so check this on cycle after newms
        ///////////////////////////////////////////////////////////////////
  else if(nxtCycle==1)
  {
    ++nxtCycle;
    if(Serial.available())
    {
      for(byte rdCnt=0; rdCnt<CMDSTR_LEN && Serial.available() ;rdCnt++)
      {
        ch = Serial.read(); 
        if(ch>=' ' && cmdStrLen<CMDSTR_LEN)
        {
          cmdStr[cmdStrLen++] = ch;
          cmdStr[cmdStrLen] = 0;
        }
        else
          ++cmdRdy;
      }
      cmdIdleCnt = 0;
    }
    else if(cmdStrLen)
    {
      if(++cmdIdleCnt > 10)       // 10 ms since last ch arrived, so end of msg stream
        ++cmdRdy;

      if(cmdRdy)                  // Command is ready to be handled
      {
          PRINT(F("DEBUG CMD>"));PRINTLN(cmdStr);
          
        ch = toupper(cmdStr[0]);
        if(ch == 'L')             // SET LISTEN MODE
        {
          commState = 0;
          PRINTLN(F("Begin LISTEN & REPLY MODE "));
        }
        else if(ch=='W')
        {
          commState = 200;
          PRINTLN(F("Begin SEND Only MODE "));
        }
        else if(ch=='M' && (commState==201 || commState==1))        // SEND MSG
        {
          byte sx;
          for(sx=2; sx<CMDSTR_LEN && cmdStr[sx]>=' ' && commBfrLen<COMMBFR_SIZ ;sx++)
            commBfr[commBfrLen++] = cmdStr[sx];
          commBfr[commBfrLen] = 0;
          PRINT(F("Queued Send:"));PRINT(commBfrLen); PRINT(F(" >"));PRINTLN((char*)commBfr);
        }
        else if(ch=='S')
          setFlag(SHOW_STS_FLAG); 
        else if(ch=='H' || ch=='?')   // SHOW HELP
          setFlag(SHOW_HLP_FLAG); 
          
        cmdRdy = 0; cmdStrLen = 0;
      }
    }
    else if(tstFlag(SHOW_STS_FLAG)) { clrFlag(SHOW_STS_FLAG); showSts(); }
    else if(tstFlag(SHOW_HLP_FLAG)) { clrFlag(SHOW_HLP_FLAG); showHlp(); }
  } //  end nxtCycle
  
        ///////////////////////////////////////////////////////////////////
        // HEARTBEAT - LOW PRIORITY HOUSEKEEPING TASKS
        ///////////////////////////////////////////////////////////////////
  else if(nxtCycle==2)
  {
    ++nxtCycle;
    if(timerExpired(HEARTBEAT_TMRX))
    {
      digitalWrite(ONBOARD_LED_PIN, heartBeat);   heartBeat = heartBeat ? 0:1;
      startTimer(HEARTBEAT_TMRX, HEARTBEAT_RATE);
    }
  }
} 

////////////////////////////////////////////////////////////////////
void showSts()
{
  for(byte x=0;x<20;x++) { PRINT("= "); if(x==10) {PRINT(TITLE);PRINT(APP_REV);PRINT(F("STATUS")); }} PRINTLN();
  PRINT(F(" commState="));PRINTLN(commState);
}

////////////////////////////////////////////////////////////////////
void showHlp()
{
  for(byte x=0;x<20;x++) { PRINT("= "); if(x==10) {PRINT(TITLE);PRINT(APP_REV);PRINT(F("HELP")); }} PRINTLN();
  PRINTLN(F("CMDS: L  ==ListenMode,  W  ==SendMode,  S  ==Status,"));
  PRINTLN(F("      M ABC   ==Send Msg ABC"));
  PRINTLN(F(" commState 0 to 10 =RcvdMode, 100 - 103 = Replying, 200-203 = Send Only Mode"));
}
