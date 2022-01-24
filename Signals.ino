/*
 * Radio Signal Code
 * CSPPRy 2019
 * Tristan Leavitt
 */
//******STANDARD SIGNAL BLOCK CODE*********//
const float VERSION = 1.406;

#define DEBUG
//#define DEBUGV

#ifdef DEBUGV
#define DEBUG
#endif

#include <RH_RF95.h>
#include <RHDatagram.h>
//#include <SPIFlash.h>  //Flash chip library
#include <SPI.h>       //SPI bus library
#include <avr/sleep.h> //ATmega Sleep library
#include <avr/power.h> //ATmega Power library
#include <avr/wdt.h>   //ATmega Watchdog Timer library

//Settings for RFM95
int NODEID = 999;
int DESTID = 999;
int DESTIDW = 999;
const int CTC = 254;

/*
 * Moteino Pins
 *  2 - Radio interupt pin
 *  8 - SPI Flash Select Pin
 * 10 - RFM69 Select Pin
 * 11 - MOSI
 * 12 - MISO
 * 13 - Serial Clock
 */

//Identifier pins setting. Internal pullups on, so LOW is 1 and HIGH is 0
//5 bits used to make addresses 1-32
//4 bits used to make addresses 1-16
//Maybe replace with a multiplexer in the future
#define ID0 A3
#define ID1 A4
#define ID2 A5
#define ID3 A6
#define ID4 9

//Pins for Signal Head LEDS
//LED Negatives
#define GREEN A0
#define AMBER A1
#define RED A2
//Only used for node 17
#define GREENW A3
#define AMBERW A4
#define REDW A5

//Capture and Release Switch Settings
#define CAPTURE 5
#define RELEASE 4
#define RELEASEW 6
#define BUTTON 3 //button interrupt pin

//Pin to detect is switch is thrown or not
#define SWITCH 7
#define SWITCH2 6

/*State of switch throw if applicable 
*0 - Main line
*1 - Diverging line
*2 - N/A
 */
uint8_t sState = 2;
uint8_t sState2 = 2;

const int LISTEN = 80;  //how many milliseconds the radio will listen for a response before retrying
const int RETRIES = 20; //how many retries to make before giving up

RH_RF95 radio; //The radio itself
RHDatagram manager(radio, NODEID);
//Initialize flash
//SPIFlash flash(SS_FLASHMEM, 0xEF30);

//Packet Structure
typedef struct
{
  uint8_t destination; //Unsigned 8 bit integer destination address, 0-255
  //uint8_t voltage; //Unsinged 8 bit integer battery voltage, 0-255 representing 00.0 - 25.5V
  bool isACK;  //Is this packet an acknowledgement
  bool isCode; //Is this packet a code control packet
  char aspect; /*New aspect color for signal head. R - red, A - amber, G - green
                *New aspect for right head on signal bridge: Q - red, B - amber, H - green
                *If isCode is true
                *A - Amber LED out
                *B - Battery good reset - **needs implementing**
                *b - Battery low alarm - **needs implementing**
                *C - Remote capture
                *D - Remote release
                *G - Green LED out
                *R - Red LED out
                *W - Wake signal
                *Z - Trigger a reset - not working?
                *S - Switch Main
                *s - Switch Diverging
                *T - Switch 2 Main
                *t - Switch 2 Diverging 
                */
} RCL;
RCL transmission; // The packet
RCL trans;

volatile char aspect = 'G'; //The current lit aspect
volatile char aspectW = 'H';
char lastAspect = 'X';
volatile bool changed = true; //For updating CTC map
volatile bool changedW = false;
volatile bool sw1Changed = false;
volatile bool sw2Changed = false;

bool out = false;
bool outW = false;

//Control locks
volatile bool LOCK = false; //Lockout capture whisker if aspect is R or A
volatile bool LOCKW = false;
bool rLock = false; //Lockout for release whisker
bool rwLock = false;
bool cLock = false;
bool dwarfLock = false;

//Control locks for delay going to green from amber
volatile bool Amber = false;
volatile bool AmberW = false;
volatile bool Green = true;
volatile bool GreenW = true;

unsigned long dimTime = 0;
unsigned int DIM = 255;

unsigned int AUTORELEASE = 0;

unsigned long t = 0;
unsigned long sleepWait = 0;

//Retry indicators
volatile bool cRetry = false;
volatile bool rRetry = true;
volatile bool rwRetry = false;
//Retry counter
int retry = 0;

volatile bool lastCap = false;
volatile bool lastRel = false;
volatile bool lastRelW = false;

//Watchdog Timer counter, Used for powering off signal head
volatile unsigned int dimCount = 0;

//relocated variables so everything is global
bool dwarfC = false;
bool rtn = false;
uint8_t id = 0;
uint8_t to = 0;
uint8_t len = sizeof(transmission);
int rssi = 0;
int led = 0;
bool state = 0;
char in = ' ';

//ISR for switch activation
void wake()
{
  //check the position of the turnout to capture the right direction
  if (NODEID == 17 && !cRetry)
  {
    int state = digitalRead(SWITCH);
    if (state != sState)
    {
      sw1Changed = true;

#ifdef DEBUG
      Serial.println(state);
#endif
    }

    sState = state;
  }

//Debounce the button
#ifdef DEBUGV
  if (digitalRead(CAPTURE) || digitalRead(RELEASE) || digitalRead(BUTTON) || (NODEID == 17 && digitalRead(RELEASEW)))
  {
    Serial.println("Before Debounce");
    Serial.print("Capture: ");Serial.println(digitalRead(CAPTURE));
    Serial.print("Release: ");Serial.println(digitalRead(RELEASE));
    if (NODEID == 17)
    {
      Serial.print("Release West: ");Serial.println(digitalRead(RELEASEW));
    }
    Serial.print("Interrupt: ");Serial.println(digitalRead(BUTTON));
  }
#endif

  for (int i = 0; i < 25; i++)
  {
    digitalRead(CAPTURE);
    digitalRead(RELEASE);
    digitalRead(BUTTON);
    if (NODEID == 17)
      digitalRead(RELEASEW);
    delay(1);
  }

#ifdef DEBUG
  if (digitalRead(CAPTURE) || digitalRead(RELEASE) || digitalRead(BUTTON) || (NODEID == 17 && digitalRead(RELEASEW)))
  {
    Serial.println("After Debounce");
    Serial.print("Capture: ");Serial.println(digitalRead(CAPTURE));
    Serial.print("Release: ");Serial.println(digitalRead(RELEASE));
    if (NODEID == 17)
    {
      Serial.print("Release West: ");Serial.println(digitalRead(RELEASEW));
    }
    Serial.print("Interrupt: ");Serial.println(digitalRead(BUTTON));
  }
#endif

  if (!cLock && (digitalRead(CAPTURE) == HIGH))
  {
    cRetry = true;
    if (NODEID != 17)
      rRetry = false;

    retry = 0;
  }
  else if (!rLock && !cRetry && !Green && (digitalRead(RELEASE) == HIGH))
  {
    rRetry = true;
    if (NODEID != 17)
      cRetry = false;

    retry = 0;
  }
  else if (NODEID == 17 && !rwLock && !GreenW && (digitalRead(RELEASEW) == HIGH))
  {
    rwRetry = true;

    retry = 0;
  }

  //Dont act on the release switch being pressed until it has been released again.
  if (rLock && !lastCap && (digitalRead(RELEASE) == LOW))
  {
    rLock = false;
  }

  if (rwLock && !lastRel && (digitalRead(RELEASEW) == LOW))
  {
    rwLock = false;
  }

  if (cLock && !lastRelW && (digitalRead(CAPTURE) == LOW))
  {
    cLock = false;
  }

  lastCap = digitalRead(CAPTURE);
  lastRel = digitalRead(RELEASE);
  lastRelW = digitalRead(RELEASEW);
}

//ISR for watchdog timer
ISR(WDT_vect)
{
  //Reset the watchdog timer to avoid random interrupts or possible resets
  wdt_reset();
  //wdt_disable();

  //increment counter for dimming
  dimCount++;
  dimTime = millis();

#ifdef DEBUG
  Serial.println("Dim++");
#endif
}

void setup()
{
  //Initialize serial connection
  Serial.begin(115200); //9600 causes garbage serial lines with sleep on Moteinos

  //Initialize ID and capture/release switch pins
  pinMode(ID0, INPUT_PULLUP); //Set ID pin for input with internal pullup resistor enabled
  pinMode(ID1, INPUT_PULLUP);
  pinMode(ID2, INPUT_PULLUP);
  pinMode(ID3, INPUT_PULLUP);
  pinMode(ID4, INPUT_PULLUP);

  //Initialize pins for head LEDS and apply power to LED
  pinMode(GREEN, OUTPUT);
  pinMode(AMBER, OUTPUT);
  pinMode(RED, OUTPUT);

  //Read ID pin and set IDs accordingly
  NODEID = bitID();
  //Once ID is set, disable internal pullup and set power to LOW to save battery
  pinMode(ID0, OUTPUT);
  pinMode(ID1, OUTPUT);
  pinMode(ID2, OUTPUT);
  pinMode(ID3, OUTPUT);
  pinMode(ID4, OUTPUT);
  digitalWrite(ID0, LOW);
  digitalWrite(ID1, LOW);
  digitalWrite(ID2, LOW);
  digitalWrite(ID3, LOW);
  digitalWrite(ID4, LOW);

  POST();

  pinMode(CAPTURE, INPUT);
  pinMode(RELEASE, INPUT);
  pinMode(BUTTON, INPUT);

  //Timing variable settings for each node
  switch (NODEID)
  {
  //East Parrish
  case 1:
    DESTID = 2;

    AUTORELEASE = 45; //~6min

    DIM = 225; //30min
    break;

  //West Etowah
  case 2:
    DESTID = 1;

    AUTORELEASE = 45; //~6min

    DIM = 225; //30min
    break;

  //West Parrish twins
  case 17:
    DESTID = 18;
    DESTIDW = 19;

    AUTORELEASE = 45; //~6min

    DIM = 450; //60min

    pinMode(RELEASEW, INPUT);
    pinMode(SWITCH, INPUT);

    sState = digitalRead(SWITCH);

    pinMode(GREENW, OUTPUT);
    pinMode(AMBERW, OUTPUT);
    pinMode(REDW, OUTPUT);

    digitalWrite(AMBERW, HIGH);
    digitalWrite(REDW, HIGH);

    rwRetry = true;

    changedW = true;
    sw1Changed = true;
    break;

  //Greyrock
  case 18:
    DESTID = 17;

    AUTORELEASE = 30; //~4min

    DIM = 450; //60min

    pinMode(SWITCH, INPUT_PULLUP);

    sState = digitalRead(SWITCH);

    sw1Changed = true;
    break;

  //Canton
  case 19:
    DESTID = 17;

    AUTORELEASE = 45; //~6min

    DIM = 450; //60min

    pinMode(SWITCH, INPUT_PULLUP);
    pinMode(SWITCH2, INPUT_PULLUP);

    sState = digitalRead(SWITCH);
    sState2 = digitalRead(SWITCH2);

    sw1Changed = true;
    sw2Changed = true;
    break;

  //Westbound Service Tracks dwarf
  case 20:
    DESTID = 19;

    AUTORELEASE = 999; //Dwarf shouldnt auto release

    DIM = 450; //60min
    rRetry = false;
    Green = true;
    break;

  //Eastbound Service Tracks dwarf
  case 21:
    DESTID = 18;

    AUTORELEASE = 999; //Dwarf shouldnt auto release

    DIM = 450; //60min

    rRetry = false;
    Green = true;
    break;
  }

#ifdef DEBUGV
  Serial.println("DEBUG MODE");
#endif

  Serial.print("Node ");Serial.print(NODEID, DEC);Serial.println(" ready");
  Serial.print("Version: ");Serial.println(VERSION, 3);

  //Initialize RFM95
  manager.init();
  manager.setThisAddress(NODEID);
  radio.setFrequency(915.0);
  radio.setSignalBandwidth(500000);
  radio.setCodingRate4(5);
  radio.setSpreadingFactor(8);
  radio.setTxPower(20, false);

  //Set signal head to green
  digitalWrite(GREEN, LOW);
  digitalWrite(AMBER, HIGH);
  digitalWrite(RED, HIGH);

  attachInterrupt(digitalPinToInterrupt(BUTTON), wake, HIGH);

  //Put radio in receive mode
  startRX();

  if (NODEID != 17 || NODEID != 18 || NODEID != 19)
  {
    //Disable TWI, Timers 1 and 2 to save power
    power_twi_disable();
    power_timer1_disable();
    power_timer2_disable();
  }

  //Initialize the timers and counter
  sleepWait = millis();
  dimTime = millis();
  dimCount = 0;
}

void loop()
{
  //Reset the watchdog timer to avoid random interrupts or possible resets
  wdt_reset();
  //wdt_disable();

  t = millis();
  do
  {
    //Check for input from radio
    if (manager.available())
    {
      id = 0;
      to = 0;
      len = sizeof(transmission);
      rssi = radio.lastRssi();

      manager.recvfrom((uint8_t *)&transmission, &len, &id, &to);

      if (len == sizeof(transmission))
      {
        //Store a copy of the packet to print after the data is processed
        trans = transmission;
        lastAspect = aspect;

        //Check that datagram's destination ID is this node or is the broadcast address and this is not a control code
        if (transmission.destination == NODEID && !transmission.isCode)
        {
          if (NODEID == 17 && id == DESTIDW)
          {
            //reset the packet's voltage to this nodes voltage
            //transmission.voltage = volt;

            //variable for analog return from LED to check for functionality
            led = 0;
            switch (transmission.aspect)
            {
            //Green case. Change head to green if it was not amber, otherwise start 8sec wait to go green
            case 'G':
              if (!transmission.isACK)
              {
                transmit(id, 'G', true, false);
              }
              else
              {
                transmit(20, 'G', true, false);
              }

              if (!AmberW || rwRetry)
              {
                digitalWrite(GREENW, LOW);
                digitalWrite(AMBERW, HIGH);
                digitalWrite(REDW, HIGH);
                LOCKW = false;

                aspectW = 'H';

                led = analogRead(GREENW);

                //Mark as changed to trigger a CTC update
                changedW = true;
              }

              //stop retrying to release
              rwRetry = false;
              retry = 0;

              GreenW = true;
              break;

            //Amber case. Set the signal head to amber, and set control locks to prevent other captures
            case 'A':
              if (!transmission.isACK)
              {
                transmit(id, 'R', true, false);
              }
              else
              {
                transmit(20, 'R', true, false);
              }
              digitalWrite(GREENW, HIGH);
              digitalWrite(AMBERW, LOW);
              digitalWrite(REDW, HIGH);
              AmberW = true;
              GreenW = false;
              LOCKW = true;

              //stop retrying to capture
              cRetry = false;
              retry = 0;

              aspectW = 'B';

              led = analogRead(AMBERW);

              dimCount = 0;
              dimTime = millis();

              //Mark as changed to trigger a CTC update
              changedW = true;
              break;

            //Red case. Set head to red and set control lock to prevent captures
            case 'R':
              if (!transmission.isACK)
              {
                transmit(id, 'A', true, false);
              }
              else
              {
                transmit(20, 'R', true, false);
              }
              digitalWrite(GREENW, HIGH);
              digitalWrite(AMBERW, HIGH);
              digitalWrite(REDW, LOW);
              GreenW = false;
              AmberW = false;
              LOCKW = true;

              if (cRetry && sState == 1)
              {
                cRetry = false;
              }

              rwRetry = false;

              retry = 0;

              aspectW = 'Q';

              led = analogRead(REDW);

              dimCount = 0;
              dimTime = millis();

              changedW = false;
              break;
            }

            sleepWait = millis();
            startRX();

            #ifdef DEBUG
            if (transmission.isACK)
            {
              Serial.print("Response time: ");Serial.println(millis() - t);
            }

            Serial.print("LED: ");Serial.println(led);
            #endif
            if (led < 50 || led > 500)
            {
              outW = true;
            }
            else
            {
              outW = false;
            }
          }
          else
          {
            //variable for analog return from LED to check for functionality
            led = 0;
            switch (transmission.aspect)
            {
            //Green case. Change head to green if it was not amber, otherwise start 8sec wait to go green
            case 'G':
              if (!transmission.isACK)
              {
                transmit(id, 'G', true, false);
              }
              if (!Amber || rRetry)
              {
                digitalWrite(GREEN, LOW);
                digitalWrite(AMBER, HIGH);
                digitalWrite(RED, HIGH);
                LOCK = false;

                aspect = 'G';

                led = analogRead(GREEN);

                //Mark as changed to trigger a CTC update
                changed = true;
              }

              if ((NODEID == 18 || NODEID == 17) && id != 21 && transmission.isACK)
              {
                transmit(21, 'G', true, false);
              }
              else if (NODEID == 19 && id != 20 && transmission.isACK)
              {
                transmit(20, 'G', true, false);
              }

              //stop retrying to release
              rRetry = false;
              retry = 0;

              if ((id == 20 || id == 21) && NODEID != 17)
              {
                rRetry = true;
              }

              Green = true;
              dwarfLock = false;
              break;

            //Amber case. Set the signal head to amber, and set control locks to prevent other captures
            case 'A':
              if (!dwarfLock)
              {
                if (!transmission.isACK)
                {
                  transmit(id, 'R', true, false);
                }
                digitalWrite(GREEN, HIGH);
                digitalWrite(AMBER, LOW);
                digitalWrite(RED, HIGH);
                Amber = true;
                Green = false;
                LOCK = true;

                aspect = 'A';

                led = analogRead(AMBER);

                if ((NODEID == 18 || NODEID == 17) && transmission.isACK && !dwarfLock)
                {
                  transmit(21, 'R', true, false);
                }
                else if (NODEID == 19 && transmission.isACK && !dwarfLock)
                {
                  transmit(20, 'R', true, false);
                }

                dimCount = 0;
                dimTime = millis();
              }
              else
              {
                transmit(id, 'R', true, false);
              }
              //stop retrying to capture
              cRetry = false;

              //Mark as changed to trigger a CTC update
              changed = true;

              retry = 0;
              break;

            //Red case. Set head to red and set control lock to prevent captures
            case 'R':
              dwarfC = false;
              if (NODEID == 21 || NODEID == 20)
              {
                dwarfC = cRetry;
              }
              if ((Green || !(Green || Amber)) && !dwarfC && !(NODEID > id && cRetry))
              {
                if (!transmission.isACK)
                {
                  transmit(id, 'A', true, false);
                }
                digitalWrite(GREEN, HIGH);
                digitalWrite(AMBER, HIGH);
                digitalWrite(RED, LOW);
                Green = false;
                Amber = false;
                LOCK = true;

                if (cRetry && (NODEID == 21 || id > NODEID || (NODEID == 17 && sState == 0)))
                {
                  cRetry = false;
                }

                rRetry = false;

                retry = 0;

                aspect = 'R';

                led = analogRead(RED);
              }
              else
              {
                transmit(id, 'R', true, false);
              }

              if ((NODEID == 18 || NODEID == 17) && id != 21 && transmission.isACK && !dwarfLock)
              {
                transmit(21, 'R', true, false);
              }
              else if (NODEID == 19 && id != 20 && transmission.isACK && !dwarfLock)
              {
                transmit(20, 'R', true, false);
              }

              dimCount = 0;
              dimTime = millis();

              changed = false;

              if ((id == 20 || id == 21) && NODEID != 17)
              {
                cRetry = true;
                LOCK = false;
                dwarfLock = true;
              }
              break;
            }

            sleepWait = millis();
            startRX();

            #ifdef DEBUG
            if (transmission.isACK)
            {
              Serial.print("Response time: ");Serial.println(millis() - t);
            }

            Serial.print("LED: ");Serial.println(led);
            #endif

            if (led < 50 || led > 500)
            {
              out = true;
            }
            else
            {
              out = false;
            }
          }
        }
        else if ((NODEID == 21 && (id == 18 || (id == 17 && trans.destination == 18))) || (NODEID == 20 && (id == 19 || (id == 17 && trans.destination == 19))))
        {
          led = 0;
          switch (trans.aspect)
          {
            //Green case. Change head to green if it was not amber, otherwise start 8sec wait to go green
            case 'g':
            case 'G':
              digitalWrite(GREEN, LOW);
              digitalWrite(AMBER, HIGH);
              digitalWrite(RED, HIGH);
              LOCK = false;

              aspect = 'G';

              led = analogRead(GREEN);

              //Mark as changed to trigger a CTC update
              changed = true;

              //stop retrying to release
              rRetry = false;
              retry = 0;

              Green = true;
              dimCount = 0;
              dimTime = millis();
              break;

            case 'a':
            case 'A':
            case 'r':
            //Red case. Set head to red and set control lock to prevent captures
            case 'R':
              if (!Amber && !cRetry)
              {
                digitalWrite(GREEN, HIGH);
                digitalWrite(AMBER, HIGH);
                digitalWrite(RED, LOW);
                Green = false;
                Amber = false;
                LOCK = true;
                cRetry = false;
                rRetry = false;

                retry = 0;

                aspect = 'R';

                led = analogRead(RED);

                changed = false;              

                dimCount = 0;
                dimTime = millis();
              }
              break;
          }

          if (led < 50 || led > 500)
          {
            out = true;
          }
          else
          {
            out = false;
          }
        }

        //restart receive mode to catch any new incoming data
        startRX();

        //If the tranmission was a control code and the destination was this node or the broadcast address
        if (trans.isCode && (trans.destination == NODEID || trans.destination == RH_BROADCAST_ADDRESS))
        {
          switch (transmission.aspect)
          {
          //remote capture
          case 'C':
            cRetry = true;
            rRetry = false;
            rwRetry = false;
            retry = 0;
            break;

          //remote release
          case 'D':
            rRetry = true;
            if (NODEID == 17)
            {
              rwRetry = true;
            }
            cRetry = false;
            retry = 0;
            break;

          //wake if asleep, and report back to CTC
          case 'W':
            led = 0;
            if (aspect == 'G' || aspect == 'X')
            {
              digitalWrite(GREEN, LOW);
              aspect = 'G';
              led = analogRead(GREEN);

              if (led < 50 || led > 500)
              {
                out = true;
              }
              else
              {
                out = false;
              }
            }
            if (NODEID == 17 && (aspectW == 'H' || aspectW == 'X'))
            {
              digitalWrite(GREENW, LOW);
              aspectW = 'H';

              led = analogRead(GREENW);

              if (led < 50 || led > 500)
              {
                outW = true;
              }
              else
              {
                outW = false;
              }
            }

            if (id == CTC)
            {
              changed = true;
              if (NODEID == 17)
              {
                changedW = true;
                sw1Changed = true;
              }
              else if (NODEID == 18)
              {
                sw1Changed = true;
              }
              else if (NODEID == 19)
              {
                sw1Changed = true;
                sw2Changed = true;
              }
            }

            cRetry = false;
            rRetry = false;
            rwRetry = false;

            transmission.isCode = false;

            dimCount = 0;
            dimTime = millis();
            retry = 0;
            break;
          }
          transmission.isCode = false;
        }
        else if (shouldWake(id, trans.destination, trans.aspect))
        {
          led = 0;
          digitalWrite(GREEN, LOW);
          aspect = 'G';
          Green = true;

          led = analogRead(GREEN);
          if (led < 50 || led > 500)
          {
            out = true;
          }
          else
          {
            out = false;
          }

          if (NODEID == 17 && (aspectW == 'H' || aspectW == 'X'))
          {
            digitalWrite(GREENW, LOW);
            aspectW = 'H';

            GreenW = true;

            led = analogRead(GREENW);

            if (led < 50 || led > 500)
            {
              outW = true;
            }
            else
            {
              outW = false;
            }
          }
          changed = true;

          transmission.isCode = false;

          dimCount = 0;
          dimTime = millis();
          sleepWait = millis();
        }

        Serial.print("Destination: ");Serial.println(trans.destination);
        Serial.print("Sender: ");Serial.println(id, DEC);
        Serial.print("Is ACK: ");Serial.println(trans.isACK ? "TRUE" : "FALSE");
        Serial.print("Is Code: ");Serial.println(trans.isCode ? "TRUE" : "FALSE");
        Serial.print("Aspect: ");Serial.println(trans.aspect);
        //Print receive signal strength from received packet
        Serial.print("[RX_RSSI:");Serial.print(rssi);Serial.println("]");
      }

      //Data packet of 1 length is ACK
      else if (len == 1 && to == NODEID)
      {
        #ifdef DEBUG
        Serial.println("Radio ACK Received");
        Serial.print('[');Serial.print(id, DEC);Serial.print("] ");Serial.println(rssi);
        #endif

        startRX();
        if (changed)
          changed = false;
        else if (changedW)
          changedW = false;
        else if (sw1Changed)
          sw1Changed = false;
        else if (sw2Changed)
          sw2Changed = false;
        retry = 0;
      }
    }
  } while ((((retry > 0) && (cRetry || rRetry || rwRetry || changed || changedW || sw1Changed || sw2Changed)) && ((t + LISTEN) >= millis())));

  //After a set number retries, give up
  if (retry >= RETRIES)
  {
    cRetry = false;
    rRetry = false;
    rwRetry = false;
    if (changed)
      changed = false;
    else if (changedW)
      changedW = false;
    else if (sw1Changed)
      sw1Changed = false;
    else if (sw2Changed)
      sw2Changed = false;
    retry = 0;
  }

  if ((NODEID == 17 || NODEID == 18 || NODEID == 19) && !cRetry)
  {
    state = digitalRead(SWITCH);
    if (state != sState)
    {
      sw1Changed = true;

      #ifdef DEBUG
      Serial.println(state);
      #endif
    }

    sState = state;
  }
  if (NODEID == 19)
  {
    state = digitalRead(SWITCH2);
    if (state != sState2)
    {
      sw2Changed = true;
    }
    sState2 = state;
  }

  wake(); //check switches even if interrupt did not occur

  if (Serial.available())
  {
    in = Serial.read();
    if (in == 'C' || in == 'c')
    {
      Serial.println("CAPTURE");
      cRetry = true;
    }
    if (in == 'R' || in == 'r')
    {
      Serial.println("RELEASE");
      rRetry = true;
    }
  }

  //If the capture switch was hit and the LOCK interlock bit is not set
  //Set up datagram for the other head and transmit it
  if (cRetry)
  {
    //wake head if it was asleep to confirm functionality
    if (aspect == 'X')
    {
      aspect = 'G';
      Green = true;
      dimCount = 0;
      dimTime = millis();
      digitalWrite(GREEN, LOW);
    }

    if (NODEID == 17 && sState == 1 && !LOCKW)
    {
      #ifdef DEBUG
      Serial.print("Sending capture West ");
      #endif

      transmit(DESTIDW, 'R', false, false);

      rwRetry = false;
    }
    else if ((NODEID != 17 && !LOCK) || (NODEID == 17 && sState == 0 && !LOCK))
    {
      #ifdef DEBUG
      Serial.println("Sending capture ");
      #endif

      transmit(DESTID, 'R', false, false);

      rRetry = false;
    }

    changed = false;
    retry++;
  }

  //If the release switch was hit
  //Ignore if the aspect is red unless its been red for  or more cycles, ~24 sec
  //Set up datagram for the other head and transmit it
  else if (rRetry)
  {
    #ifdef DEBUG
    Serial.println("Sending release");
    #endif

    transmit(DESTID, 'G', false, false);

    changed = false;
    if (NODEID != 17 || sState == 1)
    {
      cRetry = false;
    }
    rLock = true;
    retry++;
  }

  //If the release switch was hit
  //Ignore if the aspect is red unless its been red for 3 or more cycles, ~24 sec
  //Set up datagram for the other head and transmit it
  else if (NODEID == 17 && rwRetry)
  {
    #ifdef DEBUG
    Serial.println("Sending release w");
    #endif

    transmit(DESTIDW, 'G', false, false);

    changed = false;
    if (sState == 0)
    {
      cRetry = false;
    }
    rwLock = true;
    retry++;
  }

  if ((dimTime + 8000) < millis())
  {
    dimCount++;
    dimTime = millis();

    #ifdef DEBUG
    Serial.println("dim++");
    #endif
  }

  //If Amber and Green bits are set and the watchdog timer has made 1 pass
  //Return head to green and reset Amber, Green, and LOCK bits
  if ((dimCount >= 1) && Amber && Green)
  {
    digitalWrite(GREEN, LOW);
    digitalWrite(AMBER, HIGH);
    digitalWrite(RED, HIGH);
    Amber = false;
    Green = true;
    LOCK = false;

    aspect = 'G';
    changed = true;
  }
  if (NODEID == 17 && (dimCount >= 1) && AmberW && GreenW)
  {
    digitalWrite(GREENW, LOW);
    digitalWrite(AMBERW, HIGH);
    digitalWrite(REDW, HIGH);
    AmberW = false;
    GreenW = true;
    LOCKW = false;

    aspectW = 'H';
    changed = true;
  }

  //If the watchdog time has made the set number of cycles and the head is green, power down the signal head
  if (NODEID != 17 && dimCount >= DIM && aspect == 'G')
  {
    #ifdef DEBUG
    Serial.println("Dimming...");
    #endif
    digitalWrite(GREEN, HIGH);

    aspect = 'X';
    changed = true;
    Green = false;
    Amber = false;

    retry = 0;
  }
  else if (aspect == 'G' && aspectW == 'H' && NODEID == 17 && dimCount >= DIM)
  {
    #ifdef DEBUG
    Serial.println("Dimming...");
    #endif
    digitalWrite(GREEN, HIGH);
    digitalWrite(GREENW, HIGH);

    aspect = 'X';
    aspectW = 'X';
    changed = true;
    Green = true;
    GreenW = true;
    Amber = false;
    AmberW = false;
    out = false;
    outW = false;

    retry = 0;
  }

  //for two head boards, if one head is awake and the other isnt, wake the asleep one up.
  if (NODEID == 17 && aspect == 'X' && aspectW != 'X')
  {
    digitalWrite(GREEN, LOW);
    aspect = 'G';
    Green = true;
  }
  else if (NODEID == 17 && aspect != 'X' && aspectW == 'X')
  {
    digitalWrite(GREENW, LOW);
    aspectW = 'H';
    GreenW = true;
  }

  //if the head has been red for a set number of sleep cycles, release the block automatically
  if (dimCount >= AUTORELEASE && (aspect == 'R' || aspect == 'A'))
  {
    rRetry = true;
  }
  if (dimCount >= AUTORELEASE && (aspectW == 'Q' || aspectW == 'B') && NODEID == 17)
  {
    rwRetry = true;
  }

  //set radio into recieve mode before sleeping or looping again
  startRX();

  if (sleepWait > millis())
  {
    sleepWait = millis();
  }

  if ((sleepWait + 2500) < millis())
  {
    //If something has changed, send an update to the CTC board
    if (changed)
    {
      #ifdef DEBUG
      Serial.println("Updating CTC");
      #endif

      transmit(CTC, aspect, false, out);

      retry++;
      sleepWait = millis();
    }
    else if (changedW)
    {
      transmit(CTC, aspectW, false, outW);

      retry++;
      sleepWait = millis();
    }
    else if (sw1Changed)
    {
      if (sState == 0)
      {
        transmit(CTC, 's', false, false);
      }
      else if (sState == 1)
      {
        transmit(CTC, 'S', false, false);
      }

      retry++;
      sleepWait = millis();
    }
    else if (sw2Changed)
    {
      if (sState2 == 0)
      {
        transmit(CTC, 't', false, false);
      }
      else if (sState2 == 1)
      {
        transmit(CTC, 'T', false, false);
      }

      retry++;
      sleepWait = millis();
    }

    //return radio to recieve mode
    startRX();
  

    if (dimCount >= DIM && !(cRetry || rRetry || rwRetry || changed || changedW || sw1Changed || sw2Changed))
    {
      //Reset flags
      MCUSR = 0;

      //Set watchdog timer to program mode
      WDTCSR = bit(WDCE) | bit(WDE);
      //Set watchdog timer to run for ~8s and set for interrupt, not reset
      WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0);

      //Set sleep mode and attach interrupt to see if anything happens with the capture or release switches
      //Radio library already has its interrupts built in
      //Then put the arduino into deep sleep to save power
      attachInterrupt(digitalPinToInterrupt(BUTTON), wake, HIGH);
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      
      Serial.println("Entering Sleep Mode");
      
      //Delay to allow Serial buffer to clear
      delay(100);
      sleep_mode();
      //If sleep fails, disable sleep mode, power up peripherals
      sleep_disable();
      power_all_enable();
    }
  }
}

//Fuction to set the appropriate packet variables and then send the transmission
void transmit(uint8_t dest, char asp, bool ack, bool code)
{
  transmission.destination = dest;
  transmission.aspect = asp;
  transmission.isACK = ack;
  transmission.isCode = code;

  manager.sendto((uint8_t *)&transmission, sizeof(transmission), RH_BROADCAST_ADDRESS);
}

//Get address from binary setting of DIP switches or solder joints
int bitID()
{
  int address = 1;

  /*
   * AnalogRead only for Moteinos!
   * DigitalRead doesnt work on analog pins
   */
  if (analogRead(ID0) < 25)
  {
    address = address + 1;
    Serial.print("ID0:");Serial.println(analogRead(ID0));
  }
  if (analogRead(ID1) < 25)
  {
    address = address + 2;
    Serial.print("ID1:");Serial.println(analogRead(ID1));
  }
  if (analogRead(ID2) < 25)
  {
    address = address + 4;
    Serial.print("ID2:");Serial.println(analogRead(ID2));
  }
  if (analogRead(ID3) < 25)
  {
    address = address + 8;
    Serial.print("ID3:");Serial.println(analogRead(ID3));
  }
  if (digitalRead(ID4) == LOW)
  {
    address = address + 16;
    Serial.print("ID4:");Serial.println(digitalRead(ID4));
  }

  return address;
}

/*Function to put the radio in receive mode if it is not already
 * Prevents data loss from overcalling radio.receiveDone()
 */
void startRX()
{
  radio.setModeRx();
}

void POST()
{
  pinMode(CAPTURE, OUTPUT);
  pinMode(RELEASE, OUTPUT);

  digitalWrite(CAPTURE, HIGH);
  if (digitalRead(BUTTON) == HIGH)
  {
    Serial.println("CAPTURE Passed");
  }
  else
  {
    Serial.println("CAPTURE Failed");
  }
  digitalWrite(CAPTURE, LOW);

  digitalWrite(RELEASE, HIGH);
  if (digitalRead(BUTTON) == HIGH)
  {
    Serial.println("RELEASE Passed");
  }
  else
  {
    Serial.println("RELEASE Failed");
  }
  digitalWrite(RELEASE, LOW);

  if (NODEID == 17)
  {
    pinMode(RELEASEW, OUTPUT);

    digitalWrite(RELEASEW, HIGH);
    if (digitalRead(BUTTON) == HIGH)
    {
      Serial.println("RELEASEW Passed");
    }
    else
    {
      Serial.println("RELEASEW Failed");
    }
    digitalWrite(RELEASEW, LOW);
  }
}

bool shouldWake(uint8_t s, uint8_t d, char a)
{
  rtn = false;

  switch(NODEID)
  {
    case 1:
          if((s == 17 && d != CTC && a == 'A') || (d == 17 && a == 'R')) rtn = true;
          break;

    case 2: 
          if((s == 17 && d != CTC && a == 'A') || (d == 17 && a == 'R')) rtn = true;
          break;

    case 17:
          if((s == 1 && d != CTC && a == 'A') || (d == 1 && a == 'R')) rtn = true;
          break;

    case 18:
          if((s == 19 && d != CTC && a == 'A') || (d == 19 && a == 'R') || (s == 1 && d != CTC && a == 'A') || (d == 1 && a == 'R')) rtn = true;
          break;
    
    case 19:
          if((s == 18 && d != CTC && a == 'A') || (d == 18 && a == 'R') || (s == 1 && d != CTC && a == 'A') || (d == 1 && a == 'R')) rtn = true;
          break;                                          
  }

  return rtn;  
}
