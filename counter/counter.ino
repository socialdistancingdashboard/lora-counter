#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define LEDPIN 2
#define START 0

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonPin1 =  12;
const int buttonPin2 = 14;
const int buttonResetPin = 13;
int buttonState1 = 0;
int buttonState2 = 0;         
int buttonResetState = 0; 

char TTN_response[30];

volatile int counter = START;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes.

// Copy the value from Device EUI from the TTN console in LSB mode.
static const u1_t PROGMEM DEVEUI[8]= {  };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// Copy the value from Application EUI from the TTN console in LSB mode
static const u1_t PROGMEM APPEUI[8]= {  };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is. Anyway its in MSB mode.
static const u1_t PROGMEM APPKEY[16] = {  };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[5];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 180;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void do_send(osjob_t* j){
    uint8_t payload[2];
    payload[0] = counter >> 8;
    payload[1] = counter & 0xFF;


    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (uint8_t*) payload, sizeof(payload), 0);
        Serial.println(F("Sending uplink packet..."));
        digitalWrite(LEDPIN, HIGH);       
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
           case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));            

            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));            
            }

            if (LMIC.dataLen) {
              int i = 0;
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
              Serial.println(LMIC.rssi);              
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            digitalWrite(LEDPIN, LOW);      
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            lcd.clear();
            lcd.print("OTAA joining...");            
            lcd.clear();
            break;
        case EV_JOINED: {
              Serial.println(F("EV_JOINED"));
              lcd.clear();
              lcd.print("Joined!");
              // Disable link check validation (automatically enabled
              // during join, but not supported by TTN at this time).
              LMIC_setLinkCheckMode(0);
            }
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }

}

void setup() {
  Serial.begin(115200);
  delay(2500);  
  Serial.println(F("Starting..."));
  pinMode(LEDPIN,OUTPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0 ,0);
  lcd.print(counter);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(DR_SF11,14);
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  do_send(&sendjob);     // Will fire up also the join
  //LMIC_startJoining();
}

void loop() {
  os_runloop_once();
  //##################BUTTON 1##################################################
  if (debounceButton(buttonState1) == HIGH && buttonState1 == LOW) 
  {
    lcd.clear();
    counter++;
    lcd.print(counter);
    lcd.setCursor(1 ,0);
    buttonState1 = HIGH; 
   }
  else if (debounceButton(buttonState1) == LOW && buttonState1 == HIGH)
  {
    buttonState1 = LOW;
  }
   //##################BUTTON 2##################################################
  if (digitalRead(buttonPin2) == HIGH && buttonState2 == LOW) 
  {
    lcd.clear();
    counter--;
    lcd.print(counter);
    lcd.setCursor(1 ,0);
    buttonState2 = HIGH;
    //delay(10); 
   }
  else if (digitalRead(buttonPin2) == LOW && buttonState2 == HIGH)
  {
    buttonState2 = LOW;
    delay(10);
  }


     /////////////////////////////////BUTTON 3////////////////////////////////////////////////
  if (digitalRead(buttonResetPin ) == HIGH && buttonResetState == LOW) 
  {
    lcd.clear();
    lcd.setCursor(0 ,0);
    lcd.print("5 Sek halten");
    lcd.setCursor(0 ,1);
    lcd.print("fuer reset");
      for (int i = 0; i <= 5; i++) {
      lcd.setCursor(11 ,1);
      lcd.print(i);
      delay(1000);
      Serial.println(i);
        if (digitalRead(buttonResetPin) == LOW)
          break;
        if (i == 5){
          lcd.clear();
          lcd.setCursor(0 ,0);
          lcd.print("Reset");
          delay(1000);
          counter = START;
          setup();
          //Serial.println(i);
          }
  }
    buttonResetState = HIGH; 
   }
  else if (digitalRead(buttonResetPin) == LOW && buttonResetState == HIGH)
  {
    buttonResetState = LOW;
    delay(10);
  }


  
  //Serial.println(counter);
  }


boolean debounceButton(boolean state)
{
  boolean stateNow = digitalRead(buttonPin1);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(buttonPin1);
  }
  return stateNow;
  
}
