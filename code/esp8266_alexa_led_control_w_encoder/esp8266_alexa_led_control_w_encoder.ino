// Alexa voice control of an LED or simple LED strip

#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

#include "fauxmoESP.h"
#include <RotaryEncoder.h>

// Rename the credentials.sample.h file to credentials.h and 
// edit it according to your router configuration
#include "credentials.h"

#define DEV1 "Beverage" // Name that Alexa will use
#define LED_PIN 4 // Arduino GPIO4 is NodeMCU pin D2
//#define LED_PIN 16 // Blue LED on board.
#define buttonPin 0    
#define MAX_PWM 255
#define bounce_interval = 20

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// Example for Arduino UNO with input signals on pin 2 and 3
#define PIN_IN1 A2
#define PIN_IN2 A3
#elif defined(ESP8266)
// Example for ESP8266 NodeMCU with input signals on pin D5 and D6
#define PIN_IN1 D5
#define PIN_IN2 D6
#endif
                                                         
fauxmoESP fauxmo;
int v = 128;
bool ledState = LOW;        // the current state of the output pin
bool buttonState = 0;            // the current reading from the input pin
bool lastButtonState = HIGH;  // the previous reading from the input pin
bool first_button_press = 1;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 20;    // the debounce time; increase if the output flickers

int direction = 0;
// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// This interrupt routine will be called on any change of one of the input signals
void checkPosition() {
  encoder->tick();  // just call tick() to check the state.
}
#elif defined(ESP8266)
/**
 * The interrupt service routine will be called on any change of one of the input signals.
 */
IRAM_ATTR void checkPosition() {
  encoder->tick();  // just call tick() to check the state.
}
#endif

void setup() {

  // LED SETUP
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(LED_PIN,OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize the PWM pin
  analogWriteRange(MAX_PWM); // Set the PWM range (0-255)
  analogWriteFreq(1000); // Set the PWM frequency (in Hz)
  analogWrite(LED_PIN,0);
                
  Serial.begin(115200);

  setup_wifi();
  Serial.println("\n\nfauxmo = Control by Alexa for Arduino");

  fauxmo.setPort(80);
  fauxmo.enable(true);
  fauxmo.addDevice(DEV1);

  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {

      // Callback when a command from Alexa is received.
      // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
      // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
      // Just remember not to delay too much here, this is a callback, exit as soon as possible.
      // If you have to do something more involved here set a flag and process it in your main loop.

      //Serial.println("DEV1 Addressed");
      Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

      // Checking for device_id is simpler if you are certain about the order they are loaded and it does not change.
      // Otherwise comparing the device_name is safer.
    if (strcmp(device_name, DEV1)==0) {   // Dimmable LED or LED string
      if  (state==LOW) {
        analogWrite(LED_PIN,0);
        ledState = LOW;
      }
      else  {  // Control PWM with gamma correction.
            v = ((int)  ( (float)MAX_PWM * pow( (float)value / MAX_PWM, 1) )); // 2.2 applies gamma correction
            //v = (int) ((float) MAX_PWM * ((float) value / (float) MAX_PWM) );   
            analogWrite(LED_PIN,v);
            ledState = HIGH;
            //char buffer[25];
            //sprintf(buffer, "The PWM value is: %d", 255 - v);
            //Serial.println(buffer);
      }
    }
  });// end fauxmo.onSetState()  //last_toggle_status = read_toggle_switch();  // assumes this is a momentary switch
  
  // setup the rotary encoder functionality

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
   encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  //encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE); 

}// end setup()

void loop() {

  static unsigned long lastHeapTime;
  unsigned long currentMillis = millis();

  // fauxmoESP uses an async TCP server but a sync UDP server
  // Therefore, we have to manually poll for UDP packets
 fauxmo.handle();  // fauxmo uses a callback so no additional code needed here for Alexa communications
 /*
  // This is a sample code to output free heap every 5 seconds
  // This is a cheap way to detect memory leaks
  if (currentMillis - lastHeapTime > 5000){
    lastHeapTime = currentMillis;
    //Serial.printf("[MAIN] Free heap: % d bytes\n", ESP.getFreeHeap());
    //Serial.println( ledState);
  }
*/
  // Manually toggle the LED by a momentary button press
  buttonState = digitalRead(buttonPin);
  //Serial.print("buttonState: ");
  //Serial.println(buttonState);
  if (buttonState == !lastButtonState && currentMillis - lastDebounceTime > debounceDelay) {
   /* if (first_button_press = 1) {
      v=50;  //fauxmo.setState("DEV1", true, v);
      first_button_press = 0;
    } */
    lastDebounceTime = currentMillis;
    //lastButtonState = buttonState;
    // If the button has just gone HIGH, turn on LED
    if (ledState == HIGH) {
      analogWrite(LED_PIN, 0);
      ledState = LOW;
      fauxmo.setState(DEV1, false, v);
    }
    else {
      analogWrite(LED_PIN, v);
      ledState = HIGH;
      fauxmo.setState(DEV1, true, v);
    }
    Serial.print("ledState: ");
    Serial.println(ledState); 
    Serial.print("v: ");
    Serial.println(v);
    lastButtonState = 1;
    delay(800);
  }

  // Read the current position of the encoder and print out when changed
  static int pos = 0;
  encoder->tick();  // just call tick() to check the state.

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    direction = (int) encoder->getDirection();
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println(direction);
    if (direction == -1) {
      if (v - 10 >= 2) {
        v = v - 10;
        fauxmo.setState(DEV1, true, v);
      }
      else {
        v=2;
        fauxmo.setState(DEV1, true, v);
      }
      analogWrite(LED_PIN, v);
      Serial.print("v after -rotate = ");
      Serial.println(v);
    }
    else {
      if (v + 10 <= 255) v = v + 10;
      else {
        v = 255;
      }
      analogWrite(LED_PIN, v);
      fauxmo.setState(DEV1, true, v-1);
      Serial.print("v after +rotate = ");
      Serial.println(v);
    }
    Serial.print("Brilliance after rotate = ");
    Serial.println((int) round((v*((float) 100)/(float) 255)));    
    pos = newPos;
  }
}

void setup_wifi(void) {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  for (int i = 1; i <=2; i+=1 ) {
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED ON (LOW of ESP is equivalent of HIGH to on board LED)
    delay(333);                      // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage LOW
    delay(333);
  }
  digitalWrite(LED_BUILTIN, LOW);
}
