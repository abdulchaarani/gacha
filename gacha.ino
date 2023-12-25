#include <Servo.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "ColorPalette.h"

#define COIN_SENSOR A3
#define STOP_SENSOR A4
#define SERVO_PIN 4
#define DFP_RX 10
#define DRP_TX 11

SoftwareSerial mySoftwareSerial(DFP_RX, DRP_TX); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
const int nSongs = 10;
const int volume = 20;
int currentSong = 1;
int victorySong = 9999;

Servo myServo;
const int forwardServoValue = 1300;
const int backwardServoValue = 2000;
const int stopServoValue = 90;
const int threshold = 100;
int coinSensorValue = 0;
int stopSensorValue = 0;

volatile int counter = 0;
volatile int seconds = 0;

bool toggleLed = false;

inline const void setupDFP(){
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  
  // Serial.println();
  // Serial.println(F("DFRobot DFPlayer Mini Demo"));
  // Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    // Serial.println(F("Unable to begin:"));
    // Serial.println(F("1.Please recheck the connection!"));
    // Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  // Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(volume);  //Set volume value. From 0 to 30
  // myDFPlayer.play(2);  //Play the first mp3
  // myDFPlayer.play(random(0, nSongs + 1));
  // myDFPlayer.next();
  // myDFPlayer.stop();
}

inline const void forwardServo(const Servo& servo){
    servo.attach(SERVO_PIN);
    servo.writeMicroseconds(forwardServoValue);
}

inline const void backwardServo(const Servo& servo){
    servo.attach(SERVO_PIN);
    servo.writeMicroseconds(backwardServoValue);
}

inline const void stopServo(const Servo& servo){
    servo.write(stopServoValue);
    servo.detach();
}

void timerSetup(){
  // servo uses timer 1 so we use timer 2
  cli(); // Disable all interrupts
  TCCR2A = (1 << WGM21); // CTC mode
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Frequency 8Mhz / 1024 = 7813 tics/sec 
  OCR2A = 255; // compare value: 8 bit register max value        
  TIMSK2 = (1 << OCIE2A);  // Enable timer interrupts
  sei(); // Enable all interrupts
}

ISR(TIMER2_COMPA_vect){          // timer compare interrupt service routine
  counter++;
  if (counter % 32 == 0)
    seconds++;
}


enum class State{
  IDLE,
  START_SONG,
  WAIT,
  FORWARD,
  BACKWARD,
  STOP,
  VICTORY,
};

State state = State::IDLE;

void checkState(){
  switch(state){
    case State::IDLE:
      seconds = 0;
      // Serial.println("idle");
      if (coinSensorValue > threshold)
        state = State::START_SONG;
      break;

    case State::START_SONG:
      // Serial.println("start");
      // delay(5000);
      state = State::WAIT;
      break;

    case State::WAIT:
      if (seconds == 5){
        seconds = 0;
        state = State::FORWARD;
      }
      break;  

    case State::FORWARD:
      // Serial.println("forward");
      if (seconds == 6){
        seconds = 0;
        state = State::BACKWARD;
      }
      else if (stopSensorValue > threshold){
        seconds = 0;
        state = State::VICTORY;
      }
      break;

    case State::BACKWARD:
      if (seconds == 6){
        seconds = 0;
        state = State::FORWARD;
      }
      else if (stopSensorValue > threshold){
        seconds = 0;
        state = State::VICTORY;
      }
      break;

    case State::VICTORY:
      state = State::STOP;
      break;

    case State::STOP:
    if (seconds == 1)
      state = State::IDLE;
    break;
  }
}

void setup() {
  ledSetup();
  FastLED.clear();
  FastLED.show();

  setupDFP();
  pinMode(COIN_SENSOR, INPUT);
  pinMode(STOP_SENSOR, INPUT);
  timerSetup();
  Serial.begin(9600);
}

void loop() {
  if (toggleLed)
    ledLoop();

  // Output
  switch(state){
    case State::IDLE:
      break;

    case State::START_SONG:
    myDFPlayer.playMp3Folder(currentSong);
    delay(50); // to increment song
    currentSong++;
    if (currentSong > nSongs){
      currentSong = 1;
    }
    toggleLed = true;
    break;

    case State::WAIT:
    break;

    case State::FORWARD:
      forwardServo(myServo);
      break;

    case State::BACKWARD:
      backwardServo(myServo);
      break;

    case State::VICTORY:
      myDFPlayer.playMp3Folder(victorySong);
      delay(10);
      break;

    case State::STOP:
      stopServo(myServo);
      delay(10);
      toggleLed = false;
      FastLED.clear();
      FastLED.show();
      break;
  }


  coinSensorValue = analogRead(COIN_SENSOR);
  stopSensorValue = analogRead(STOP_SENSOR);

  checkState();
}