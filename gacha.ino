#include <Servo.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define COIN_SENSOR A3
#define STOP_SENSOR A4
#define SERVO_PIN 9
#define DFP_RX 10
#define DRP_TX 11

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
const int nSongs = 3;

Servo myServo;
const int forwardServoValue = 1300;
const int backwardServoValue = 2000;
const int stopServoValue = 90;
const int threshold = 100;
int coinSensorValue = 0;
int stopSensorValue = 0;

volatile int counter = 0;
volatile int seconds = 0;

inline const void setupDFP(){
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
  // myDFPlayer.play(2);  //Play the first mp3
  // myDFPlayer.play(random(0, nSongs + 1));
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
  FORWARD,
  BACKWARD,
  STOP
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
      seconds = 0;
      while(seconds < 10){
        delay(0);
        Serial.println("Waiting");
      }
      // state = State::FORWARD;
      break;

    case State::FORWARD:
      // Serial.println("forward");
      if (seconds == 6){
        seconds = 0;
        state = State::BACKWARD;
      }
      else if (stopSensorValue > threshold){
        seconds = 0;
        state = State::STOP;
      }
      break;

    case State::BACKWARD:
      if (seconds == 6){
        seconds = 0;
        state = State::FORWARD;
      }
      else if (stopSensorValue > threshold){
        seconds = 0;
        state = State::STOP;
      }
      break;
    
    case State::STOP:
    if (seconds == 1)
      state = State::IDLE;
    break;
  }
}

void setup() {
  setupDFP();
  pinMode(COIN_SENSOR, INPUT);
  pinMode(STOP_SENSOR, INPUT);
  timerSetup();
  Serial.begin(9600);
}

void loop() {
  
  // Output
  switch(state){
    case State::IDLE:
      stopServo(myServo);
      break;

    case State::START_SONG:
      myDFPlayer.next();

    case State::FORWARD:
      forwardServo(myServo);
      break;

    case State::BACKWARD:
      backwardServo(myServo);
      break;

    case State::STOP:
      myDFPlayer.stop();
      break;
  }


  coinSensorValue = analogRead(COIN_SENSOR);
  stopSensorValue = analogRead(STOP_SENSOR);

  checkState();
}


  // coinSensorValue = analogRead(COIN_SENSOR);
  // if (coinSensorValue > threshold)
  //   forwardServo(myServo);

  // stopSensorValue = analogRead(STOP_SENSOR);
  // Serial.println(stopSensorValue);
  // // delay(100);

  // if (stopSensorValue > threshold)
  //   stopServo(myServo);