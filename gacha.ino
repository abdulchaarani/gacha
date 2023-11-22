#include <Servo.h>

#define COIN_SENSOR A3
#define STOP_SENSOR A4
#define SERVO_PIN 9

Servo myServo;
const int forwardServoValue = 1300;
const int backwardServoValue = 2000;
const int stopServoValue = 90;
const int threshold = 100;
int coinSensorValue = 0;
int stopSensorValue = 0;

volatile int counter = 0;
volatile int seconds = 0;

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
        state = State::FORWARD;
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

    case State::FORWARD:
      forwardServo(myServo);
      break;

    case State::BACKWARD:
      backwardServo(myServo);
      break;

    case State::STOP:
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