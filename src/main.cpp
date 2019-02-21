/*
 * allTeensy Code Version 3.0.0
 * This code is specific to the teensy 3.5
 * 
 */

/*
 * REMOTE CONFIGURATION
 * 
 * Code starts in calibration mode, move sticks to extents to calibrate
 * Send any serial information to arduino to proceed once done
 * 
 * throttleOut, rollOut, pitchOut, and yawOut are the calibrated values.
 * throttleOut ranges from 0 to 100, while the rest range from -100 to 100.
 * NOTE: THIS CODE HAS YAW RANGE changed from 0 to 70 as per SmoothSteering Code.
 * 
 */

#include <Filters.h>
#include <AccelStepper.h>

// Interrupts configuration
#define CHANNEL1 36
#define CHANNEL2 35
#define CHANNEL3 34
#define CHANNEL4 33

// Stepper pin configurations
#define STEERDPIN 24
#define STEERPPIN 25
#define ACCELDPIN 26
#define ACCELPPIN 27
#define BRAKEDPIN 28
#define BRAKEPPIN 29

#define JITTERZONE 3

// Software limited steps for the steppers
#define MINSTEER -3000
#define MAXSTEER 3000
#define MINACCEL 0
#define MAXACCEL 700
#define MINBRAKE -1500
#define MAXBRAKE 0

// Encoder pin definitions
#define ENCODER 30
#define ENCODERD 31

// setting up steppers, accel and brakes requires additional timing delay,
// hence the arbitary pin 11, 12 as dir pin
AccelStepper steering(1, STEERPPIN, STEERDPIN);
AccelStepper accelerator(1, ACCELPPIN, ACCELDPIN);
AccelStepper brakes(1, BRAKEPPIN, BRAKEDPIN);

// Filters out changes faster that 0.5 Hz.
float filterFrequency = 0.5;

// Create a one pole (RC) lowpass filter
FilterOnePole steerFilter(LOWPASS, filterFrequency, 0.0);
FilterOnePole accelFilter(LOWPASS, filterFrequency);
FilterOnePole brakeFilter(LOWPASS, filterFrequency);

// Remote control variable setups
volatile int throttle = 0;
volatile int prev_throttle = 0;
volatile int roll = 0;
volatile int prev_roll = 0;
volatile int pitch = 0;
volatile int prev_pitch = 0;
volatile int yaw = 0;
volatile int prev_yaw = 0;

int throttleTop = 0;
int throttleBtm = 0;
int rollTop = 0;
int rollBtm = 0;
int pitchTop = 0;
int pitchBtm = 0;
int yawTop = 0;
int yawBtm = 0;

int throttleOut = 0;
int rollOut = 0;
int pitchOut = 0;
int yawOut = 0;

// Final sensor values to be used by the program
long sensorValueS;
long sensorValueB;
long sensorValueA;

// Variables for implementing a deadzone to avoid jitter
long sensorValueALast = 0;
long sensorValueBLast = 0;
long sensorValueSLast = 0;

// Encoder code
const int ticksPerRev = 500;
volatile long counter = 0;
float angle = 0;

// Function forward declaration
void motorInit();
void remoteInit();
void encoderInit();
void remoteCalc();
void remoteDebug();
void encoderCalc();
void remoteCalibration();
void remoteVal();
void rising1();
void falling1();
void rising2();
void falling2();
void rising3();
void falling3();
void rising4();
void falling4();
void encoderRising();

void setup(){
    Serial.begin(115200);
    delay(3000);
    motorInit();
    Serial.println("Start");
    remoteInit();
    encoderInit();
    pinMode(STEERDPIN, OUTPUT);
    pinMode(STEERPPIN, OUTPUT);
    pinMode(BRAKEDPIN, OUTPUT);
    pinMode(BRAKEPPIN, OUTPUT);
    pinMode(ACCELDPIN, OUTPUT);
    pinMode(ACCELPPIN, OUTPUT);
    yawOut = 0;
    steering.setCurrentPosition(steering.currentPosition());
}

void loop(){
    remoteCalc();
    // remoteDebug();
    // sensorValueS = map(analogRead(potPinS), 0, 1023, 0, 70);
    // encoderCalc();
    sensorValueS = steerFilter.input(yawOut);
    sensorValueA = accelFilter.input(throttleOut);
    sensorValueB = brakeFilter.input(pitchOut);
    
    if(sensorValueS > sensorValueSLast + JITTERZONE || sensorValueS < sensorValueSLast - JITTERZONE){
        // if(sensorValueS > steering.currentPosition() && digitalRead(STEERDPIN) == HIGH){
        //     digitalWrite(STEERDPIN, LOW);
        //     delayMicroseconds(10);
        // }
        // else if(sensorValueS < steering.currentPosition() && digitalRead(STEERDPIN) == LOW){
        //     digitalWrite(STEERDPIN, HIGH);
        //     delayMicroseconds(10);
        // }
        steering.moveTo(sensorValueS);
        sensorValueSLast = sensorValueS;
    }
    if(sensorValueA > sensorValueALast + JITTERZONE || sensorValueA < sensorValueALast - JITTERZONE){
        // if(sensorValueA > accelerator.currentPosition() && digitalRead(ACCELDPIN) == LOW){
        //     digitalWrite(ACCELDPIN, HIGH);
        //     delayMicroseconds(10);
        // }
        // else if(sensorValueA < accelerator.currentPosition() && digitalRead(ACCELDPIN) == HIGH){
        //     digitalWrite(ACCELDPIN, LOW);
        //     delayMicroseconds(10);
        // }
        accelerator.moveTo(sensorValueA);
        sensorValueALast = sensorValueA;
    }
    if(sensorValueB > sensorValueBLast + JITTERZONE || sensorValueB < sensorValueBLast - JITTERZONE){
        // if(sensorValueB > brakes.currentPosition() && digitalRead(BRAKEDPIN) == LOW){
        //     digitalWrite(BRAKEDPIN, HIGH);
        //     delayMicroseconds(10);
        // }
        // else if(sensorValueB < brakes.currentPosition() && digitalRead(BRAKEDPIN) == HIGH){
        //     digitalWrite(BRAKEDPIN, LOW);
        //     delayMicroseconds(10);
        // }
        brakes.moveTo(sensorValueB);
        sensorValueBLast = sensorValueB;
    }
    
    steering.run();
    accelerator.run();
    brakes.run();
    // Serial.print(sensorValueS);
    // Serial.print("\t");
    // Serial.print(steering.currentPosition());
    // Serial.print("\t");
    // Serial.print(sensorValueA);
    // Serial.print("\t");
    // Serial.println(accelerator.currentPosition());
}

// Initialisation and setup for steppers
void motorInit(){
    steering.setMaxSpeed(20000.0);
    steering.setAcceleration(20000.0);
    steering.setPinsInverted(true,false,false);
    steering.setMinPulseWidth(20);
    brakes.setMaxSpeed(10000.0);
    brakes.setAcceleration(10000.0);
    brakes.setMinPulseWidth(20);
    accelerator.setMaxSpeed(150000.0);
    accelerator.setAcceleration(150000.0);
    accelerator.setMinPulseWidth(20);
}

// Initialisation and setup of remote control
void remoteInit(){
    pinMode(CHANNEL1,INPUT);
    pinMode(CHANNEL2,INPUT);
    pinMode(CHANNEL3,INPUT);
    pinMode(CHANNEL4,INPUT);
    attachInterrupt(digitalPinToInterrupt(CHANNEL1), rising1, RISING);
    attachInterrupt(digitalPinToInterrupt(CHANNEL2), rising2, RISING);
    attachInterrupt(digitalPinToInterrupt(CHANNEL3), rising3, RISING);
    attachInterrupt(digitalPinToInterrupt(CHANNEL4), rising4, RISING);
    // remoteCalibration();
    remoteVal();
}

// calibration program for remote control
void remoteCalibration(){
    delay(500);
    throttleTop = throttleBtm = throttle;
    rollTop = rollBtm = roll;
    pitchTop = pitchBtm = pitch;
    yawTop = yawBtm = yaw;

    Serial.println("Calibrating... Move the sticks around...");
    Serial.println("Press any key to continue...");
    while(!Serial.available()){
        if(throttle > throttleTop){
            throttleTop = throttle;
        }
        if(throttle < throttleBtm){
            throttleBtm = throttle;
        }
        if(roll > rollTop){
            rollTop = roll;
        }
        if(roll < rollBtm){
            rollBtm = roll;
        }
        if(pitch > pitchTop){
            pitchTop = pitch;
        }
        if(pitch < pitchBtm){
            pitchBtm = pitch;
        }
        if(yaw > yawTop){
            yawTop = yaw;
        }
        if(yaw < yawBtm){
            yawBtm = yaw;
        }
        // Serial.println(yaw);
    }
    while(Serial.available()){
        Serial.read();
    }
}

void remoteVal(){
    throttleBtm = 983;
    throttleTop = 2007;
    rollBtm = 983;
    rollTop = 2005;
    pitchBtm = 987;
    pitchTop = 2012;
    yawBtm = 985;
    yawTop = 2010;
}

// Remaps raw remote control values to useable ones
void remoteCalc(){
    throttleOut = map(throttle, throttleBtm, throttleTop, MINACCEL, MAXACCEL);
    rollOut = map(roll, rollBtm, rollTop, -100, 100);
    pitchOut = map(pitch, pitchBtm, pitchTop, MINBRAKE, MAXBRAKE);
    yawOut = map(yaw, yawBtm, yawTop, MINSTEER, MAXSTEER);

    // Serial.print(throttleBtm);
    // Serial.print("\t");
    // Serial.print(throttleTop);
    // Serial.print("\t");
    // Serial.print(rollBtm);
    // Serial.print("\t");
    // Serial.print(rollTop);
    // Serial.print("\t");
    // Serial.print(pitchBtm);
    // Serial.print("\t");
    // Serial.print(pitchTop);
    // Serial.print("\t");
    // Serial.print(yawBtm);
    // Serial.print("\t");
    // Serial.println(yawTop);
}

// Debug program for printing all remote control values
void remoteDebug(){
    Serial.print(throttleOut);
    Serial.print("\t | \t");
    Serial.print(rollOut);
    Serial.print("\t | \t");
    Serial.print(pitchOut);
    Serial.print("\t | \t");
    Serial.println(yawOut);
}

void encoderInit(){
    attachInterrupt(digitalPinToInterrupt(ENCODER), encoderRising, RISING);
}

void encoderCalc(){
    angle = ((float)counter/ticksPerRev)*360;
}

// Remote Interrupt Service Routines
void rising1() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL1), falling1, FALLING);
  prev_throttle = micros();
}
 
void falling1() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL1), rising1, RISING);
  throttle = micros()-prev_throttle;
}

void rising2() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL2), falling2, FALLING);
  prev_roll = micros();
}
 
void falling2() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL2), rising2, RISING);
  roll = micros()-prev_roll;
}

void rising3() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL3), falling3, FALLING);
  prev_pitch = micros();
}
 
void falling3() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL3), rising3, RISING);
  pitch = micros()-prev_pitch;
}

void rising4() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL4), falling4, FALLING);
  prev_yaw = micros();
}
 
void falling4() {
  attachInterrupt(digitalPinToInterrupt(CHANNEL4), rising4, RISING);
  yaw = micros()-prev_yaw;
}

void encoderRising(){
  if(!digitalRead(ENCODERD)){
    counter++;
  }
  else{
    counter--;
  }
}