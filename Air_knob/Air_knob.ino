/*

This is an example how to use Touch Intrrerupts
The sketh will tell when it is touched and then relesased as like a push-button

This method based on touchInterruptSetThresholdDirection() is only available for ESP32
*/
#include <Arduino.h>
#include <BLEMidi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "circ_array.h"

#define T_BUTS T2
#define TOUCH T3

// inertial data
float_t acel[3] = {0, 0, 0}, gyro[2] = {0, 0}, init_g[3] = {0, 0, 0};

// output rotation x,y
circ_array a0_smooth, a1_smooth;

//current and previous time and index                                          
uint32_t t[2];
bool curr = 0;

// thresholds for the 3 buttons
int thresh1 = 60;
int thresh2 = 30;
int thresh3 = 15;

//thresholds for the trigger
uint8_t t_tresh = 55;

// octave value
int value = 2;

// button state for trigger and buttons
bool touch = 0;
bool strip = 0;

uint8_t midi_cc[4];
uint8_t midi_note[2] = {0, 0};

int args1[2] = {0, 0};
int args2[2] = {T_BUTS, 0};

// abstraction of the IMU sensor
Adafruit_MPU6050 mpu;

/* ---- ROTATION FUNCTIONS ---- */

float_t getInertialData(){                                       // stores the sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  acel[0] = int(100*a.acceleration.x)/100.0;
  acel[1] = int(100*a.acceleration.y)/100.0;
  acel[2] = int(100*a.acceleration.z)/100.0;
  gyro[0] = int(100*g.gyro.x)/100.0;
  gyro[1] = int(100*g.gyro.y)/100.0;
  t[curr] = micros();
  return (t[curr]-t[!curr])*0.000001;
}

void getAccAngles(float_t in[3], float_t out[2]) {
  float_t roll, pitch;
  roll = sqrt((in[1] * in[1]) + (in[2] * in[2]));
  pitch = sqrt((in[0] * in[0]) + (in[2] * in[2]));
  
  out[0] = atan2(in[0], roll);
  out[1] = atan2(in[1], pitch);
}

void getMidiCc() {
  float_t dt = getInertialData();
  float_t app[2], gyro_a[2], angles[2];
  getAccAngles(acel, app);
  gyro_a[0] = gyro[0]*dt;
  gyro_a[1] = gyro[1]*dt;
  angles[0] = (app[0]*0.95 + gyro_a[0]*0.05)*(180/PI);
  angles[1] = (app[1]*0.95 + gyro_a[1]*0.05)*(180/PI);
  app[0] = angles[0] > 90 ? 90 : angles[0]; 
  app[0] = app[0] < -90 ? -90 : app[0];
  midivals[curr*2] = 127 - int((app[0] + 90)*(127/180.0));
  a0_smooth.insert(midivals[curr*2]);
  midivals[curr*2] = a0_smooth.getValue();

  app[1] = angles[1] > 90 ? 90 : angles[1]; 
  app[1] = app[1] < -90 ? -90 : app[1];
  midivals[curr*2 + 1] = int((app[1] + 90)*(127/180.0));
  a1_smooth.insert(midivals[curr*2 + 1]);
  midi_cc[curr*2 + 1] = a1_smooth.getValue();
}

void getMidiNote() {
  // float_t val = analogRead(33);
  // midi_note[curr] = 3;
}

/* ---- TOUCH FUNCTIONS ---- */

void touchButton(bool *bt, uint8_t pin, uint8_t tresh, void (*when_pressed)(void*), void (*when_released)(void*), void* args){
  if(touchRead(pin) < 55 && !*bt){
    Serial.printf("touch pressed\n");
    (*when_pressed)(args);
    *bt = true;
  } else if (touchRead(pin) > 55 && *bt){
    Serial.printf("touch released\n");
    (*when_released)(args);
    *bt = false;
  }
}

void evaluate(void*args){
  uint16_t t_val = touchRead(((int*)args)[0]);
  if(t_val > thresh2) value = 3;
  else if (t_val > thresh3) value = 2;
  else value = 1;
  Serial.printf("value = %i\n", value);
}

void empty(void* i){
  Serial.printf("empty function\n");
}

/* ---- ARDUINO FUNCTIONS ---- */

void setup() {
  Serial.begin(115200);
  delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(10);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  
  BLEMidiServer.begin("MI.MU gloves dei poveri");
  Serial.print("Waiting for connections");
  while(!BLEMidiServer.isConnected()) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n Bluetooth connected.");
}

void loop(){
  if (!BLEMidiServer.isConnected()) {
    Serial.print("Waiting for connections");
    while(!BLEMidiServer.isConnected()) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("");
  }
  getMidiCc();
  getMidiNote();
  // Serial.printf("roll: %i --- pitch: %i\n", midi_cc[curr_cc*2 + 0], midi_cc[curr_cc*2 + 1]);
  touchButton(&touch, TOUCH, t_tresh, empty, empty, (void*)args1);
  touchButton(&strip, T_BUTS, thresh1,  evaluate, empty,(void*)args2);
  if(midi_cc[curr*2] != midi_cc[!curr*2]) BLEMidiServer.controlChange(1, 12, midi[0]);
  if(midi_cc[curr*2 + 1] != midi_cc[!curr*2 + 1]) BLEMidiServer.controlChange(1, 13, midi[1]);
  curr = !curr;
  delay(10);
}
