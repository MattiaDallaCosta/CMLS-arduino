#include <Arduino.h>
#include <BLEMidi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "circ_array.h"

#define T_BUTS 15                                               // last pin on the right from the top (usb plug on the bottom)
#define TOUCH 33                                                // last pin on the right  from the top (usb plug on the bottom)
#define FLEX 13                                                 // 4th pin from the top left (usb plug on the bottom)

// inertial data
float_t acel[3] = {0, 0, 0}, gyro[2] = {0, 0};

// output rotation x,y smoothing circular array
circ_array a0_smooth, a1_smooth;

//current and previous time and index 
uint32_t t[2];
bool curr = 0;

// thresholds for the 3 buttons
int thresh1 = 45;
int thresh2 = 36;
int thresh3 = 12;

//thresholds for the trigger
uint8_t t_tresh = 55;

// octave value
uint8_t octave = 2;
uint32_t long_octave = 0;
uint32_t idx = 0;

// button state for trigger and buttons
bool touch = 0;
bool strip = 0;

// midi cc and notes for current and previous instant
uint8_t midi_cc[4] = {0, 0, 0, 0};
uint8_t midi_note[2] = {0, 0};
bool curr_n = 0;


// abstraction of the IMU sensor
Adafruit_MPU6050 mpu;

/* ---- ROTATION FUNCTIONS ---- */

float_t getInertialData() {                                       // stores the sensor data end evaluates time from last data retrival
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

void getAccAngles(float_t in[3], float_t out[2]) {                // calculate pitch and roll (out[2]) from accelerometer data (in[3])
  float_t roll, pitch;
  roll = sqrt((in[1] * in[1]) + (in[2] * in[2]));
  pitch = sqrt((in[0] * in[0]) + (in[2] * in[2]));
  
  out[0] = atan2(in[0], roll);
  out[1] = atan2(in[1], pitch);
}

void getMidiCc() {                                                // evaluates the Midi CC values for the IMU
  float_t dt = getInertialData();
  float_t app[2], gyro_a[2], angles[2];
  getAccAngles(acel, app);
  gyro_a[0] = gyro[0]*dt;                                         // evaluates the angle from the angular velocity
  gyro_a[1] = gyro[1]*dt;
  angles[0] = (app[0]*0.9 + gyro_a[0]*0.1)*(180/PI);              // sensor fusion to reduce error weight (and conversion to degrees)
  angles[1] = (app[1]*0.9 + gyro_a[1]*0.1)*(180/PI);
  app[0] = angles[0] > 90 ? 90 : angles[0];                       // limits the value to the usefull range
  app[0] = app[0] < -90 ? -90 : app[0];
  midi_cc[curr*2] = 127 - int((app[0] + 90)*(127/180.0));         // maps 180 values in 127 (possible midi values)
  a0_smooth.insert(midi_cc[curr*2]);                              // circular array for smoothing
  midi_cc[curr*2] = a0_smooth.getValue();

  app[1] = angles[1] > 90 ? 90 : angles[1];                       // same as above
  app[1] = app[1] < -90 ? -90 : app[1];
  midi_cc[curr*2 + 1] = int((app[1] + 90)*(127/180.0));
  a1_smooth.insert(midi_cc[curr*2 + 1]);
  midi_cc[curr*2 + 1] = a1_smooth.getValue();
  //printf("midi_cc[0] = %i --- midi_cc[1] = %i\n", midi_cc[0], midi_cc[1]);
}

void getMidiNote() {                                                                  // get the MIDI note from the resitor sensor
  curr_n = !curr_n;
  int out;
  float_t val = analogRead(FLEX);                                                     //it get the analog value from the resistor sensor
  if (val < 1800) val = 1800;                                                         //as the values are not so proportional to the flexion of the hand we have to fix it
  else if (val > 3400) val = 3400;
  out = int((val - 1800)/(1600/12));                                                  //it converts the fixed analog data in the 12 values of the notes
  // Serial.printf("val = %f, out = %i\n", val, out);
  midi_note[curr_n] =  out + octave;                                                  //the MIDI note is given by the note plus the octave that we chose from the 3 buttons
}

/* ---- TOUCH FUNCTIONS ---- */

void touchButton(bool *bt, uint8_t pin, uint8_t tresh, void (*when_pressed)(uint8_t),
                 void (*still_on)(uint8_t), void (*when_released)(uint8_t)) {         // function used to emulate the behaviour of a button using a touch pin
  if(touchRead(pin) < tresh && !*bt) {                                                // what should be done if the button is pressed
    (*when_pressed)(pin);
    *bt = true;
  } else if (touchRead(pin) < tresh && *bt) {                                         // what should be done when the button is continued to be pressed
    (*still_on)(pin);
  } 
  else if (touchRead(pin) > tresh && *bt) {                                           // what should be done when the button is released
    (*when_released)(pin);
    *bt = false;
  }
}

void empty(uint8_t ags){}

void evaluate(uint8_t arg) {                                                          // behaviour of the 3 button touchpin when is being pressed (bot at the beginning and while hold down)
  uint16_t t_val = touchRead(arg);                                                    // gets the value of the pin (differet based on the button pressed)
  Serial.printf("touchread = %i\n", t_val);
  if(t_val > thresh2) long_octave += 2;                                               // adds the value of the button on the specific moment based on some thresholds (the input value oscilates and needs to be averaged for precision)
  else if (t_val > thresh3) long_octave += 1;
  //Serial.printf("long_octave = %i --- touchread = %i\n", long_octave, t_val);
  idx++;
}

void setOctave(uint8_t arg) {                                                         // behaviour of the 3 button touchpin when is released
  float_t fval = long_octave/float_t(idx);                                            // averages the summed values from the prior function to obtain the value of the octave
  octave = 48 + int(round(fval))*12;                                                       // sets the first note of the octave which will be playable (lowest octave starts on C3)
  Serial.printf("value = %i --- not rounded = %f --- idx = %i\nfirst note = %i\n", int(round(fval)), fval, idx, octave);
  idx = 0;                                                                            // resets the values
  long_octave = 0;
}

void sendNote(uint8_t arg) {                                                          // behaviour of the trigger button when pressed
  Serial.printf("in sendNote\n");             
  getMidiNote();                                                                      // evaluates the current midi note and sends a noteOn
  BLEMidiServer.noteOn(1,midi_note[curr_n], 127);
}

void checkNote(uint8_t arg) {                                                         // behaviour of the trigger button when is continued to be pressed
  //Serial.printf("in CheckNote\n");
  getMidiNote();                                                                      // evaluates the current note
  if(midi_note[curr_n] != midi_note[!curr_n]){                                        // if different from previous one send noteOf of the previous one and noteOn of current one
    BLEMidiServer.noteOff(1, midi_note[!curr_n], 127);
    BLEMidiServer.noteOn(1, midi_note[curr_n], 127);
  }
}

void stopNote(uint8_t arg) {                                                          // behaviour of trigger button when released
  Serial.printf("in StopNote\n");
  BLEMidiServer.noteOff(1, midi_note[curr_n], 127);                                   // sends noteOff of current note
}



/* ---- ARDUINO FUNCTIONS ---- */

void setup() {
  Serial.begin(115200);
  delay(10);

  if (!mpu.begin()) {                                                                 // init of mpu
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);                                    // init of mpu (cotd)
  mpu.setMotionDetectionThreshold(10);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  
  BLEMidiServer.begin("MI.MU gloves dei poveri");                                     // starts the Bluetooth Ble Server used for sending midi messages
  Serial.print("Waiting for connections");
  while(!BLEMidiServer.isConnected()) {                                               // waits for connectionss
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n Bluetooth connected.");
}

void loop() {
  if (!BLEMidiServer.isConnected()) {                                                 // if a disconnection happens a new connection is waited for
    Serial.print("Waiting for connections");
    while(!BLEMidiServer.isConnected()) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("");
  }
  getMidiCc();                                                                        // evaluates CC values from IMU
  //Serial.printf("roll: %i --- pitch: %i\n", midi_cc[curr*2 + 0], midi_cc[curr*2 + 1]);
  touchButton(&strip, T_BUTS, thresh1, evaluate, evaluate, setOctave);                //sets the 2 touch inputs as buttons
  //touchButton(&strip, T_BUTS, thresh1, evaluate, empty, setOctave);                //sets the 2 touch inputs as buttons
  touchButton(&touch, TOUCH, t_tresh, sendNote, checkNote, stopNote);
  if(midi_cc[curr*2] != midi_cc[!curr*2])                                             // sends midi CC for pitch
    BLEMidiServer.controlChange(1, 12, midi_cc[curr*2]);
  if(midi_cc[curr*2 + 1] != midi_cc[!curr*2 + 1])                                     // sends midi CC for roll
    BLEMidiServer.controlChange(1, 13, midi_cc[curr*2 + 1]);
  curr = !curr;
  delay(10);
}