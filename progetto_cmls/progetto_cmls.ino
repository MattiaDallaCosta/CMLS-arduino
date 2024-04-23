#include <Arduino.h>
#include <BLEMidi.h>

// #define LED 4

typedef struct {
  int16_t pos[3], vel[3];
  int16_t dir_vec[3];
} state_t; // state containing the current position velocity and orientation vectors

int16_t acel[3], gyro[3], init_g[3]; // sensor data
uint8_t curr = 0; // index to deduce the current state (treated as boolean)
state_t state[2]; // current and previous state (used for calculations)

void Xrot(float_t angle, int16_t vect[3]) { // pitch rotation matrix
    int8_t oldvec[3];
    memcpy(oldvec, vect, 3);

    vect[0] = oldvec[0];
    vect[1] = oldvec[1]*cos(angle) - oldvec[2]*sin(angle);
    vect[2] = oldvec[1]*sin(angle) + oldvec[2]*cos(angle);
}

void Yrot(float_t angle, int16_t vect[3]) { // roll rotation matrix
    int8_t oldvec[3];
    memcpy(oldvec, vect, 3);

    vect[0] = oldvec[0]*cos(angle) + oldvec[2]*sin(angle);
    vect[1] = oldvec[1];
    vect[2] = oldvec[2]*cos(angle) - oldvec[0]*sin(angle);
}

// papperooooo


void Zrot(float_t angle, int16_t vect[3]) { //yaw rotation matrix
    int8_t oldvec[3];
    memcpy(oldvec, vect, 3);

    vect[0] = oldvec[0]*cos(angle) - oldvec[1]*sin(angle);
    vect[1] = oldvec[0]*sin(angle) + oldvec[1]*cos(angle);
    vect[2] = oldvec[2];
}

void RotApply(int16_t vect[3], float angles[3]){ // using convention (X,Y,Z) for rotation application
  Xrot(angles[0], vect); 
  Yrot(angles[1], vect);
  Zrot(angles[2], vect);
}

void Stateinit(state_t s, int16_t){
   memset(s.pos, 0, 3);
   memset(s.vel, 0, 3);
   memset(s.dir_vec, 0, 3);

  // TODO;
}

void getInertialData(int16_t a[3], int16_t g[3]){
    // TODO;
}

void updateState(){
   // get data from sensors

};


void setup() {
  Serial.begin(115200);
  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("Basic MIDI device");
  Serial.println("Waiting for connections...");
  //BLEMidiServer.enableDebugging();  // Uncomment if you want to see some debugging output from the library
}

void loop() {
  if(BLEMidiServer.isConnected()) {     // If we've got a connection, we send an A4 during one second, at full velocity (127)
    Serial.println("connected");
      BLEMidiServer.noteOn(1, 69, 127);
      delay(1000);
      BLEMidiServer.noteOff(1, 69, 127);      // Then we stop the note and make a delay of one second before returning to the beginning of the loop
      delay(1000);
  } else {Serial.println("not connected"); delay(1000);}
  Serial.flush();
}