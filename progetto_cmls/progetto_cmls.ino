#include <Arduino.h>
#include <BLEMidi.h>

// #define LED 4

typedef struct {
  int16_t pos[3], vel[3];
  int16_t dir_vec[3]; // 1 0 0
} state_t; // state containing the current position velocity and orientation vectors

typedef struct {
  int16_t v[3];
} Vec3_t;

int16_t acel[6], gyro[6], init_g[3]; // sensor data
uint8_t curr = 0, cur_sen = 1; // index to deduce the current state (treated as boolean)
state_t state[2]; // current and previous state (used for calculations)

Vec3_t accellerazione[2];

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

void Stateinit(state_t s){
   memset(s.pos, 0, 6);
   memset(s.vel, 0, 6);
   memset(s.dir_vec, 0, 6);
   s.dir_vec[0] = 1;
}

void getInertialData(){ //prende i dati dei sensori e li mette nei corrispettivi vettori
  cur_sens = !cur_sens;

  acel[(cur_sens * 3 + 0)] = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acel[(cur_sens * 3 + 1)] = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acel[(cur_sens * 3 + 2)] = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyro[(cur_sens * 3 + 0)] = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro[(cur_sens * 3 + 1)] = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro[(cur_sens * 3 + 2)] = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void updateState(){
  
   // get data from sensors
   // state[curr].
   state[curr].pos[0];
   state[curr].pos[1];
   state[curr].pos[2];
    // curvall - oldval

  curr = !curr;
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
