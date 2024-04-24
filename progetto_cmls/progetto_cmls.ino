#include <Arduino.h>
#include <BLEMidi.h>

// #define LED 4

//the b-frame is the frame of the body while the e-frame is the frame of the environament

typedef struct {
  float_t pos[3], vel[3];
  float_t dir_mat[9]; // rotation matrix of the b-frame in relation to the e-frame initialized as [[1 0 0],[0 1 0],[0 0 1]]
} state_t; // state containing the current position velocity and orientation vectors

int16_t acel[6], gyro[6], init_g[3]; // sensor data
uint8_t curr = 0, cur_sen = 1; // index to deduce the current state (treated as boolean)
uint32_t time[2];
state_t state[2]; // current and previous state (used for calculations)

void eul2Rotm(float_t angle[3], float rotm[9]){ // define rotation matrix from euler angles (output of gyro)
  rotm[0] = cos(angle[1])*cos(angle[2]);
  rotm[1] = cos(angle[1])*sin(angle[2]);
  rotm[2] = 0 - sin(angle[1]);
  rotm[3] = sin(angle[0])*sin(angle[1])*cos(angle[2]) - cos(angle[0])*sin(angle[2]);
  rotm[4] = sin(angle[0])*sin(angle[1])*sin(angle[2]) + cos(angle[0])*cos(angle[2]);
  rotm[5] = sin(angle[0])*cos(angle[1]);
  rotm[6] = cos(angle[0])*sin(angle[1])*cos(angle[2]) + sin(angle[0])*sin(angle[2]);
  rotm[7] = cos(angle[0])*sin(angle[1])*sin(angle[2]) - sin(angle[0])*cos(angle[2]);
  rotm[8] = cos(angle[0])*cos(angle[1]);
};

void rotmul(float_t out[9], float_t a[9], float_t b[9]) {
  out[0] = ;
  out[1] = ;
  out[2] = ;
  out[3] = ;
  out[4] = ;
  out[5] = ;
  out[6] = ;
  out[7] = ;
  out[8] = ;
}

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



void Zrot(float_t angle, int16_t vect[3]) { // yaw rotation matrix
    int8_t oldvec[3];
    memcpy(oldvec, vect, 3);

    vect[0] = oldvec[0]*cos(angle) - oldvec[1]*sin(angle);
    vect[1] = oldvec[0]*sin(angle) + oldvec[1]*cos(angle);
    vect[2] = oldvec[2];
}

void RotApply(int16_t vect[3], float angles[3]){ // applies the rotetion defined as the angles[3] vector to a 3d vector (vect[3])
  Zrot(angles[2], vect);
  Yrot(angles[1], vect);
  Xrot(angles[0], vect);                         // using convention (Z,Y,X) for rotation application  
}

void Stateinit(state_t s){
   memset(s.pos, 0, 6);
   memset(s.vel, 0, 6);
   memset(s.dir_vec, 0, 18);
   s.dir_vec[0] = 1;
}

uint32_t getInertialData(uint8_t c){ //prende i dati dei sensori e li mette nei corrispettivi vettori
  acel[(c * 3 + 0)] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acel[(c * 3 + 1)] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acel[(c * 3 + 2)] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyro[(c * 3 + 0)] = (Wire.read() << 8 | Wire.read())/131 + 0.56;    // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro[(c * 3 + 1)] = (Wire.read() << 8 | Wire.read())/131 - 2;       // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
  gyro[(c * 3 + 2)] = (Wire.read() << 8 | Wire.read())/131 + 0.79;    // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  time[c] = millis();
  return time[c]/time[!c]; // returns the time elapsed from the last call
}

void updateState(){
  curr = !curr;
  uint32_t dt = getInertialData(curr);
  // calculates current position and velocity from the current and old vals of the accelerometer and the previous state
  state[curr].vel[0] = (state[!curr].vel[0] + acel[(cur_sens * 3 + 0)] - acel[(!cur_sen * 3 + 0)])/dt;
  state[curr].vel[1] = (state[!curr].vel[1] + acel[(cur_sens * 3 + 1)] - acel[(!cur_sen * 3 + 1)])/dt;
  state[curr].vel[2] = (state[!curr].vel[2] + acel[(cur_sens * 3 + 2)] - acel[(!cur_sen * 3 + 2)])/dt;
  state[curr].pos[0] = (state[!curr].pos[0] +  state[curr].vel[0] -  state[!curr].vel[0])/dt;
  state[curr].pos[1] = (state[!curr].pos[1] +  state[curr].vel[1] -  state[!curr].vel[1])/dt;
  state[curr].pos[2] = (state[!curr].pos[2] +  state[curr].vel[2] -  state[!curr].vel[2])/dt;
};


void setup() {
  Serial.begin(115200);
  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("MI.MU gloves dei poveri");
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
