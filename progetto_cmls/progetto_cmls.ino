#include <Arduino.h>
#include <BLEMidi.h>

// #define LED 4

//the b-frame is the frame of the body while the e-frame is the frame of the environament

typedef struct {
  float_t pos[3], vel[3];
  float_t dir_mat[9]; // rotation matrix of the b-frame in relation to the e-frame initialized as [[1 0 0],[0 1 0],[0 0 1]]
} state_t; // state containing the current position velocity and orientation vectors

float_t acel[6], gyro[6], init_g[3]; // sensor data
uint8_t curr = 0, cur_sen = 1; // index to deduce the current state (treated as boolean)
uint32_t time[2];
state_t state[2]; // current and previous state (used for calculations)

void eul2Rotm(float_t eul[3], float rotm[9]){ // define rotation matrix from euler angles (output of gyro)
  rotm[0] = cos(eul[1])*cos(eul[2]);
  rotm[1] = cos(eul[1])*sin(eul[2]);
  rotm[2] = 0 - sin(eul[1]);
  rotm[3] = sin(eul[0])*sin(eul[1])*cos(eul[2]) - cos(eul[0])*sin(eul[2]);
  rotm[4] = sin(eul[0])*sin(eul[1])*sin(eul[2]) + cos(eul[0])*cos(eul[2]);
  rotm[5] = sin(eul[0])*cos(eul[1]);
  rotm[6] = cos(eul[0])*sin(eul[1])*cos(eul[2]) + sin(eul[0])*sin(eul[2]);
  rotm[7] = cos(eul[0])*sin(eul[1])*sin(eul[2]) - sin(eul[0])*cos(eul[2]);
  rotm[8] = cos(eul[0])*cos(eul[1]);
};

void rotm2Eul(float_t rotm[9], float_t eul[3]){

}

void matMul(float_t out[9], float_t delta[9], float_t old[9]) { // matrix matrix multiplication
  out[0] = delta[0]*old[0] + delta[1]*old[3] + delta[2]*old[6];
  out[1] = delta[0]*old[1] + delta[1]*old[4] + delta[2]*old[7];
  out[2] = delta[0]*old[2] + delta[1]*old[5] + delta[2]*old[8];
  out[3] = delta[3]*old[0] + delta[4]*old[3] + delta[5]*old[6];
  out[4] = delta[3]*old[1] + delta[4]*old[4] + delta[5]*old[7];
  out[5] = delta[3]*old[2] + delta[4]*old[5] + delta[5]*old[8];
  out[6] = delta[6]*old[0] + delta[7]*old[3] + delta[8]*old[6];
  out[7] = delta[6]*old[1] + delta[7]*old[4] + delta[8]*old[7];
  out[8] = delta[6]*old[2] + delta[7]*old[5] + delta[8]*old[8];
}

void matVecMul(float_t out[3], float_t mat[9], float vec[3]) { // matrix vector multiplication
  old[0] = mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2];
  old[1] = mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2];
  old[2] = mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2];
}

void stateInit(state_t s) { // initializes the state variables for the given state_t variable
   memset(s.pos, 0, 6);
   memset(s.vel, 0, 6);
   memset(s.dir_mat, 0, 18);
   s.dir_mat[0] = 1;
   s.dir_mat[4] = 1;
   s.dir_mat[8] = 1;
}

void getAccelData(){ // must be used only in the initialization state
  acel[0] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acel[1] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acel[2] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

float_t getInertialData(uint8_t c){ // stores the sensor data
  acel[(c * 3 + 0)] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acel[(c * 3 + 1)] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acel[(c * 3 + 2)] = (Wire.read() << 8 | Wire.read())/16384.0;       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyro[(c * 3 + 0)] = (Wire.read() << 8 | Wire.read())/131 + 0.56;    // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro[(c * 3 + 1)] = (Wire.read() << 8 | Wire.read())/131 - 2;       // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
  gyro[(c * 3 + 2)] = (Wire.read() << 8 | Wire.read())/131 + 0.79;    // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  time[c] = millis();
  return (time[c]-time[!c])*0.001; // returns the time elapsed from the last call in seconds
}

void updateState(){
  curr = !curr;
  float_t dt = getInertialData(curr);
  float_t curr_g[3];
  /* TODO: 1) calculate rotation angles from angular velocity of gyro
   *       2) calculate new rotation matrix (get rotation matrix from euler angles and multiply it for the old rotation matrix)
   *       3) rotate initial gravity to obtain current gravity
   *       4) subtract current gravity from acel measurements
   *       5) calculate new velocity and position from accel measurements (v = v0 + a*t and p = p0 + v*t + a+t^2)
   */
};


void setup() {
  Serial.begin(115200);
  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("MI.MU gloves dei poveri");
  Serial.println("Waiting for connections...");
  //BLEMidiServer.enableDebugging();  // Uncomment if you want to see some debugging output from the library
  stateInit(state[0]);
  stateInit(state[1]);
  float_t elapsed = 0;
  while (elapsed < 1){  // initializing the gravity vector by getting data for a second and "averaging" it
    getAccelData();
    init_g[0] += acel[0];
    init_g[1] += acel[1];
    init_g[2] += acel[2];
    init_g[0] /= 2;
    init_g[1] /= 2;
    init_g[2] /= 2;
    elapsed += millis()*0.001;
    delay(10);
  }
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
