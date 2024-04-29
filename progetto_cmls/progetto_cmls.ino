#include <Arduino.h>
#include <BLEMidi.h>
#include <Wire.h>

// #define LED 4

//the b-frame is the frame of the body while the e-frame is the frame of the environament

typedef struct {
  float_t pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
  float_t dir_mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};                                         // rotation matrix of the b-frame in relation to the e-frame initialized as [[1 0 0],[0 1 0],[0 0 1]]
} state_t;                                                    // state containing the current position velocity and orientation vectors

float_t acel[3] = {0, 0, 0}, gyro[3] = {0, 0, 0}, init_g[3] = {0, 0, 0};                          // sensor data
bool curr = 0;                                             // index to deduce the current state (treated as boolean)
uint32_t t[2];
state_t state[2];                                             // current and previous state (used for calculations)

bool test_low = 1, c_t = 0, touch[2] = {0, 0};
uint32_t threshold = 55;

// ---- MATRIX OPERATIONS ----

void eul2Rotm(float_t eul[3], float rotm[9]) {                // define rotation matrix from euler angles (output of gyroscope)
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
  // TODO
}

void matMul(float_t out[9], float_t delta[9], float_t old_mat[9]) { // matrix matrix multiplication
  out[0] = delta[0]*old_mat[0] + delta[1]*old_mat[3] + delta[2]*old_mat[6];
  out[1] = delta[0]*old_mat[1] + delta[1]*old_mat[4] + delta[2]*old_mat[7];
  out[2] = delta[0]*old_mat[2] + delta[1]*old_mat[5] + delta[2]*old_mat[8];
  out[3] = delta[3]*old_mat[0] + delta[4]*old_mat[3] + delta[5]*old_mat[6];
  out[4] = delta[3]*old_mat[1] + delta[4]*old_mat[4] + delta[5]*old_mat[7];
  out[5] = delta[3]*old_mat[2] + delta[4]*old_mat[5] + delta[5]*old_mat[8];
  out[6] = delta[6]*old_mat[0] + delta[7]*old_mat[3] + delta[8]*old_mat[6];
  out[7] = delta[6]*old_mat[1] + delta[7]*old_mat[4] + delta[8]*old_mat[7];
  out[8] = delta[6]*old_mat[2] + delta[7]*old_mat[5] + delta[8]*old_mat[8];
}

void matVecMul(float_t out[3], float_t mat[9], float vec[3]) {            // matrix vector multiplication
  out[0] = mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2];
  out[1] = mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2];
  out[2] = mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2];
}

void matTrans(float_t out[3], float_t in[3]) {                            // transposes the in matrix in the out matrix (since rotation matrices are orthogonal it is equal to inverting it)
  out[0] = in[0];
  out[1] = in[3];
  out[2] = in[6];
  out[3] = in[1];
  out[4] = in[4];
  out[5] = in[7];
  out[6] = in[2];
  out[7] = in[5];
  out[8] = in[8];
}

// ---- PHYSICAL OPERATIONS ----

void getAcelData() {                                                      // must be used only in the initialization state
  acel[0] = (Wire.read() << 8 | Wire.read())/16384.0;
  acel[1] = (Wire.read() << 8 | Wire.read())/16384.0;
  acel[2] = (Wire.read() << 8 | Wire.read())/16384.0;
}

float_t getInertialData(uint8_t c){                                       // stores the sensor data
  acel[0] = (Wire.read() << 8 | Wire.read())/16384.0;
  acel[1] = (Wire.read() << 8 | Wire.read())/16384.0;
  acel[2] = (Wire.read() << 8 | Wire.read())/16384.0;
  gyro[0] = (Wire.read() << 8 | Wire.read())/131 + 0.56;
  gyro[1] = (Wire.read() << 8 | Wire.read())/131 - 2;
  gyro[2] = (Wire.read() << 8 | Wire.read())/131 + 0.79;
  t[c] = millis();
  return (t[c]-t[!c])*0.001;                                             // returns the time elapsed from the last call in seconds
}

void updateState() {
  curr = !curr;
  float_t dt = getInertialData(curr);
  float_t rotation[3], drot[9], rot_acel[3], trans_dir[9];

  rotation[0] = (gyro[0]*dt)*2*PI;
  rotation[1] = (gyro[1]*dt)*2*PI;
  rotation[2] = (gyro[2]*dt)*2*PI;

  eul2Rotm(rotation, drot);                                               // calculating rotation matrix from old rotation matrix and gyroscope data
  matMul(state[curr].dir_mat,drot,state[!curr].dir_mat);

  
  matTrans(trans_dir, state[curr].dir_mat);                               // transposing the rotation matrix from e-frame to b-frame to obtain the inverse from b-frame to e-frame
  matVecMul(rot_acel, trans_dir, acel);                                   // rotating the acceleratoion to e-frame

  rot_acel[0] -= init_g[0];
  rot_acel[1] -= init_g[1];
  rot_acel[2] -= init_g[2];

  state[curr].vel[0] = state[!curr].vel[0] + rot_acel[0]*dt;              // calculating velocity and position
  state[curr].vel[1] = state[!curr].vel[1] + rot_acel[1]*dt;
  state[curr].vel[2] = state[!curr].vel[2] + rot_acel[2]*dt;
  state[curr].pos[0] = state[!curr].pos[0] + (state[curr].vel[0])*dt;
  state[curr].pos[1] = state[!curr].pos[1] + (state[curr].vel[1])*dt;
  state[curr].pos[2] = state[!curr].pos[2] + (state[curr].vel[2])*dt;
};

// touch Interrupt

void gotTouchEvent(){
  if (touch[!t_c] != touch[t_c]) {
    touch[t_c] = !touch[t_c];
    test_low = !test_low;
    // Touch ISR will be inverted: Lower <--> Higher than the Threshold after ISR event is noticed
    touchInterruptSetThresholdDirection(test_low);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing bluetooth");

  BLEMidiServer.begin("MI.MU gloves dei poveri");
  Serial.println("Waiting for connections...");
  while(!BLEMidiServer.isConnected()) delay(100);
  Serial.println("Bluetooth connected.");

  touchAttachInterrupt(T2, gotTouchEvent, threshold);
  touchInterruptSetThresholdDirection(test_low);

  Serial.print("Starting gravity estimation hold your hand still for 1 second.");
  float_t elapsed = 0;
  int16_t i = 0;
  while (elapsed < 1){                                                    // initializing the gravity vector by getting data for a second and averaging it
    getAcelData();
    init_g[0] += acel[0];
    init_g[1] += acel[1];
    init_g[2] += acel[2];
    elapsed += millis()*0.001;
    i++;
    delay(10);
  }
  init_g[0] /= i;
  init_g[1] /= i;
  init_g[2] /= i;
  t[0] = millis();
  t[1] = t[0];
  Serial.print("Inital gravity calculation done. Used ");
  Serial.print(i);
  Serial.println(" accelerometer measurements");
}

void loop() {
  if(touch[!c_t] != touch[c_t]){
    if(touch[c_t]){
      Serial.println("  ---- Touch was Pressed"); // note on and other stuff
    } else {
      Serial.println("  ---- Touch was Released"); // note off and other stuff
    }
    c_t = !c_t;
  }
  if(BLEMidiServer.isConnected()) {     // If we've got a connection, we send an A4 during one second, at full velocity (127)
    Serial.println("connected");
      BLEMidiServer.noteOn(1, 69, 127);
      delay(1000);
      BLEMidiServer.noteOff(1, 69, 127);      // Then we stop the note and make a delay of one second before returning to the beginning of the loop
      delay(1000);
  } else {Serial.println("not connected"); delay(1000);}
  Serial.flush();
}
