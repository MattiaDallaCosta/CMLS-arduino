#include <Arduino.h>
#include <BLEMidi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

#define TOUCH 2

#define E_MEA 5
#define E_EST 5
#define NOISE 0.5

#define E_MEA2 2
#define E_EST2 2
#define NOISE2 0.01
//the b-frame is the frame of the body while the e-frame is the frame of the environament

typedef struct {
  float_t pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
  float_t dir_mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};                                       // rotation matrix of the b-frame in relation to the e-frame initialized as [[1 0 0],[0 1 0],[0 0 1]]
} state_t;                                                    // state containing the current position velocity and orientation vectors

float_t acel[3] = {0, 0, 0}, gyro[3] = {0, 0, 0}, init_g[3] = {0, 0, 0};                          // sensor data
bool curr = 0;                                             // index to deduce the current state (treated as boolean)
uint32_t t[2];
state_t state[2];                                             // current and previous state (used for calculations)

int threshold = 40;
bool touchActive = false;
bool lastTouchActive = false;
bool testingLower = true;

uint8_t note = 60;
uint8_t note_tresh =3;
uint8_t vel_val = 60;
uint8_t vel_tresh =3;

SimpleKalmanFilter vect(E_MEA,E_EST,NOISE);
SimpleKalmanFilter g0(E_MEA2,E_EST2,NOISE2);
SimpleKalmanFilter g1(E_MEA2,E_EST2,NOISE2);
SimpleKalmanFilter g2(E_MEA2,E_EST2,NOISE2);

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
    // Calcola l'angolo di pitch
  eul[1] = -asin(rotm[2]); // sin(-pitch) = -rotm[2]

  // Calcola l'angolo di yaw e roll
  if (cos(eul[1]) != 0) {
    eul[0] = atan2(rotm[5] / cos(eul[1]), rotm[8] / cos(eul[1])); // roll
    eul[2] = atan2(rotm[1] / cos(eul[1]), rotm[0] / cos(eul[1])); // yaw
  } else {
    // Situazione singolare, pitch = +-90 gradi
    eul[0] = atan2(rotm[3], rotm[4]); // roll arbitrario
    eul[2] = 0; // yaw arbitrario
  }
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

float_t getInertialData(){                                       // stores the sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  acel[0] = int8_t(10*a.acceleration.x)/10.0;
  acel[1] = int8_t(10*a.acceleration.y)/10.0;
  acel[2] = int8_t(10*a.acceleration.z)/10.0;
  gyro[0] = int8_t(10*g.gyro.x)/10.0;
  gyro[1] = int8_t(10*g.gyro.y)/10.0;
  gyro[2] = int8_t(10*g.gyro.z)/10.0;
  t[curr] = micros();
  return (t[curr]-t[!curr])*0.000001;
}

void updateState() {
  curr = !curr;
  float_t dt = getInertialData();
  float_t rotation[3], drot[9], rot_acel[3] = {0, 0, 0}, trans_dir[9];
  float_t rot_g[3], aacel[3];

  rotation[0] = (gyro[0]*dt);
  rotation[1] = (gyro[1]*dt);
  rotation[2] = (gyro[2]*dt);

  eul2Rotm(rotation, drot);                                               // calculating rotation matrix from old rotation matrix and gyroscope data
  matMul(state[curr].dir_mat,drot,state[!curr].dir_mat);
  
  matTrans(trans_dir, state[curr].dir_mat);                               // transposing the rotation matrix from e-frame to b-frame to obtain the inverse from b-frame to e-frame
  matVecMul(rot_acel, trans_dir, acel);                                   // rotating the acceleratoion to e-frame
  
  //Serial.println("--------");
  
  //Serial.printf("rotated \naccel values: %f --- %f --- %f\ngyro values: %f --- %f --- %f\n", rot_acel[0], rot_acel[1], rot_acel[2], gyro[0], gyro[1], gyro[2]);
  float_t pre_mod = sqrt(rot_acel[0]*rot_acel[0] + rot_acel[1]*rot_acel[1]+ rot_acel[2]*rot_acel[2]);
  
  rot_acel[0] -= init_g[0];
  rot_acel[1] -= init_g[1];
  rot_acel[2] -= init_g[2];

  float_t accel_mod = sqrt(rot_acel[0]*rot_acel[0] + rot_acel[1]*rot_acel[1]+ rot_acel[2]*rot_acel[2]);
  float_t corrected_mod = vect.updateEstimate(accel_mod);

  rot_acel[0] /= accel_mod;
  rot_acel[1] /= accel_mod;
  rot_acel[2] /= accel_mod;
  
  rot_acel[0] *= corrected_mod;
  rot_acel[1] *= corrected_mod;
  rot_acel[2] *= corrected_mod;

  rot_acel[0] = int8_t(rot_acel[0]*10)/10.0;
  rot_acel[1] = int8_t(rot_acel[1]*10)/10.0;
  rot_acel[2] = int8_t(rot_acel[2]*10)/10.0;

  Serial.printf("rotated \naccel values: %f --- %f --- %f\ngyro values: %f --- %f --- %f\n", rot_acel[0], rot_acel[1], rot_acel[2], gyro[0], gyro[1], gyro[2]);
  Serial.printf("vect module = %f\n", pre_mod);
  Serial.printf("no gravity vect module = %f\n", accel_mod);
  Serial.printf("corrected vect module = %f\n", corrected_mod);
  
  state[curr].vel[0] = int8_t(100*(state[!curr].vel[0] + rot_acel[0]*dt))/100.0;              // calculating velocity and position
  state[curr].vel[1] = int8_t(100*(state[!curr].vel[1] + rot_acel[1]*dt))/100.0;
  state[curr].vel[2] = int8_t(100*(state[!curr].vel[2] + rot_acel[2]*dt))/100.0;
  state[curr].pos[0] = (state[!curr].pos[0] + (state[curr].vel[0])*dt);
  state[curr].pos[1] = (state[!curr].pos[1] + (state[curr].vel[1])*dt);
  state[curr].pos[2] = (state[!curr].pos[2] + (state[curr].vel[2])*dt);
};

// touch Interrupt

void gotTouchEvent(){
  if (lastTouchActive != testingLower) {
    touchActive = !touchActive;
    testingLower = !testingLower;
    // Touch ISR will be inverted: Lower <--> Higher than the Threshold after ISR event is noticed
    touchInterruptSetThresholdDirection(testingLower);
    if(lastTouchActive != touchActive){
    lastTouchActive = touchActive;
    if(touchActive){
      Serial.println("Touch was Pressed\n"); // note on and other stuff
      Serial.printf("Vel X : %f\nVel Y: %f\nVel Z: %f\nPos X : %f\nPos Y: %f\nPos Z: %f\n\n", state[curr].vel[0], state[curr].vel[1], state[curr].vel[2], state[curr].pos[0], state[curr].pos[1], state[curr].pos[2]);
    } else {
      Serial.println("Touch was Released\n ---====---"); // note off and other stuff
    }
  }
  }
}

// ---- MIDI FUNCTIONS ----
/*deve comparare la vecchia posizione con la nuova ma solo della posizione x e se la differenza è più grande/piccolo di un certo valore allora aggiorna
la nota aumentandola; stessa cosa per le y con la velocità però.


void notesFromPosition(){
 bool is_higer = state[curr].pos[0] > state[!curr].pos[0]
 float_t delta0 = is_higer ? state[curr].pos[0] - state[!curr].pos[0] : state[!curr].pos[0] - state[curr].pos[0];
 if (delta0 > note_tresh) {
  int8_t var0 = int8_t(delta0 / note_tresh ); 
  var0 *= is_higer ? 1 : -1 ; 
  note += var0;
}
else {
    int8_t pitch_w = int8_t((delta0/ note_tresh)*8191);
    pitch_w = is_higer ? pitch_w : -pitch_w;
   }

   bool v_higer = state[curr].pos[1] > state[!curr].pos[1]
 float_t delta1 = v_higer ? state[curr].pos[1] - state[!curr].pos[1] : state[!curr].pos[1] - state[curr].pos[1];
if (delta1 > vel_tresh) {
  int8_t var1 = int8_t(delta0 / note_tresh ); 
  var1 *= v_higer ? 1 : -1 ; 
  vel_val += var1;
}


}*/

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing bluetooth");
  
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

  touchAttachInterrupt(TOUCH, gotTouchEvent, threshold);
  touchInterruptSetThresholdDirection(testingLower);

  Serial.println("Starting gravity estimation hold your hand still for 1 second.");
  uint8_t i = 0;
  float_t elapsed = 0;
  t[0] = micros();
  t[1] = t[0];
  while(elapsed < 1){
    elapsed += getInertialData();
    init_g[0] += acel[0];
    init_g[1] += acel[1];
    init_g[2] += acel[2];
    i++;
  }
  init_g[0] /= i;
  init_g[1] /= i;
  init_g[2] /= i;

  Serial.printf("Mean of %i values\ngravity X : %f\ngravity Y: %f\ngravity Z: %f\n", i, init_g[0], init_g[1], init_g[2]);
  Serial.printf("vect module = %f\n", sqrt(init_g[0]*init_g[0] + init_g[1]*init_g[1]+ init_g[2]*init_g[2]));
}


void loop() {
  if (!BLEMidiServer.isConnected()) {
    Serial.print("Waiting for connections");
    while(!BLEMidiServer.isConnected()) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("");
  }
  updateState();
  
  // Serial.printf("accel X : %f\naccel Y: %f\naccel Z: %f\n", acel[0], acel[1], acel[2]);

  //Serial.printf("Vel X : %f\nVel Y: %f\nVel Z: %f\nPos X : %f\nPos Y: %f\nPos Z: %f\n\n", state[curr].vel[0], state[curr].vel[1], state[curr].vel[2], state[curr].pos[0], state[curr].pos[1], state[curr].pos[2]);
  //delay(10);
  //delay(1000);
  // if(BLEMidiServer.isConnected()) {     // If we've got a connection, we send an A4 during one second, at full velocity (127)
  //   Serial.println("connected");
  //     BLEMidiServer.noteOn(1, 69, 127);
  //     delay(1000);
  //     BLEMidiServer.noteOff(1, 69, 127);      // Then we stop the note and make a delay of one second before returning to the beginning of the loop
  //     delay(1000);
  // } else {Serial.println("not connected"); delay(1000);}
  // Serial.flush();
}
