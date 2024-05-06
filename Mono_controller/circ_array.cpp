#include <sys/_stdint.h>
#include "circ_array.h"

circ_array::circ_array():i(0) {
  memset(this->array, 0, C_SZ);
}

circ_array::~circ_array() {}

void circ_array::insert(int value) {
  this->array[this->i] = value;
  this->i = (this->i+1)%C_SZ;
}

uint8_t circ_array::getValue() {
  uint32_t long_app = 0;
  for (uint8_t j = 0; j < C_SZ; j++){
    long_app += this->array[j];
  }
  return long_app/C_SZ;
}

void circ_array::printArray(){
  Serial.printf("i = %i\n", this->i);
  for (uint8_t j = 0; j < C_SZ; j++) {
    Serial.printf("array[%i] = %i  ", j, this->array[j]);
    if(j%10 == 9) Serial.println();
  }
}