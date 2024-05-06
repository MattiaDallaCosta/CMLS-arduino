#pragma once
#include <Arduino.h>
#define C_SZ 100


class circ_array {
  private:
  uint8_t array[C_SZ];
  uint8_t i;

  public:
  circ_array();
  ~circ_array();
  void insert(int value);
  uint8_t getValue();
  void printArray();
};
