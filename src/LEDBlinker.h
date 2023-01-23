#ifndef _LEDBLINKER_H_
#define _LEDBLINKER_H_

class LEDBlinker {
 private:
  int _pin;
  unsigned long _onInterval;   // in ms
  unsigned long _offInterval;  // in ms
  int _state;
  unsigned long _lastToggleTimestamp;  // in ms

  void _setup();

 public:
  LEDBlinker(int pin, unsigned long onInterval, unsigned long offInterval);
  // LEDBlinker(int pin);
  void setHigh();
  void setLow();
  void setInterval(unsigned long onInterval, unsigned long offInterval);
  void tick();
};

#endif