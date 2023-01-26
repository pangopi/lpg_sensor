#include <Arduino.h>
#include "LEDBlinker.h"

LEDBlinker::LEDBlinker(int pin, unsigned long onInterval, unsigned long offInterval) {
  _pin = pin;
  _onInterval = onInterval;
  _offInterval = offInterval;
  _setup();
}

// LEDBlinker(int pin) {
//   // Create a non blinking LED, starts in the off state
//   _pin = pin;
//   _onInterval = 0;
//   _offInterval = 0;
// }

void LEDBlinker::_setup() {
  _state = LOW;
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
}

void LEDBlinker::setHigh() {
  // Turn on the LED without blinking, resets the intervals
  digitalWrite(_pin, HIGH);
  _state = HIGH;
  _onInterval = 2000;
  _offInterval = 0;
}

void LEDBlinker::setLow() {
  // Turn off the LED without blinking, resets the interval
  digitalWrite(_pin, LOW);
  _state = LOW;
  _onInterval = 0;
  _offInterval = 2000;
}

void LEDBlinker::setInterval(unsigned long onInterval, unsigned long offInterval) {
  // Set a new interval
  _onInterval = onInterval;
  _offInterval = offInterval;
}

void LEDBlinker::setPulse(unsigned long pulse_ms, unsigned long pulseEvery_ms = 1000) {
  setInterval(pulse_ms, pulseEvery_ms - pulse_ms);
}

void LEDBlinker::tick() {
  unsigned long currentMillis = millis();

  switch (_state)
  {
  case LOW:
    if (_onInterval == 0) {
      // Keep the LED permanently off
      break;
    }
    if (currentMillis - _lastToggleTimestamp > _offInterval) {
      // Turn on when off interval is over
      _lastToggleTimestamp = millis();
      _state = HIGH;
      digitalWrite(_pin, _state);
    }
    break;
  case HIGH:
    if (_offInterval == 0) {
      // Keep the LED permanently on
      break;
    }
    if (currentMillis - _lastToggleTimestamp > _onInterval) {
      // Turn off when on interval is over
      _lastToggleTimestamp = millis();
      _state = LOW;
      digitalWrite(_pin, _state);
    }
    break;
  
  default:
    break;
  }
}
