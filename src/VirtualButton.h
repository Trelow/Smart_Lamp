#ifndef _VirtualButton_h
#define _VirtualButton_h
#define _VB_DEB 50
#define _VB_CLICK 400
#include <Arduino.h>
#ifndef VB_DEB
#define VB_DEB _VB_DEB
#endif
#ifndef VB_CLICK
#define VB_CLICK _VB_CLICK
#endif

class VButton {
public:
  void setHoldTimeout(uint16_t tout) { _holdT = tout >> 5; }
  void setStepTimeout(uint16_t tout) { _stepT = tout >> 5; }
  bool poll(bool s) {
    uint16_t prev = _flags;
    if (s || readF(9))
      pollBtn(s);
    return (prev != _flags);
  }
  void reset() { _flags = 0; }
  bool busy() { return readF(9); }
  bool press() { return checkF(3); }
  bool release() { return checkF(10); }
  bool click() { return checkF(0); }
  bool held() { return checkF(1); }
  bool hold() { return readF(4); }
  bool step() { return checkF(2); }
  bool releaseStep() { return checkF(12); }
  bool held(uint8_t clk) { return (clicks == clk) ? checkF(1) : 0; }
  bool hold(uint8_t clk) { return (clicks == clk) ? readF(4) : 0; }
  bool step(uint8_t clk) { return (clicks == clk) ? checkF(2) : 0; }
  bool releaseStep(uint8_t clk) { return (clicks == clk) ? checkF(12) : 0; }
  bool hasClicks(uint8_t num) { return (clicks == num && checkF(7)) ? 1 : 0; }
  uint8_t hasClicks() { return checkF(6) ? clicks : 0; }
  bool timeout(uint16_t tout) {
    return ((uint16_t)(millis() & 0xFFFF) - _debTmr > tout && checkF(15));
  }
  uint8_t clicks = 0;

private:
  void pollBtn(bool state) {
    uint16_t ms = millis() & 0xFFFF;
    uint16_t debounce = ms - _debTmr;
    if (state) {
      setF(9);
      if (!readF(8)) {
        if (readF(14)) {
          if (debounce > VB_DEB) {
            _flags |= 0b100001000;
            _debTmr = ms;
          }
        } else {
          setF(14);
          if (debounce > VB_CLICK || readF(5)) {
            clicks = 0;
            _flags &= ~0b0011000011101111;
          }
          _debTmr = ms;
        }
      } else {
        if (!readF(4)) {
          if (debounce >= (uint16_t)(_holdT << 5)) {
            _flags |= 0b00110010;
            _debTmr = ms;
          }
        } else {
          if (debounce > (uint16_t)(_stepT << 5)) {
            _flags |= 0b0010000000000100;
            _debTmr = ms;
          }
        }
      }
    } else {
      if (readF(8)) {
        if (debounce > VB_DEB) {
          if (!readF(4)) {
            setF(0);
            clicks++;
          }
          _flags &= ~0b100010000;
          _debTmr = ms;
          _flags |= (1 << 10) | (1 << 15);
          if (checkF(13))
            setF(12);
        }
      } else if (clicks && !readF(5)) {
        if (debounce > VB_CLICK)
          _flags |= 0b11100000;
      } else
        clrF(9);
      checkF(14);
    }
  }
  bool checkF(const uint8_t val) { return readF(val) ? clrF(val), 1 : 0; }
  inline void setF(const uint8_t x) __attribute__((always_inline)) {
    _flags |= 1 << x;
  }
  inline void clrF(const uint8_t x) __attribute__((always_inline)) {
    _flags &= ~(1 << x);
  }
  inline bool readF(const uint8_t x) __attribute__((always_inline)) {
    return _flags & (1 << x);
  }
  uint16_t _flags = 0;
  uint8_t _holdT = 1000 >> 5;
  uint8_t _stepT = 500 >> 5;
  uint16_t _debTmr = 0;
};
#endif
