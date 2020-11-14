#ifndef STUBS_H
#define STUBS_H

#define PROGMEM
#define pgm_read_byte_near(x) (*(x))

// For inducing interrupts while testing
void noInterrupts();
void interrupts();
void inject_fill_interrupt(int mark);

#define _BV(x) 0
#define ISR(x) void x()
#define inline
#define micros() 0

#include <math.h>

#define abs(x) ((x) > 0 ? (x) : -(x))
#define min(a, b) ((a) > (b) ? (b) : (a))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define digitalWrite(a, b) \
  {}
#define pinMode(a, b) \
  {}

extern char TCCR4A;
extern char TCCR4B;
extern char TCCR4C;
extern char TIMSK4;
extern char TIFR4;
extern unsigned short OCR4A;
extern unsigned short OCR4B;

#define test(x, msg) \
  if (!(x)) {        \
    puts(msg);       \
    assert(false);   \
  };
#endif
