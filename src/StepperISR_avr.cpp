#include "FastAccelStepper.h"
#include "StepperISR.h"

#if defined(ARDUINO_ARCH_AVR)

#define Stepper_Toggle(X) TCCR4A = (TCCR4A | _BV(COM4##X##0)) & ~_BV(COM4##X##1)
#define Stepper_Zero(X) TCCR4A = (TCCR4A | _BV(COM4##X##1)) & ~_BV(COM4##X##0)
#define Stepper_Disconnect(X) \
  TCCR4A = (TCCR4A & ~(_BV(COM4##X##1) | _BV(COM4##X##0)))
#define Stepper_IsToggling(X) \
  ((TCCR4A & (_BV(COM4##X##0) | _BV(COM4##X##1))) == _BV(COM4##X##0))
#define Stepper_IsDisconnected(X) \
  ((TCCR4A & (_BV(COM4##X##0) | _BV(COM4##X##1))) == 0)

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  _initVars();

  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  if (step_pin == stepPinStepperA) {
    isChannelA = true;
    noInterrupts();
    Stepper_Disconnect(A);
    TCCR4C = _BV(FOC4A);     // force compare to ensure disconnect
    TIMSK4 &= ~_BV(OCIE4A);  // disable compare A interrupt
    interrupts();
  }
  if (step_pin == stepPinStepperB) {
    isChannelA = false;
    noInterrupts();
    Stepper_Disconnect(B);
    TCCR4C = _BV(FOC4B);     // force compare to ensure disconnect
    TIMSK4 &= ~_BV(OCIE4B);  // disable compare B interrupt
    interrupts();
  }
}

#define AVR_STEPPER_ISR(CHANNEL, queue, ocr, foc)                     \
  ISR(TIMER4_COMP##CHANNEL##_vect) {                                  \
    uint8_t rp = queue.read_idx;                                      \
    if (Stepper_IsToggling(CHANNEL)) {                                \
      TCCR4C = _BV(foc); /* clear bit */                              \
      struct queue_entry* e = &queue.entry[rp & QUEUE_LEN_MASK];      \
      if (e->steps-- > 0) {                                           \
        /* perform another steps_dir with this queue entry */         \
        ocr += queue.ticks;                                           \
        return;                                                       \
      }                                                               \
      rp++;                                                           \
      queue.read_idx = rp;                                            \
    } else if (!Stepper_IsDisconnected(CHANNEL)) {                    \
      rp++;                                                           \
      queue.read_idx = rp;                                            \
    }                                                                 \
    if (rp == queue.next_write_idx) {                                 \
      /* queue is empty => set to disconnect */                       \
      Stepper_Disconnect(CHANNEL);                                    \
      /* disable compare interrupt */                                 \
      TIMSK4 &= ~_BV(OCIE4##CHANNEL);                                 \
      /* force compare to ensure disconnect */                        \
      TCCR4C = _BV(FOC1##CHANNEL);                                    \
      queue.isRunning = false;                                        \
      queue.queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;                \
      return;                                                         \
    }                                                                 \
    /* command in queue */                                            \
    struct queue_entry* e = &queue.entry[rp & QUEUE_LEN_MASK];        \
    ocr += (queue.ticks = e->ticks);                                  \
    /* assign to skip and test for not zero */                        \
    if (e->steps == 0) {                                              \
      Stepper_Zero(CHANNEL);                                          \
    } else {                                                          \
      Stepper_Toggle(CHANNEL);                                        \
    }                                                                 \
    if (e->toggle_dir) {                                              \
      uint8_t dirPin = queue.dirPin;                                  \
      digitalWrite(dirPin, digitalRead(dirPin) == HIGH ? LOW : HIGH); \
    }                                                                 \
  }
AVR_STEPPER_ISR(A, fas_queue_A, OCR4A, FOC4A)
AVR_STEPPER_ISR(B, fas_queue_B, OCR4B, FOC4B)

void StepperQueue::startQueue() {
  isRunning = true;
  if (isChannelA) {
    noInterrupts();
    TIFR4 = _BV(OCF4A);     // clear interrupt flag
    TIMSK4 |= _BV(OCIE4A);  // enable compare A interrupt
    OCR4A = TCNT1 + 40;     // definite start point
    interrupts();
  } else {
    noInterrupts();
    TIFR4 = _BV(OCF4B);     // clear interrupt flag
    TIMSK4 |= _BV(OCIE4B);  // enable compare A interrupt
    OCR4B = TCNT1 + 40;     // definite start point
    interrupts();
  }
}
void StepperQueue::forceStop() {
  if (isChannelA) {
    /* disable compare interrupt */
    TIMSK4 &= ~_BV(OCIE4A);
    /* set to disconnect */
    Stepper_Disconnect(A);
    /* force compare to ensure disconnect */
    TCCR4C = _BV(FOC4A);
  } else {
    /* disable compare interrupt */
    TIMSK4 &= ~_BV(OCIE4B);
    /* set to disconnect */
    Stepper_Disconnect(B);
    /* force compare to ensure disconnect */
    TCCR4C = _BV(FOC4B);
  }
  isRunning = false;
  queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;

  // empty the queue
  read_idx = next_write_idx;
}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}
#endif
