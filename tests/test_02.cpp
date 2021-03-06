#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"

char TCCR4A;
char TCCR4B;
char TCCR4C;
char TIMSK4;
char TIFR4;
unsigned short OCR4A;
unsigned short OCR4B;

StepperQueue fas_queue[NUM_QUEUES];

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#include "RampChecker.h"

void init_queue() {
  fas_queue[0].read_idx = 0;
  fas_queue[1].read_idx = 0;
  fas_queue[0].next_write_idx = 0;
  fas_queue[1].next_write_idx = 0;
}

void basic_test_with_empty_queue() {
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0, 0);
  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  assert(s.isQueueEmpty());
  s.setSpeed(10000);
  s.setAcceleration(100);
  s.fill_queue();
  assert(s.isQueueEmpty());
  s.move(1000);
  s.fill_queue();
  assert(!s.isQueueEmpty());
  for (int i = 0; i < 1000; i++) {
    if (false) {
      printf(
          "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
          "pos = %d  QueueEmpty=%s\n",
          i, fas_queue[0].read_idx, fas_queue[0].next_write_idx, s.targetPos(),
          s.getPositionAfterCommandsCompleted(),
          s.isQueueEmpty() ? "yes" : "no");
    }
    if (!s.isRampGeneratorActive()) {
      break;
    }
    s.fill_queue();
    while (!s.isQueueEmpty()) {
      rc.check_section(
          &fas_queue_A.entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
      fas_queue[0].read_idx++;
    }
  }
  test(!s.isRampGeneratorActive(), "too many commands created");
  printf("min_dt=%u\n", rc.min_dt);
  test(rc.min_dt == 160000, "max speed not reached");
}

void test_with_pars(const char *name, int32_t steps, uint32_t travel_dt,
                    uint32_t accel, bool reach_max_speed, float min_time,
                    float max_time, float allowed_ramp_time_delta) {
  printf("Test %s test_with_pars steps=%d travel_dt=%d accel=%d dir=%s\n", name,
         steps, travel_dt, accel, reach_max_speed ? "CW" : "CCW");
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0, 0);
  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  assert(s.isQueueEmpty());
  s.setSpeed(travel_dt);
  s.setAcceleration(accel);
  s.fill_queue();
  assert(s.isQueueEmpty());
  s.move(steps);
  s.fill_queue();
  assert(!s.isQueueEmpty());
  float old_planned_time_in_buffer = 0;
  char fname[100];
  sprintf(fname, "test_02_%s.gnuplot", name);
  FILE *gp_file = fopen(fname, "w");
  fprintf(gp_file, "$data <<EOF\n");
  for (int i = 0; i < steps; i++) {
    if (true) {
      printf(
          "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
          "pos = %d  QueueEmpty=%s\n",
          i, fas_queue[0].read_idx, fas_queue[0].next_write_idx, s.targetPos(),
          s.getPositionAfterCommandsCompleted(),
          s.isQueueEmpty() ? "yes" : "no");
    }
    if (!s.isRampGeneratorActive()) {
      break;
    }
    s.fill_queue();
    uint32_t from_dt = rc.total_ticks;
    while (!s.isQueueEmpty()) {
      rc.check_section(
          &fas_queue_A.entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
      fas_queue[0].read_idx++;
      fprintf(gp_file, "%.6f %.2f %d\n", rc.total_ticks / 1000000.0,
              16000000.0 / rc.last_dt, rc.last_dt);
    }
    uint32_t to_dt = rc.total_ticks;
    float planned_time = (to_dt - from_dt) * 1.0 / 16000000;
    printf("planned time in buffer: %.6fs (old=%.6fs)\n", planned_time,
           old_planned_time_in_buffer);
    // This must be ensured, so that the stepper does not run out of commands
    assert((i == 0) || (old_planned_time_in_buffer > 0.005));
    old_planned_time_in_buffer = planned_time;
  }
  fprintf(gp_file, "EOF\n");
  fprintf(gp_file, "plot $data using 1:2 with linespoints\n");
  fprintf(gp_file, "pause -1\n");
  fclose(gp_file);
  printf("TEST=%s\n", name);
  test(!s.isRampGeneratorActive(), "too many commands created");
  printf("Total time  %f < %f < %f ?\n", min_time, rc.total_ticks / 16000000.0,
         max_time);
  test(rc.total_ticks / 16000000.0 > min_time, "ramp too fast");
  test(rc.total_ticks / 16000000.0 < max_time, "ramp too slow");
  if (reach_max_speed) {
    printf("%d = %d ?\n", rc.min_dt, travel_dt * 16);
    test(rc.min_dt == travel_dt * 16, "max speed not reached");
  } else {
    printf("%d > %d ?\n", rc.min_dt, travel_dt * 16);
    test(rc.min_dt > travel_dt * 16, "max speed reached");
  }
  float up_time, down_time;
  if (reach_max_speed) {
    printf("Ramp time up/coast/down/total=");
    up_time = 1.0 * rc.accelerate_till / 16000000.0;
    down_time = (1.0 * rc.total_ticks - 1.0 * rc.coast_till) / 16000000.0;
    printf(" %f", 1.0 * rc.accelerate_till / 16000000.0);
    printf(" %f", 1.0 * (rc.coast_till - rc.accelerate_till) / 16000000.0);
    printf(" %f", 1.0 * (rc.total_ticks - rc.coast_till) / 16000000.0);
    assert(rc.total_ticks > rc.coast_till);
  } else {
    printf("Ramp time up/down/total =");
    up_time = 1.0 * rc.accelerate_till / 16000000.0;
    down_time = (1.0 * rc.total_ticks - 1.0 * rc.accelerate_till) / 16000000.0;
    printf(" %f", 1.0 * rc.accelerate_till / 16000000.0);
    printf(" %f", 1.0 * (rc.total_ticks - rc.accelerate_till) / 16000000.0);
  }
  printf(" %f\n", 1.0 * rc.total_ticks / 16000000.0);
  // turned off
  // test(abs(up_time - down_time) <
  //         0.5 * (up_time + down_time) * allowed_ramp_time_delta,
  //     "assymmetric ramp");
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  printf("CHECKSUM for %d/%d/%d: %d\n", steps, travel_dt, accel, s.checksum);
#endif
}

int main() {
  basic_test_with_empty_queue();
  //             steps  ticks_us  accel    maxspeed  min/max_total_time
  // jumps in speed in real on esp32
  test_with_pars("f1", 1000, 4300, 10000, true, 4.5 - 0.2, 4.5 + 0.2, 0.5);

  // ramp time 2s, 400 steps TODO
  test_with_pars("f2", 10000, 5000, 100, true, 2 * 2.0 + 46.0 - 1.0,
                 2 * 2.0 + 46.0 + 2.0, 0.2);
  // ramp time 0.02s, 4 steps
  test_with_pars("f3", 1600, 5000, 10000, true, 7.9, 8.1, 0.2);
  // ramp time 0.2s, 20 steps
  test_with_pars("f4", 1600, 5000, 1000, true, 2 * 0.2 + 7.8 - 0.1 - 0.1,
                 2 * 0.2 + 7.8 + 0.1, 0.2);
  // ramp time 1s, 5000 steps
  test_with_pars("f5", 15000, 100, 10000, true, 2 * 1.0 + 0.5 - 0.17,
                 2 * 1.0 + 0.5 + 0.1, 0.2);
  // ramp time 0.02s, 4 steps
  test_with_pars("f6", 100, 5000, 10000, true, 2 * 0.02 + 0.48 - 0.1,
                 2 * 0.02 + 0.48 + 0.1, 0.2);
  // ramp time 2s, 20000 steps => only ramp 0.22s
  test_with_pars("f7", 500, 50, 10000, false, 2 * 0.22 - 0.1, 2 * 0.22 + 0.11,
                 0.2);
  // ramp time 4s, 8000 steps
  test_with_pars("f8", 128000, 250, 1000, true, 2 * 2.0 + 30.0 - 0.1,
                 2 * 2.0 + 30.0 + 0.1 + 1.9, 0.2);
  // ramp time 4s, 8000 steps
  test_with_pars("f9", 72000, 250, 1000, true, 2 * 2.0 + 15.0 - 0.1,
                 2 * 2.0 + 15.0 + 0.1 + 2 * 1.7, 0.2);
  // ramp time 4s, 8000 steps
  test_with_pars("f10", 44000, 250, 1000, true, 2 * 2.0 + 7.5 - 0.1,
                 2 * 2.0 + 7.5 + 0.1 + 2 * 1.7, 0.2);
  // ramp time 4s, 8000 steps
  test_with_pars("f11", 16002, 250, 1000, true, 2 * 2.0 + 0.0 - 0.1,
                 2 * 2.0 + 0.0 + 0.1 + 4.0, 0.2);
  // ramp time 50s => 2s
  test_with_pars("f12", 1000, 20, 1000, false, 2 * 1.0 - 0.1, 2 * 1.0 + 0.1,
                 0.2);

  // ramp time 50s, thus with 500steps max speed not reached. 250steps need 10s
  //  test_with_pars("f13", 500, 4000, 5, false, 20.0 - 0.6, 20.0 + 0.2, 0.2);
  ////  test_with_pars("f14", 2000, 4000, 5, false, 40.0 - 0.6, 40.0 + 0.2,
  /// 0.2);
  // ramp time 50s with 6250 steps => 4000 steps at max speed using 1s
  test_with_pars("f15", 12600, 4000, 5, true, 100.0 - 0.7, 100.0 + 0.2, 0.2);
  // ramp time 50s with 6250 steps => 4000 steps at max speed using 16s
  test_with_pars("f16", 16500, 4000, 5, true, 116.0 - 0.7, 116.0 + 0.2, 0.2);

  // jumps in speed in real => WORKS NOW
  test_with_pars("f17", 256000, 40, 5000, true, 15.2 - 0.1, 15.2 + 0.2, 0.2);

  // ramp time  625s, 7812500 steps
  // test_with_pars("f18", 2000000, 40, 40, false, 2*223.0, 2*223.0);

  // slow ramp time
  //  test_with_pars("f19", 1000, 10, 1, false, 62.0, 63.0, 1.0);

  // name, steps, travel_dt, accel, reach_max_speed, min_time, max_time,
  // allowed_ramp_time_delta slow ramp time Those are anomalies (see github
  // issue #8) on avr, but not on PC
  //  test_with_pars("f20", 50000, 270000, 10, true, 62.0, 63.0, 1.0);
  //  test_with_pars("f20", 20, 270000, 1, true, 62.0, 63.0, 1.0);

  printf("TEST_02 PASSED\n");
}
