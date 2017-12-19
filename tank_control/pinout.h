#ifndef PINOUT_H
#define PINOUT_H
/* ------------------------------------------------------------------------- *\
| Pinout File - Tank XY Control                                               |
| larry12193@gmail.com                                                        |
|  -------------------------------------------------------------------------  |
| Revision:                                                                   |
|  =========================================================================  |
| 2017-03-11: Initial commit                                                  |
| --------------------------------------------------------------------------  |
| 2017-04-30: Finalized pinout added                                          |
\* ------------------------------------------------------------------------- */

#define Y_ENC_A     2   // Y axis quadrature encoder signal A
#define Y_ENC_B     1   // Y axis quadrature encoder signal B

#define Y_STOP_1    22  // Y axis endstop 1
#define Y_STOP_2    23  // Y axis endstop 2

#define Y_DIR_PIN   35  // Y axis stepper driver direction
#define Y_STEP_PIN  34  // Y axis stepper driver step
#define Y_ENB_PIN   36  // Y axis stepper driver enable

#define X_ENC_A     3   // X axis quadrature encoder signal A
#define X_ENC_B     0   // X axis quadrature encoder signal B

#define X_STOP_1    20  // X axis endstop 1
#define X_STOP_2    21  // X axis endstop 2

#define X_DIR_PIN   37  // X axis stepper driver direction
#define X_STEP_PIN  38  // X axis stepper driver step
#define X_ENB_PIN   39  // X axis stepper driver enable

#endif
