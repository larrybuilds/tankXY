/* ------------------------------------------------------------------------- *\
| Tank XY Control                                                             |
| larry12193@gmail.com                                                        |
|  -------------------------------------------------------------------------  |
| Revision:                                                                   |
|  =========================================================================  |
| 2017-03-11: Initial commit                                                  |
\* ------------------------------------------------------------------------- */


// Adapted from:
//------------------------------------------------------------------------------
// 2 Axis CNC Demo
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

#include "stp_drv_6575.h"
#include "config.h"
#include "pinout.h"
#include <stdint.h>

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

char  buffer[MAX_BUF];  // where we store the message until we get a newline
int   sofar;            // how much is in the buffer
float px, py;      // location

// speeds
float fr =     0;  // human version
long  step_delay;  // machine version

// settings
char mode_abs=1;   // absolute mode?

STP_DRV_6575 xMotor(X_AXIS_SPR, X_DIR_PIN, X_ENB_PIN, X_STEP_PIN);
STP_DRV_6575 yMotor(Y_AXIS_SPR, Y_DIR_PIN, Y_ENB_PIN, Y_STEP_PIN);

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  nfr = nfr/PULLY_CIRC*X_AXIS_SPR;
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_STEPRATE || nfr<MIN_STEPRATE) {  // don't allow crazy feed rates
    return;
  }
  step_delay = 1000000.0/nfr;
  fr = nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {  
  long i;
  long over= 0;

//  Serial.print("Newx:");
//  Serial.print(newx);
//  Serial.print(" Newy");
//  Serial.println(newy);

 
  
  newx = newx/PULLY_CIRC*X_AXIS_SPR;
  newy = newy/PULLY_CIRC*X_AXIS_SPR;
  
  
//  Serial.print("Newx:");
//  Serial.print(newx);
//  Serial.print(" Newy");
//  Serial.println(newy);
  long dx  = newx-px;
  long dy  = newy-py;
  int dirx = dx>0?CCW:CW;
  int diry = dy>0?CCW:CW;  // because the motors are mounted in opposite directions
//  Serial.print("Dirx:");
//  Serial.print(dirx);
//  Serial.print(" Diry:");
//  Serial.print(diry);
//  Serial.print(" Dx:");
//  Serial.print(dx);
//  Serial.print(" Dy:");
//  Serial.println(dy);
  
  dx = abs(dx);
  dy = abs(dy);
  
  xMotor.setDir(dirx);
  yMotor.setDir(diry);
  
  if(dx>dy) {
    over = dx/2;
//    Serial.println("dx>dy");
    for(i=0; i<dx; ++i) {
      xMotor.step(1);
      over += dy;
      if(over>=dx) {
        over -= dx;
        yMotor.step(1);
      }
      pause(step_delay);
    }
  } else {
//    Serial.println("dx<dy");
    over = dy/2;
    for(i=0; i<dy; ++i) {
      yMotor.step(1);
      over += dx;
      if(over >= dy) {
        over -= dy;
        xMotor.step(1);
      }
      pause(step_delay);
    }
  }
   px = newx;
   py = newy;
  
}


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a = atan2(dy,dx);
  if(a<0) a = (PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = (px*PULLY_CIRC/X_AXIS_SPR) - cx;
  float dy = (py*PULLY_CIRC/X_AXIS_SPR) - cy;
  float radius=sqrt(dx*dx+dy*dy);
  
//  Serial.print("dx = ");
//  Serial.print(dx);
//  Serial.print("    dy = ");
//  Serial.print(dy);
//  Serial.print("    px = ");
//  Serial.print(px);
//  Serial.print("    py = ");
//  Serial.print(py);
//  Serial.print("    cx = ");
//  Serial.print(cx);
//  Serial.print("    cy = ");
//  Serial.println(cy);
  
  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
} 


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 

void runArc() {
  // Set feedrate to 10mm/s
  feedrate(10);
  // Run 0.9m diameter, 180deg arc clockwise
  arc(450,0,900,0,1);
  delay(500);
  // Run 0.9m diameter, 180deg arc counter-clockwise
  arc(450,0,0,0,-1);
}

void raster() {
  delay(2000);
  // Set feedrate to 10mm/s
  feedrate(10);
  // Move 0.9m towards the desk
  line(900,0);
  // Move back to origin
  line(0,0);
}

void square() {
  delay(2000);
  // Set feedrate to 10mm/s
  feedrate(10);
  // Move 0.7m towards the desk
  line(700,0);
  waiting();  
  line(700,700);
  waiting();
  line(0,700);
  waiting();
  line(0,0);
}

void waiting() {
  Serial.println ("Hit enter to continue");     // signal initalization done
  while(Serial.available() == 0){
  }
}
}
/**
 * display helpful information
 */
void help() {
  Serial.print(F("Medium Tank XY-Stage Control"));
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(mm)] [Y(mm)] [F(mm/s)]; - line"));
  Serial.println(F("G01 [X(mm)] [Y(mm)] [F(mm/s)]; - line"));
  Serial.println(F("G02 [X(mm)] [Y(mm)] [I(mm)] [J(mm)] [F(mm/s)]; - clockwise arc"));
  Serial.println(F("G03 [X(mm)] [Y(mm)] [I(mm)] [J(mm)] [F(mm/s)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(mm)] [Y(mm)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("M254; - run Arc scan"));
  Serial.println(F("M255; - raster scan"));
  Serial.println(F("M256; - square scan"));
  Serial.println(F("All commands must end with a newline."));
  Serial.print("Maximum feedrate:");
  Serial.println(MAX_FEEDRATE);
  Serial.print("Minimum feedrate:");
  Serial.println(MIN_FEEDRATE);
  Serial.print("mm/step:");
  Serial.println(MM_PER_STEP);
  
}

void disable() {
  xMotor.disable();
  yMotor.disable();
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parsenumber('F',fr));
      arc(parsenumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0) );
    break;
  case 255:
    help();
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  case 254:  runArc(); break;
  case 255:  raster(); break;
  case 256:  square(); break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
  xMotor.enable();
  yMotor.enable();
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  delay(5000);
  //setup_controller();  
  position(0,0);  // set staring position
  feedrate((MAX_FEEDRATE + MIN_FEEDRATE)/2);  // set default speed
  help();  // say hello
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if((c=='\n') || (c == '\r')) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/
