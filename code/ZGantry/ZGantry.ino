// A4988 stepper driver controlling NEMA 17 Motor through Arduino Uno R4 w CNC Shield 

#include <AccelStepper.h>
#include "Serial.h"

// A4988 set to "Y" Driver seat
#define STEP_PIN    3
#define DIR_PIN     6
#define EN_PIN      8

// --- Motor / motion parameters ---
const int STEPS_PER_REV = 200;     // 1.8 deg/step motor, full-step
const int MICROSTEPS    = 1;       // set via A4988 MS1/MS2/MS3 pins; update this to match
const long STEPS_PER_REV_MICRO = (long)STEPS_PER_REV * MICROSTEPS;

// Tune these to your mechanics (load, belt/screw ratio, driver current limit)
const float MAX_SPEED_SPS     = 2000.0;  // steps/sec at full speed
const float ACCELERATION_SPS2 = 4000.0;  // steps/sec^2 ramp rate

char buf[32];
uint8_t idx = 0;
bool goingForward = true;

// AccelStepper in DRIVER mode: 1 = STEP/DIR interface (what the A4988 uses)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Active Low

  stepper.setMaxSpeed(MAX_SPEED_SPS);
  stepper.setAcceleration(ACCELERATION_SPS2);

  Serial.begin(9600);
  while(!Serial);
  Serial.println("Init");
}

void loop() {
  // Example move sequence — replace with your actual motion calls.
  // stepper.run() must be called as often as possible (every loop iteration,
  // no blocking delays) to keep step pulses flowing smoothly.


  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        buf[idx] = '\0';
        handleCommand(buf);
        idx = 0;
      }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }

  stepper.run();  // non-blocking; call every iteration, do NOT use delay() anywhere in loop()
}

void handleCommand(const char* cmd) {
  if (cmd[0] == 'F') 
  {                    
    // e.g. "F1600" = move 1600 steps Forward
    long steps = atol(cmd + 1);
    stepper.move(steps);
      Serial.print("Writing");
      Serial.println(steps);
  }
  else if (cmd[0] == 'B')
  {
    // e.g. "B1600" = move 1600 steps backwards
    long steps = atol(cmd + 1);
    stepper.move(-1 * steps);
      Serial.print("Writing ");
      Serial.println(steps);
  }
}
