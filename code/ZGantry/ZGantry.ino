// Using 3 Stepper motors to control X, Z, and elevator. 
// Z and Elevator (Y) steppers are classic NEMA 17
// X is a NEMA 23 Stepper with it's own closed loop driver (CL57Y)

// TODO: Need to add arithmetic to account for different sensitivity settings for CL57Y

#include "Serial.h"
#include "AccelStepper.h"

// #define SKIP_HOMING 

// pinouts
#define X_STEP_PIN      2
#define X_DIR_PIN       5
#define Y_STEP_PIN      3
#define Y_DIR_PIN       6
#define NEN_PIN         8
#define Z_STEP_PIN      4
#define Z_DIR_PIN       7
#define A_DIR_PIN       12
#define A_STEP_PIN      13
#define X_ENDSTOP       9
#define Y_ENDSTOP       10 
#define Z_ENDSTOP       11

enum FSM {
    HOME,
    HOME_X, 
    HOME_Z, 
    HOME_ELEVATOR,
    PICK_DESCEND, 
    PICK_ASCEND, 
    MOVE_FORWARD, 
    PLACE_DESCEND, 
    RETURN, 
    PAUSE, 
    FAULT    
} FSM_STATE; 

// I need to figure these out later
// TODO: Add parameters per motor
const int STEPS_PER_REV = 200;     // 1.8 deg/step motor, full-step
const int MICROSTEPS    = 1;       // set via A4988 MS1/MS2/MS3 pins; update this to match
const long STEPS_PER_REV_MICRO = (long)STEPS_PER_REV * MICROSTEPS;

// Making these adjustable
const float X_MAX_SPEED     = 2000.0;  // steps/sec at full speed
const float X_ACCELERATION = 4000.0;  // steps/sec^2 ramp rate
const float Y_MAX_SPEED     = 2000.0;  // steps/sec at full speed
const float Y_ACCELERATION = 4000.0;  // steps/sec^2 ramp rate
const float Z_MAX_SPEED     = 2000.0;  // steps/sec at full speed
const float Z_ACCELERATION = 4000.0;  // steps/sec^2 ramp rate

// For Serial 
char buf[32];
uint8_t idx = 0;

// For Motors 
int16_t x_step_count; 
int16_t y_step_count; 
int16_t z_step_count; 
int16_t a_step_count; 

// AccelStepper in DRIVER mode: 1 = STEP/DIR interface (what the A4988 uses)
AccelStepper Xstepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper Ystepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN); // Elevator
AccelStepper Zstepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper Astepper(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN); 

void setup() {
    pinMode(NEN_PIN, OUTPUT);
    pinMode(X_ENDSTOP, INPUT_PULLUP);
    pinMode(Y_ENDSTOP, INPUT_PULLUP);
    pinMode(Z_ENDSTOP, INPUT_PULLUP);
    digitalWrite(NEN_PIN, LOW);

    Xstepper.setMaxSpeed(X_MAX_SPEED);
    Xstepper.setAcceleration(X_ACCELERATION);
    Ystepper.setMaxSpeed(Y_MAX_SPEED);
    Ystepper.setAcceleration(Y_ACCELERATION);
    Zstepper.setMaxSpeed(Z_MAX_SPEED);
    Zstepper.setAcceleration(Z_ACCELERATION);

    Serial.begin(115200);
    while(!Serial);
    Serial.println("Init");
}

uint32_t old_time = micros(); 
uint32_t current_time = micros(); 

void loop() {
    // Read input for parameters
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                handleCommand(buf);
                idx = 0;
            }
        } 
        else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
    Xstepper.run();  
    Ystepper.run();  
    Zstepper.run();  
    bool MotorsOn = false;
    MotorsOn = (Xstepper.distanceToGo() || 
                Ystepper.distanceToGo() ||
                Zstepper.distanceToGo() || 
                Astepper.distanceToGo() ); 
    digitalWrite(NEN_PIN, MotorsOn ? LOW : HIGH); 

    if (!MotorsOn) // NEGEDGE 
    {
        delay(10); // Tune for minimum until steps aren't missed. 
        // Need to figure out how to axe this while ensuring that steps aren't missed
        switch (FSM_STATE)
        {
            case HOME: 
                FSM_STATE = HOME_ELEVATOR;
                Zstepper.move(-2000); 

                #ifdef SKIP_HOMING
                    FSM_STATE = PICK_DESCEND; 
                #endif

                break;

            case HOME_X: 
                if (!digitalRead(X_ENDSTOP)) 
                {
                    // FSM_STATE = HOME_Z;
                    x_step_count = 0; 
                    Xstepper.move(-1300); 

                }                
                else 
                {
                    Xstepper.move(1); 
                }
                Serial.println(digitalRead(X_ENDSTOP));
                break; 

            case HOME_Z: 
                // Xstepper.move(-10); 
                Serial.println("ENDSTOP");
                FSM_STATE = HOME_ELEVATOR; 
                break;

            case HOME_ELEVATOR: 
                Zstepper.move(10); 

                break; 

            case PICK_DESCEND: 
                Ystepper.move(200);
                FSM_STATE = PICK_ASCEND;
                break;

            case PICK_ASCEND: 
                Ystepper.move(-200);
                FSM_STATE = MOVE_FORWARD;
                break;

            case MOVE_FORWARD: 
                Xstepper.move(-500); 
                FSM_STATE = PLACE_DESCEND;
                break;

            case PLACE_DESCEND: 
                // Later add X movement to match belt 
                Ystepper.move(200);
                FSM_STATE = RETURN;
                break;

            case RETURN:
                Xstepper.move(500); 
                Ystepper.move(-200);
                FSM_STATE = PICK_DESCEND;
                break;

            case PAUSE: 
                break;

            case FAULT: 
                while(1); 
                break;

            default: 
                Serial.println("FSM DEFAULT CASE"); 
                FSM_STATE = PICK_ASCEND;
                break; 
        }
        if (x_step_count > 1300 || x_step_count < 0) 
        {
            Serial.println("x_step_count out of bounds");
            FSM_STATE = FAULT; 
        }
        Serial.print("FSM_STATE = ");
        Serial.println(FSM_STATE);
    }
}

void handleCommand(const char* cmd) {
    long steps = 0; 
    switch (cmd[0])
    {
        
    case 'Z':
        // e.g. "Z1600" = move 1600 steps Forward
        steps = atol(cmd + 1);
        Zstepper.move(steps);
        Serial.print("Writing Forward ");
        Serial.println(steps);
        break;

    case 'z':
        // e.g. "z1600" = move 1600 steps backwards
        steps = atol(cmd + 1);
        Zstepper.move(-1 * steps);
        Serial.print("Writing Backwards ");
        Serial.println(steps);
        break;

    case 'Y':
        // e.g. "Y1600" = move 1600 steps Forward
        steps = atol(cmd + 1);
        Ystepper.move(steps);
        Serial.print("Writing Forward");
        Serial.println(steps);
        break;

    case 'y':
        // e.g. "y1600" = move 1600 steps backwards
        steps = atol(cmd + 1);
        Ystepper.move(-1 * steps);
        Serial.print("Writing Backwards ");
        Serial.println(steps);
        break;

    case 'X':
        // e.g. "X1600" = move 1600 steps Forward
        steps = atol(cmd + 1);
        Xstepper.move(steps);
        Serial.print("Writing Forward");
        Serial.println(steps);
        break;

    case 'x':
        // e.g. "x1600" = move 1600 steps backwards
        steps = atol(cmd + 1);
        Xstepper.move(-1 * steps);
        Serial.print("Writing Backwards ");
        Serial.println(steps);
        break;

    case 'S':
        Xstepper.setMaxSpeed(atof(cmd + 1));
        break;

    case 'A':
        Xstepper.setAcceleration(atof(cmd + 1));
        break;

    default: 
        Serial.println("Default Switch Triggered. Uh Oh");
    }
}
