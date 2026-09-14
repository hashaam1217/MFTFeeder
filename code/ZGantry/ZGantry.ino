// Using 3 Step0per motors to control X, Z, and elevator. 
// Z and Elevator (Y) steppers are classic NEMA 17
// X is a NEMA 23 Stepper with it's own closed loop driver (CL57Y)

// TODO: Need to add arithmetic to account for different sensitivity settings for CL57Y

#include "Serial.h"
#include "AccelStepper.h"
#include "ArduinoLog.h"

// #define SKIP_HOMING 

// pinouts
#define X_STEP_PIN      2
#define X_DIR_PIN       5
#define Y_STEP_PIN      3
#define Y_DIR_PIN       6
#define NEN_PIN         8
#define Z_STEP_PIN      4
#define Z_DIR_PIN       7
// #define A_DIR_PIN       12
// #define A_STEP_PIN      13
#define SOLENOID_PIN    12
#define X_ENDSTOP       9
#define Y_ENDSTOP       10 
#define Z_ENDSTOP       11

#define DESCEND_DEPTH 2400 //2400 default 

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
const float Y_MAX_SPEED     = 20000.0;  // steps/sec at full speed
const float Y_ACCELERATION = 4000.0;  // steps/sec^2 ramp rate
const float Z_MAX_SPEED     = 10 * 2000.0;  // steps/sec at full speed
const float Z_ACCELERATION = 10 * 4000.0;  // steps/sec^2 ramp rate

// For Serial 
char buf[32];
uint8_t idx = 0;

// For Motors 
int32_t x_step_count; 
int32_t y_step_count; 
int32_t z_step_count; 
int32_t a_step_count; 

// AccelStepper in DRIVER mode: 1 = STEP/DIR interface (what the A4988 uses)
AccelStepper Xstepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper Ystepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN); // Elevator
AccelStepper Zstepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
#ifdef A_DIR_PIN
    AccelStepper Astepper(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN); 
#endif

void setup() {
    pinMode(NEN_PIN, OUTPUT);
    pinMode(X_ENDSTOP, INPUT_PULLUP);
    pinMode(Y_ENDSTOP, INPUT_PULLUP);
    pinMode(Z_ENDSTOP, INPUT_PULLUP);
    pinMode(SOLENOID_PIN, OUTPUT); 
    digitalWrite(SOLENOID_PIN, LOW);
    digitalWrite(NEN_PIN, LOW);

    Xstepper.setMaxSpeed(X_MAX_SPEED);
    Xstepper.setAcceleration(X_ACCELERATION);
    Ystepper.setMaxSpeed(Y_MAX_SPEED);
    Ystepper.setAcceleration(Y_ACCELERATION);
    Zstepper.setMaxSpeed(Z_MAX_SPEED);
    Zstepper.setAcceleration(Z_ACCELERATION);

    Serial.begin(115200);
    while(!Serial);
    Log.begin   (LOG_LEVEL_VERBOSE, &Serial);

    Log.traceln("Init");
}

uint32_t old_time = micros(); 
uint32_t current_time = micros(); 

FSM OLD_FSM_STATE; 

void loop() {
    Xstepper.run();  
    Ystepper.run();  
    Zstepper.run();  
    bool MotorsOn = false;
    MotorsOn = (Xstepper.distanceToGo() || 
                Ystepper.distanceToGo() ||
                Zstepper.distanceToGo() 
                ); 
    digitalWrite(NEN_PIN, MotorsOn ? LOW : HIGH); 


    if (!MotorsOn) // NEGEDGE 
    {
        if (!digitalRead(X_ENDSTOP) && FSM_STATE != HOME_X) 
        {
            FSM_STATE = FAULT;
            Log.fatalln("X_ENDSTOP triggered");
            Xstepper.stop();
        }                

        
        // Need to figure out how to axe this while ensuring that steps aren't missed
        switch (FSM_STATE)
        {
            // FSM STATE = 0
            case HOME: 
                Log.traceln("Starting Homing");

                FSM_STATE = HOME_X;
                Log.traceln("Starting Homing X");

                #ifdef SKIP_HOMING
                    FSM_STATE = PICK_DESCEND; 
                #endif
                    Ystepper.move( -64000); 
                    y_step_count += -6400; 

                break;

            // FSM STATE = 1
            case HOME_X: 
                if (!digitalRead(X_ENDSTOP)) 
                {
                    FSM_STATE = HOME_Z;
                    Log.traceln("Starting Homing Z");

                    x_step_count = 0; 
                    Xstepper.move(-100); // 1300 max motion 
                    x_step_count += 100; 
                }                

                else 
                {
                    Xstepper.move(1); 
                }
                break; 

            // FSM STATE = 2
            case HOME_Z: 
                Log.noticeln("HOME_Z is currently skipped");
                FSM_STATE = HOME_ELEVATOR; 
                Log.traceln("Starting Homing Elevator");
                break;

            // FSM STATE = 3
            case HOME_ELEVATOR: 
                if (!digitalRead(Y_ENDSTOP)) 
                {
                    FSM_STATE = PICK_DESCEND;
                    y_step_count = 0; 

                    // Ystepper.move( -6400); 
                    // y_step_count += -6400; 
                }                
                else 
                {
                    // High number because max microstepping enabled
                    Ystepper.move(1600); 
                    y_step_count += 1600; 
                }
                break; 

            // FSM STATE = 4
            case PICK_DESCEND: 
                // Default position is max high
                Zstepper.move(-1 * DESCEND_DEPTH);
                digitalWrite(SOLENOID_PIN, HIGH); 
                FSM_STATE = PICK_ASCEND;
                break;

            // FSM STATE = 5
            case PICK_ASCEND: 
                Zstepper.move(DESCEND_DEPTH);
                FSM_STATE = MOVE_FORWARD;
                break;

            // FSM STATE = 6
            case MOVE_FORWARD: 
                Xstepper.move(-1200); 
                x_step_count += 1200;
                FSM_STATE = PLACE_DESCEND;
                break;

            // FSM STATE = 7
            case PLACE_DESCEND: 
                // Later add X movement to match belt 
                Zstepper.move(-1 * DESCEND_DEPTH);
                FSM_STATE = RETURN;
                digitalWrite(SOLENOID_PIN, LOW); 
                break;

            // FSM STATE = 8
            case RETURN:
                Xstepper.move(1200); 
                x_step_count += -1200; 
                Zstepper.move(DESCEND_DEPTH);
                FSM_STATE = HOME_ELEVATOR;
                break;

            case PAUSE: 
                break;

            case FAULT: 
                Log.fatalln("ENTERED FAULT STATE");
                while(1); 
                break;

            default: 
                Log.errorln("FSM DEFAULT CASE"); 
                FSM_STATE = PICK_ASCEND;
                break; 
        }
        if (x_step_count > 1300 || x_step_count < 0) 
        {
            Log.fatalln("x_step_count out of bounds");
            Log.fatalln("x_step_count: %d", x_step_count);
            FSM_STATE = FAULT; 
        }
        Log.traceln("FSM_STATE = %d", FSM_STATE);
    }
}
