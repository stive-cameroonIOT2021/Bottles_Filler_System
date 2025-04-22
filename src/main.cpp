#include <AccelStepper.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
/* -------------------------------------Debug message--------------------------------------------------- */
#define DEBUG 1 // Enable debug messages

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif
/* -------------------------------------Steppers setup--------------------------------------------------- */
#define STEP_PIN_1 4
#define DIR_PIN_1 5
#define LEFT_LIMIT_1 8
#define RIGHT_LIMIT_1 9

#define STEP_PIN_2 3
#define DIR_PIN_2 6
#define LEFT_LIMIT_2 7
#define RIGHT_LIMIT_2 10

// Lead screw parameters
#define LEAD_SCREW_PITCH 2.0  // mm per revolution
#define STEPS_PER_REV 200     // 1.8° stepper motor
#define MICROSTEPPING 16      // Microstepping setting

// Motion parameters
#define ACCELERATION 500 // Steps per second^2

// Calculate steps per mm
#define STEPS_PER_MM ((STEPS_PER_REV * MICROSTEPPING) / LEAD_SCREW_PITCH)

// Initialize AccelStepper (Driver mode: STEP, DIR)
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

float feedRateMMperSec = 2000; // Speed of movement
bool movingRight1 = true;
bool movingRight2 = true;

// Global variable to reverse limit switch logic
bool reverseLimitSwitchLogic = true; // Set to true to reverse logic
/* ---------------------------------------------------------------------------------------- */
#define Relay_WaterPump A1 // Pin for water pump relay
/* ---------------------------------------------------------------------------------------- */
#define Buzzer 12 // Pin for buzzer
/* ---------------------------------------------------------------------------------------- */
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
/* ---------------------------------------------------------------------------------------- */
// Servo motor setup 
#define SERVO_PIN 11 // Pin for servo motor
Servo servoMotor; // Create a Servo object
/* ---------------------------------------------------------------------------------------- */
// States
enum State {
    WAIT_FOR_BOTTLE,
    GRAB_BOTTLE,
    FILL_BOTTLE,
    MOVE_TO_CAPPING,
    CAPPING_DOWN,
    CAPPING_Z_HOME,
    MOVE_TO_DROP,
    DROP_BOTTLE,
    X_HOMING
  };
  
  State currentState = WAIT_FOR_BOTTLE;

unsigned long stateStartTime = 0;
const unsigned long delayBeforeGrab = 3000;
const unsigned long cappingDuration = 3000;
/* ---------------------------------------------------------------------------------------- */
const int bottlePresentSensorPin = A0;
/* ---------------------------------------------------------------------------------------- */
const int waterflowsensorPin = 2; // Pin for water flow sensor
/* ---------------------------------------------------------------------------------------- */
const int valvePin = A2; // Pin for valve control
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */



/* --------------------------------------FUNCTIONS-------------------------------------------------- */
// Function prototypes
void moveStepperAndCheckLimit(AccelStepper &stepper, float distance_mm, int rightLimitPin, int leftLimitPin);
void moveStepper(AccelStepper &stepper, float distance_mm);
bool readLimitSwitch(int pin);
void setFeedRate(AccelStepper &stepper, float feedRateMMperSec);
void homeStepper(AccelStepper &stepper, int limitPin);

void setup() {
    pinMode(LEFT_LIMIT_1, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_1, INPUT_PULLUP);
    pinMode(LEFT_LIMIT_2, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_2, INPUT_PULLUP);

    pinMode(bottlePresentSensorPin, INPUT_PULLUP); // Set bottle present sensor pin as input with pull-up resistor
    pinMode(waterflowsensorPin, INPUT_PULLUP); // Set water flow sensor pin as input with pull-up resistor
    
    pinMode(valvePin, OUTPUT); // Set valve control pin as output
    digitalWrite(valvePin, LOW); // Turn off the valve initially

    pinMode(Relay_WaterPump, OUTPUT); // Set water pump relay pin as output
    digitalWrite(Relay_WaterPump, HIGH); // Turn off the water pump initially

    pinMode(Buzzer, OUTPUT); // Set buzzer pin as output
    Serial.begin(115200);

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System starting...");
    delay(4000); // Display message for 2 seconds

    stepper1.setAcceleration(ACCELERATION);
    stepper2.setAcceleration(ACCELERATION);
    setFeedRate(stepper1, feedRateMMperSec);
    setFeedRate(stepper2, feedRateMMperSec);

    servoMotor.attach(SERVO_PIN); // Attach the servo motor to the specified pin
    servoMotor.write(90); // Set initial position of the servo motor

    // Homing Stepper 1 to the left
    /* DEBUG_PRINTLN("Homing Stepper 1 to the left...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Homing Stepper 1");
    homeStepper(stepper1, LEFT_LIMIT_1);

    // Homing Stepper 2 up
    DEBUG_PRINTLN("Homing Stepper 2 up...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Homing Stepper 2");
    homeStepper(stepper2, LEFT_LIMIT_2); */

    stepper1.setCurrentPosition(0); // Set Stepper 1 position to 0 after homing
    stepper2.setCurrentPosition(0); // Set Stepper 2 position to 0 after homing

    moveStepperAndCheckLimit(stepper1, 40.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);
    moveStepperAndCheckLimit(stepper1, -40.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);

    moveStepperAndCheckLimit(stepper2, 20.0, RIGHT_LIMIT_2, LEFT_LIMIT_2);
    moveStepperAndCheckLimit(stepper2, -20.0, RIGHT_LIMIT_2, LEFT_LIMIT_2);
    while(1);
}

void loop() {
    // ✅ Step 1: Wait for bottle at fill point
    // ✅ Step 2: After t sec → Servo grabs bottle
    // ✅ Step 3: Fill bottle (until sensor says full)
    // ✅ Step 4: X-stepper → move to capping station
    // ✅ Step 5: Z-stepper → move down to cap when down dc motor spin to cap
    // ✅ Step 6: After cappingDuration → Z-stepper homes (Z = 0)
    // ✅ Step 7: X-stepper → move to drop station
    // ✅ Step 8: Drop bottle (servo motor moves to drop position)
    // ✅ Step 9: X-stepper → move to home position(X = 0)
    // ✅ Step 10: Repeat

    switch (currentState) {
    case WAIT_FOR_BOTTLE:
        // Wait for bottle at fill point
        DEBUG_PRINTLN("Waiting for bottle...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Waiting for bottle");
        // Simulate waiting for a bottle (replace with actual sensor check)
        delay(2000); // Simulate waiting time
        stateStartTime = millis(); // Record the start time
        currentState = GRAB_BOTTLE;
        break;
    
    default:
        break;
    }

    /* // Move Stepper 1 forward 40mm and check the limit switches
    moveStepperAndCheckLimit(stepper1, 40.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);
    
    // Move Stepper 1 back 30mm after checking limit
    moveStepperAndCheckLimit(stepper1, -30.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);

    // Move Stepper 2 forward 40mm and check the limit switches
    moveStepperAndCheckLimit(stepper2, 40.0, RIGHT_LIMIT_2, LEFT_LIMIT_2);

    // Move Stepper 2 back 30mm after checking limit
    moveStepperAndCheckLimit(stepper2, -30.0, RIGHT_LIMIT_2, LEFT_LIMIT_2); */
}

void moveStepperAndCheckLimit(AccelStepper &stepper, float distance_mm, int rightLimitPin, int leftLimitPin) {
    // Move the stepper by the specified distance
    moveStepper(stepper, distance_mm); 

    // Wait until the movement is completed or the limit switch is hit
    while (stepper.isRunning()) {
        stepper.run(); // Move the motor continuously

        // If the right limit switch is triggered (non-reversed logic)
        if (readLimitSwitch(rightLimitPin) == LOW) {
            DEBUG_PRINTLN("Limit switch hit, stopping.");
            stepper.stop();
            break;
        }
        // If the left limit switch is triggered (non-reversed logic)
        if (readLimitSwitch(leftLimitPin) == LOW) {
            DEBUG_PRINTLN("Left limit switch hit, stopping.");
            stepper.stop();
            break;
        }
    }
}

void moveStepper(AccelStepper &stepper, float distance_mm) {
    long steps = (distance_mm / 10.0) * STEPS_PER_MM;
    stepper.move(steps);
}

bool readLimitSwitch(int pin) {
    // Use global reverseLimitSwitchLogic to determine logic
    return reverseLimitSwitchLogic ? (digitalRead(pin) == HIGH) : (digitalRead(pin) == LOW);
}

void setFeedRate(AccelStepper &stepper, float feedRateMMperSec) {
    float speedStepsPerSec = feedRateMMperSec * STEPS_PER_MM;
    stepper.setMaxSpeed(speedStepsPerSec);
}

// Homing function for stepper motors
void homeStepper(AccelStepper &stepper, int limitPin) {
    stepper.setCurrentPosition(0); // Ensure the motor starts at position 0
    setFeedRate(stepper, 10);  // Slow down for homing

    // Move towards the limit switch until it's hit
    while (readLimitSwitch(limitPin) == HIGH) {
        stepper.move(-10000); // Move towards the limit switch
        stepper.run();
    }

    stepper.stop(); // Stop the motor once the limit switch is hit
    stepper.setCurrentPosition(0); // Reset the position to 0
    DEBUG_PRINTLN("Homing complete.");
}
