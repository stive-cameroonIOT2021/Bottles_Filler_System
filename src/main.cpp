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
//X-axis stepper motor pins
#define STEP_PIN_1 4
#define DIR_PIN_1 5
#define LEFT_LIMIT_1 8
#define RIGHT_LIMIT_1 9

//Z-axis stepper motor pins
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

// Global variables to reverse stepper motor direction
bool reverseDirection_X = false;  // Normal direction for X-axis (stepper1)
bool reverseDirection_Z = true;  // Normal direction for Z-axis (stepper2)
/* ---------------------------------------------------------------------------------------- */
// Global variables to reverse limit switch logic for each stepper
bool reverseLimitSwitchLogic_X = true;  // Reverse logic for X-axis (stepper1)
bool reverseLimitSwitchLogic_Z = false; // Normal logic for Z-axis (stepper2)
/* ---------------------------------------------------------------------------------------- */
#define Relay_WaterPump A1 // Pin for water pump relay
/* ---------------------------------------------------------------------------------------- */
#define Buzzer A6 // Pin for buzzer
#define Push_Btn 12
/* ---------------------------------------------------------------------------------------- */
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
/* ---------------------------------------------------------------------------------------- */
// Servo motor setup 
#define SERVO_PIN_UP 11 // Pin for servo motor
#define SERVO_PIN_DOWN 13 // Pin for servo motor
Servo servoMotor_up; // Create a Servo object
Servo servoMotor_down; // Create a Servo object
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
const unsigned long cappingDuration = 2000;
/* ---------------------------------------------------------------------------------------- */
const int bottlePresentSensorPin = A0;
/* ---------------------------------------------------------------------------------------- */
//const int waterflowsensorPin = 2; // Pin for water flow sensor
/* ---------------------------------------------------------------------------------------- */
const int ConvoyMotor = A2; // Pin for valve control
const int IrsensorPin = A3; // Pin for IR sensor
/* ---------------------------------------------------------------------------------------- */
const int cappingMotorPin = 2; // Pin for capping motor (DC motor)
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
    pinMode(IrsensorPin, INPUT_PULLUP); // Set IR sensor pin as input with pull-up resistor

    pinMode(bottlePresentSensorPin, INPUT_PULLUP); // Set bottle present sensor pin as input with pull-up resistor
    pinMode(Push_Btn, INPUT);
    //pinMode(waterflowsensorPin, INPUT_PULLUP); // Set water flow sensor pin as input with pull-up resistor
    
    pinMode(ConvoyMotor, OUTPUT); // Set valve control pin as output
    digitalWrite(ConvoyMotor, HIGH); // Turn off the convoyor initially

    pinMode(Relay_WaterPump, OUTPUT); // Set water pump relay pin as output
    digitalWrite(Relay_WaterPump, HIGH); // Turn off the water pump initially

    pinMode(cappingMotorPin, OUTPUT); // Set capping motor pin as output
    digitalWrite(cappingMotorPin, HIGH); // Turn off the capping motor initially

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

    servoMotor_up.attach(SERVO_PIN_UP); // Attach the servo motor to the specified pin
    servoMotor_up.write(0); // Set initial position of the servo motor

    servoMotor_down.attach(SERVO_PIN_DOWN); // Attach the servo motor to the specified pin
    servoMotor_down.write(0); // Set initial position of the servo motor

    // Homing Stepper 1 to the left
     DEBUG_PRINTLN("Homing Stepper 1 to the left...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Homing Stepper 1");
    homeStepper(stepper1, LEFT_LIMIT_1);
    //moveStepperAndCheckLimit(stepper1, 200.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);//This is a trial to check the limit switch logic
    //moveStepperAndCheckLimit(stepper1, -90.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);//This is a trial to check the limit switch logic

    //while(1);

    // Homing Stepper 2 up
    DEBUG_PRINTLN("Homing Stepper 2 up...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Homing Stepper 2");
    homeStepper(stepper2, RIGHT_LIMIT_2);

    stepper1.setCurrentPosition(0); // Set Stepper 1 position to 0 after homing
    stepper2.setCurrentPosition(0); // Set Stepper 2 position to 0 after homing

    //moveStepperAndCheckLimit(stepper1, 300.0, RIGHT_LIMIT_1, LEFT_LIMIT_1);//This is a trial to check the limit switch logic

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bottle Filler");

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
    // ✅ Step 10: Reset to Step 1

if(currentState == WAIT_FOR_BOTTLE){
        //DEBUG_PRINTLN("Waiting for bottle...");
        int state = digitalRead(IrsensorPin); // Read the IR sensor state
        if (state == LOW) { // Bottle detected
            //DEBUG_PRINTLN("Bottle detected!");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Bottle Filler");
            lcd.setCursor(0, 1);
            lcd.print("Bottle detected!");
            delay(2000);
            digitalWrite(ConvoyMotor, HIGH);//Turn off relay convoy
            currentState = GRAB_BOTTLE;
            delay(2000); // Simulate waiting time
        } else {
            //DEBUG_PRINTLN("Waiting for bottle...");
            lcd.setCursor(0, 0);
            lcd.print("Bottle Filler");
            lcd.setCursor(0, 1);
            lcd.print("Waiting for bottle");

            digitalWrite(ConvoyMotor, LOW);//Turn on relay convoy
        }
}
else if(currentState == GRAB_BOTTLE){
        // Grab the bottle with the servo motor
        //DEBUG_PRINTLN("Grabbing bottle...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Bottle Filler");
        lcd.setCursor(0, 1);
        lcd.print("Grabbing bottle...");
        servoMotor_up.write(110); // Move servo to grab position
        servoMotor_down.write(110); // Move servo to grab position

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Bottle Filler");
        lcd.setCursor(0, 1);
        lcd.print("Waiting press...");

        bool press = !digitalRead(Push_Btn);
        while(!press){
            press = !digitalRead(Push_Btn);
        }

        delay(2000); // Simulate grabbing time
        currentState = MOVE_TO_CAPPING;
}

else if(currentState == MOVE_TO_CAPPING){

    // Move Stepper 1 to the capping station
    DEBUG_PRINTLN("Moving to capping station...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bottle Filler");
    lcd.setCursor(0, 1);
    lcd.print("Moving to capping");
    moveStepperAndCheckLimit(stepper1, 185.0, RIGHT_LIMIT_1, LEFT_LIMIT_1); // Move to capping station
    currentState = CAPPING_DOWN;
}

else if(currentState == CAPPING_DOWN){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bottle Filler");
    lcd.setCursor(0, 1);
    lcd.print("Capping down...");
    // Move Stepper 2 down to cap
    moveStepperAndCheckLimit(stepper2, 22.0, RIGHT_LIMIT_2, LEFT_LIMIT_2); // Move down to cap

    DEBUG_PRINTLN("Capping down, activating capping motor...");
    digitalWrite(cappingMotorPin, LOW); // Turn on capping motor (DC motor)
    delay(cappingDuration); // Wait for capping duration
    digitalWrite(cappingMotorPin, HIGH); // Turn off capping motor

    DEBUG_PRINTLN("Capping complete, moving to capping home...");
    currentState = CAPPING_Z_HOME; // Move to capping down state
}

else if(currentState == CAPPING_Z_HOME){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bottle Filler");
    lcd.setCursor(0, 1);
    lcd.print("Capping up...");

    homeStepper(stepper2, RIGHT_LIMIT_2); // Home Stepper 2 after capping
    DEBUG_PRINTLN("Capping complete, homing Stepper 2...");

    currentState = MOVE_TO_DROP; // Move to drop state
}

else if(currentState == MOVE_TO_DROP){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bottle Filler");
    lcd.setCursor(0, 1);
    lcd.print("Dropping bottle...");
    servoMotor_up.write(0); // Move servo to drop position
    servoMotor_down.write(0);// Move servo to drop position
    moveStepperAndCheckLimit(stepper1, 300.0, RIGHT_LIMIT_1, LEFT_LIMIT_1); // Move back to drop station
    delay(4000); // Simulate waiting time
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bottle Filler");
    lcd.setCursor(0, 1);
    lcd.print("Homing Stepper 1");
    homeStepper(stepper1, LEFT_LIMIT_1); // Home Stepper 1 to the left
    currentState = WAIT_FOR_BOTTLE; // Reset to wait for next bottle

    lcd.clear();
}
else{}


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
    // Apply direction reversal logic
    bool isStepper1 = (&stepper == &stepper1);
    float adjusted_distance_mm = (isStepper1 && reverseDirection_X) || (!isStepper1 && reverseDirection_Z) ? -distance_mm : distance_mm;
    
    long steps = (adjusted_distance_mm / 10.0) * STEPS_PER_MM; // Convert mm to steps
    stepper.move(steps);  // Relative move

    while (stepper.distanceToGo() != 0) {
        stepper.run();

        // Check limit switches
        if (readLimitSwitch(rightLimitPin) == LOW && steps > 0) {
            DEBUG_PRINTLN("Right limit reached!");
            stepper.stop();
            break;
        }
        if (readLimitSwitch(leftLimitPin) == LOW && steps < 0) {
            DEBUG_PRINTLN("Left limit reached!");
            stepper.stop();
            break;
        }
    }
}

void moveStepper(AccelStepper &stepper, float distance_mm) {
    // Apply direction reversal logic
    bool isStepper1 = (&stepper == &stepper1);
    float adjusted_distance_mm = (isStepper1 && reverseDirection_X) || (!isStepper1 && reverseDirection_Z) ? -distance_mm : distance_mm;
    
    long steps = (adjusted_distance_mm / 10.0) * STEPS_PER_MM; // Convert mm to steps
    stepper.move(steps);
}

bool readLimitSwitch(int pin) {
    // Determine which stepper's limit switch is being read and apply the appropriate logic
    if (pin == RIGHT_LIMIT_1 || pin == LEFT_LIMIT_1) {
        return reverseLimitSwitchLogic_X ? (digitalRead(pin) == HIGH) : (digitalRead(pin) == LOW);
    } else if (pin == RIGHT_LIMIT_2 || pin == LEFT_LIMIT_2) {
        return reverseLimitSwitchLogic_Z ? (digitalRead(pin) == HIGH) : (digitalRead(pin) == LOW);
    }
    // Default case (should not occur with proper pin usage)
    return digitalRead(pin) == LOW;
}

void setFeedRate(AccelStepper &stepper, float feedRateMMperSec) {
    float speedStepsPerSec = feedRateMMperSec * STEPS_PER_MM;
    stepper.setMaxSpeed(speedStepsPerSec);
}

// Homing function for stepper motors
void homeStepper(AccelStepper &stepper, int limitPin) {
    stepper.setCurrentPosition(0); // Ensure the motor starts at position 0
    setFeedRate(stepper, 100);  // Slow down for homing

    // Apply direction reversal logic for homing
    bool isStepper1 = (&stepper == &stepper1);
    bool reverseDirection = isStepper1 ? reverseDirection_X : reverseDirection_Z;
    long homingDirection = reverseDirection ? 10000 : -10000; // Reverse homing direction if needed

    // Move towards the limit switch until it's hit
    while (readLimitSwitch(limitPin) == HIGH) {
        stepper.move(homingDirection); // Move towards the limit switch
        stepper.run();
    }

    stepper.stop(); // Stop the motor once the limit switch is hit
    stepper.setCurrentPosition(0); // Reset the position to 0
    DEBUG_PRINTLN("Homing complete.");
}
