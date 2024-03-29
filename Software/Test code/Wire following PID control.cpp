#include <SimpleTimer.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Configuration
#define SUBTRACTIVE_MOTOR_SPEED     // Comment for additive and subtractive motor speed
#define CALCULATE_DERIVATIVE_ERROR  // Comment to remove derivative gain from error calculations
#define CALCULATE_INTEGRAL_ERROR    // Comment to remove integral gain from error calculations
//#define DEBUG_MODE                  // Comment to display voltage and time instead of debug values

// Pin definitions
#define LEFT_SENSOR 1       // ANALOG
#define RIGHT_SENSOR 2      // ANALOG
#define CENTER_SENSOR 3     // ANALOG
#define BATTERY_SENSOR 4    // ANALOG
#define BUTTON_INPUT 0      // ANALOG
#define LEFT_MOTOR 11       // PWM
#define RIGHT_MOTOR 3       // PWM
#define LEFT_TURN_LED 12     // DIGITAL
#define RIGHT_TURN_LED 13    // DIGITAL

// Keypad button definitions
#define RIGHT  0
#define UP     1
#define DOWN   2
#define LEFT   3
#define SELECT 4
#define NONE   5

// LCD cursor definitions
#define TOP 0
#define BOTTOM 1

// Wire States
#define TOO_LEFT -1
#define TOO_RIGHT 1
#define OFF_WIRE 5

// Turn indicator LEDs
SimpleTimer turnSignalTimer;
bool turnSignalState = false;

// "Clock" speed
unsigned long loopPeriod;
unsigned long lastLoop;

enum state
{
    menu,
    moveStraight,
    turnLeft,
    turnRight
};

struct Parameter
{
    String Name;
    byte Value;
    int Index;
}; 

// Prevents LCD flicker
short lcdRefreshCount = 0;
short lcdRefreshPeriod = 200;

// Prevents excess battery readings
short batteryCount = 10;
short batteryPeriod = 10000;
volatile float batteryVoltage = 0.0;

// Sensors
volatile short left = 0;
volatile short right = 0;
volatile short leftDetected = false;
volatile short rightDetected = false;

// Smart intersection detection
#define INTERSECTION_COUNT_LEFT 2
#define INTERSECTION_COUNT_RIGHT 3
#define INTERSECTION_COUNT_ORIGIN 4

volatile short center = 0;
volatile short centerOldHigh = false;
volatile short intersectionSignalRecent = 0;
volatile short centerTimeoutCount = 0;
volatile short intersectionTurnFlag = 0;

// Control
float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
short derivativeCounter = 0;
short integralOffsetPeriod = 50;
short integralOffsetCounter = integralOffsetPeriod;

float error = 0.0;
float previousError = 0.0;

// State tracking
state currentState = menu;
short menuIndex = 0;

// Clock
/*volatile unsigned int clockTime = 0;
volatile int clockRun = false;
volatile unsigned int clockStart = 0;
*/
SimpleTimer lapTimer;
long lapTime;

// EEPROM values
Parameter proportionalGain   = {"P-gain",      0, 1}; 
Parameter integralGain       = {"I-gain",      0, 2}; 
Parameter derivativeGain     = {"D-gain",      0, 3}; 
Parameter speed              = {"Speed",       0, 4}; 
Parameter thresholdLeft      = {"L-Thresh",    0, 5}; 
Parameter thresholdRight     = {"R-Thresh",    0, 6}; 
Parameter centerSchmittHigh  = {"C-Schmitt H", 0, 7};
Parameter centerSchmittLow   = {"C-Schmitt L", 0, 8};
Parameter integralCap        = {"I-Cap",       0, 9};
Parameter centerTimeoutLimit = {"Timeout",     0, 10};
Parameter turnTime           = {"Turn time",   0, 11};

// Make sure any EEPROM value is added to the array
Parameter *parameters[] = 
{
    &proportionalGain, &integralGain, &derivativeGain,
    &speed, &thresholdLeft, &thresholdRight, &centerSchmittHigh, 
    &centerSchmittLow, &integralCap, &centerTimeoutLimit, &turnTime
};
#define PARAMETER_COUNT (sizeof(parameters)/sizeof(Parameter*)) // MUST EQUAL NUMBER OF PARAMETERS

inline void TurnSignal()
{
    if (intersectionTurnFlag == INTERSECTION_COUNT_LEFT)
    {
        digitalWrite(LEFT_TURN_LED, turnSignalState);
        digitalWrite(RIGHT_TURN_LED, 0);
    }
    else if (intersectionTurnFlag == INTERSECTION_COUNT_RIGHT)
    {   
        digitalWrite(LEFT_TURN_LED, 0);
        digitalWrite(RIGHT_TURN_LED, turnSignalState);
    }
    else
    {
        digitalWrite(LEFT_TURN_LED, 0);
        digitalWrite(RIGHT_TURN_LED, 0);
    }
    turnSignalState = !turnSignalState;
}

// Increment lap time
void IncrementTime()
{
    lapTime++;
}

// Clears the LCD screen
inline void Clear()
{
    lcd.clear();
}

// Prints a string to the LCD display, with an optional integer value beside it
inline void Print(String text, int value = -1) 
{
    lcd.print(text);
    if (value != -1) lcd.print(value);
}

// Changes the LCD cursor location
inline void Cursor(int row, int column)
{
    lcd.setCursor(column, row);
}

// SETUP
void setup()
{
    // For algorithm speed checking
    loopPeriod = 0;
    lastLoop = micros();

    // Lap timer
    lapTimer.setInterval(100, IncrementTime);
    lapTimer.disable(0);

    // Initialize LCD
    lcd.begin(16, 2);   
    lcd.setCursor(0,0);
    
    // Turn signals
    pinMode(LEFT_TURN_LED, OUTPUT);
    pinMode(RIGHT_TURN_LED, OUTPUT);
    digitalWrite(LEFT_TURN_LED, 0);
    digitalWrite(RIGHT_TURN_LED, 0);
    turnSignalTimer.setInterval(50, TurnSignal);

    // Force motors off by default
    MotorSpeed(LEFT_MOTOR, 0);
    MotorSpeed(RIGHT_MOTOR, 0);

    // Parameters need to be loaded from EEPROM
    LoadFromEEPROM();

    // Intro text
    Clear();
    Cursor(TOP, 0);
    Print("FAST ORANGE");
    Cursor(BOTTOM, 0);
    #ifdef DEBUG_MODE
    Print("Debug Mode");
    #else
    Print("Race Mode");
    #endif
    delay(1000);

    currentState = menu;
}

// LOOP
void loop()
{
    switch(currentState)
    {
        case menu:
        MotorSpeed(LEFT_MOTOR, 0);
        MotorSpeed(RIGHT_MOTOR, 0);
        ShowMenu();
        break;

        case moveStraight:
        Update();
        ProcessMovementAnalog();
        break;

        case turnLeft:
        case turnRight:
        Turn();
        break;
    }
    turnSignalTimer.run();
    lapTimer.run();
}


// Returns the current button being pressed.
// Can detect one button being pressed at a time.
inline int ReadButton()
{
    int value = analogRead(BUTTON_INPUT);   
    if (value > 1000) return NONE;
    if (value < 50)   return RIGHT;  
    if (value < 250)  return UP; 
    if (value < 450)  return DOWN; 
    if (value < 650)  return LEFT; 
    if (value < 850)  return SELECT;  
    return NONE;
}

// Variables used to modify rate of change of values in menu
int previousButton = UP;
int holdCounter = 0;

// Display the menu on screen
void ShowMenu()
{
    // Show menu item on top row
    Clear();
    Cursor(TOP, 0);
    Print(parameters[menuIndex]->Name);
    Print(" ", parameters[menuIndex]->Value);

    // Show sensor info on bottom row. Useful for threshold calibration
    Cursor(BOTTOM, 0);
    Print(" ", analogRead(LEFT_SENSOR));
    Print(" ", analogRead(RIGHT_SENSOR));
    Print(" ", analogRead(CENTER_SENSOR));
    Print(" ");
    lcd.print(float(lapTime)/10.0); Print("s");

    switch(ReadButton())
    {
        case UP:    // Increase item value
        holdCounter = (previousButton == UP) ? (holdCounter + 1) : 0;
        previousButton = UP;
        parameters[menuIndex]->Value += 1 + (holdCounter / 20);
        break;
        case DOWN:  // Lower item value
        holdCounter = (previousButton == DOWN) ? (holdCounter + 1) : 0;
        previousButton = DOWN;
        parameters[menuIndex]->Value -= (1 + (holdCounter / 20));
        break;
        case LEFT:  // Next menu item
        menuIndex = (menuIndex > 0) ? (menuIndex - 1) : (PARAMETER_COUNT - 1);
        break;
        case RIGHT: // Previous menu item
        menuIndex = (menuIndex < PARAMETER_COUNT - 1) ? (menuIndex + 1) : 0;
        break;
        case SELECT:// Exit menu
        delay(500);
        if (ReadButton() == SELECT)
        {
            Clear();
            Cursor(TOP, 0);
            Print("Exiting menu");
            SaveToEEPROM(); // Save values to EEPROM before exiting
            delay(750);
            currentState = moveStraight;
            intersectionTurnFlag = 0;
            lapTime = 0;
            batteryCount = 1;
            Clear();
            return;
        }
        break;
    }
    delay(150); 
}

// Loads all values from the EEPROM
void LoadFromEEPROM()
{
    for(int i = 0; i < PARAMETER_COUNT; i++)
        parameters[i]->Value = EEPROM.read(parameters[i]->Index);
}

// Saves all values to the EEPROM
void SaveToEEPROM()
{
    for(int i = 0; i < PARAMETER_COUNT; i++)
        EEPROM.write(parameters[i]->Index, parameters[i]->Value);
}

inline void DisplaySensorValues()
{
    if(lcdRefreshCount == 0)  // Mitigates screen flicker and time consuming operations
    {
        #ifdef DEBUG_MODE
        // Sensors on top line        
        Cursor(BOTTOM, 0);

        #ifdef USE_SMART_INTERSECTIONS
        Print("Signal: ", intersectionTurnFlag);
        #else
        Print("Intersection: ", intersectionIndex + 1);
        #endif
        
        #else
        Cursor(TOP, 0); Print("                "); Cursor(TOP, 0);
        Print("Time: "); lcd.print(float(lapTime)/10.0); Print("s");
        #endif
    }
    #ifndef DEBUG_MODE
    if(batteryCount == 0)
    {
        Cursor(BOTTOM, 0);
        batteryVoltage = float(analogRead(BATTERY_SENSOR)) / 1024.0 * 5.0;
        Print("Battery: "); lcd.print(batteryVoltage); Print("V");
    }
    #endif
}

inline void CenterSmartUpdate()
{
    center = analogRead(CENTER_SENSOR);

    if (centerTimeoutCount)
        centerTimeoutCount += 1;

    if (!centerOldHigh && (center > centerSchmittHigh.Value)) // On switch-to-high
    {
        centerOldHigh = true; // Switch current center state to high

        if((intersectionSignalRecent + 1) == INTERSECTION_COUNT_ORIGIN) // If the start/stop signal is encountered, toggle clock.
        // This is done on when it switches high (instead of waiting for timeout) to save a bit of clock-time.
        {
            lapTimer.toggle(0);
            if (!lapTimer.isEnabled(0))
            {
               // Stop motors before entering menu
                MotorSpeed(LEFT_MOTOR, 0);
                MotorSpeed(RIGHT_MOTOR, 0);

                Clear();
                Cursor(TOP, 0);
                Print("Finish!");
                currentState = menu;
                delay(800);
                return;
            }

        }

        switch(intersectionTurnFlag)
        {
            case INTERSECTION_COUNT_LEFT: // First intersection after left signal
            currentState = turnLeft;
            break;
            case INTERSECTION_COUNT_RIGHT: // First intersection after right signal
            currentState = turnRight;
            break;
            default: // Else
            currentState = moveStraight;
            intersectionSignalRecent += 1;
            break;
        }

        centerTimeoutCount = 0; // Stop timeout counter
        intersectionTurnFlag = 0; // Reset flag
    }
    else if (centerOldHigh && (center < centerSchmittLow.Value)) // On switch-to-low
    {
        centerOldHigh = false; // Swtich current center state to low
        centerTimeoutCount = 1; // Start timeout counter
    }

    if (centerTimeoutCount >= (centerTimeoutLimit.Value * 2)) // Timeout
    {
        if (intersectionTurnFlag < intersectionSignalRecent) // Do not let false signals effect the flag
            intersectionTurnFlag = intersectionSignalRecent; // Set flag

        intersectionSignalRecent = 0; // Reset signal counter
        centerTimeoutCount = 0; // Stop timeout counter
    }
}

// Updates the sensors and machine state
void Update()
{
    if ((lcdRefreshCount == 0) && (currentState != menu) && (ReadButton() == SELECT))
    {
        delay(750);
        if(ReadButton() == SELECT) // debounce MENU button
        {
            // Stop motors before entering menu
            MotorSpeed(LEFT_MOTOR, 0);
            MotorSpeed(RIGHT_MOTOR, 0);

            Clear();
            Cursor(TOP, 0);
            Print("Entering menu");
            currentState = menu;
            delay(1000);
            lapTimer.disable(0);
            return;
        }
    }

    // Read raw sensor values
    left = analogRead(LEFT_SENSOR);
    right = analogRead(RIGHT_SENSOR);
    leftDetected = left > thresholdLeft.Value;
    rightDetected = right > thresholdRight.Value;
    if (leftDetected || rightDetected)
    {
        CenterSmartUpdate();
    }

    // Display values on LCD
    lcdRefreshCount = (lcdRefreshCount <= 0) ? lcdRefreshPeriod : (lcdRefreshCount - 1);
    batteryCount = (batteryCount <= 0) ? batteryPeriod : (batteryCount - 1);
    DisplaySensorValues();   
}

void ProcessMovementAnalog()
{
    if (!leftDetected && !rightDetected) error = (previousError <= 0) ? -500 : 500;
    else error = float(left) - float(right);

    // Proportional
    proportional = error * float(proportionalGain.Value) / 10.0;
    derivative = (error - previousError) * float(derivativeGain.Value) / 50.0;
    integral += (error * float(integralGain.Value)) / 50.0;
    
    if (integral > integralCap.Value) 
        integral = integralCap.Value;
    else if (integral < -integralCap.Value) 
        integral = -integralCap.Value;

    #ifndef CALCULATE_INTEGRAL_ERROR
    integral = 0;
    #endif;

    #ifndef CALCULATE_DERIVATIVE_ERROR
    derivative = 0;
    #endif;

    int baseSpeed;
    if (center > centerSchmittHigh.Value)
        baseSpeed = 1000;
    else    
        baseSpeed = (speed.Value * 4.0);

    int mLeft  = baseSpeed + (proportional + derivative + integral);
    int mRight = baseSpeed - (proportional + derivative + integral);

    #ifdef SUBTRACTIVE_MOTOR_SPEED
    if (mLeft > mRight) 
        mLeft = baseSpeed;
    else
        mRight = baseSpeed;
    #endif

    MotorSpeed(LEFT_MOTOR, mLeft);
    MotorSpeed(RIGHT_MOTOR, mRight);
}

// Modifies the motor speed given a value between 0 and 100
inline void MotorSpeed(int motor, int speed)
{
	if (speed > 1000) speed = 1000;
	else if (speed < 0) speed = 0;
    analogWrite(motor, speed/4.0);
}

void Turn()
{
    turnSignalState = true;
    TurnSignal();

    int direction;
    if (currentState == turnLeft) direction = 1;
    else if (currentState == turnRight) direction = -1;
    else return;

    Clear();
    Print("TURN");

    MotorSpeed(LEFT_MOTOR, 1000 * direction);
    MotorSpeed(RIGHT_MOTOR, 1000 * -direction);
    delay(4.0 * turnTime.Value);

    previousError = direction;
    integral = 0;
    currentState = moveStraight;
    Clear();
}

