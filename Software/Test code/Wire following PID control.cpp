#include <LiquidCrystal.h>
#include <EEPROM.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Configuration
#define SUBTRACTIVE_MOTOR_SPEED     // Comment for additive and subtractive motor speed
#define CALCULATE_DERIVATIVE_ERROR  // Comment to remove derivative gain from error calculations
#define CALCULATE_INTEGRAL_ERROR    // Comment to remove integral gain from error calculations
#define CALCULATE_ANALOG_PID        // Comment to calculate PID error using boolean logic instead of analog values
#define USE_SMART_INTERSECTIONS     // Comment to use the intersection map instead of sensing intersection signals

// Pin definitions
#define LEFT_SENSOR 1       // ANALOG
#define RIGHT_SENSOR 2      // ANALOG
#define CENTER_SENSOR 3     // ANALOG
#define BATTERY_SENSOR 4    // ANALOG
#define BUTTON_INPUT 0      // ANALOG
#define LEFT_MOTOR 11       // PWM
#define RIGHT_MOTOR 3       // PWM
#define SENSOR_RESET 2      // DIGITAL

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
int lcdRefreshCount = 0;
int lcdRefreshPeriod = 200;

// Prevents excess battery readings
int batteryCount = 0;
int batteryPeriod = 10000;
volatile float batteryVoltage = 0.0;

// Sensors
volatile int left = 0;
volatile int right = 0;
volatile int leftDetected = false;
volatile int rightDetected = false;

// Smart intersection detection
#define INTERSECTION_COUNT_LEFT 2
#define INTERSECTION_COUNT_RIGHT 3
#define INTERSECTION_COUNT_ORIGIN 4
int centerTimeoutLimit = 500;
volatile int center = 0;
volatile int centerOldHigh = false;
volatile int intersectionSignalRecent = 0;
volatile int centerTimeoutCount = 0;
volatile int intersectionTurnFlag = 0;

// Mapped intersection detection
enum intersectionCommand
{
    straight,
    leftTurn,
    rightTurn,
    stop
};

int intersections[] =
{
    straight,
    straight,
    straight,
    straight,
    straight,
    leftTurn,
    straight,
    straight,
    rightTurn,
    straight,
    straight,
};
#define INTERSECTION_COUNT (sizeof(intersections)/sizeof(int)) // MUST EQUAL NUMBER OF PARAMETERS
int intersectionIndex = -1;
volatile int centerNew = false;
volatile int centerRisingEdge = false;
volatile int centerHighDetected = false;
volatile int centerLowDetected = false;

// Control
float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
int derivativeCounter = 0;
int integralOffsetPeriod = 50;
int integralOffsetCounter = integralOffsetPeriod;

float error = 0.0;
float previousError = 0.0;

// State tracking
state currentState = menu;
int menuIndex = 0;

// EEPROM values
Parameter proportionalGain  = {"P-gain",      0, 1}; 
Parameter integralGain      = {"I-gain",      0, 2}; 
Parameter derivativeGain    = {"D-gain",      0, 3}; 
Parameter speed             = {"Speed",       0, 4}; 
Parameter thresholdLeft     = {"L-Thresh",    0, 5}; 
Parameter thresholdRight    = {"R-Thresh",    0, 6}; 
Parameter centerSchmittHigh = {"C-Schmitt H", 0, 7};
Parameter centerSchmittLow  = {"C-Schmitt L", 0, 8};
Parameter integralCap       = {"I-Cap",       0, 9};

// Make sure any EEPROM value is added to the array
Parameter *parameters[] = 
{
    &proportionalGain, &integralGain, &derivativeGain,
    &speed, &thresholdLeft, &thresholdRight, &centerSchmittHigh, 
    &centerSchmittLow, &integralCap
};
#define PARAMETER_COUNT (sizeof(parameters)/sizeof(Parameter*)) // MUST EQUAL NUMBER OF PARAMETERS

// Clears the LCD screen
void Clear()
{
    lcd.clear();
}

// Prints a string to the LCD display, with an optional numerical value beside it
void Print(String text, int value = -1) 
{
    lcd.print(text);
    if (value != -1) lcd.print(value);
}

// Changes the LCD cursor location
void Cursor(int row, int column)
{
    lcd.setCursor(column, row);
}

// SETUP
void setup()
{
    // Initialize LCD
    lcd.begin(16, 2);   
    lcd.setCursor(0,0);
    pinMode(SENSOR_RESET, OUTPUT);

    // Force motors off by default
    MotorSpeed(LEFT_MOTOR, 0);
    MotorSpeed(RIGHT_MOTOR, 0);

    // Parameters need to be loaded from EEPROM
    LoadFromEEPROM();

    // Intro text
    Clear();
    Cursor(TOP, 0);
    Print("Fast Orange");
    delay(1000);
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
            #ifdef CALCULATE_ANALOG_PID
            ProcessMovementAnalog();
            #else
            ProcessMovementDigital();
            #endif
        break;

        case turnLeft:
        case turnRight:
            Turn();
        break;
    }
}

// Drains the capacitors on the peak detector so that can be read again
void SensorReset(int microseconds = 10)
{
    digitalWrite(SENSOR_RESET, HIGH);
    delayMicroseconds(microseconds);
    digitalWrite(SENSOR_RESET, LOW);
}

// Returns the current button being pressed.
// Can detect one button being pressed at a time.
int ReadButton()
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
    SensorReset();

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

void DisplaySensorValues()
{
    if(lcdRefreshCount == 0)  // Mitigates screen flicker and time consuming operations
    {
        // Sensors on top line
        Cursor(TOP, 0); 
        Print("L:", left);
        Print(" R:", right);
        Print(" C:", center);
        Print("     ");
        
        Cursor(BOTTOM, 0);

        #ifdef USE_SMART_INTERSECTIONS
            Print("Signal: ", intersectionTurnFlag);
        #else
            Print("Intersection: ", intersectionIndex + 1);
        #endif
    }

    
    // Only update battery voltage once in a while
    //if(batteryCount == 0)
    //{
    //    batteryVoltage = float(analogRead(BATTERY_SENSOR)) / 1024.0 * 5.0;
        // Battery voltage on bottom line
    //    Cursor(BOTTOM, 0);
    //    Print("Batt: "); lcd.print(batteryVoltage); Print("V");
    //}
}


void CenterMapUpdate()
{
    center = analogRead(CENTER_SENSOR);
    centerHighDetected = center > centerSchmittHigh.Value;
    centerLowDetected  = center > centerSchmittLow.Value;

    if(!centerLowDetected)
    {
        centerNew = true;
    }

    if(centerNew && centerHighDetected)
    {
        delay(50);
        if (analogRead(CENTER_SENSOR) > centerSchmittHigh.Value)
        {
            centerRisingEdge = true;
            delay(50);
            centerNew = false;
            intersectionIndex++;
            if (intersectionIndex == INTERSECTION_COUNT) 
                intersectionIndex = 0;
            
            switch(intersections[intersectionIndex])
            {
                case straight:
                currentState = moveStraight;
                break;
                case leftTurn:
                currentState = turnLeft;
                break;
                case rightTurn:
                currentState = turnRight;
                break;
            }
        }
    }
}

void CenterSmartUpdate()
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
            //toggleClock();
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

    if (centerTimeoutCount >= centerTimeoutLimit) // Timeout
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
    if ((currentState != menu) && (ReadButton() == SELECT))
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
        }
    }

    // Read raw sensor values
    left = analogRead(LEFT_SENSOR);
    right = analogRead(RIGHT_SENSOR);
    leftDetected = left > thresholdLeft.Value;
    rightDetected = right > thresholdRight.Value;
    if (leftDetected || rightDetected)
    {
        #ifdef USE_SMART_INTERSECTIONS
        CenterSmartUpdate();
        #else
        CenterMapUpdate();
        #endif
    }
    SensorReset(); // Drain capacitor in preparation for next sensor reading

    // Display values on LCD
    lcdRefreshCount = (lcdRefreshCount <= 0) ? lcdRefreshPeriod : (lcdRefreshCount - 1);
    batteryCount = (batteryCount <= 0) ? batteryPeriod : (batteryCount - 1);
    DisplaySensorValues();   
}

// Calculates PID values for a single iteration of movement
void ProcessMovementDigital()
{
    if (leftDetected && rightDetected) error = 0; // No error if both sensors see the wire
    else if (!leftDetected && rightDetected) error = TOO_LEFT;
    else if (leftDetected && !rightDetected) error = TOO_RIGHT;
    else if (!leftDetected && !rightDetected) error = (previousError <= TOO_LEFT) ? -OFF_WIRE : OFF_WIRE;

    // Proportional
    proportional = error * float(proportionalGain.Value);

    // Integral
    #ifdef CALCULATE_INTEGRAL_ERROR    
    if (integralOffsetCounter < 0)
    {
        integral += (error * integralGain.Value);
        integralOffsetCounter = integralOffsetPeriod;
    }
    else
    { 
        integralOffsetCounter--;
    }

    if (integral > integralCap.Value) 
        integral = integralCap.Value;
    else if (integral < -integralCap.Value) 
        integral = -integralCap.Value;
    #else
    integral = 0;
    #endif

    // Derivative
    #ifdef CALCULATE_DERIVATIVE_ERROR
    derivative = (error - previousError) / float(derivativeCounter) * float(derivativeGain.Value);
    #else
    derivative = 0;
    #endif

    int baseSpeed = (speed.Value * 4.0);
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
    
    if(previousError != error)
    {
        previousError = error;
        derivativeCounter = 1;
    }
    else derivativeCounter++;
}

void ProcessMovementAnalog()
{
    if (!leftDetected && !rightDetected) error = (previousError <= 0) ? -500 : 500;
    else error = float(left) - float(right);

    // Proportional
    proportional = error * float(proportionalGain.Value);
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

    int baseSpeed = (speed.Value * 4.0);
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
void MotorSpeed(int motor, int speed)
{
	if (speed > 1000) speed = 1000;
	else if (speed < 0) speed = 0;
    analogWrite(motor, speed/4.0);
}

void Turn()
{
    int direction;
    if (currentState == turnLeft) direction = 1;
    else if (currentState == turnRight) direction = -1;
    else return;

    Clear();
    Print("TURN");

    // Move past intersection a small amount
    
    MotorSpeed(LEFT_MOTOR, 300);
    MotorSpeed(RIGHT_MOTOR, 300);
    delay(400);

    /*
    // Turn until the line has been lost
    MotorSpeed(LEFT_MOTOR, direction * 100);
    MotorSpeed(RIGHT_MOTOR, -direction * 100);
    do
    {
        left = analogRead(LEFT_SENSOR);
        right = analogRead(RIGHT_SENSOR);
        leftDetected = left > thresholdLeft.Value;
        rightDetected = right > thresholdRight.Value;
        SensorReset();
    } while (leftDetected || rightDetected);
    */

    MotorSpeed(LEFT_MOTOR, 500 * direction);
    MotorSpeed(RIGHT_MOTOR, 500 * -direction);
    delay(400);

    previousError = direction;
    currentState = moveStraight;

    Clear();
}