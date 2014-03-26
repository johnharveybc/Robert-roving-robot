#include <LiquidCrystal.h>
#include <EEPROM.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Configuration
#define SUBTRACTIVE_MOTOR_SPEED     // Comment for additive and subtractive motor speed
#define CALCULATE_DERIVATIVE_ERROR  // Comment to remove derivative gain from error calculations

// Pin definitions
#define LEFT_SENSOR 2       // ANALOG
#define RIGHT_SENSOR 1      // ANALOG
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

struct Parameter {
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
int batteryVoltage = 0;

// Sensors
int left = 0;
int right = 0;
int leftDetected = false;
int rightDetected = false;

float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
int derivativeCounter = 0;

int error = 0.0;
int previousError = 0.0;

// State tracking
state currentState = menu;
int menuIndex = 0;

// EEPROM values
#define PARAMETER_COUNT 6 // MUST EQUAL NUMBER OF PARAMETERS
Parameter proportionalGain  = {"P-gain",   0, 1}; 
Parameter integralGain      = {"I-gain",   0, 2}; 
Parameter derivativeGain    = {"D-gain",   0, 3}; 
Parameter speed             = {"Speed",    0, 4}; 
Parameter thresholdLeft     = {"L-Thresh", 0, 5}; 
Parameter thresholdRight    = {"R-Thresh", 0, 6}; 

// Make sure any EEPROM value is added to the array
Parameter *parameters[] = 
{
    &proportionalGain, &integralGain, &derivativeGain,
    &speed, &thresholdLeft, &thresholdRight
};

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
        ProcessMovement();
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
    Print("L: ", analogRead(LEFT_SENSOR));
    Print(" R: ", analogRead(RIGHT_SENSOR));

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
        parameters[menuIndex]->Value += 1 + (holdCounter / 20);
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

// Updates the sensors and machine state
void Update()
{   
    // Check if MENU button is being held down
    if ((currentState != menu) && (ReadButton() == SELECT))
    {
        delay(750);
        if (ReadButton() == SELECT) // debounce MENU button
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

    left = analogRead(LEFT_SENSOR);
    right = analogRead(RIGHT_SENSOR);
    leftDetected = left > thresholdLeft.Value;
    rightDetected = right > thresholdRight.Value;
    SensorReset(); // Drain capacitor in preparation for next sensor reading
    lcdRefreshCount = (lcdRefreshCount <= 0) ? lcdRefreshPeriod : (lcdRefreshCount - 1);
    batteryCount = (batteryCount <= 0) ? batteryPeriod : (batteryCount - 1);
}

// Calculates PID values for a single iteration of movement
void ProcessMovement()
{
    if (leftDetected && rightDetected) error = 0; // No error if both sensors see the wire
    else if (!leftDetected && rightDetected) error = TOO_LEFT;
    else if (leftDetected && !rightDetected) error = TOO_RIGHT;
    else if (!leftDetected && !rightDetected) error = (previousError <= TOO_LEFT) ? -OFF_WIRE : OFF_WIRE;

    proportional = error * float(proportionalGain.Value);
    derivative = (error - previousError) / float(derivativeCounter) * float(derivativeGain.Value);
    
    int baseSpeed = (speed.Value * 4.0);
    int mLeft  = baseSpeed + (proportional + derivative);
    int mRight = baseSpeed - (proportional + derivative);

    #ifndef CALCULATE_DERIVATIVE_ERROR
    mLeft -= derivative;
    mRight += derivative;
    #endif

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

    if(lcdRefreshCount == 0)  // Mitigates screen flicker and time consumping operations
    {
        // Sensors on top line
        Cursor(TOP, 0); 
        Print("L: ", left); Print("   ");
        Print(" R: ", right); Print("   ");
    }
    
    if(batteryCount == 0) return;
    {
        // Battery voltage on bottom line
        Cursor(BOTTOM, 0);
        Print("Batt: ", batteryVoltage); Print("    ");
    }  
}

// Modifies the motor speed given a value between 0 and 100
void MotorSpeed(int motor, int speed)
{
	if (speed > 1000) speed = 1000;
	else if (speed < 0) speed = 0;
    analogWrite(motor, speed/4.0);
}