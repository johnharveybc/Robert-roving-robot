#include <LiquidCrystal.h>
#include <EEPROM.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Pin definitions
#define LEFT_SENSOR 1  // ANALOG
#define RIGHT_SENSOR 2 // ANALOG
#define BUTTON_INPUT 0 // ANALOG
#define LEFT_MOTOR 11 // PWM
#define RIGHT_MOTOR 3 // PWM
#define SENSOR_RESET 13 // DIGITAL

// Keypad definitions
#define RIGHT  0
#define UP     1
#define DOWN   2
#define LEFT   3
#define SELECT 4
#define NONE   5

// LCD definitions
#define TOP 0
#define BOTTOM 1

// Wire States
#define TOO_LEFT -1
#define TOO_RIGHT 1
#define OFF_WIRE 5

struct Parameter {
    String Name;
    byte Value;
    int Index;
}; 

// Prevents LCD flicker
int lcdRefreshCount = 0;
int lcdRefreshPeriod = 200;

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
bool MENU = true;
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
Parameter parameters[] = 
{
    proportionalGain, integralGain, derivativeGain,
    speed, thresholdLeft, thresholdRight
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
    lcd.begin(16, 2);   // Initialize LCD
    lcd.setCursor(0,0);
    pinMode(SENSOR_RESET, OUTPUT);

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
    if (MENU)
    {
        ShowMenu();
    }
    else
    {
        Update();
        ProcessMovement();
    }
}

// Drains the capacitors on the peak detector so that can be read again
void SensorReset(int microseconds = 25)
{
    digitalWrite(13, HIGH);
    delayMicroseconds(microseconds);
    digitalWrite(13, LOW);
    delayMicroseconds(microseconds);
}

// Returns the current button being pressed
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

void ShowMenu()
{
    Clear();
    Cursor(TOP, 0);
    Print(parameters[menuIndex].Name);
    Print(" ", parameters[menuIndex].Value);

    switch(ReadButton())
    {
        case UP:
        parameters[menuIndex].Value++;
        break;
        case DOWN:
        parameters[menuIndex].Value--;
        break;
        case LEFT:
        menuIndex = (menuIndex > 0) ? (menuIndex - 1) : (PARAMETER_COUNT - 1);
        break;
        case RIGHT:
        menuIndex = (menuIndex < PARAMETER_COUNT - 1) ? (menuIndex + 1) : 0;
        break;
        case SELECT:
        delay(1000);
        if (ReadButton() == SELECT)
        {
            Clear();
            Cursor(TOP, 0);
            Print("Exiting menu");
            SaveToEEPROM();
            delay(1000);
            MENU = false;
            break;
        }
        break;
    }
    delay(150);
}

// Loads all values from the EEPROM
void LoadFromEEPROM()
{
    for(int i = 0; i < PARAMETER_COUNT; i++)
        parameters[i].Value = EEPROM.read(parameters[i].Index);
}

// Saves all values to the EEPROM
void SaveToEEPROM()
{
    for(int i = 0; i < PARAMETER_COUNT; i++)
        EEPROM.write(parameters[i].Index, parameters[i].Value);
}

// Updates the sensors and machine state
void Update()
{   
    // Check if MENU button is being held down
    if (ReadButton() == SELECT)
    {
        delay(1000);
        if (ReadButton() == SELECT)
        {
            Clear();
            Cursor(TOP, 0);
            Print("Entering menu");
            MENU = true;
            delay(1000);
        }
    }

    left = analogRead(LEFT_SENSOR);
    right = analogRead(RIGHT_SENSOR);
    leftDetected = left > thresholdLeft.Value;
    rightDetected = right > thresholdRight.Value;
    SensorReset();
    lcdRefreshCount = (lcdRefreshCount <= 0) ? lcdRefreshPeriod : (lcdRefreshCount - 1);
}

// Calculates PID values for a single iteration of movement
void ProcessMovement()
{
    if (leftDetected && rightDetected) error = 0;
    else if (!leftDetected && rightDetected) error = TOO_LEFT;
    else if (leftDetected && !rightDetected) error = TOO_RIGHT;
    else if (!leftDetected && !rightDetected) error = (previousError <= TOO_LEFT) ? -OFF_WIRE : OFF_WIRE;

    proportional = error * float(proportionalGain.Value);
    derivative = (error - previousError) / float(derivativeCounter) * float(derivativeGain.Value);
    MotorSpeed(LEFT_MOTOR, speed.Value + (proportional + derivative));
    MotorSpeed(RIGHT_MOTOR, speed.Value - (proportional + derivative));
    
    if(previousError != error)
    {
        previousError = error;
        derivativeCounter = 1;
    }
    else derivativeCounter++;

    if(lcdRefreshCount != 0) return; // Mitigates screen flicker
    Clear();
    Cursor(TOP, 0);
    Print("L: ", left);
    Cursor(BOTTOM, 0);
    Print("R: ", right);
}

// Modifies the motor speed given a value between 0 and 100
void MotorSpeed(int motor, int speed)
{
    analogWrite(motor, speed/4);
}

