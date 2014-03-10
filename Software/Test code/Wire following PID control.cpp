#include <LiquidCrystal.h>

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

// State tracking
bool MENU = true;
int lcdRefreshCount = 0;
int lcdRefreshPeriod = 200;

// Sensors
int thresholdLeft = 300;
int leftDetected = false;
int thresholdRight = 300;
int rightDetected = false;

int speed = 100.0;
int error = 0.0;
int previousError = 0.0;

float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
int derivativeCounter = 0;

int proportionalGain = 1.0;
int integralGain = 1.0;
int derivativeGain = 1.0;

void setup()
{
    lcd.begin(16, 2);   // Initialize LCD
    lcd.setCursor(0,0);
}

void loop()
{
    Update();
    ProcessMovement();
    delay(5);
}

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

void Print(String text, int value = -1) 
{
    lcd.print(text);
    if (value != -1) lcd.print(value);
}

void Cursor(int row, int column)
{
    lcd.setCursor(column, row);
}

void Update()
{   
    if (ReadButton() == SELECT) MENU = true;
    left = analogRead(LEFT_SENSOR);
    right = analogRead(RIGHT_SENSOR);
    leftDetected = left > thresholdLeft;
    rightDetected = right > thresholdRight;
    SensorReset();
    lcdRefreshCount = (lcdRefreshCount <= 0) ? lcdRefreshPeriod : (lcdRefreshCount - 1);
}

void ProcessMovement()
{
    if (leftDetected && rightDetected) error = 0;
    else if (!leftDetected && rightDetected) error = TOO_LEFT;
    else if (leftDetected && !rightDetected) error = TOO_RIGHT;
    else if (!leftDetected && !rightDetected) error = (previousError <= TOO_LEFT) ? -OFF_TAPE : OFF_TAPE;

    proportional = error * proportionalGain;
    derivative = (error - previousError) / (float)derivativeCounter * derivativeGain;
    MotorSpeed(LEFT_MOTOR, speed + (proportional + derivative));
    MotorSpeed(RIGHT_MOTOR, speed - (proportional + derivative));
    
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

void MotorSpeed(int motor, int speed)
{
    analogWrite(motor, speed/4);
}

void SensorReset(int microseconds = 10)
{
    digitalWrite(SENSOR_RESET, HIGH);
    delayMicroseconds(microseconds);
    digitalWrite(SENSOR_RESET, LOW);
}

void Clear()
{
    lcd.clear();
}