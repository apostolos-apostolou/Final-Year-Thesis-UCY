#include <util/atomic.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// Sabertooth driver settings
const int sabertoothAddress = 128;    // Sabertooth address
const int txPin = 11;                 // TX pin for software serial to Sabertooth

// Sabertooth driver setup
SoftwareSerial SWSerial(NOT_A_PIN, txPin); // RX on no pin (unused), TX on pin 11 (to S1)
Sabertooth ST(sabertoothAddress, SWSerial);  // Sabertooth with address 128

// Pins right motor
#define ENCA_R 2 // Interrupt pin
#define ENCB_R 4

// Pins left motor
#define ENCA_L 3 // Interrupt pin
#define ENCB_L 5

// Globals
float countsPerRevolution = 246.00; // Encoder's counts per shaft revolution
String serialCommand; // Variable to store serial input

// Globals for right motor
long prevT_R = 0; // Previus time 
int posPrev_R = 0; // Previous position
volatile int pos_i_R = 0; // Encoder position
volatile float velocity_i_R = 0; // Velocity measured in encoder's counts/second
float v1Filt_R = 0; // Filtered motor rad/s
float v1Prev_R = 0; // Previous rad/s
float eintegral_R = 0; // Sum of error/intergral
float prevErr_R = 0; // Previous error

// Globals for left motor - same as for right motor
long prevT_L = 0;
int posPrev_L = 0;
volatile int pos_i_L = 0;
volatile float velocity_i_L = 0;
float v1Filt_L = 0;
float v1Prev_L = 0;
float eintegral_L = 0;
float prevErr_L = 0;

// Set target velocities in rad/s
float vt_R = 0;  // Target for right motor in rad/s
float vt_L = 0;  // Target for left motor in rad/s

// PID constants
float kp = 2.55; // Proportional
float ki = 2.15; // Integral
float kd = 0.35; // Derivative

void setup() 
{
  //Serial begin
  Serial.begin(19200);
  
  // Sabertooth serial setup
  SWSerial.begin(9600);
  ST.autobaud();

  // Setup right motor encoder pins
  pinMode(ENCA_R, INPUT_PULLUP); // Need pullup resistor
  pinMode(ENCB_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoderRight, RISING); // Go to readEncoder function when interrupt pin in triggered

  // Setup left motor encoder pins
  pinMode(ENCA_L, INPUT_PULLUP);
  pinMode(ENCB_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoderLeft, RISING);
}

void loop() 
{
  // Check serial to read the received command
  getCommand(); 

  // Right motor control
  controlMotor('R', pos_i_R, posPrev_R, prevT_R, v1Filt_R, v1Prev_R, eintegral_R, prevErr_R, vt_R);

  // Left motor control
  controlMotor('L', pos_i_L, posPrev_L, prevT_L, v1Filt_L, v1Prev_L, eintegral_L, prevErr_L, vt_L);

  // 1ms delay so that sampling frequency is 1kHz (needed for the filter implementation)
  delay(1); 
}

// Function to implement PID
void controlMotor(char motor, volatile int &pos_i, int &posPrev, long &prevT, float &v1Filt, float &v1Prev, float &eintegral, float &prevErr, float vt) 
{
  // Read the position in an atomic block
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  { 
    pos = pos_i;
  }

  // Calculate velocity
  long currT = micros(); // Current timestamp in microseconds 
  float deltaT = ((float)(currT - prevT)) / 1.0e6;  // Time difference in seconds
  float velocity = (pos - posPrev) / deltaT; // Motor speed in counts/second
  posPrev = pos; // Set current position as previous position for the next loop

  // Convert count/s to rad/s
  float v1 = (velocity / countsPerRevolution) * 2.0 * 3.14159; // Radians per second

  // Multiply speed by 10 so i can you smaller PID Gains
  v1 = v1*10.0;
  vt = vt*10.0;

  // Low-pass filter (25 Hz cutoff) for reduce the noise of the measured rad/s
  v1Filt = 0.8544 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1; // Set current rad/s as previous rad/s for the next loop
  
  // PID control
  float e = vt - v1Filt; // Calculate error
  eintegral = eintegral + e * deltaT; // Calculate the integral of the error
  float derivative = (e - prevErr) / (currT - prevT); // Calculate the derivative of the error
  prevT = currT; // Set current time as previous time for the next loop
  prevErr = e; // Set current error as previous error for the next loop

  float u = (kp * e) + (ki * eintegral) + (kd * derivative); // Calculate control signal

  // Set motor speed and direction
  int dir = (u < 0) ? -1 : 1; // If control signal is below zero, direction is negative
  int signal = (int)fabs(u); // Get absolute value of control signal
  if (signal > 127) // Limit signal to a max value of 127  - Designed for the Sabertooth 2x12 motor driver
  { 
    signal = 127;
  }

  // If target speed is 0, set motor's speed to 0
  if(vt == 0)
    signal = 0;
  
  setMotor(dir, signal, motor); // Give signal to the motor

  //Serial.println(vt);
  //Serial.println(v1);
}

// Function to control the motors
void setMotor(int dir, int speed, char motor)
{
  if (dir == 1) // Forward direction
  { 
    if (motor == 'R') 
    {
      ST.motor(2, -speed); // Right motor forward
    }else 
    {
      ST.motor(1, -speed); // Left motor forward
    }
  }else // Backwards
  { 
    if (motor == 'R') 
    { 
      ST.motor(2, speed); // Right motor backward
    }else 
    {
      ST.motor(1, speed); // Left motor backward
    }
  }
}

// Get right encoder's count
void readEncoderRight() 
{
  int b = digitalRead(ENCB_R);
  int increment = (b > 0) ? 1 : -1;
  pos_i_R += increment;
}

// Get left encoder's count
void readEncoderLeft() 
{
  int b = digitalRead(ENCB_L);
  int increment = (b > 0) ? -1 : 1;
  pos_i_L += increment;
}

void getCommand()
{
  if(Serial.available() > 0)
  {
    String serialCommand = Serial.readStringUntil('\n');
    char dec = serialCommand[0];

    if(dec == 'm') // Set desired speed in rad/s: 'm vt_R vt_L'
    {
      int index = 0;
      String arr[3];
      for (int i = 0; i < 3; i++)
      {
         index = serialCommand.indexOf(' ');
         arr[i] = serialCommand.substring(0, index);
         serialCommand = serialCommand.substring(index+1);
      }
      vt_L= arr[1].toFloat();
      vt_R = arr[2].toFloat();
      Serial.println("Wheel speed command received");
    }
    else if(dec == 'p') // Set PID Value 'p Kp Ki Kd'
    {
      int index = 0;
      String arr[4];
      for (int i = 0; i < 4; i++)
      {
         index = serialCommand.indexOf(' ');
         arr[i] = serialCommand.substring(0, index);
         serialCommand = serialCommand.substring(index+1); 
      }
      kp = arr[1].toFloat();
      ki = arr[2].toFloat();
      kd = arr[3].toFloat();
      Serial.println("PID Gains command received");
    }
    else if(dec == 'e') // Print encoder counts
    {
      Serial.print(pos_i_L);
      Serial.print(" ");
      Serial.println(pos_i_R);
    }
    else if(dec == 'r') // Reset encoder count to 0 for both motors
    {
      pos_i_L = 0;
      pos_i_R = 0;
      Serial.println("Encoder reset command received");
    }
  }
}