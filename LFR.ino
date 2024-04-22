// Number of sensors. Float has been used to ensure that there are no errors during execution.
float Sensor_Count = 8;

// Pin numbers for the sensor inputs. Since there are 8 sensors, there's an array of 8
int Sensor_Pins[8] = {2, 3, 4, 5, 6, 7, 8, 9};

// Pin numbers for the motor
int LMP =  10;   // Pin # for left motor
int RMP = 11;   // Pin # for right motor

// PID Constants
// These were tweaked for an array of 8 sensors
float Kp = 2.7;     // Proportional gain
float Ki = 0.015;   // Integral gain
float Kd = 0.18;    // Derivative gain

// Variables for PID control
float Error = 0;
float Last_Error = 0;
float Integral = 0;
float Derivative = 0;
float PID_Value = 0;

void setup() 
{
  // Initializes sensor pins
  for (int i = 0; i < Sensor_Count; i++) 
  {
    pinMode(Sensor_Pins[i], INPUT);
  }

  // Initializes motor pins
  pinMode(LMP, OUTPUT);
  pinMode(RMP, OUTPUT);
}

void loop() 
{
  // Reads sensor values
  int Sensor_Values[8];

  for (int i = 0; i < Sensor_Count; i++) 
  {
    Sensor_Values[i] = digitalRead(Sensor_Pins[i]);
  }

  // Error Calculation
  float Weighted_Sum = 0;
  float Sum_of_Weights = 0;

  for (int i = 0; i < Sensor_Count; i++) 
  {
      Weighted_Sum += Sensor_Values[i] * i; // Multiplies sensor value by its position
      Sum_of_Weights += Sensor_Values[i];
  }

  float Line_Position = Weighted_Sum / Sum_of_Weights; // Calculates weighted average
  float Correct_Position = (Sensor_Count - 1) / 2.0; // Center of the sensor array
  Error = Correct_Position - Line_Position;

  // Normalizes error to a range between -1 to 1
  Error = Error / (Sensor_Count / 2.0);



  // Calculates PID control value
  Integral += Error;
  Derivative = Error - Last_Error;
  PID_Value = (Kp * Error) + (Ki * Integral) + (Kd * Derivative);

  int LMS = 200 + PID_Value;  // Base left motor speed + PID_Value
  int RMS = 200 + PID_Value;  // Base right motor speed + PID_Value

  // Ensures that motor speeds are within allowable range
  LMS = constrain(LMS, 0, 255);
  RMS = constrain(RMS, 0, 255);

  // Set motor speeds
  analogWrite(LMP, LMS);
  analogWrite(RMP, RMS);

  // Update last error
  Last_Error = Error;
}
