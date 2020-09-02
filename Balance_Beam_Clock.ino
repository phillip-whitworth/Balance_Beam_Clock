
/*   Balance Beam Clock
     
    This file was completed using multiple libraries from adafruit and arduino websites.
    They are awesome and deserve all  the credit!

    The idea behind this was to combine a balance beam with a clock to track the time in a unique way.

    We used an IR sensor, servo, arduino feather with servo hat and an LED display.

    This was the code for a final project in ME 401 - Mechatronics. The PID Controlled Balance beam also inculed positions along the beam to indicate the single digit minute
    Team members : Phillip Whitworth, Kyle Bryant, Jordan Raymond

    
*/
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include "Adafruit_LEDBackpack.h"


#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define STARTPOS  7.5// This is the lowest reading position from the IR Sensor
#define ENDPOS    50  // This is the highest reading position from the IR Sensor
#define num_pos  60


Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_PWMServoDriver servo1 = Adafruit_PWMServoDriver();

int incomingByte = 0;
int clock_time[4];
int timer_set = 0;
int current_secs = 0;
int previous_secs = 0;
double Setpoint, Input, Output;
double ball_pos[num_pos];
double Kp = 7.3967, Ki = 2.141, Kd = 3.721;

int irSensor1Pin = A5;
int irSensorSmoothing1[20];
int irSensorSmoothing2[40];
int servoSmoothing[20];
int range_check = 0;
double dist = 0.0;

PID balance(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(57600);
  matrix.begin(0x70);
  clock_time[0] = 2; // this is the tens of minutes
  clock_time[1] = 0; // this is the minutes
  clock_time[2] = 0; // this is the tens of seconds
  clock_time[3] = 1; // this is the second tick

  servo1.begin();
  servo1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(3000);
  float pos_total = ENDPOS - STARTPOS;
  float pos_step = pos_total / num_pos;
  for (int i = 0; i < num_pos; i++)
  {
    ball_pos[i] = STARTPOS + pos_step * i;
  }

  Input = 25;
  Setpoint = ball_pos[30];

  balance.SetMode(AUTOMATIC);
}
int servo_position = 1500;
int Output1 = 330;
void loop()
{
  int i = 0;
  if (Serial.available() > 0) // this area is used to set the clock time.
  {
    incomingByte = Serial.read();
    if (incomingByte == 'a')
    {
      clock_time[0] += 1;
    }
    if (incomingByte == 's')
    {
      clock_time[1] += 1;
    }
    if (incomingByte == 'd')
    {
      clock_time[2] += 1;
    }
    if (incomingByte == 'f')
    {
      clock_time[3] += 1;
    }
    if (incomingByte == 'k')
    {
      servo_position += 20;
    }
    if (incomingByte == 'l')
    {
      servo_position -= 20;
    }
    if (incomingByte == 'z')
    {
      Kp += 0.1;
    }
    if (incomingByte == 'x')
    {
      Kp -= 0.1;
    }
    if (incomingByte == 'c')
    {
      Ki += 0.1;
    }
    if (incomingByte == 'v')
    {
      Ki -= 0.1;
    }
    if (incomingByte == 'b')
    {
      Kd += 0.1;
    }
    if (incomingByte == 'n')
    {
      Kd -= 0.1;
    }
    balance.SetTunings(Kp, Ki, Kd, DIRECT);
  }

  //update the time
  current_secs = millis();
  if (current_secs - previous_secs >= 1000)
  {
    clock_time[3] += 1;
    previous_secs = current_secs;
  }
  //bind the display numbers to the bounds of a clock.

  if (clock_time[0] > 5)
  {
    clock_time[0] = 0;
  }
  if (clock_time[1] > 9)
  {
    clock_time[1] = 0;
    clock_time[0] += 1;
  }
  if (clock_time[2] > 5)
  {
    clock_time[2] = 0;
    clock_time[1] += 1;
  }
  if (clock_time[3] > 9)
  {
    clock_time[3] = 0;
    clock_time[2] += 1;
  }

  //displays the time on the clock
  boolean drawDots = false;

  matrix.writeDigitNum(0, clock_time[0], drawDots);
  matrix.writeDigitNum(1, clock_time[1], drawDots);
  matrix.writeDigitNum(3, clock_time[2], drawDots);
  matrix.writeDigitNum(4, clock_time[3], drawDots);

  matrix.drawColon(true);
  matrix.writeDisplay();

  int ball_point = clock_time[0] * 10 + clock_time[1]; // this sets the minutes as a 2 digit int

  //Serial.print(ball_point); Serial.print(": ");
  //Serial.println(ball_pos[ball_point]);

  Setpoint = ball_pos[ball_point];  // this setup can get used to change the setpoint of the ball

  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("---Kp: ");
  Serial.print(Kp);
  Serial.print("---Ki: ");
  Serial.print(Ki);
  Serial.print("---Kd: ");
  Serial.print(Kd);




  if (range_check == 0)
  {
    //  Getting data from the IR
    // Input = {({ IRsignal reading }]}
    int sensorTotal = 0;
    int i = 0;
    for (i = 19; i >= 0; i--)
    {
      irSensorSmoothing1[i + 1] = irSensorSmoothing1[i];
      sensorTotal += irSensorSmoothing1[i + 1];
      // Serial.print(i); Serial.print(": "); Serial.println(irSensorSmoothing[i]);
    }
    irSensorSmoothing1[0] = analogRead(irSensor1Pin);
    sensorTotal += irSensorSmoothing1[0];
    int irSensor1Value = sensorTotal / 20;
    //Serial.print("IR1:");
    //Serial.print(irSensor1Value);
    dist = 26091 * pow(irSensor1Value, -1.251);
    //Serial.print(" ---- distance [cm]: ");
    //Serial.println(dist);

    Input = dist;
    if (abs(Setpoint - Input) < 1)
    {
      range_check = 1;
    }
  }
  else
  {
    int sensorTotal = 0;

    for (i = 39; i >= 0; i--)
    {
      irSensorSmoothing2[i + 1] = irSensorSmoothing2[i];
      sensorTotal += irSensorSmoothing2[i + 1];
      // Serial.print(i); Serial.print(": "); Serial.println(irSensorSmoothing[i]);
    }
    irSensorSmoothing2[0] = analogRead(irSensor1Pin);
    sensorTotal += irSensorSmoothing2[0];
    int irSensor1Value = sensorTotal / 40;
    //Serial.print("IR1:");
    //Serial.print(irSensor1Value);
    dist = 26091 * pow(irSensor1Value, -1.251);
    //Serial.print(" ---- distance [cm]: ");
    //Serial.println(dist);

    Input = dist;
    if (abs(Setpoint - Input) > 1)
    {
      range_check = 0;
    }
  }

  Serial.print("-----input: ");
  Serial.print(Input);
  balance.Compute();

  Serial.print("-----Output: ");
  Serial.println(Output);


  int servoTotal = 0;
  for (i = 19; i >= 0; i--)
  {
    servoSmoothing[i + 1] = servoSmoothing[i];
    servoTotal += servoSmoothing[i + 1];
    // Serial.print(i); Serial.print(": "); Serial.println(irSensorSmoothing[i]);
  }
  servoSmoothing[0] = Output;
  servoTotal += servoSmoothing[0];
  int servoValue = servoTotal / 20;


  //Servo control needs to go here.   range is 255 - 405

  servo1.setPWM(4, 0, servoValue);

  //
  //  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  //    servo1.setPWM(4, 0, pulselen);
  //  }
  //
  //
}
