// An Chi Chen and John Cast and Hrishikesh Bhegade 
// Includes
#include <math.h>

// Parameter that define what environment to render
// #define ENABLE_VIRTUAL_WALL
// #define ENABLE_MASS_SPRING_DAMPER
// #define ENABLE_TELEOP_PART_A
// #define ENABLE_TELEOP_PART_B
// #define ENABLE_TELEOP_PART_C
// #define ENABLE_TELEOP_PART_D
#define ENABLE_VOICE_COIL_EXAMPLE

#define E_Value 2.71828     // Value for e

// Pin declares
int pwmPin = 5; // PWM output pin for voice coil 
// int dirPin = 8; // direction output pin for leader motor 
// int pwmPin_f = 6; // PWM output pin for follower motor 
// int dirPin_f = 7; // direction output pin for follower motor 
// int sensorPosPin = A2; // input pin for MR sensor
// int fsrPin = A3; // input pin for FSR sensor
// int sensorPosPin_f = A4; // input pin for follower MR sensor output

// Leader Position tracking variables
// int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
// int rawPos = 0;         // current raw reading from MR sensor
// int lastRawPos = 0;     // last raw reading from MR sensor
// int lastLastRawPos = 0; // last last raw reading from MR sensor
// int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
// int tempOffset = 0;
// int rawDiff = 0;
// int lastRawDiff = 0;
// int rawOffset = 0;
// int lastRawOffset = 0;
// const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred, follower too
// boolean flipped = false;
// double OFFSET = 980; // follower too
// double OFFSET_NEG = 15; // follower too

// Follower position tracking variables
// int updatedPos_f = 0;     // keeps track of the latest updated value of the MR sensor reading
// int rawPos_f = 0;         // current raw reading from MR sensor
// int lastRawPos_f = 0;     // last raw reading from MR sensor
// int lastLastRawPos_f = 0; // last last raw reading from MR sensor
// int flipNumber_f = 0;     // keeps track of the number of flips over the 180deg mark
// int tempOffset_f = 0;
// int rawDiff_f = 0;
// int lastRawDiff_f = 0;
// int rawOffset_f = 0;
// int lastRawOffset_f = 0;
// // const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
// boolean flipped_f = false;
// // double OFFSET = 980;
// // double OFFSET_NEG = 15;

// Kinematics variables
// Leader
// double xh = 0;           // Position of the handle [m]
// double theta_s = 0;      // Angle of the sector pulley in deg
// double xh_prev;          // Distance of the handle at previous time step
// double xh_prev2;
// double dxh;              // Velocity of the handle
// double dxh_prev;
// double dxh_prev2;
// double dxh_filt;         // Filtered velocity of the handle
// double dxh_filt_prev;
// double dxh_filt_prev2;

// Follower
// double xh_f = 0;           // Position of the handle [m]
// double theta_s_f = 0;      // Angle of the sector pulley in deg
// double xh_f_prev;          // Distance of the handle at previous time step
// double xh_f_prev2;
// double dxh_f;              // Velocity of the handle
// double dxh_f_prev;
// double dxh_f_prev2;
// double dxh_f_filt;         // Filtered velocity of the handle
// double dxh_f_filt_prev;
// double dxh_f_filt_prev2;

// Force output variables motor 1
// double force = 0;                 // Force at the handle
// double Tp = 0;                    // Torque of the motor pulley
double duty = 0;                  // Duty cylce (between 0 and 255)
unsigned int output = 0;          // Output command to the motor

// Force output variables motor 2
// double force_f = 0;                 // Force at the handle
// double Tp_f = 0;                    // Torque of the motor pulley
// double duty_f = 0;                  // Duty cylce (between 0 and 255)
// unsigned int output_f = 0;          // Output command to the motor

// *************** Parameter for the haptic rendering *******************
// Parameter for virtual wall
// double x_wall = 0.0005;//0.005;                  // Position of the virtual wall
// double k_wall = 1000;                     // Maximum stiffness of the virtual wall

// Parameter for mass spring damper simulation
// double m_msd = 2;        // mass of the mass spring damper simulation in kg
// double b_msd = 1;        // damping coefficient of the mass spring damper simulation in Ns/m
// double k_msd = 300;       // spring coefficient of the mass spring damper simulation in N/m
// double k_user = 1000;     // spring coefficient linking the user's

// position with the position of the mass
// double dt = 0.0006;      // time step used for simulation
// double x_msd = 0;        // position of the mass
// double v_msd = 0;        // velocity of the mass
// double a_msd = 0;        // acceleration of the mass
// double xi_msd = 0.0005;//0.005;   // initial position of the mass
// double vi_msd = 0;       // initial velocity of the mass
// double ai_msd = 0;       // initial acceleration of the mass

int counter = 0;
int on_timer = 0;
bool timer_on = false;

// // kp and kd constants for Leader and Follower
// double kp1 = 100;
// double kp2 = 100;
// double kd1 = 1;
// double kd2 = 1;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup()
{
  // Set up serial communication
  Serial.begin(9600);

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  // // Input pins
  // pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  // pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input
  // pinMode(sensorPosPin_f, INPUT);  // set follower's MR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  // pinMode(dirPin, OUTPUT);  // dir pin for motor A
  // pinMode(pwmPin_f, OUTPUT);  // PWM pin for motor B
  // pinMode(dirPin_f, OUTPUT);  // dir pin for motor B

  // Initialize motors
  // analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  // digitalWrite(dirPin, LOW);  // set direction
  // analogWrite(pwmPin_f, 0);     // set to not be spinning (0/255)
  // digitalWrite(dirPin_f, LOW);  // set direction

  // Initialize position valiables
  // Leader
  // lastLastRawPos = analogRead(sensorPosPin);
  // lastRawPos = analogRead(sensorPosPin);
  // flipNumber = 0;

  // Follower
  // lastLastRawPos_f = analogRead(sensorPosPin_f);
  // lastRawPos_f = analogRead(sensorPosPin_f);
  // flipNumber_f = 0;
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{

  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  //
  // Leader
  //
  // Get voltage output by MR sensor
  // rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // // Calculate differences between subsequent MR sensor readings
  // rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  // lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  // rawOffset = abs(rawDiff);
  // lastRawOffset = abs(lastRawDiff);

  // // Update position record-keeping vairables
  // lastLastRawPos = lastRawPos;
  // lastRawPos = rawPos;

  // // Keep track of flips over 180 degrees
  // if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
  //   if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
  //     flipNumber--;              // cw rotation
  //   } else {                     // if(rawDiff < 0)
  //     flipNumber++;              // ccw rotation
  //   }
  //   flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  // } else {                        // anytime no flip has occurred
  //   flipped = false;
  // }
  // updatedPos = rawPos + flipNumber * OFFSET; // need to update pos based on what most recent offset is

  // //
  // // Follower
  // //
  // // Get voltage output by MR sensor
  // rawPos_f = analogRead(sensorPosPin_f);  //current raw position from MR sensor

  // // Calculate differences between subsequent MR sensor readings
  // rawDiff_f = rawPos_f - lastRawPos_f;          //difference btwn current raw position and last raw position
  // lastRawDiff_f = rawPos_f - lastLastRawPos_f;  //difference btwn current raw position and last last raw position
  // rawOffset_f = abs(rawDiff_f);
  // lastRawOffset_f = abs(lastRawDiff_f);

  // // Update position record-keeping vairables
  // lastLastRawPos_f = lastRawPos_f;
  // lastRawPos_f = rawPos_f;

  // // Keep track of flips over 180 degrees
  // if ((lastRawOffset_f > flipThresh) && (!flipped_f)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
  //   if (lastRawDiff_f > 0) {       // check to see which direction the drive wheel was turning
  //     flipNumber_f--;              // cw rotation
  //   } else {                     // if(rawDiff < 0)
  //     flipNumber_f++;              // ccw rotation
  //   }
  //   flipped_f = true;            // set boolean so that the next time through the loop won't trigger a flip
  // } else {                        // anytime no flip has occurred
  //   flipped_f = false;
  // }
  // updatedPos_f = rawPos_f + flipNumber_f * OFFSET; // need to update pos based on what most recent offset is

  // //*************************************************************
  // //*** Section 2. Compute position in meters *******************
  // //*************************************************************

  // //
  // // Leader
  // //
  // double rh = .09038; // meters
  // double rs = 0.076; // meters
  // double rp = 0.004815; // meters
  // double m = -0.0106;//-0.0103;//-0.0103;
  // double b = 7.2888;//-0.8914;//2.5591;

  // // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  // double theta_s = m*updatedPos + b;
  
  // // Compute the position of the handle (in meters) based on ts (in radians)
  // xh = (rh*rp/rs)*theta_s*M_PI/180.0; // meters

  // // Calculate the velocity of the handle
  // dxh = (double)(xh - xh_prev) / 0.001;

  // // Calculate the filtered velocity of the handle using an infinite impulse response filter
  // dxh_filt = .9 * dxh + 0.1 * dxh_prev;

  // // Record the position and velocity
  // xh_prev2 = xh_prev;
  // xh_prev = xh;

  // dxh_prev2 = dxh_prev;
  // dxh_prev = dxh;

  // dxh_filt_prev2 = dxh_filt_prev;
  // dxh_filt_prev = dxh_filt;

  // //
  // // Follower
  // //
  // double rh_f = 0.075;   //[m]
  // double rp_f = 0.007;
  // double rs_f = 0.073;
  // double m_f = -0.0122;//-0.0127;//-0.0124;
  // double b_f = 5.1228;//10.856;//-9.2383;

  // // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  // double theta_s_f = m_f*updatedPos_f + b_f;
  
  // // Compute the position of the handle (in meters) based on ts (in radians)
  // xh_f = (rh_f*rp_f/rs_f)*theta_s_f*M_PI/180.0; // meters

  // // Calculate the velocity of the handle
  // dxh_f = (double)(xh_f - xh_f_prev) / 0.001;

  // // Calculate the filtered velocity of the handle using an infinite impulse response filter
  // dxh_f_filt = .9 * dxh_f + 0.1 * dxh_f_prev;

  // // Record the position and velocity
  // xh_f_prev2 = xh_f_prev;
  // xh_f_prev = xh_f;

  // dxh_f_prev2 = dxh_f_prev;
  // dxh_f_prev = dxh_f;

  // dxh_f_filt_prev2 = dxh_f_filt_prev;
  // dxh_f_filt_prev = dxh_f_filt;

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************

#ifdef ENABLE_VOICE_COIL_EXAMPLE
  // duty = 1.0;
  // output = (int)(duty * 255);  // convert duty cycle to output signal
  // Serial.println(output, 4);
  // analogWrite(pwmPin, (int)25); // output the signal
  
  // counter++;
  // if (counter % 10000 == 0) {
  //   digitalWrite(pwmPin, HIGH);
  //   timer_on = true;
  //   counter = 0;
  // } else if (timer_on) {
  //   on_timer++;
    
  //   if (on_timer >= 750) {
  //     on_timer = 0;
  //     timer_on = false;
  //     digitalWrite(pwmPin, LOW);
  //   }
  // }  
  // Serial.println(timer_on);

  digitalWrite(pwmPin, HIGH);
  delay(100);//0.001);
  digitalWrite(pwmPin, LOW);
  delay(200);//100);
  // Serial.println("low");
#endif

//   // Forces algorithms
// #ifdef ENABLE_VIRTUAL_WALL
//   if (xh > x_wall)
//   {
//     //to do: what is the force when contacting wall?
//     force = k_wall*(x_wall - xh); // N
//   }
//   else
//   {
//     //to do: what is the force when not contacting wall?
//     force = 0;
//   }

//   //using this counter is optional, just limits how many times we print to Processing (increasing baud rate can also help if need be)
//   if (counter >= 100) {
//     counter = 0;
//     Serial.println(xh, 4); //print to processing

//   } else {
//     counter++;
//   }
// #endif


// #ifdef ENABLE_MASS_SPRING_DAMPER
//   // Calculate the relevant state at this time step
//   v_msd = v_msd + a_msd * dt;
//   x_msd = x_msd + v_msd * dt;

//   // calculate force and a_msd 
//   // Determine whether the user's position is in front
//   float Fsystem = -v_msd*b_msd - k_msd*(x_msd - xi_msd); // mass/spring damper forces of mass
//   float Fuser = k_user*(xh - x_msd); // spring force of user engaging mass
//   if (xh > x_msd)
//   {
//     // to do: what is the force when the xh > x_msd
//     force = Fsystem + Fuser;
//     // what is the acceleration of the msd? (include effect of contact with user as well as other forces)
//     a_msd = force / m_msd;
//   }
//   else
//   {
//     // to do: what is the force when the xh < x_msd
//     force = 0;
//     // what is the acceleration of the msd? 
//     a_msd = Fsystem / m_msd;
//   }

//   //using this counter is optional, just limits how many times we print to Processing (increasing baud rate can also help if need be)
//   if (counter >= 100) {
//     counter = 0;
//     //print to processing
//     Serial.print(xh, 4);
//     Serial.print(' ');
//     Serial.println(x_msd, 4);

//   } else {
//     counter++;
//   }
// #endif

// #ifdef ENABLE_TELEOP_PART_A
//   // Serial.print(xh, 4);
//   // Serial.print(' ');
//   // Serial.println(xh_f, 4);
//   // Serial.println(updatedPos);
  
//   double kp1 = 1000;//2000;
//   double kp2 = 1000;//2000;
//   double kd1 = 1;
//   double kd2 = 1;
  
//   force = kp1*(xh_f - xh) + kd1*(dxh_f - dxh);
//   force_f = kp2*(xh - xh_f) + kd2*(dxh - dxh_f);
// #endif

// #ifdef ENABLE_TELEOP_PART_B
//   force = kp1*(xh_f - 4*xh) + kd1*(dxh_f - dxh);
//   force_f = kp2*(4*xh - xh_f) + kd2*(dxh - dxh_f);
// #endif

// #ifdef ENABLE_TELEOP_PART_C
//   force = kp1*(xh_f - xh) + kd1*(dxh_f - dxh);
//   force_f = 10*(kp2*(xh - xh_f) + kd2*(dxh - dxh_f));
// #endif

// #ifdef ENABLE_TELEOP_PART_D
//   // kp and kd constants for Leader and Follower
//   double kp1 = 1000;//10;//2000;
//   double kp2 = 1000;//10;//2000;
//   double kd1 = 1;//1000;
//   double kd2 = 1;//1000;

//   force = kp1*(xh_f - xh) + kd1*(dxh_f - dxh);
//   force_f = kp2*(xh - xh_f) + kd2*(dxh - dxh_f);
// #endif

//   //Step C.2
//   Tp = rp / rs * rh * force;  // Compute the require motor pulley torque (Tp) to generate that force
//   Tp_f = rp_f / rs_f * rh_f * force_f;  // Compute the require motor pulley torque (Tp) to generate that force
//   // Serial.println(Tp, 4);

//   //*************************************************************
//   //*** Section 4. Force output (do not change) *****************
//   //*************************************************************

//   // Determine correct direction for motors' torques
//   // motor 1
//   if (force > 0) {
//     digitalWrite(dirPin, HIGH);
//   } else {
//     digitalWrite(dirPin, LOW);
//   }
//   // motor 2
//   if (force_f > 0) {
//     digitalWrite(dirPin_f, HIGH);
//   } else {
//     digitalWrite(dirPin_f, LOW);
//   }

//   // Compute the duty cycle required to generate Tp (torque at the motor pulley)
//   // motor 1
//   duty = sqrt(abs(Tp) / 0.03);
//   // motor 2
//   duty_f = sqrt(abs(Tp_f) / 0.03);

//   // Make sure the duty cycle is between 0 and 100%
//   // motor 1
//   if (duty > 1) {
//     duty = 1;
//   } else if (duty < 0) {
//     duty = 0;
//   }
//   // motor 2
//   if (duty_f > 1) {
//     duty_f = 1;
//   } else if (duty_f < 0) {
//     duty_f = 0;
//   }

//   // motor 1
//   output = (int)(duty * 255);  // convert duty cycle to output signal
//   analogWrite(pwmPin, output); // output the signal

//   // motor 2
//   output_f = (int)(duty_f * 255);  // convert duty cycle to output signal
//   analogWrite(pwmPin_f, output_f); // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}