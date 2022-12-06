//Quadrature Encoder
//Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.
//Mechatronics 530.421 Lab 2

int encoder0PinA = 4;
int encoder0PinB = 5;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int encoder0PinBLast = LOW;
int n1 = LOW;
int n2 = LOW;

int pwmPin = 5; // PWM output pin for motor 
int dirPin = 8; // direction output pin for motor 
double duty = 0; // Duty cylce (between 0 and 255)
unsigned int output = 0; // Output command to the motor

// INPUT: Potentiometer should be connected to 5V and GND
int potPin = A3; // Potentiometer output connected to analog pin 3
int potVal = 0; // Variable to store the input from the potentiometer



// declare variable values for vib

int vibPin = 6; // PWM pin for vibration actuator 
int tar1 = 240;    // encoder position of vib act row 1
int tar2 = 0;   // encoder position of vib act row 2
int tarRange = 20; // range around tar values that vib act should be actuated
int cirEncoderPos;   // position of encoder limited to 0 - 480

int vibDigReadPin1 = A4;
int vibDigWritePin2 = 3;

int vib1;



void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor
  pinMode(dirPin, OUTPUT);  // dir pin for motor
  pinMode(vibPin, OUTPUT); // PWM pin for vib act
  pinMode(vibDigReadPin1, INPUT);

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  Serial.begin(9600);

  // start motor
  //duty = 0.0;//0.005;
  //digitalWrite(dirPin, LOW);
  //output = (int)(duty * 255);  // convert duty cycle to output signal
  //analogWrite(pwmPin, output); // output the signal



}

//Increase or decrease pos depends on user's definition of positive pos.
void loop() {
  vib1 = analogRead(vibDigReadPin1);


  if (vib1 % 400 == 5){
    Serial.println(vib1);
  }
  


  if (vib1 == HIGH) {
    analogWrite(vibPin, (int)125);
  } 
  else {
    analogWrite(vibPin, (int)0);
  }

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