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
int potPin = A5; // Potentiometer output connected to analog pin 3
int potVal = 0; // Variable to store the input from the potentiometer

void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor
  pinMode(dirPin, OUTPUT);  // dir pin for motor

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  Serial.begin(9600);

  // start motor
  //duty = 0.0;//0.005;
  digitalWrite(dirPin, LOW);
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal
}

//Increase or decrease pos depends on user's definition of positive pos.
void loop() {
  n1 = digitalRead(encoder0PinA);
  n2 = digitalRead(encoder0PinB);
//Using rising edge of the channel A
  if ((encoder0PinALast == LOW) && (n1 == HIGH)) {
    if (n2 == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  }
//Using falling edge of the channel A
  else if ((encoder0PinALast == HIGH) && (n1 == LOW)) {
    if (n2 == HIGH) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  }
//Using rising edge of the channel B
  else if ((encoder0PinBLast == LOW) && (n2 == HIGH)) {
    if (n1 == HIGH) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  } 
//Using falling edge of the channel B
  else if ((encoder0PinBLast == HIGH) && (n2 == LOW)) {
    if (n1 == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  } 
  encoder0PinALast = n1;
  encoder0PinBLast = n2;
  Serial.print("encoder");Serial.println(encoder0Pos);Serial.println ("");
  // Serial.println(output);

  potVal = analogRead(potPin);   // read the potentiometer value at the input pin

  if (potVal < 341)  // Lowest third of the potentiometer's range (0-340)
  {                  
    potVal = (potVal * 3) / 4; // Normalize to 0-255
  }
  else if (potVal < 682) // Middle third of potentiometer's range (341-681)
  {
    potVal = ( (potVal-341) * 3) / 4; // Normalize to 0-255
  }
  else  // Upper third of potentiometer"s range (682-1023)
  {
    potVal = ( (potVal-683) * 3) / 4; // Normalize to 0-255
  }

  duty = potVal / 255.0; // convert potVal to 0-100% range
  digitalWrite(dirPin, HIGH);
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal

  // Serial.println(potVal);
  //Serial.println(output);
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