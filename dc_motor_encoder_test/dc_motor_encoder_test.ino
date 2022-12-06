//Quadrature Encoder
//Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.
//Mechatronics 530.421 Lab 2

int encoder0PinA = 9;
int encoder0PinB = 10;
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

int vibDigWritePin1 = 2;
int vibDigWritePin2 = 3;

int count = 0;

void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor
  pinMode(dirPin, OUTPUT);  // dir pin for motor
  pinMode(vibPin, OUTPUT); // PWM pin for vib act
  pinMode(vibDigWritePin1, OUTPUT);

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

  //if (encoder0Pos % 480 == 0) {
  //  Serial.print("encoder");Serial.println(encoder0Pos);Serial.println ("");
  //}
  
//   Serial.println(output);


  cirEncoderPos = encoder0Pos % 480;
  
  //digitalWrite(vibDigWritePin1, LOW);

  //if (encoder0Pos % 480 == 2) {
  //  Serial.println(encoder0Pos);
  //}

  if (cirEncoderPos > (tar1-(tarRange/2)) && cirEncoderPos < (tar1+(tarRange/2)) ) {    
    
    digitalWrite(vibDigWritePin1, HIGH);
    analogWrite(vibPin, (int)255);
  }
  else{
    digitalWrite(vibDigWritePin1, LOW);
    analogWrite(vibPin, (int)0);
  }

  //analogWrite(vibPin, (int)125);

  count = count + 1;
  if (count % 400 == 0){
    analogWrite(vibDigWritePin1, count);
  }
  

  Serial.println(count);


  potVal = analogRead(potPin);   // read the potentiometer value at the input pin

  //Serial.println(potVal);

  if (potVal < 140)  // Lowest half of the potentiometer's range (0-512) -> rotate clockwise
  {       
    //potVal = potVal - 123;           
    potVal = 255*(140 - potVal)/ 140; // Normalize to 0-255
    digitalWrite(dirPin, LOW);
  }
  else if (potVal > 190)  // Upper half of potentiometer"s range (512-1023) -> rotate counter clockwise
  {
    potVal = 255*(potVal-190)/833; // Normalize to 0-255
    digitalWrite(dirPin, HIGH);
  }
  else {
    potVal = 0;
  }






  // if (encoder0Pos % 480 == 0) {
  //   Serial.println(potVal);
  // }

  duty = potVal / 255.0; // convert potVal to 0-100% range
  Serial.println(duty);
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal

  // Serial.println(potVal);
  // Serial.println(output);
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