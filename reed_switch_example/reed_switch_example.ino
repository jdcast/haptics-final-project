const int REED_PIN = 2;
const int LED_PIN = 3;
int pwmPinVM = 6; // PWM output pin for vibration motor
int dirPinVM = 7; // direction output pin for vibration motor

void setup() {
	Serial.begin(9600);
	pinMode(REED_PIN, INPUT_PULLUP);
	pinMode(LED_PIN, OUTPUT);
  pinMode(pwmPinVM, OUTPUT);  // PWM pin for vibration motor
  pinMode(dirPinVM, OUTPUT);  // dir pin for vibration motor
}

void loop() {
	int proximity = digitalRead(REED_PIN);

	if (proximity == LOW) {
		Serial.println("Switch closed");
		// digitalWrite(LED_PIN, HIGH);

    digitalWrite(dirPinVM, HIGH);
    analogWrite(pwmPinVM, 54);  // output the signal, PWM duty cycle defined for 0-255; we give a noticeable but not overwhelming vibration
	}
	else {
		Serial.println("Switch opened");
		// digitalWrite(LED_PIN, LOW);

    digitalWrite(dirPinVM, LOW);
    analogWrite(pwmPinVM, 0);
	}
}