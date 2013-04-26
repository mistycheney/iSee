// Do not remove the include below
#include "MotorTest.h"

int countA = 0, countB = 0, countC = 0, countD = 0;

//PIN A
inline void funcA1() {
	PCintPort::pinState != digitalReadFast(PINA2) ? countA++ : countA--;
}
inline void funcA2() {
	PCintPort::pinState != digitalReadFast(PINA1) ? countA-- : countA++;
}
//PIN B
inline void funcB1() {
	PCintPort::pinState != digitalReadFast(PINB2) ? countB++ : countB--;
}
inline void funcB2() {
	PCintPort::pinState != digitalReadFast(PINB1) ? countB-- : countB++;
}
//PIN C
inline void funcC1() {
	PCintPort::pinState != digitalReadFast(PINC2) ? countC++ : countC--;
}
inline void funcC2() {
	PCintPort::pinState != digitalReadFast(PINC1) ? countC-- : countC++;
}
//PIN D
inline void funcD1() {
	PCintPort::pinState != digitalReadFast(PIND2) ? countD++ : countD--;
}
inline void funcD2() {
	PCintPort::pinState != digitalReadFast(PIND1) ? countD-- : countD++;
}

void setup() {
	pinMode(PINA1, INPUT);
	digitalWrite(PINA1, HIGH);
	PCintPort::attachInterrupt(PINA1, &funcA1, CHANGE);
	pinMode(PINA2, INPUT);
	digitalWrite(PINA2, HIGH);
	PCintPort::attachInterrupt(PINA2, &funcA2, CHANGE);
	pinMode(DIRA, OUTPUT);
	digitalWrite(DIRA, LOW);
	pinMode(PWMA, OUTPUT);
	digitalWrite(PWMA, LOW);

	pinMode(PINB1, INPUT);
	digitalWrite(PINB1, HIGH);
	PCintPort::attachInterrupt(PINB1, &funcB1, CHANGE);
	pinMode(PINB2, INPUT);
	digitalWrite(PINB2, HIGH);
	PCintPort::attachInterrupt(PINB2, &funcB2, CHANGE);
	pinMode(DIRB, OUTPUT);
	digitalWrite(DIRB, LOW);
	pinMode(PWMB, OUTPUT);
	digitalWrite(PWMB, LOW);

	pinMode(PINC1, INPUT);
	digitalWrite(PINC1, HIGH);
	PCintPort::attachInterrupt(PINC1, &funcC1, CHANGE);
	pinMode(PINC2, INPUT);
	digitalWrite(PINC2, HIGH);
	PCintPort::attachInterrupt(PINC2, &funcC2, CHANGE);
	pinMode(DIRC, OUTPUT);
	digitalWrite(DIRC, LOW);
	pinMode(PWMC, OUTPUT);
	digitalWrite(PWMC, LOW);

	pinMode(PIND1, INPUT);
	digitalWrite(PIND1, HIGH);
	PCintPort::attachInterrupt(PIND1, &funcD1, CHANGE);
	pinMode(PIND2, INPUT);
	digitalWrite(PIND2, HIGH);
	PCintPort::attachInterrupt(PIND2, &funcD2, CHANGE);
	pinMode(DIRD, OUTPUT);
	digitalWrite(DIRD, LOW);
	pinMode(PWMD, OUTPUT);
	digitalWrite(PWMD, LOW);

	Serial.begin(115200);
	Serial.println("Start");

}

void loop() {

	// write signal to motor pins
//	analogWrite(PWMA, 0);
//	digitalWrite(DIRA, 0);
//	analogWrite(PWMB, 0);
//	digitalWrite(DIRB, 0);
//	analogWrite(PWMC, 0);
//	digitalWrite(DIRC, 0);
	analogWrite(PWMD, 0);
//	digitalWrite(DIRD, 0);

	Serial.print(countA);
	Serial.print(" ");
	Serial.print(countB);
	Serial.print(" ");
	Serial.print(countC);
	Serial.print(" ");
	Serial.println(countD);

}
