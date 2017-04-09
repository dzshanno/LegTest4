#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// define servo names

#define s1RF  0
#define s2RF  1
#define s3RF  2
#define s1LF  4
#define s2LF  5
#define s3LF  6
#define s1RB  8
#define s2RB  9
#define s3RB  10
#define s1LB  12
#define s2LB  13
#define s3LB  14




///////////////////////////////////////////////////////////
double cal1RF = 0;
double cal2RF = 0;
double cal3RF = 0;

double cal1LF = 180;
double cal2LF = 155;
double cal3LF = 10;

double cal1LB = 180;
double cal2LB = 0;
double cal3LB = 180;

double cal1RB = -51;
double cal2RB = 160;
double cal3RB = 30;

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


double RFx, RFy, RFz;
double LFx, LFy, LFz;
double RBx, RBy, RBz;
double LBx, LBy, LBz;

double dist1 = 2.3; // length of hip
double femur = 8.0; // length of femur (hip to knee )
double tibia = 8.0; // length of bottom bone ( knee - foot)

double ava = 2;

////////////////////////////////////////////////////////////
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


void setup() {
	Serial.begin(9600);

	Serial.print("Setting things up");
	// set up pwm shield
	pwm.begin();
	pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	yield();
	//

	// set up starting positions

	RFx = 3;
	RFy = 3;
	RFz = 3;

	LFx = 5;
	LFy = 0;
	LFz = 5;

	RBx = 3;
	RBy = 3;
	RBz = 3;

	LBx = 3;
	LBy = 0;
	LBz = 3;

	Serial.print("OK - Lets go");

}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void loop() {

	Serial.print("Lets shake a leg");
	//MoveServo(4,90);
	movLegLF(5, 5, 5, 20, 30);
	delay(2000);
	movLegLF(-5, 5, 5, 20, 30);
	//MoveServo(4,120);
	delay(2000);

}
//  Various gait patterns
void walk2(double down, double up, int div, int time) {

	movLegs(9, up, 5.5, 9, down, 7, 9, down, 7, 9, up, 8.5, div, time);
	movLegs(9, down, 10, 9, down, 10, 9, down, 4, 9, down, 4, div, time);
	movLegs(9, down, 7, 9, up, 8.5, 9, up, 5.5, 9, down, 7, div, time);
	movLegs(9, down, 4, 9, down, 4, 9, down, 10, 9, down, 10, div, time);

}

void walk99(int div, int time) {

	movLegs(8, 7, RFz - ava, 8, 2.5, 13, 10, 3.5, LFz - ava, 8, 7, LBz + ava, div, time);
	movLegs(8, 7, RFz - ava, 8, 2.5, 6.5, 10, 3.8, LFz - ava, 8, 7, LBz + ava, div, time);
	movLegs(8, 7, RFz - ava, 8, 7, 0, 10, 4, LFz - ava, 8, 6, LBz + ava, div, time);

	movLegs(8, 2, 2, 8, 7, RBz + ava, 10, 5, LFz - ava, 8, 6, LBz + ava, div, time);
	movLegs(8, 2, 8.5, 8, 7, RBz + ava, 10, 5.5, LFz - ava, 8, 6, LBz + ava, div, time);
	movLegs(8, 6, 17, 8, 7, RBz + ava, 10, 5, LFz - ava, 8, 6, LBz + ava, div, time);

	movLegs(8, 6, RFz - ava, 8, 7, RBz + ava, 10, 5, LFz - ava, 8, 3, 17, div, time);
	movLegs(8, 6, RFz - ava, 8, 7, RBz + ava, 10, 5, LFz - ava, 8, 3, 6.5, div, time);
	movLegs(8, 6, RFz - ava, 8, 7, RBz + ava, 10, 5, LFz - ava, 8, 7, 0, div, time);

	movLegs(8, 6, RFz - ava, 8, 7, RBz + ava, 10, 1.5, 2, 8, 7, LBz + ava, div, time);
	movLegs(8, 6, RFz - ava, 8, 7, RBz + ava, 10, 1.5, 8.5, 8, 7, LBz + ava, div, time);
	movLegs(8, 6, RFz - ava, 8, 7, RBz + ava, 10, 4, 19, 8, 7, LBz + ava, div, time);

}



void sit(int div, int time) {
	movLegs(12, 1, 12, 12, 1, 12, 15, 0.5, 15, 15, 0.5, 15, div, time);
}

void wakeup(int div, int time) {
	movLegs(9, 6.5, 9, 9, 6.5, 9, 9, 6.5, 9, 9, 6.5, 9, div, time);
}

void walk(int div, int time) {
	moveAll(1.5, 0, 0, div, time);

	movLegRB(RBx, 2.5, RBz, div, time);
	movLegRB(11, 2.5, 3, div, time);
	movLegRB(RBx, 6.5 + 0.75, RBz, div, time);

	movLegRF(RFx, 1, RFz, div, time);
	movLegRF(11, 1, 11, div, time);
	movLegRF(RFx, 6.5, RFz, div, time);

	moveAll(-3, 0, 4, div, time);

	movLegLB(LBx, 2.5, LBz, div, time);
	movLegLB(11, 2.5, 3, div, time);
	movLegLB(LBx, 6.5, LBz, div, time);

	movLegLF(LFx, 2.5, LFz, div, time);
	movLegLF(11, 2.5, 11, div, time);
	movLegLF(LFx, 6.5 + 0.75, LFz, div, time);

	moveAll(1.5, 0, 4, div, time);

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Functions to move the legs

// Move all legs at the same time
void movLegs(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, double x4, double y4, double z4, int div, int time) {

	double subx1 = (x1 - RFx) / div;
	double suby1 = (y1 - RFy) / div;
	double subz1 = (z1 - RFz) / div;
	double subx2 = (x2 - RBx) / div;
	double suby2 = (y2 - RBy) / div;
	double subz2 = (z2 - RBz) / div;
	double subx3 = (x3 - LFx) / div;
	double suby3 = (y3 - LFy) / div;
	double subz3 = (z3 - LFz) / div;
	double subx4 = (x4 - LBx) / div;
	double suby4 = (y4 - LBy) / div;
	double subz4 = (z4 - LBz) / div;

	for (int i = 0; i < div; i++) {
		RFx += subx1;
		RFy += suby1;
		RFz += subz1;
		RBx += subx2;
		RBy += suby2;
		RBz += subz2;
		LFx += subx3;
		LFy += suby3;
		LFz += subz3;
		LBx += subx4;
		LBy += suby4;
		LBz += subz4;
		MoveServo(s1RF, cal1RF - ang1(RFx, RFy, RFz));
		MoveServo(s2RF, cal2RF - ang2(RFx, RFy, RFz));
		double sca31 = ang3(RFx, RFy, RFz) - cal3RF;
		if (sca31 > 4) {
			MoveServo(s3RF, sca31);
		}
		else {
			MoveServo(s3RF, 5);
		}
		MoveServo(s1LF, ang1(LFx, LFy, LFz) + cal1LF);
		MoveServo(s2LF, ang2(LFx, LFy, LFz));
		double sca33 = cal3LF - ang3(LFx, LFy, LFz);
		if (sca33 < 171) {
			MoveServo(s3LF, sca33);
		}
		else {
			MoveServo(s3LF, 170);
		}
		MoveServo(s1RB, ang1(RBx, RBy, RBz) + cal1RB);
		MoveServo(s2RB, ang2(RBx, RBy, RBz) - cal2RB);
		double sca32 = cal3RB - ang3(RBx, RBy, RBz);
		if (sca32 < 180) {
			MoveServo(s3RB, sca32);
		}
		else {
			MoveServo(s3RB, 180);
		}
		MoveServo(s1LB, cal1LB - ang1(LBx, LBy, LBz));
		MoveServo(s2LB, cal2LB - ang2(LBx, LBy, LBz));
		double sca34 = ang3(LBx, LBy, LBz) - cal3LB;
		if (sca34 > 10) {

			MoveServo(s3LB, sca34);
		}
		else {

			MoveServo(s3LB, 9);
		}
		delay(time);
	}
}


// Move only leg RF
void movLegRF(double x, double y, double z, int div, int time) {

	double subx = (x - RFx) / div;
	double suby = (y - RFy) / div;
	double subz = (z - RFz) / div;
	for (int i = 0; i < div; i++) {
		RFx += subx;
		RFy += suby;
		RFz += subz;
		MoveServo(s1RF, cal1RF - ang1(RFx, RFy, RFz));
		MoveServo(s2RF, cal2RF - ang2(RFx, RFy, RFz));

		double sca3 = ang3(RFx, RFy, RFz) - cal3RF;
		if (sca3 > 4) {
			MoveServo(s3RF, sca3);
		}
		else {
			MoveServo(s3RF, 5);
		}
		delay(time);
	}
}

// Move only leg LF
void movLegLF(double x, double y, double z, int div, int time) {

	double subx = (x - LFx) / div;
	double suby = (y - LFy) / div;
	double subz = (z - LFz) / div;
	for (int i = 0; i < div; i++) {
		LFx += subx;
		LFy += suby;
		LFz += subz;
		MoveServo(s1LF, cal1LF - ang1(LFx, LFy, LFz));
		MoveServo(s2LF, cal2LF - ang2(LFx, LFy, LFz));

		double sca3 = cal3LF + ang3(LFx, LFy, LFz);
		Serial.print(ang3(LFx, LFy, LFz));
		Serial.print(",");
		MoveServo(s3LF, sca3);
		delay(time);
	}
}

// Move only leg RB
void movLegRB(double x, double y, double z, int div, int time) {

	double subx = (x - RBx) / div;
	double suby = (y - RBy) / div;
	double subz = (z - RBz) / div;
	for (int i = 0; i < div; i++) {
		RBx += subx;
		RBy += suby;
		RBz += subz;
		MoveServo(s1RB, ang1(RBx, RBy, RBz) + cal1RB);
		MoveServo(s2RB, cal2RB - ang2(RBx, RBy, RBz));
		// updated for tzshanno
		double sca3 = ang3(RBx, RBy, RBz) - cal3RB;
		if (sca3 < 180) {
			MoveServo(s3RB, sca3);
		}
		else {
			MoveServo(s3RB, 180);
		}
		delay(time);
	}
}

void movLegLB(double x, double y, double z, int div, int time) {

	double subx = (x - LBx) / div;
	double suby = (y - LBy) / div;
	double subz = (z - LBz) / div;
	for (int i = 0; i < div; i++) {
		LBx += subx;
		LBy += suby;
		LBz += subz;
		MoveServo(s1LB, cal1LB - ang1(LBx, LBy, LBz));
		MoveServo(s2LB, ang2(LBx, LBy, LBz) + cal2LB);

		double sca3 = cal3LB - ang3(LBx, LBy, LBz);
		Serial.print(ang3(LFx, LFy, LFz));
		Serial.print(",");
		if (sca3 > 10) {
			MoveServo(s3LB, sca3);
		}
		else {
			MoveServo(s3LB, 9);
		}
		delay(time);
	}
}

// Move only the body
void moveAll(double x, double y, double z, int div, int time) {

	double subx = x / div;
	double suby = y / div;
	double subz = z / div;

	for (int i = 0; i < div; i++) {
		RFx += subx;
		RFy += suby;
		RFz -= subz;

		LFx -= subx;
		LFy += suby;
		LFz -= subz;

		RBx += subx;
		RBy += suby;
		RBz += subz;

		LBx -= subx;
		LBy += suby;
		LBz += subz;

		MoveServo(s1RF, cal1RF - ang1(RFx, RFy, RFz));
		MoveServo(s2RF, cal2RF - ang2(RFx, RFy, RFz));
		MoveServo(s3RF, ang3(RFx, RFy, RFz) - cal3RF);

		MoveServo(s1LF, ang1(LFx, LFy, LFz) + cal1LF);
		MoveServo(s2LF, ang2(LFx, LFy, LFz));
		MoveServo(s3LF, cal3LF - ang3(LFx, LFy, LFz));

		MoveServo(s1RB, ang1(RBx, RBy, RBz) + cal1RB);
		MoveServo(s2RB, ang2(RBx, RBy, RBz) - cal2RB);
		MoveServo(s3RB, cal3RB - ang3(RBx, RBy, RBz));

		MoveServo(s1LB, cal1LB - ang1(LBx, LBy, LBz));
		MoveServo(s2LB, cal2LB - ang2(LBx, LBy, LBz));
		MoveServo(s3LB, ang3(LBx, LBy, LBz) - cal3LB);
		delay(time);
	}
}


// Inverse kinesmatics calculations
// Angle for servo 1
int ang1(double x, double y, double z) {
	double a1 = atan2(z, x);
	return (a1 * 57.3);
}

// Angle for servo 2
int ang2(double x, double y, double z) {
	double hip = sqrt(pow(y, 2) + pow((x - dist1), 2));
	double al1 = acos(y / hip);
	double al2 = acos((pow(tibia, 2) - pow(femur, 2) - pow(hip, 2)) / (-2 * femur * hip));
	double a2 = al1 + al2;
	return (a2 * 57.3);
}

// Angle for servo 3
int ang3(double x, double y, double z) {
	double hip = sqrt(pow(y, 2) + pow((x - dist1), 2));
	double a3 = acos((pow(hip, 2) - pow(tibia, 2) - pow(femur, 2)) / (-2 * tibia * femur));
	return (a3 * 57.3);
}

void MoveServo(byte ServoNum, byte Angle) {
	// Move a particular servo to a particular angle

	// Angle moved is determined by the pulselength sent to the Servo Shield
	//Determine correct Pulselength
	double Pulselength = map(Angle, 0, 180, SERVOMIN, SERVOMAX);
	pwm.setPWM(ServoNum, 0, Pulselength);


}