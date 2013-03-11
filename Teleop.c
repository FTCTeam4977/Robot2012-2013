#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     proto,          sensorI2CCustom9V)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     rightDrive,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     rightRotate,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     elevatorRight, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     rightDrive2,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     leftRotate,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     leftDrive,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     elevatorLeft,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     leftDrive2,    tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define GYRO S3
//#define LIFTOI_DEBUG

#include "JoystickDriver.c"
#include "drivers/hitechnic-superpro.h"
#include "FTC_ValueUtils.h"
#include "FTC_Gyro.c"
#include "FTC_PID.c"
#include "LEDDriver.c"

#include "LordSwerve.c"

int liftPosition[] = {-400,-4500,-7400};
int driveMode = 2;
#include "lift.c"


bool isTurning = false;
/* Start of
 * liftOI
 */

void liftOI(){
	if ( joy2Btn(4) )
		setLiftPosition(liftPosition[2]);
	else if ( joy2Btn(3) )
		setLiftPosition(liftPosition[1]);
	else if ( joy2Btn(2) )
		setLiftPosition(liftPosition[0]);
	int liftRate;
	if ( dbc(joystick.joy2_y1, 15) ) // Large
	{
		if ( joystick.joy2_y1 < 0 && nMotorEncoder[elevatorLeft] >= -400 )
			liftRate = 0;
		else
		{
			liftRate = -joystick.joy2_y1;
			lift.target = nMotorEncoder[elevatorLeft];
		}
		motor[elevatorRight] = -liftRate;
		motor[elevatorLeft] = liftRate;
	}
	else if ( dbc(joystick.joy2_y2, 15) ) // Fine
	{
		if ( joystick.joy2_y2 < 0 && nMotorEncoder[elevatorLeft] >= -400 )
			liftRate = 0;
		else
		{
			liftRate = -(int)floor((float)joystick.joy2_y2*0.50);
			lift.target = nMotorEncoder[elevatorLeft];
		}
		motor[elevatorRight] = -liftRate;
		motor[elevatorLeft] = liftRate;
	}
	else
	{
		motor[elevatorRight] = 0;
		motor[elevatorLeft] = 0;
	}
}
bool swerveInverted = false;
void swerveOI()
{

	if (	joy1Btn(8) )
		swerveInverted = true;
	else if (	joy1Btn(8) )
		swerveInverted = false;

	if ( joy1Btn(1) ) //tank
	{
		driveMode = 1;
	}
	else if ( joy1Btn(2) ) // crab
	{
		driveMode = 2;
	}


	if ( driveMode == 1 ) // Tank mode
	{
		setModulePositions(1440);
		setModuleSpeed(LEFT, joystick.joy1_y1);
		setModuleSpeed(RIGHT, joystick.joy1_y2);
	}
	else // Crab + Rotation
	{
		if ( abs(joystick.joy1_x2) > 10 )
		{
			setModulePositions(1440);
			int rate = joystick.joy1_x2;
			setModuleSpeed(LEFT, rate);
			setModuleSpeed(RIGHT, -rate);
			isTurning = true;
		}
		else
		{
			isTurning = false;
			int magnitude = sqrt(pow(joystick.joy1_y1,2)+pow(joystick.joy1_x1,2));
			int coordRotateAngle = 90+(swerveInverted?180:0);
			int x = rotateX(joystick.joy1_x1, joystick.joy1_y1, coordRotateAngle);
			int y = rotateY(joystick.joy1_x1, joystick.joy1_y1, coordRotateAngle);
			int angle = radiansToDegrees(atan2(y,x));
			if ( magnitude > 20 )
				setModulePositions(angle*8);
			setModuleSpeed(LEFT, magnitude);
			setModuleSpeed(RIGHT, magnitude);
		}
	}
}

/* End of
 * swerveOI
 */

bool disabled = true;
task FMSwatcher()
{
	int last = 0;
	while ( true )
	{
		int delta = ntotalMessageCount-last;
		disabled = (delta == 0);
		wait1Msec(500);
	}
}

task main()
{
	writeValueToLEDs(0);
	bDisplayDiagnostics = false;
	initGyro();
	writeValueToLEDs(IDLE);
	waitForStart();
	writeValueToLEDs(RED);
	StartTask(FMSwatcher);
	initSwerve();
	updateSwerve();
	initLift();
	setLiftPosition(-400);
	setModulePositions(1440);
	while ( true )
	{
		getJoystickSettings(joystick);

		if ( joy1Btn(1) && joy1Btn(2) )
			initSwerve();

		swerveOI();
		liftOI();
		debugPID(driveModules[LEFT].turnPID);
		updateGyro();
		if ( !disabled )
			updateSwerve(isTurning);
		else
		{
			motor[leftDrive] = 0;
			motor[leftDrive2] = 0;
			motor[leftRotate] = 0;
			motor[rightDrive] = 0;
			motor[rightDrive2] = 0;
			motor[rightRotate] = 0;
		}
	}
}
