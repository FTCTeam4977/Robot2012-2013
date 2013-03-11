#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     proto,          sensorI2CCustom9V)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     HTSMUX,              sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     rightDrive,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     rightRotate,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     elevatorRight, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     rightDrive2,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     leftRotate,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     leftDrive,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     elevatorLeft,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     leftDrive2,    tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "autonoLib.c"
#include "drivers/hitechnic-irseeker-v2.h"
const tMUXSensor beaconSensor = msensor_S4_4;

task main()
{
	StopTask(readMsgFromPC);
	bDisplayDiagnostics = false;
	autonomousOptimized = true;
	writeValueToLEDs(0);
	initSwerve();
	motor[leftRotate] = 0;
	motor[rightRotate] = 0;
	wait1Msec(500);
	resetDistance();
	setModulePositions(1440);
	while ( !driveDistance(1000) ) updateSwerve();
	resetDistance();

	setModuleSpeed(LEFT, 0);
	setModuleSpeed(RIGHT, 0);
	updateSwerve();
	writeValueToLEDs(IDLE);
	StartTask(readMsgFromPC);
	lineFollowerInit();
	motor[leftDrive] = 0;
	motor[leftDrive2] = 0;
	motor[leftRotate] = 0;
	motor[rightDrive] = 0;
	motor[rightDrive2] = 0;
	motor[rightRotate] = 0;
	waitForStart();
	writeValueToLEDs(RED);
	initSwerve();
	initLift();

	setModulePositions(1440);
	resetDistance();

	initGyro();
	while ( !driveToOtherSide() )
	{
			updateSwerve();
			updateLift();
			updateGyro();
	}
}
