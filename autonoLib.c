
#define GYRO S3

#include "JoystickDriver.c"
#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-light.h"
#include "drivers/hitechnic-superpro.h"
#include "FTC_ValueUtils.h"
#include "FTC_Gyro.c"
#include "FTC_PID.c"
#include "LEDDriver.c"

#include "lineFollower.c"
#include "LordSwerve.c"
#include "lift.c"

#define stateWaitFor(c) if ( c ) state++

unsigned long timeout = 0;
void setTimeoutForNextStep(int newTimeout)
{
	timeout = nPgmTime+newTimeout;
}

bool isTimedOut()
{
	return timeout < nPgmTime;
}



void strafeStraight(int power, int angle = 0)
{
	long turnError = limitVar((getGyroAngle()-angle)*16, 720);
	if ( power > 0 )
	{
		setModulePosition(LEFT, (1440+720)+turnError);
		setModulePosition(RIGHT, (1440+720));
	}
	else if ( power < 0 )
	{
		setModulePosition(LEFT, (1440+720));
		setModulePosition(RIGHT, (1440+720)+turnError);
	}
	else
	{
		setModulePosition(LEFT, (1440+720));
		setModulePosition(RIGHT, (1440+720));
	}
}


bool strafeDriveDistance(long distance, long angle=0)
{
	static PID pid;
	static bool init = false;
	static long loopsStable = 0;
	if ( !init )
	{
		initPID(pid, 0.09);
		init = true;
	}
	pid.target = distance;

	long result = limitVar(calcPID(pid, getAvgDistance()), 100);
	nxtDisplayString(0, "%i", pid.error);

	long turnError = limitVar((getGyroAngle()-angle)*16, 720);
	if ( result > 0 )
	{
		setModulePosition(LEFT, (1440+720)+turnError);
		setModulePosition(RIGHT, (1440+720));
	}
	else if ( result < 0 )
	{
		setModulePosition(LEFT, (1440+720));
		setModulePosition(RIGHT, (1440+720)+turnError);
	}
	else
	{
		setModulePosition(LEFT, (1440+720));
		setModulePosition(RIGHT, (1440+720));
	}

	setModuleSpeed(LEFT, result);
	setModuleSpeed(RIGHT, result);
	if ( abs(pid.error) < 100 )
		loopsStable++;
	else
		loopsStable = 0;
	writeDebugStreamLine("Strafe out: %i Current: %d", result, getAvgDistance());
	return (loopsStable>10);
}

bool driveDistance(long distance, long angle=0)
{
	static PID pid;
	static bool init = false;
	static int loopsStable = 0;
	debugPID(pid);
	if ( !init )
	{
		initPID(pid, 0.1);
		init = true;
	}
	pid.target = distance;

	long result = limitVar(calcPID(pid, getAvgDistance()), 100);
	debugPID(pid);

	long turnError = limitVar((getGyroAngle()-angle)*17, 50);


	setModuleSpeed(LEFT, result-turnError);
	setModuleSpeed(RIGHT, result+turnError);
		writeDebugStreamLine("D: %i T: %i F: %i", result, turnError, result+turnError);
	if ( abs(pid.error) < 100 )
		loopsStable++;
	else
		loopsStable = 0;
	return (loopsStable>10);
}




bool lineFollow(long expectedDistance = 2000)
{
	switch ( getLineLocation() )
	{
		case MIDDLELINE:
			setModuleSpeed(LEFT, 55);
			setModuleSpeed(RIGHT, 55);
			break;
		case RIGHTLINE:
			setModuleSpeed(LEFT, 46);
			setModuleSpeed(RIGHT, -46);
			break;
		case LEFTLINE:
			setModuleSpeed(LEFT, -46);
			setModuleSpeed(RIGHT, 46);
			break;
	}
	return (getAvgDistance()>expectedDistance);
}

bool driveToCenter(bool blue = false)
{
	static int state = 0;
	static unsigned long flipAt = 0;
	static bool direction = false;
	writeDebugStreamLine("State: %i", state);
	switch ( state )
	{
		case 0:
			setModulePositions(1440);
			stateWaitFor(modulesAtTarget(50));
			break;
		case 1:
			stateWaitFor(driveDistance(7590));
			break;
		case 2:
			setLiftPosition(-4680);
			setModulePositions(1440+720);
			resetDistance();
			stateWaitFor(modulesAtTarget(50));
			setTimeoutForNextStep(2000);
			break;
		case 3:
			strafeStraight((blue?-90:90));
			setModuleSpeed(LEFT, (blue?-90:90));
			setModuleSpeed(RIGHT, (blue?-90:90));
			stateWaitFor(isTimedOut());
			break;
		case 4:
			resetDistance();
			state++;
			setModuleSpeed(LEFT, 30);
			setModuleSpeed(RIGHT, 30);
			setTimeoutForNextStep(6000);
			direction = true;
			flipAt = nPgmTime+2000;
			break;
		case 5:
			if ( direction )
			{
				strafeStraight(38);
				setModuleSpeed(LEFT, 38);
				setModuleSpeed(RIGHT, 38);
				if ( flipAt < nPgmTime )
				{
					direction = false;
					flipAt = nPgmTime+2000;
				}
			}
			else
			{
				strafeStraight(-38);
				setModuleSpeed(LEFT, -38);
				setModuleSpeed(RIGHT, -38);
				if ( flipAt < nPgmTime )
				{
					direction = true;
					flipAt = nPgmTime+1500;
				}
			}
			stateWaitFor((getLineLocation()!=-1)||isTimedOut());
		break;
		case 6:
			setModuleSpeed(LEFT, 0);
			setModuleSpeed(RIGHT, 0);
			resetDistance();
			state++;
			break;
		case 7:
			setModulePositions(1440);
			stateWaitFor(modulesAtTarget(50));
			resetDistance();
			setTimeoutForNextStep(8000);
		break;

		case 8:
				stateWaitFor(lineFollow(2440)||isTimedOut());
			break;
		case 9:
			setModuleSpeed(LEFT, 0);
			setModuleSpeed(RIGHT, 0);
			setLiftPosition(-1400);
			resetDistance();
		 	stateWaitFor(liftAtTarget());
		 	setTimeoutForNextStep(1000);
			break;
		case 10:
			setModuleSpeed(LEFT, -60);
			setModuleSpeed(RIGHT, -60);
			stateWaitFor(isTimedOut());
			break;
		default:
			return true;
	}
	return false;
}

bool driveToOtherSide(bool blue = false)
{
	static int state = 0;
	static bool direction = false;
	static long flipAt = 0;
	switch ( state )
	{
		case 0:
			setModulePositions(1440);
			stateWaitFor(modulesAtTarget(50));
			break;
		case 1:
			stateWaitFor(driveDistance(-9700));
			break;
		case 2:
				setModulePositions(1440+720);
				resetDistance();
				stateWaitFor(modulesAtTarget(50));
				setTimeoutForNextStep(800);
			break;
		case 3:
			setModuleSpeed(LEFT, -60);
			setModuleSpeed(RIGHT, -60);
			stateWaitFor(isTimedOut());
			break;
		case 4:
			setModulePositions(1440);
			resetDistance();
			stateWaitFor(modulesAtTarget(50));
			break;
		case 5:
			stateWaitFor(driveDistance(-1000));
			break;
		case 6:
			setLiftPosition(-4680);
			setModulePositions(1440+720);
			resetDistance();
			stateWaitFor(modulesAtTarget(50));
			setTimeoutForNextStep(2000);
			break;
		case 7:
			strafeStraight((blue?90:-90));
			setModuleSpeed(LEFT, (blue?90:-90));
			setModuleSpeed(RIGHT, (blue?90:-90));
			stateWaitFor(isTimedOut());
			break;
		case 8:
			resetDistance();
			state++;
			strafeStraight(-50);
			setModuleSpeed(LEFT, -50);
			setModuleSpeed(RIGHT, -50);
			setTimeoutForNextStep(8000);
			direction = true;
			flipAt = nPgmTime+5000;
			break;
		case 9:
/*			if ( direction )
			{
				strafeStraight(38);
				setModuleSpeed(LEFT, 38);
				setModuleSpeed(RIGHT, 38);
				if ( flipAt < nPgmTime )
				{
					direction = false;
					flipAt = nPgmTime+2000;
				}
			}
			else
			{
				strafeStraight(-38);
				setModuleSpeed(LEFT, -38);
				setModuleSpeed(RIGHT, -38);
				if ( flipAt < nPgmTime )
				{
					direction = true;
					flipAt = nPgmTime+2000;
				}
			}*/
			stateWaitFor((getLineLocation()!=-1)||isTimedOut());
		break;
		case 10:
			setModuleSpeed(LEFT, 0);
			setModuleSpeed(RIGHT, 0);
			resetDistance();
			state++;
			break;
		case 11:
			setModulePositions(1440);
			stateWaitFor(modulesAtTarget(50));
			resetDistance();
			setTimeoutForNextStep(8000);
		break;

		case 12:
				stateWaitFor(lineFollow(2440)||isTimedOut());
			break;
		case 13:
			setModuleSpeed(LEFT, 0);
			setModuleSpeed(RIGHT, 0);
			setLiftPosition(-1400);
			resetDistance();
		 	stateWaitFor(liftAtTarget());
		 	setTimeoutForNextStep(1000);
			break;
		case 14:
			setModuleSpeed(LEFT, -60);
			setModuleSpeed(RIGHT, -60);
			stateWaitFor(isTimedOut());
			break;
		default:
			return true;
	}
	return false;
}

bool driveToSide()
{
	static int state = 0;
	switch ( state )
	{
		case 0:
			setLiftPosition(-4680);
			stateWaitFor(driveDistance(11000));
			break;
		case 1:
			setLiftPosition(-2400);
			resetDistance();
			stateWaitFor(liftAtTarget());
			break;
		case 2:
			stateWaitFor(driveDistance(-5000));
			break;
		default:
			return true;
		}
		return false;
}



bool defence()
{
	static int state = 0;

	switch ( state )
	{
		case 0:
			setModulePositions(1440);
			stateWaitFor(modulesAtTarget(50));
			setTimeoutForNextStep(4000);
			break;
		case 1:
			setModuleSpeed(LEFT, 100);
			setModuleSpeed(RIGHT, 100);
			break;
		default:
			return true;
	}
	return isTimedOut();
}
