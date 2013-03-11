#ifndef Lift_h
#define Lift_h

PID lift;

void initLift()
{
	initPID(lift);
	lift.target = nMotorEncoder[elevatorLeft];
}

void updateLift()
{
	if ( lift.target < lift.current ) // going up
		lift.Kp = 0.6;
	else if ( lift.target > lift.current )
		lift.Kp = 0.3;
	int liftRate = calcPID(lift, nMotorEncoder[elevatorLeft]);
	if ( abs(liftRate) < 50 )
		liftRate = 0;
	motor[elevatorRight] = -liftRate;
	motor[elevatorLeft] = liftRate;
}

bool liftAtTarget(int thresh = 100)
{
	return abs(lift.error) < thresh;
}

void setLiftPosition(int position)
{
	lift.target = position;
}
#endif
