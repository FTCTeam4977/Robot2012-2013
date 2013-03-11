#ifndef LordSwerve_h
#define LordSwerve_h

bool autonomousOptimized = false; // If enabled, some features that can distort position readings will be disabled

int rotateTheta(int angle, int by)
{
	angle += by;
	if ( angle > 360 )
		angle -= 360;
	if ( angle < 0 )
		angle += 360;
	return angle;
}

int rotateX(int x, int y, float theta)
{
  float cosA = cos(theta*(PI/180.0));
  float sinA = sin(theta*(PI/180.0));
  return (int)x*cosA-y*sinA;
}

int rotateY(int x, int y, float theta)
{
  float cosA = cos(theta*(PI/180.0));
  float sinA = sin(theta*(PI/180.0));
  return (int)x*sinA+y*cosA;
}

typedef enum
{
	LEFT=0,
	RIGHT=1
} SwerveSide;

typedef struct
{
	SwerveSide side;

	PID turnPID;
	int position;
	tMotor turnMotor;
	int positionOffset;
	int potOffset;

	tMotor driveMotor;
	tMotor driveMotor2;
	int driveSpeed;
	bool driveReversed;
	int driveEncoderZero;

} SwerveModuleSet;

SwerveModuleSet driveModules[2];

void setModulePositions(int target);
int getDrivingAngle();

int inverse(int value)
{
  if ( value > 1440 )
   	value -= 1440;
  else
  	value += 1440;
  return value;
}
int constrainEncoder(int input);

void initModuleSet(SwerveSide side, tMotor turnMotor, int potOffset, float P, float I, float D, tMotor driveMotor, tMotor driveMotor2)
{
	nxtDisplayString(0, "Module %i Init", side);
	driveModules[side].turnMotor = turnMotor;

	initPID(driveModules[side].turnPID, P, I, D);
	driveModules[side].turnPID.continous = true;
	driveModules[side].turnPID.maxInput = 2880;
	driveModules[side].driveReversed = false;
	driveModules[side].driveMotor = driveMotor;
	driveModules[side].driveMotor2 = driveMotor2;
	driveModules[side].potOffset = potOffset;
	driveModules[side].driveEncoderZero = 0;

	PID zeroPID;
	initPID(zeroPID, 0.7, 0.001);
	zeroPID.target = 512;
	bool zeroed = false;
	int loopsStable = 0;
	int timeout = nPgmTime+500;
	while ( !zeroed )
	{
		int rate = calcPID(zeroPID, (1024-(HTSPBreadADC(proto, (int)side, 10)+potOffset)));
		motor[turnMotor] = rate;
		motor[driveMotor] = -(rate/2);
		if ( abs(zeroPID.error) < 20 )
			loopsStable++;
		//debugPID(zeroPID);
		zeroed = (loopsStable>20)||(nPgmTime>timeout);
	}
	motor[turnMotor] = 0;
	motor[driveMotor] = 0;
	wait1Msec(100);
	driveModules[side].positionOffset = (1024-(HTSPBreadADC(proto, (int)side, 10)+potOffset))*2.8125;

	nMotorEncoder[driveMotor] = 0;
	nMotorEncoder[turnMotor] = 0;
	driveModules[side].position = constrainEncoder(nMotorEncoder[driveModules[side].turnMotor]+driveModules[side].positionOffset);
}

float normalPConstant = 0.2;
float turningPConstant = 0.69;


void initSwerve()
{
	nxtDisplayString(0, "Swerve Init");
	initModuleSet(LEFT, leftRotate, 0, 0.2,0,0, leftDrive, leftDrive2);
	initModuleSet(RIGHT, rightRotate, -19, 0.2,0,0, rightDrive, rightDrive2); // was 2 D
	setModulePositions(1440);
	eraseDisplay();
}

int constrainEncoder(int input)
{
	if ( input < 0  )
	{
		while ( input < -2880 )
			input += 2880;
		return 2880-abs(input);
	}
	else
	{
		while ( input > 2880 )
			input -= 2880;
	}
	return input;
}

bool modulesAtTarget(int maxError = 20)
{
	for ( int i = 0; i < 2; i++ )
	{
		if ( abs(driveModules[i].turnPID.error) > maxError )
			return false;
	}
	return true;
}

void updateSwerve(bool isTurning = false)
{
	static int tmpEncoderStore[2] = {0,0};
	int distances[2];

	for ( int i = 0; i < 2; i++ )
	{

		SwerveModuleSet &s = driveModules[i];

		if ( isTurning )
			s.turnPID.Kp = turningPConstant;
		else
			s.turnPID.Kp = normalPConstant;

		if ( false && abs(s.driveSpeed) < 2 && !autonomousOptimized )
		{
			motor[s.turnMotor]  = 0;
			motor[s.driveMotor] = 0;
			motor[s.driveMotor2] = 0;
		}
		else
		{
			s.position = constrainEncoder(nMotorEncoder[s.turnMotor]+s.positionOffset);

			int turnRate = calcPID(s.turnPID, s.position);
			if ( abs(turnRate) < 5 )
				turnRate = 0;
			motor[s.turnMotor] = turnRate;
			if ( abs(s.driveSpeed) < 20 || ( abs(driveModules[0].turnPID.error) > 600 || abs(driveModules[1].turnPID.error) > 600) )
			{
				motor[s.driveMotor] = (-turnRate)/2;
				motor[s.driveMotor2] = (-turnRate)/2;
			}
			else
			{
				motor[s.driveMotor]  = s.driveSpeed;
				motor[s.driveMotor2] = s.driveSpeed;
			}
		}
		if ( autonomousOptimized )
		{
			distances[i] = nMotorEncoder[s.driveMotor]-tmpEncoderStore[i];
			tmpEncoderStore[i] = nMotorEncoder[s.driveMotor];
		}
	}


}

int calcModuleError(int cur, int target)
{
	int error = target-cur;
	if ( abs(error) > 1440 )
  	{
	  	if ( error > 0 )
	  		error = error - 2880;
	  	else
	  		error = error + 2880;
  	}
  	return error;
}

bool wheelReversalIsShorter(SwerveSide number, int target)
{
	return abs(calcModuleError(driveModules[number].position, target)) > abs(calcModuleError(driveModules[number].position, inverse(target)));
}

void setModulePositions(int target)
{
	bool useInverse = (wheelReversalIsShorter(LEFT, target) || wheelReversalIsShorter(RIGHT, target)) && !autonomousOptimized;

	if ( useInverse )
		target = inverse(target);

	for ( int i = 0; i < 2; i++ )
	{
		driveModules[i].driveReversed = useInverse;
		driveModules[i].turnPID.target = target;
	}
}

void setModulePosition(SwerveSide side, int target, bool useShortest=true)
{
	bool useInverse = useShortest&&wheelReversalIsShorter(side, target);

	if ( useInverse )
		target = inverse(target);
	driveModules[side].driveReversed = useInverse;
	driveModules[side].turnPID.target = target;
}

void setModuleSpeed(SwerveSide side, int driveSpeed)
{
	if ( driveModules[side].driveReversed )
		driveSpeed = -driveSpeed;
	driveModules[side].driveSpeed = -driveSpeed;
}

int getDrivingAngle()
{
	int output;
	output = constrainEncoder(driveModules[0].position);
	output /= 8; // Convert to degrees
	return rotateTheta(output, -90);
}

long getDistance(SwerveSide side)
{
	long current = -(nMotorEncoder[driveModules[side].driveMotor]);

	if ( !autonomousOptimized && driveModules[side].driveReversed )
		current = -current;
	return current;
}

long getAvgDistance()
{
	return getDistance(RIGHT);
//	return (getDistance(LEFT)+getDistance(RIGHT))/2;
}

void resetDistance(SwerveSide side)
{
	nMotorEncoder[driveModules[side].driveMotor] = 0;
	nMotorEncoder[driveModules[side].driveMotor] = 0;
}

void resetDistance()
{
	resetDistance(LEFT);
	resetDistance(RIGHT);
}

#endif
