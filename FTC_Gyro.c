#ifndef FTC_GYRO_h
#define FTC_GYRO_h

#include "FTC_ValueUtils.h"
#include "drivers/hitechnic-gyro.h"




typedef struct
{
	float degsec;
	float deg;
	int offset;
} gyroData_t;
gyroData_t gyroData;

void updateGyro()
{
	static float lastTime = nPgmTime;
		gyroData.degsec = HTGYROreadRot(GYRO);
		gyroData.deg += gyroData.degsec*(float)((nPgmTime-lastTime)/1000);
		lastTime = nPgmTime;
}

void initGyro()
{
	hogCPU();

	nxtDisplayBigTextLine(0,"Gyro");
	nxtDisplayString(2, "The gyro");
	nxtDisplayString(3,"is calibrating!");
	nxtDisplayString(5,"DO NOT MOVE THE");
	nxtDisplayString(6, " ROBOT.");

	wait1Msec(1000);

	gyroData.degsec = 0;
	gyroData.deg = 0;
	gyroData.offset = HTGYROstartCal(GYRO);

	eraseDisplay();
	releaseCPU();
}

int getGyroAngle()
{
	return (int)floor(gyroData.deg);
}

#endif
