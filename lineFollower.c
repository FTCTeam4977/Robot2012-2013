#define LEFTLINE 3
#define MIDDLELINE 2
#define RIGHTLINE 1

const tMUXSensor light1 = msensor_S4_1;
const tMUXSensor light2 = msensor_S4_2;
const tMUXSensor light3 = msensor_S4_3;

void lineFollowerInit()
{
	LSsetActive(light1);
	LSsetActive(light2);
	LSsetActive(light3);
}

int getLineLocation()
{
	if ( LSvalRaw(light2) > 1320 )
		return MIDDLELINE;
	else if ( LSvalRaw(light1) > 1320 )
		return RIGHTLINE;
	else if ( LSvalRaw(light3) > 1320 )
		return LEFTLINE;
	return -1;
}
