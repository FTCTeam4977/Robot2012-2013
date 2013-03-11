#define IDLE 1
#define RED 2
#define BLUE 3

void writeValueToLEDs(int value)
{
	static bool isFirstCall = true;
	if ( isFirstCall )
	{
		isFirstCall = false;
		HTSPBsetupIO(proto, 0xFF);
	}

	value = value << 2; // Shift by 2 to align with wiring
	HTSPBwriteIO(proto, value);
}
