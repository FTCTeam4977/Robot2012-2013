#define B2 2
#define B3 6

#define NOCOMM 0
#define IDLE 1
#define RED 2
#define BLUE 3
#define RAMPDOWN 4
#define AUTON 5

#define FRONT_WEIGHTED 8
#define BACK_WEIGHTED 9
#define BOTH_WEIGHTED 10


//#define SETPOINT_F 190 // RED
#define SETPOINT_F 308 // BLUE



float brightness = 0; // changed in fade loop
  
void setRGB(float r, float g, float b, float brightness = 1.0f)
{
  r = 40*(r/100);
  g = 40*(g/100);
  b = 40*(b/100);
  
  analogWrite(15, r*brightness);
  analogWrite(14, g*brightness);
  analogWrite(12, b*brightness);
}


void setup()
{
  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  
  pinMode(B1, INPUT); // B1
  pinMode(B2, INPUT); // B2
  pinMode(B3, INPUT); // B3
  Serial.begin(9600);
//  Serial.end();
}

boolean frontIsHeavy(int calibrationSet)
{
  if ( calibrationSet == IDLE )
    return false;
  static int reading = analogRead(2);
  reading = (reading*0.75)+(analogRead(2)*0.25);
  Serial.println(reading);
  return (reading < SETPOINT_F);
}


boolean backIsHeavy()
{
  static int reading = analogRead(0);
  reading = (reading*0.75)+(analogRead(0)*0.25);
  return !(reading > (SETPOINT_F==RED?400:440));
}

int getValueFromNXT()
{
  int output = 0;
  if ( digitalRead(B2) )
    output += 1;
  if ( digitalRead(B3) )
    output += 2;  
  return output;
} 

void loop()
{

  unsigned long brightnessTref = 0;
  int state = getValueFromNXT();
  if ( frontIsHeavy(state) )
    state = FRONT_WEIGHTED;
  switch (state)
  {
    case NOCOMM:
      setRGB(100, 30, 0);
      break;
    case IDLE:
      setRGB(0,50,0);
      break;
    case BLUE:
      setRGB(0,0,100);
      break;
    case RED:
      setRGB(100,0,0);
      break;
    case FRONT_WEIGHTED:
      setRGB(100,100,100, brightness);
      break;
  }

  if ( brightnessTref < millis() )
  {
    static float intBrightness = 0;
    static boolean flip = false;
    if ( intBrightness >= 100 )
      flip = true;
    else if ( intBrightness <= 0 )
      flip = false;
    intBrightness += (flip?-1:1);
    brightness = intBrightness/100.0f;
    brightnessTref = millis()+100;
  }
  
}
