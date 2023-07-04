#include <movingAvg.h>

typedef struct Pid_S{
  float oldState; 				/**<old state*/
  float iState;   				/**<integrator state*/
  float iMin;     				/**<minimum integrator state*/
  float iMax;    				/**<maximum integrator state*/
  float pGain;    				/**<proportional gain*/
  float iGain;    				/**<integral gain*/
  float dGain;    				/**<derivative gain*/
  int used;     			/**<in-use state*/
} Pid_t;

Pid_t	Pid[3];

const int PID_ID_SHOULDER = 0;
const int PID_ID_ELBOW    = 1;
const int PID_ID_ANKLE    = 2;

const int ANKLE_PIN = 15; // The ankle gpio pin
const int ELBOW_PIN = 0; // The elbow gpio pin
const int SHOULDER_PIN = 33; // The shoulder gpio pin

const int SWITCH_ON_OFF = 16;

const int SWITCH_STANDING_WALKING = 19;

const int ENABLE_A = 17;
const int ENABLE_B = 18;

const int A_IN_SHOULDER = 26;
const int A_IN_ELBOW = 25;
const int A_IN_ANKLE = 32;

const int POW_LED = 22;

const int PWM_OUT_SHOULDER = 2;
const int PWM_OUT_ELBOW = 1;
const int PWM_OUT_ANKLE = 0;

//Cilinders Max in
//Schouder: 86, Elleboog: 198gr, Enkel: 199gr
//Cilinders Max uit
//Schouder: 31, Elleboog: 120 , Enkel:  50
//Cilinders Base
//Schouder: 80, Elleboog: 192 , Enkel:  78

//Schouder cilinder: 639+415 = 1054  hoek:  -12gr    :0: +35gr   totaal: 47gr       gemeten: 54gr  correctie: 285/360 * 54 =  42,75
//Elleboog cilinder: 715+454 = 1169 hoek:   +20gr    :0: +84,5gr totaal: 64,5gr     gemeten: 78gr  correctie: 285/360 * 78 =  61,75
//Enkel cilinder: 820+600 = 1420 hoek:      -106,7gr :0: +4,2gr  totaal: 110,9gr    gemeten: 149gr correctie: 285/360 * 149 = 118
//Potmeter 285gr

const int SHOULDER_MAX_IN = 86;
const int SHOULDER_MAX_OUT = 31;
const int SHOULDER_BASE = 80;

const int ELBOW_MAX_IN = 198;
const int ELBOW_MAX_OUT = 120;
const int ELBOW_BASE = 192;

const int ANKLE_MAX_IN = 199;
const int ANKLE_MAX_OUT = 50;
const int ANKLE_BASE = 78;

int shoulder = 0; // Needs to be shoulder sensor input
int elbow = 0; // Needs to be elbow sensor input
int ankle = 0; // Needs to be ankle sensor input

signed int errorShoulder = 0;
signed int errorElbow = 0;
signed int errorAnkle = 0;

float outputShoulder = 0;
float outputElbow = 0;
float outputAnkle = 0;


float shoulderTarget = SHOULDER_BASE; // The target rotation of the shoulder, was 52
float elbowTarget = ELBOW_BASE; // The target rotation of the elbow, was 52
float ankleTarget = ANKLE_BASE; // The target rotation of the ankle, was 170

float elbowSpeedMultiplier = 1; // Speed multiplier of the elbow
float ankleSpeedMultiplier = 2; // Speed multiplier of the ankle

float speedMultiplier = 1; // Global speed multiplier
bool footOnGround = false; // Is the foot on the ground yes/no (If on the ground, calculate flat position for ankle)

int frequency = 1000; // The pwm frequency

int resolution = 16; // The pwm resolution

int baseValue = 5000; // The base amount the valve opens
int cilinderBaseSpeed = 0; //Maximum cilinderBaseSpeed is speed shoulder
int cilinderElbowSpeed = 0; //speed elbow cilinder
int cilinderAnkleSpeed = 0;  //speed ankle ciliner
int counter = 0;
int state = 0 ;

int shoulderPosition1;
int shoulderPosition2;
int shoulderPosition3;
int shoulderPosition4;
int shoulderPosition5;

int elbowPosition1;
int elbowPosition2;
int elbowPosition3;

int anklePosition1;
int anklePosition2;
int anklePosition3;

bool onOffSwitch;

bool running = false;

int AVERAGE_SIZE = 50;
movingAvg averageFilterShoulder(AVERAGE_SIZE);
movingAvg averageFilterElbow(AVERAGE_SIZE);
movingAvg averageFilterAnkle(AVERAGE_SIZE);

// Setup the pwm outputs for valves
void setup() {
  Serial.begin(57600);

  averageFilterShoulder.begin();
  averageFilterElbow.begin();
  averageFilterAnkle.begin();

  pinMode(SWITCH_ON_OFF, INPUT_PULLUP);
  pinMode(SWITCH_STANDING_WALKING, INPUT_PULLUP);

  ledcSetup(PWM_OUT_SHOULDER, frequency, resolution);
  ledcAttachPin(SHOULDER_PIN, 2);

  ledcSetup(PWM_OUT_ELBOW, frequency, resolution);
  ledcAttachPin(ELBOW_PIN, 1);

  ledcSetup(PWM_OUT_ANKLE, frequency, resolution);
  ledcAttachPin(ANKLE_PIN, 0);


  pinMode(ENABLE_A, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);
  pinMode(POW_LED, OUTPUT);
  pinMode(A_IN_SHOULDER, INPUT);
  pinMode(A_IN_ELBOW, INPUT);
  pinMode(A_IN_SHOULDER, INPUT);

  //Set PID values
  PidSetGain(PID_ID_SHOULDER, 200 * speedMultiplier,0,0);
  PidSetGain(PID_ID_ELBOW, 200 * speedMultiplier * elbowSpeedMultiplier,0,0);
  PidSetGain(PID_ID_ANKLE, 200 * speedMultiplier * ankleSpeedMultiplier,0,0);

  //Set Base values, zodat poot op grond staat
  shoulderTarget = SHOULDER_BASE;
  elbowTarget = ELBOW_BASE;
  ankleTarget = ANKLE_BASE; 

  delay(1000);
}


void loop() {
  float vA = analogRead(A_IN_ANKLE);
  float vE = analogRead(A_IN_ELBOW);
  float vS = analogRead(A_IN_SHOULDER);
  
  ankle = float(360)/float(4095)*vA;  //12bits   -> moet eigenlijk 285 graden zijn, meer kan potmeter niet
  elbow = float(360)/float(4095)*vE;  //12bits 
  shoulder = float(360)/float(4095)*vS; //12bits

  averageFilterShoulder.reading(shoulder);
  shoulder = averageFilterShoulder.getAvg();
  averageFilterElbow.reading(elbow);
  elbow = averageFilterElbow.getAvg();
  averageFilterAnkle.reading(ankle);
  ankle = averageFilterAnkle.getAvg();

  onOffSwitch = digitalRead(SWITCH_ON_OFF);

  running = digitalRead(SWITCH_STANDING_WALKING);
   //actual = Scale(ActCurrent, IMin, IMax, 0, 100); 		//ActCurrent -> Actual measured current
   //error = (Setpoint * Max / 100) - (actual); 			//Calculate error in % related to Setpoint
   //output = PidUpdate(ucPIDid, error, actual); 			//PID control output in %

  //Ieder 10k ticks send debug info
  if (counter == 100)
    {
      DebugInfo();
      counter = 0; 
    }
  counter++;

  digitalWrite(ENABLE_A, !onOffSwitch);  //als switch off dan gaat L298 uit en stuurt ventiel naar uit (outout 1 & output 2)
  digitalWrite(ENABLE_B, !onOffSwitch);  //als switch off dan gaat L298 uit en stuurt ventiel naar uit (output 3 & output 4)
  digitalWrite(POW_LED, LOW);  //zet blauwe led aan

  //Begrezing maximale uitsturing cilinder
  cilinderBaseSpeed  = int(LimitCalc(float(baseValue * speedMultiplier), 16384)); 
  cilinderElbowSpeed = int(LimitCalc(float(cilinderBaseSpeed * elbowSpeedMultiplier), 16384));
  cilinderAnkleSpeed = int(LimitCalc(float(cilinderBaseSpeed * ankleSpeedMultiplier), 16384));

  //Berekening afwijking tov setpoint
  errorShoulder = shoulderTarget - shoulder;
  errorElbow    = elbowTarget - elbow;
  errorAnkle    = ankleTarget - ankle;

  //Excecute PID regeling
  outputShoulder = PidUpdate(PID_ID_SHOULDER, errorShoulder, shoulder );
  outputShoulder = -LimitCalc(outputShoulder, 16384);
  outputElbow = PidUpdate(PID_ID_ELBOW, errorElbow, elbow );
  outputElbow = -LimitCalc(outputElbow, 16384);
  outputAnkle = PidUpdate(PID_ID_ANKLE, errorAnkle, ankle );
  outputAnkle = -LimitCalc(outputAnkle, 16384);

  //Schrijf PWM uitsturing naar outputs
  ledcWrite(PWM_OUT_SHOULDER, 32767 + outputShoulder);
  ledcWrite(PWM_OUT_ELBOW   , 32767 + outputElbow);
  ledcWrite(PWM_OUT_ANKLE   , 32767 + outputAnkle);

  // Base position: Trillen kwam doordat de klep niet naar neutraal gaat wanneer de positie bereikt is. Maar terug de andere kant op
  if (!running) {
    shoulderTarget = SHOULDER_BASE;
    elbowTarget = ELBOW_BASE;
    ankleTarget = ANKLE_BASE;
  } else {
    // Set points
    shoulderPosition1 = SHOULDER_MAX_IN - 5; //was 95  -> nu 81
    shoulderPosition2 = SHOULDER_MAX_OUT + 10; //was 85  -> nu 41
    shoulderPosition3 = shoulderPosition2 + 20; // was 70 -> nu 61
    shoulderPosition4 = shoulderPosition3 + 10;// was 90  -> nu 71
    shoulderPosition5 = shoulderPosition4 - 30;// 65 -> nu 41 

    elbowPosition1 = ELBOW_MAX_IN - 5;  //was 95  -> nu 193 
    elbowPosition2 = ELBOW_MAX_OUT + 10;  // was 145 -> nu 130 
    elbowPosition3 = elbowPosition2 + 5; // was 150 -> nu 135

    anklePosition1 = ANKLE_MAX_IN - 5;  //was 90  -> nu 194
    anklePosition2 = ANKLE_MAX_OUT + 20; //was 135  -> nu 70
    anklePosition3 = anklePosition2 + 15 ; //was 160  -> nu 85

    // Shoulder swings backwards and forwards in a swinging motion. Depending on this motion the other parts move.
    if (compareValues(shoulderTarget, SHOULDER_BASE) && compareValues(shoulder, SHOULDER_BASE)) {
      shoulderTarget = shoulderPosition1;  //was 95 nu max uitslag - 5 graden
      elbowTarget = elbowPosition1; //was 95 nu max uitslag - 5 graden
      ankleTarget = anklePosition1; //was 90 nu max uitslag -5 graden
      ankleSpeedMultiplier = 4;
      elbowSpeedMultiplier = 3;
      state = 1;
    }

        // Shoulder swings backwards and forwards in a swinging motion. Depending on this motion the other parts move.
    if (compareValues(shoulderTarget, shoulderPosition2) && compareValues((int)shoulder, shoulderPosition2)) {
      shoulderTarget = shoulderPosition1;  //was 95 nu max uitslag - 5 graden
      elbowTarget = elbowPosition1; //was 95 nu max uitslag - 5 graden
      ankleTarget = anklePosition1; //was 90 nu max uitslag -5 graden
      ankleSpeedMultiplier = 4;
      elbowSpeedMultiplier = 3;
      state = 1;
    }

    if (compareValues(shoulderTarget, shoulderPosition1) && compareValues((int)shoulder, shoulderPosition1)) {
      shoulderTarget = shoulderPosition2;
      elbowTarget = elbowPosition2;
      elbowSpeedMultiplier = 1;
      footOnGround = true;
      state = 2;
    }

    if (compareValues(shoulderTarget, shoulderPosition1) && compareValues((int)shoulder, shoulderPosition3)) {
      elbowTarget = elbowPosition3;
      ankleTarget = anklePosition2;
      elbowSpeedMultiplier = 2;
      ankleSpeedMultiplier = 2;
      state = 3;
    }

    if (compareValues(shoulderTarget, SHOULDER_BASE) && compareValues((int)shoulder, shoulderPosition4)) {
      elbowSpeedMultiplier = 1;
      elbowTarget = elbowPosition3;
      footOnGround = true;
      state = 4;
    }

    if (compareValues(shoulderTarget, SHOULDER_BASE) && compareValues((int)shoulder, shoulderPosition5)) {
      footOnGround = false;
      ankleSpeedMultiplier = 1.5;
      ankleTarget = anklePosition3;
      state  = 5;
    }

    if (footOnGround) {
      ankleTarget = 90 - shoulder + elbow;
    }
    
  }

  // Delay om te zorgen dat de analogRead functie werkt
  delay(10);
}

bool compareValues(int one, int two) {
  return one <= two + 4 && one >= two -4;
}

//Added debug info
/*********************/
void DebugInfo()
/********************/
{
      Serial.print(", Schouder M: ");
      Serial.print(shoulder);
      Serial.print(" S: ");
      Serial.print(shoulderTarget);
      Serial.print(" Err: ");
      Serial.print(errorShoulder);
      Serial.print(" PWM: ");
      Serial.print(outputShoulder);

      Serial.print(", Elleboog M: ");
      Serial.print(elbow);
      Serial.print(" S: ");
      Serial.print(elbowTarget);
      Serial.print(" Err: ");
      Serial.print(errorElbow);
      Serial.print(" PWM: ");
      Serial.print(outputElbow);

      Serial.print(", Enkel M: ");
      Serial.print(ankle);
      Serial.print(" S: ");
      Serial.print(ankleTarget);
      Serial.print(" Err: ");
      Serial.print(errorAnkle);
      Serial.print(" PWM: ");
      Serial.print(outputAnkle);

      Serial.print(", Input: ");
      Serial.print(onOffSwitch);
      Serial.print(", State: ");
      Serial.println(state);
}

int OFFSET = 6500;

//Added function
/****************************************/
float LimitCalc(float Value, int Limit)
/***************************************/
//Calculate limit
{
  if (Value >= 0 )
  {
    Value += OFFSET;
    if (Value > Limit) 
    {
      Value = Limit;
    }
  }
  else 
  {
    Value -= OFFSET;
    if (Value < -Limit) 
    {
      Value = -Limit;
    }    
  }
  return Value;
}

//Added library functions
/****************************************************************************/
void PidSetGain (int id, float pGain, float iGain, float dGain)
/****************************************************************************/
{
  Pid[id].pGain = pGain;
  Pid[id].iGain = iGain;
  Pid[id].dGain = dGain;
}

/****************************************************************************/
void PidSetIntegratorLimits (int id, float iMin, float iMax)
/****************************************************************************/
{
  Pid[id].iMin = iMin;
  Pid[id].iMax = iMax;
}

/****************************************************************************/
float PidUpdate (int id, float error, float position)
/****************************************************************************/
{
  float pTerm, dTerm, iTerm;
  static int teller = 0;
  
  // Calculate the proportional term
  pTerm = Pid[id].pGain * error;

  // Calculate the integral state with appropriate limiting
  //printf("\t id%d g%f  s%f  mx%f mn%f", id, Pid[id].iGain, Pid[id].iState, Pid[id].iMax, Pid[id].iMin);
  if(Pid[id].iGain) {
		Pid[id].iState += error;
		if (Pid[id].iState > Pid[id].iMax)
		{
			Pid[id].iState = Pid[id].iMax;
		}
		else if (Pid[id].iState < Pid[id].iMin)
		{
			Pid[id].iState = Pid[id].iMin;
		}
  } else {
  	Pid[id].iState = 0;
  }
  iTerm = Pid[id].iState * Pid[id].iGain;
  
  // Calculate the derivative state
  dTerm = Pid[id].dGain * (position - Pid[id].oldState);
  
  // Save old state
  Pid[id].oldState = position;

  return pTerm + iTerm - dTerm;
}  

//Scales any input value between UiMin and UiMax to an output value between UoMin and UoMax
//Also works with inverted input or output range
//Input value is automatically limited between UiMin and UiMax
signed long Scale(signed long Uin, signed long UiMin, signed long UiMax, signed long UoMin, signed long UoMax)
/**************************************************************************************************************************/
{
	signed long long Ui0 = 0;
	signed long Uis = 0;
	signed long Uos = 0;
	signed long Out = 0;

	if (UiMax >= UiMin) { //Normal input range
		if (Uin < UiMin)
			Uin = UiMin; //Limit input to min
		//if (Uin > UiMax) Uin = UiMax; //Limit input to max
	} else { //Inverted input range
		if (Uin > UiMin)
			Uin = UiMin; //Limit input to min
		if (Uin < UiMax)
			Uin = UiMax; //Limit input to max
	}

	Ui0 = Uin - UiMin; //Calculate input zero
	Uis = UiMax - UiMin; //Calculate input span
	Uos = UoMax - UoMin; //Calculate output span

	Out = (UoMin + (Ui0 * Uos) / Uis);

	//uart_printf(TRACE_PORT,40,40, "Scale: %5d", Out);

	return Out;
}