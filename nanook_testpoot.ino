float shoulder = 0; // Needs to be shoulder sensor input
float elbow = 0; // Needs to be elbow sensor input
float ankle = 0; // Needs to be ankle sensor input

float shoulderTarget = 52; // The target rotation of the shoulder
float elbowTarget = 52; // The target rotation of the elbow
float ankleTarget = 170; // The target rotation of the ankle
float elbowSpeedMultiplier = 1; // Speed multiplier of the elbow
float ankleSpeedMultiplier = 2; // Speed multiplier of the ankle

float speedMultiplier = 1; // Global speed multiplier
bool footOnGround = false; // Is the foot on the ground yes/no (If on the ground, calculate flat position for ankle)

int anklePin = 24; // The ankle gpio pin
int elbowPin = 25; // The elbow gpio pin
int shoulderPin = 27; // The shoulder gpio pin

int frequency = 1000; // The pwm frequency

int resolution = 16; // The pwm resolution

int baseValue = 5000; // The base amount the valve opens

// Setup the pwm outputs for valves
void setup() {
  ledcSetup(0, frequency, resolution);
  ledcAttachPin(anklePin, 0);
  ledcSetup(1, frequency, resolution);
  ledcAttachPin(elbowPin, 1);
  ledcSetup(2, frequency, resolution);
  ledcAttachPin(shoulderPin, 2);
}

// DutyCycle 64 max one way
// DutyCycle 191 max other way
// DutyCycle 127 halfway

void loop() {

  // Handle shoulder movement. Move towards target rotation
  if (shoulder > shoulderTarget) {
    // Open valve to retract cilinder by a set amount * speedMultiplier
    ledcWrite(2, 32767 - (baseValue * speedMultiplier));
  } else if (shoulder < shoulderTarget) {
    // Open valve to extend cilinder by a set amount * speedMultiplier
    ledcWrite(2, 32767 + (baseValue * speedMultiplier));
  }

  // Handle elbow movement. Move towards target rotation
  if (elbow > elbowTarget) {
    // Open valve to retract cilinder by a set amount * speedMultiplier * elbowSpeedMultiplier
    ledcWrite(1, 32767 - (baseValue * speedMultiplier * elbowSpeedMultiplier));
  } else if (elbow < elbowTarget) {
    // Open valve to extend cilinder by a set amount * speedMultiplier * elbowSpeedMultiplier
    ledcWrite(1, 32767 + (baseValue * speedMultiplier * elbowSpeedMultiplier));
  }

  // Handle ankle movement. Move towards target rotation
  if (ankle > ankleTarget) {
    // Open valve to retract cilinder by a set amount * speedMultiplier * ankleSpeedMultiplier
    ledcWrite(0, 32767 - (baseValue * speedMultiplier * ankleSpeedMultiplier));
  } else if (ankle < ankleTarget) {
    // Open valve to extend cilinder by a set amount * speedMultiplier * ankleSpeedMultiplier
    ledcWrite(0, 32767 + (baseValue * speedMultiplier * ankleSpeedMultiplier));
  }

  // Shoulder swings backwards and forwards in a swinging motion. Depending on this motion the other parts move.
  if (shoulderTarget == 52 && (int)shoulder == 52) {
    shoulderTarget = 95;
    ankleTarget = 90;
    ankleSpeedMultiplier = 4;
    elbowSpeedMultiplier = 3;
    elbowTarget = 95;
  }

  if (shoulderTarget == 95 && (int)shoulder == 95) {
    shoulderTarget = 52;
    elbowSpeedMultiplier = 1;
    elbowTarget = 145;
    footOnGround = true;
  }

  if (shoulderTarget == 95 && (int)shoulder == 70) {
    elbowTarget = 150;
    elbowSpeedMultiplier = 2;
    ankleSpeedMultiplier = 2;
    ankleTarget = 135;
  }
  if (shoulderTarget == 52 && (int)shoulder == 90) {
    elbowSpeedMultiplier = 1;
    elbowTarget = 150;
    footOnGround = true;
  }
  if (shoulderTarget == 52 && (int)shoulder == 65) {
    footOnGround = false;
    ankleSpeedMultiplier = 1.5;
    ankleTarget = 160;
  }
  if (footOnGround) {
    ankleTarget = 90 - shoulder + elbow;
  }
}