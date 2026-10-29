#include "BioloidController.h"
#include "ax12.h"
#include "poses.h"

BioloidController bioloid = BioloidController(1000000);

const int SERVOCOUNT = 18;
const int STAND_OFFSET = 100;
const int WALK_CYCLES = 2;
int id;
int pos;
boolean IDCheck;
boolean RunCheck;

void setup() {

  pinMode(0, OUTPUT);

  id = 1;
  pos = 0;
  IDCheck = 1;
  RunCheck = 0;
  Serial.begin(9600);
  delay(500);

  Serial.println("###########################");
  Serial.println("Serial Communication Established.");
  CheckVoltage();

  // Walking is menu option 8, see Walk().

  MenuOptions();

  RunCheck = 1;
}

void loop() {

  int inByte = Serial.read();

  switch (inByte) {

  case '1':
    ScanServo();
    break;

  case '2':
    MoveCenter();
    break;

  case '3':
    RelaxServos();
    break;

  case '4':
    LeftLegTest();
    break;

  case '5':
    RightLegTest();
    break;

  case '6':
    CheckVoltage();
    break;

  case '7':
    LEDTest();
    break;

  case '8':
    Walk();
    break;
  }
}

// Walk forward on flat ground: center, stand, then WALK_CYCLES tripod steps.
void Walk() {
  delay(100);
  Serial.println("###########################");
  Serial.print("Walking ");
  Serial.print(WALK_CYCLES);
  Serial.println(" cycles");
  Serial.println("###########################");

  // readPose()/setNextPose() do nothing until loadPose() runs, so start from
  // Center.
  bioloid.loadPose(Center);
  bioloid.readPose();
  bioloid.interpolateSetup(1000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }

  LowerLegs();

  firstStep();

  for (int i = 0; i < WALK_CYCLES; i++) {
    LowerLegs();

    LiftRightLegs();

    RightLegTransition();

    LowerLegs();

    LiftLeftLegs();

    LeftLegTransition();
  }

  LowerLegs();

  if (RunCheck == 1) {
    MenuOptions();
  }
}

// Stand on flat ground: move each femur/tibia STAND_OFFSET down from Center.
// Coxas don't move. Replaces upstream TerrainTest(), which needed foot switches
// the simulator doesn't have.
void LowerLegs() {

  delay(100);
  Serial.println("###########################");
  Serial.println("Lowering Legs to Flat Ground");
  Serial.println("###########################");
  delay(1000);

  bioloid.readPose();

  for (int i = 1; i <= SERVOCOUNT; i++) {
    bioloid.setNextPose(i, bioloid.getCurPose(i));
  }

  // Each leg is (coxa, femur, tibia) = (n, n+2, n+4), n = 1,2,7,8,13,14.
  // Odd n is a left leg, even n a right leg; their directions are mirrored.
  for (int i = 1; i <= 13; i += 6) {
    for (int coxa = i; coxa <= i + 1; coxa++) {
      int down = (coxa % 2) ? STAND_OFFSET : -STAND_OFFSET;
      bioloid.setNextPose(coxa + 2,
                          pgm_read_word_near(Center + coxa + 2) + down);
      bioloid.setNextPose(coxa + 4,
                          pgm_read_word_near(Center + coxa + 4) - down);
    }
  }

  bioloid.interpolateSetup(500);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
}

// First step: lift the left tripod and swing it forward.
void firstStep() {
  int pos;

  delay(100);

  Serial.println("###########################");
  Serial.println("Transitioning Left Legs");
  Serial.println("###########################");
  delay(1000);

  bioloid.readPose();

  for (int i = 1; i <= SERVOCOUNT; i++) {
    pos = bioloid.getCurPose(i);
    if (i != 1 && i != 7 && i != 14 && i != 3 && i != 5 && i != 9 && i != 11 &&
        i != 16 && i != 18) {
      bioloid.setNextPose(i, pos);
    }
  }

  bioloid.setNextPose(1, 512);
  bioloid.setNextPose(7, 412);
  bioloid.setNextPose(14, 562);

  bioloid.setNextPose(3, 312);
  bioloid.setNextPose(5, 812);
  bioloid.setNextPose(9, 312);
  bioloid.setNextPose(11, 812);
  bioloid.setNextPose(16, 712);
  bioloid.setNextPose(18, 212);

  delay(1000);
  bioloid.interpolateSetup(2000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
}

// Lift the left tripod: front-left, rear-left and middle-right legs.
void LiftLeftLegs() {

  int pos;
  delay(100);

  Serial.println("###########################");
  Serial.println("Lifting Left Legs");
  Serial.println("###########################");
  delay(1000);

  bioloid.readPose();

  for (int i = 1; i <= SERVOCOUNT; i++) {
    pos = bioloid.getCurPose(i);
    if (i != 3 && i != 5 && i != 9 && i != 11 && i != 16 && i != 18) {
      bioloid.setNextPose(i, pos);
    }
  }

  bioloid.setNextPose(3, 312);
  bioloid.setNextPose(5, 812);
  bioloid.setNextPose(9, 312);
  bioloid.setNextPose(11, 812);
  bioloid.setNextPose(16, 712);
  bioloid.setNextPose(18, 212);

  delay(1000);
  bioloid.interpolateSetup(1000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
}

// Lift the right tripod: front-right, rear-right and middle-left legs.
void LiftRightLegs() {

  int pos;
  delay(100);

  Serial.println("###########################");
  Serial.println("Lifting Right Legs");
  Serial.println("###########################");
  delay(1000);

  bioloid.readPose();

  for (int i = 1; i <= SERVOCOUNT; i++) {
    pos = bioloid.getCurPose(i);
    if (i != 4 && i != 6 && i != 10 && i != 12 && i != 15 && i != 17) {
      bioloid.setNextPose(i, pos);
    }
  }

  bioloid.setNextPose(4, 712);
  bioloid.setNextPose(6, 212);
  bioloid.setNextPose(10, 712);
  bioloid.setNextPose(12, 212);
  bioloid.setNextPose(15, 312);
  bioloid.setNextPose(17, 812);

  delay(1000);
  bioloid.interpolateSetup(1000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
}

// Swing the lifted left tripod forward and push the other legs back.
void LeftLegTransition() {
  int pos;

  delay(100);

  Serial.println("###########################");
  Serial.println("Transitioning Left Legs");
  Serial.println("###########################");
  delay(1000);

  bioloid.readPose();

  for (int i = 1; i <= SERVOCOUNT; i++) {
    pos = bioloid.getCurPose(i);
    if (i != 1 && i != 7 && i != 14 && i != 2 && i != 13 && i != 8 && i != 16 &&
        i != 18) {
      bioloid.setNextPose(i, pos);
    }
  }
  bioloid.setNextPose(1, 512);
  bioloid.setNextPose(7, 412);
  bioloid.setNextPose(14, 562);

  bioloid.setNextPose(2, 412);
  bioloid.setNextPose(8, 512);
  bioloid.setNextPose(13, 512);
  bioloid.setNextPose(16, (bioloid.getCurPose(16) - 10));
  bioloid.setNextPose(18, (bioloid.getCurPose(18) + 10));

  delay(1000);
  bioloid.interpolateSetup(2000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
}

// Swing the lifted right tripod forward and push the other legs back.
void RightLegTransition() {
  int pos;

  delay(100);

  Serial.println("###########################");
  Serial.println("Transitioning Right Legs");
  Serial.println("###########################");
  delay(1000);

  bioloid.readPose();

  for (int i = 1; i <= SERVOCOUNT; i++) {
    pos = bioloid.getCurPose(i);
    if (i != 1 && i != 7 && i != 14 && i != 2 && i != 13 && i != 8 && i != 15 &&
        i != 17) {
      bioloid.setNextPose(i, pos);
    }
  }

  bioloid.setNextPose(1, 612);
  bioloid.setNextPose(2, 512);
  bioloid.setNextPose(7, 512);
  bioloid.setNextPose(8, 612);
  bioloid.setNextPose(13, 462);
  bioloid.setNextPose(14, 512);
  bioloid.setNextPose(15, (bioloid.getCurPose(15) + 10));
  bioloid.setNextPose(17, (bioloid.getCurPose(17) - 10));

  delay(1000);
  bioloid.interpolateSetup(2000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
}

void ScanServo() {
  id = 1;
  Serial.println("###########################");
  Serial.println("Starting Servo Scanning Test.");
  Serial.println("###########################");

  while (id <= SERVOCOUNT) {
    pos = ax12GetRegister(id, 36, 2);
    Serial.print("Servo ID: ");
    Serial.println(id);
    Serial.print("Servo Position: ");
    Serial.println(pos);

    if (pos <= 0) {
      Serial.println("###########################");
      Serial.print("ERROR! Servo ID: ");
      Serial.print(id);
      Serial.println(
          " not found. Please check connection and verify correct ID is set.");
      Serial.println("###########################");
      IDCheck = 0;
    }

    // Upstream used `id = (id++)%SERVOCOUNT;`, which is undefined behavior
    // and left id stuck at 1.
    id++;
    delay(500);
  }
  if (IDCheck == 0) {
    Serial.println("###########################");
    Serial.println("ERROR! Servo ID(s) are missing from Scan. Please check "
                   "connection and verify correct ID is set.");
    Serial.println("###########################");
  } else {
    Serial.println("All servo IDs present.");
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
}

void CheckVoltage() {
  // LiPo battery safety check
  float voltage = (ax12GetRegister(1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.println("###########################");
  Serial.print("System Voltage: ");
  Serial.print(voltage);
  Serial.println(" volts.");

  if (voltage < 10.0) {
    Serial.println("Voltage levels below 10v, please charge battery.");
    //  while(1);
  }
  if (voltage > 10.0) {
    Serial.println("Voltage levels nominal.");
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
  Serial.println("###########################");
}

void MoveCenter() {
  delay(100);
  bioloid.loadPose(Center);
  bioloid.readPose();
  Serial.println("###########################");
  Serial.println("Moving servos to centered position");
  Serial.println("###########################");
  delay(1000);
  bioloid.interpolateSetup(1000);
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
}

void LeftLegTest() {
  Serial.println("###########################");
  Serial.println("Initializing Left Leg Tests");
  Serial.println("###########################");
  delay(500);
  id = 1;
  while (id <= (SERVOCOUNT - 1)) {

    Serial.print("Moving Servo ID: ");
    Serial.println(id);
    SetPosition(id, 412);
    delay(1000);
    SetPosition(id, 512);
    delay(1000);
    id = id + 2;

    Serial.print("Moving Servo ID: ");
    Serial.println(id);
    SetPosition(id, 712);
    delay(1000);
    SetPosition(id, 512);
    delay(1000);
    id = id + 2;

    Serial.print("Moving Servo ID: ");
    Serial.println(id);
    SetPosition(id, 712);
    delay(1000);
    SetPosition(id, 512);
    delay(1000);
    id = id + 2;
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
}

void RightLegTest() {

  Serial.println("###########################");
  Serial.println("Initializing Right Leg Tests");
  Serial.println("###########################");
  delay(500);
  id = 2;
  while (id <= SERVOCOUNT) {

    Serial.print("Moving Servo ID: ");
    Serial.println(id);
    SetPosition(id, 612);
    delay(1000);
    SetPosition(id, 512);
    delay(1000);
    id = id + 2;

    Serial.print("Moving Servo ID: ");
    Serial.println(id);
    SetPosition(id, 312);
    delay(1000);
    SetPosition(id, 512);
    delay(1000);
    id = id + 2;

    Serial.print("Moving Servo ID: ");
    Serial.println(id);
    SetPosition(id, 312);
    delay(1000);
    SetPosition(id, 512);
    delay(1000);
    id = id + 2;
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
}

void MenuOptions() {

  Serial.println("###########################");
  Serial.println("Please enter option 1-8 to run individual tests again.");
  Serial.println("1) Servo Scanning Test");
  Serial.println("2) Move Servos to Center");
  Serial.println("3) Relax Servos");
  Serial.println("4) Perform Left Leg Sign Test");
  Serial.println("5) Perform Right Leg Sign Test");
  Serial.println("6) Check System Voltage");
  Serial.println("7) Perform LED Test");
  Serial.println("8) Walk Forward (flat ground)");
  Serial.println("###########################");
}

void RelaxServos() {
  id = 1;
  Serial.println("###########################");
  Serial.println("Relaxing Servos.");
  Serial.println("###########################");
  while (id <= SERVOCOUNT) {
    Relax(id);
    id++; // see ScanServo()
    delay(50);
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
}

void LEDTest() {
  id = 1;
  Serial.println("###########################");
  Serial.println("Running LED Test");
  Serial.println("###########################");
  while (id <= SERVOCOUNT) {
    ax12SetRegister(id, 25, 1);
    Serial.print("LED ON - Servo ID: ");
    Serial.println(id);
    delay(3000);
    ax12SetRegister(id, 25, 0);
    Serial.print("LED OFF - Servo ID: ");
    Serial.println(id);
    delay(3000);
    id++; // was `id = id++;`, see ScanServo()
  }

  if (RunCheck == 1) {
    MenuOptions();
  }
}
