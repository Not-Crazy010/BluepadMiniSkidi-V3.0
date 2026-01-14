//Made by not_crazy_man on discord
//profboots discord if you need help: https://discord.gg/yM52pQr82e
#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define bucketServoPin 23
#define auxServoPin 22
#define lightPin0 18
#define lightPin1 5
#define leftMotor0 33
#define leftMotor1 32
#define rightMotor0 25
#define rightMotor1 26
#define ArmMotor0 21
#define ArmMotor1 19

Servo bucketServo;
Servo auxServo;

bool lightsOn = false;
int auxpos = 15;
int bucketpos = 170;

//settings
bool flipArmMotor = false;
bool flipLeftMotor = false;
bool flipRightMotor = false;
int howFastIsBucket = 200; //bigger is slower
int howFastIsAux = 150; //bigger is slower
int lightHoldTime = 1000; //1000 = 1s delay between you can turn the lights off and on

void processThrottle(ControllerPtr ctl) {
  //right motor
  if(!flipRightMotor){
    if (ctl->r1()) {
      moveMotor(rightMotor0, rightMotor1, -256);
    }else if (ctl->r2()) {
      moveMotor(rightMotor0, rightMotor1, 256);
    }
  }else{
    if (ctl->r1()) {
      moveMotor(rightMotor1, rightMotor0, -256);
    }else if (ctl->r2()) {
      moveMotor(rightMotor1, rightMotor0, 256);
    }
  }
  if(!ctl->r1() && !ctl->r2()){
    moveMotor(rightMotor1, rightMotor0, 0);
  }
  //left motor
  if(!flipLeftMotor){
    if (ctl->l1()) {
      moveMotor(leftMotor0, leftMotor1, -256);
    }else if (ctl->l2()) {
      moveMotor(leftMotor0, leftMotor1, 256);
    }
  }else{
    if (ctl->l1()) {
      moveMotor(leftMotor1, leftMotor0, -256);
    }else if (ctl->l2()) {
      moveMotor(leftMotor1, leftMotor0, 256);
    }
  }
  if(!ctl->l1() && !ctl->l2()){
    moveMotor(leftMotor1, leftMotor0, 0);
  }
}
void processArm(int axisValue) {
  int adjustedArmValue = axisValue / 2;
  if(adjustedArmValue > 15 || adjustedArmValue < -15){// this is the bounds of the controller input
    if (!flipArmMotor) {
      moveMotor(ArmMotor0, ArmMotor1, adjustedArmValue);
    } else {
      moveMotor(ArmMotor1, ArmMotor0, adjustedArmValue);
    }
  }else{
    moveMotor(ArmMotor0, ArmMotor1, 0);
  }
}
void processBucket(int axisValue) {
  int adjustedBucketValue = (axisValue / howFastIsBucket);
  bucketpos -= adjustedBucketValue;//if you need to flip the servo movement just flip the places of the -= and +=
  if(bucketpos < 170 && bucketpos > 0){//bounds of servo movement
    bucketServo.write(bucketpos);
  }
  else{
    bucketpos += adjustedBucketValue;//if you need to flip the servo movement just flip the places of the -= and +=
  }
}
void processAux(int axisValue) {
  int adjustedAuxValue = (axisValue / howFastIsAux); 
  auxpos += adjustedAuxValue;//if you need to flip the servo movement just flip the places of the -= and +=
  if(auxpos < 110 && auxpos > 15){//bounds of servo
    auxServo.write(auxpos);
  }
  else{
    auxpos -= adjustedAuxValue;//if you need to flip the servo movement just flip the places of the -= and +=
  }
}
void processLight(bool buttonValue) {
  if (buttonValue) {
    if (lightsOn) {
      digitalWrite(lightPin0, LOW);
      digitalWrite(lightPin1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(lightPin0, HIGH);
      digitalWrite(lightPin1, LOW);
      lightsOn = true;
    }
    delay(lightHoldTime);
  }
}
void moveMotor(int motorPin0, int motorPin1, int velocity) {// analogWrite is 0 to 256 if you want to do a digitalWrite then put in something like (moveMotor(rightMotor0, rightMotor1, 256); or moveMotor(rightMotor0, rightMotor1, -256);)
  if (velocity >= 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity <= -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
    
  }
}
//all the setup stuff is under this
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}
void processGamepad(ControllerPtr ctl) {
  //Throttle
  processThrottle(ctl);
  //Bucket
  processBucket(ctl->axisRY());
  //Aux
  processAux(ctl->axisRX());
  //Rasing and lowering of Arm
  processArm(ctl->axisY());
  //lights
  processLight(ctl->x());
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}
// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(ArmMotor0, OUTPUT);
  pinMode(ArmMotor1, OUTPUT);
  pinMode(lightPin0, OUTPUT);
  pinMode(lightPin1, OUTPUT);
  digitalWrite(lightPin0, LOW);
  digitalWrite(lightPin1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);


  bucketServo.attach(bucketServoPin);
  bucketServo.write(bucketpos);
  auxServo.attach(auxServoPin);
  auxServo.write(auxpos);

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
}
// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
}