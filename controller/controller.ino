#include <SoftwareSerial.h>

// Definitions
#define BRAKE_PIN 4                       // Controls the brakes
#define STOP_PIN 5                        // Controls the start and stop
#define PIEZO_PIN 6                       // Controls the piezo
#define ROTATION_PIN 7                    // Controls the rotation of the motor
#define STATE_PIN 8                       // Bluetooth state pin
#define SPEED_R_PIN 9                     // Controls the speed of the right motor
#define SPEED_L_PIN 10                    // Controls the speed of the left motor

// Define a struct to return multiple values of the controller message
struct ControllerMessage {
  int vrx;
  int vry;
  int button;
  int prog;
};

// Constants
const float smoothingFactorAcc = 0.05;        // Smoothing factor for gradual speed changes (acceleration)
const float smoothingFactorDec = 0.15;        // Smoothing factor for gradual speed changes (deceleration)
const unsigned long expectedInterval = 100;   // Expected interval in milliseconds

// Variables
bool connectPhone = false;                // Allow phone connection
int motorSpeedR = 0;                      // Current right motor speed
int motorSpeedL = 0;                      // Current right motor speed
int mode = 0;                             // Skateboard default mode
String controllerMessage = "";            // Controller message
ControllerMessage message;                // Controller message results
unsigned long lastReceivedTime = 0;       // Time when the last message was received
unsigned long currentTime = 0;            // Current time for connection errors

// Libraries initialization
SoftwareSerial BTSerial(2, 3);            // RX, TX
SoftwareSerial PhoneSerial(4, 5);         // RX, TX

int programming_mode() {
  // Initialize variables
  String phoneMessage = "";               // Phone message
  char p;                                 // Character variable

  // Listen to the phone
  PhoneSerial.listen();

  // Debug
  Serial.println("Entering programming mode...");

  // Then, wait until a new line character is received
  while (p != '\n') {
    // Check if phone serial is available
    if (PhoneSerial.available()) {
      // Read character
      p = PhoneSerial.read();

      // Check if character is not a new line
      if (p != '\n') {
        // Append the character to the message
        phoneMessage += p;
      }
    }
  }

  // Set the type of driving
  mode = phoneMessage.toInt();

  // Debug
  Serial.println(phoneMessage); 
  Serial.println("Exiting programming mode...");

  return mode;
}

ControllerMessage get_controller_message() {
  // This boolean indicates the current status of the message
  bool message_incomplete = true;
  
  // Initialize the message string and the ControllerMessage struct
  String controllerMessage = "";
  ControllerMessage message;

  // Keep waiting until the message is complete
  while (message_incomplete) {
    // Set current time
    currentTime = millis();

    // If the connection with the controller is available
    if (BTSerial.available()) {
      // Register the character received
      char c = BTSerial.read();

      // Update last received time
      lastReceivedTime = currentTime;

      // If we received the end of line, the controller message is complete
      if (c == '\n') {
        // Print the message (optional)
        // Serial.println(controllerMessage);

        // Example: 1024,256,1,0
        // Find indexes of the commas
        int index1 = controllerMessage.indexOf(',');              
        int index2 = controllerMessage.indexOf(',', index1 + 1); 
        int index3 = controllerMessage.indexOf(',', index2 + 1); 

        // Extract substrings between commas and convert them to integers
        message.vrx = controllerMessage.substring(0, index1).toInt();
        message.vry = controllerMessage.substring(index1 + 1, index2).toInt();
        message.button = controllerMessage.substring(index2 + 1, index3).toInt();
        message.prog = controllerMessage.substring(index3 + 1).toInt();

        // Message is fully received
        message_incomplete = false;
      } else {
        // Append the received character to the message
        controllerMessage += c;
      }
    }
      // Check if there is any anomaly
      /*
      if (currentTime - lastReceivedTime > 200) {
        digitalWrite(BRAKE_PIN, LOW);
        // Debug
        Serial.println("BRAKE ACTIVE");
      } else {
          digitalWrite(BRAKE_PIN, HIGH);
      }
      */
  }

  // Rteturn the message
  return message;
}

void easy_mode() {
  // Easy mode: Only VRX is considered, performance are limited
  if (message.button == 1) {
    digitalWrite(BRAKE_PIN, LOW);
    // Debug
    Serial.println("BRAKE ACTIVE");
  } else {
    digitalWrite(BRAKE_PIN, HIGH);
  }

  // Since we want to use only the upper half of the joystick we have to map it properly
  // Decellerate
  if (message.vrx <= 502) {
    // Map joystick to PWM range
    int targetSpeed = map(message.vrx, 0, 502, 0, 127);  
    // Apply smoothing for gradual change in speed
    motorSpeedR = ((1 - smoothingFactorDec) * motorSpeedR) - (smoothingFactorDec * targetSpeed);
    motorSpeedL = motorSpeedR;
    // motorSpeed must be above 0
    if (motorSpeedR < 0) {motorSpeedR = 0;}
    if (motorSpeedL < 0) {motorSpeedL = 0;}
  }
  // Accellerate
  else {
    // Map joystick to PWM range
    int targetSpeed = map(message.vrx, 503, 1023, 0, 255);  
    // Apply smoothing for gradual change in speed
    motorSpeedR = (smoothingFactorAcc * targetSpeed) + ((1 - smoothingFactorAcc) * motorSpeedR);
    motorSpeedL = motorSpeedR;
  }

  // Debug
  Serial.println("Speed: " + String(motorSpeedR));

  // Set motor speed
  analogWrite(SPEED_R_PIN, motorSpeedR);
  analogWrite(SPEED_L_PIN, motorSpeedL);
}
/*
void driving_mode() {
  // Easy mode: Only VRX is considered, performance are limited
  if (message.button == 1) {
    digitalWrite(BRAKE_PIN, HIGH);
    // Debug
    Serial.println("BRAKE ACTIVE");
  } else {
    digitalWrite(BRAKE_PIN, LOW);
  }

  // Decellerate
  if (message.vrx <= 502) {
    // Map joystick to PWM range
    int targetSpeed = map(message.vrx, 0, 502, 0, 127);  
    // Apply smoothing for gradual change in speed
    motorSpeed = ((1 - smoothingFactorDec) * motorSpeed) - (smoothingFactorDec * targetSpeed);
    // motorSpeed must be above 0
    if (motorSpeed < 0) {motorSpeed = 0;}
  }
  // Accellerate
  else {
    // Map joystick to PWM range
    int targetSpeed = map(message.vrx, 503, 1023, 0, 255);  
    // Apply smoothing for gradual change in speed
    motorSpeed = (smoothingFactorAcc * targetSpeed) + ((1 - smoothingFactorAcc) * motorSpeed);
  }

  // Debug
  Serial.println("Speed: " + String(motorSpeed));

  // Set motor speed
  analogWrite(SPEED_R_PIN, motorSpeed);
  analogWrite(SPEED_L_PIN, motorSpeed);
}
*/

void setup() {
  // Initialize Serial communications
  Serial.begin(9600);
  BTSerial.begin(38400);
  PhoneSerial.begin(9600);

  // Set pinMode of each port
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(ROTATION_PIN, OUTPUT);
  pinMode(SPEED_R_PIN, OUTPUT);
  pinMode(SPEED_L_PIN, OUTPUT);
  pinMode(STATE_PIN, INPUT);
  
  // Set the intial states
  digitalWrite(BRAKE_PIN, HIGH);
  digitalWrite(STOP_PIN, LOW);
  digitalWrite(PIEZO_PIN, LOW);
  digitalWrite(ROTATION_PIN, LOW);

  // Set motor speed
  analogWrite(SPEED_R_PIN, 0);
  analogWrite(SPEED_L_PIN, 0);

  // Start by listening to controller
  BTSerial.listen();  

  // Arduino is ready for the loop
  Serial.println("Master ready, waiting for data from slave...");
}

void loop() {
  // Get the message from the controller
  message = get_controller_message();

  // Check if controller is in programming mode
  if (message.prog == 1) {
    // TODO: Check if it is working correctly
    // Select the driving mode
    mode = programming_mode();
  }

  // After checking if correct mode is selected, run motors with selected config
  switch(mode) {
    case 0: 
      easy_mode();
      break;
  }
}