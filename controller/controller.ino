#include <SoftwareSerial.h>

// Definitions
#define SPEED_R_PIN 9                     // Controls the speed of the right motor
#define SPEED_L_PIN 10                    // Controls the speed of the left motor
#define ROTATION_PIN 8                    // Controls the rotation of the motor
#define STOP_PIN 7                        // Controls the start and stop
#define GREEN_LED_PIN 12                  // Status led
#define RED_LED_PIN 11                    // Speed mode led 
#define CURRENT_PIN A3                    // Current sensor

// Libraries initialization
SoftwareSerial BTSerial(2, 3);            // RX, TX
SoftwareSerial PhoneSerial(4, 5);         // RX, TX

// Variables
String controllerMessage = "";            // Controller message
String phoneMessage = "";                 // Phone message
String mode = "easy economy";             // Skateboard default mode
bool connectPhone = false;                // Allow phone connection
float smoothingFactorAcc = 0.05;          // Smoothing factor for gradual speed changes (acceleration)
float smoothingFactorDec = 0.10;          // Smoothing factor for gradual speed changes (deceleration)
int motorSpeed = 0;                       // Current motor speed

void setup() {
  // Initialize Serial communications
  Serial.begin(9600);
  BTSerial.begin(38400);
  PhoneSerial.begin(9600);

  // Set pinMode of each port
  pinMode(SPEED_R_PIN, OUTPUT);
  pinMode(SPEED_L_PIN, OUTPUT);
  pinMode(ROTATION_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  // Set the intial states
  digitalWrite(ROTATION_PIN, HIGH);
  digitalWrite(STOP_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  // Set motor speed
  analogWrite(SPEED_R_PIN, 0);
  analogWrite(SPEED_L_PIN, 0);

  // Start by listening to controller
  BTSerial.listen();  

  // Arduino is ready for the loop
  Serial.println("Master ready, waiting for data from slave...");
  
}

void loop() {
  // If the connection with the controller is available
  if (BTSerial.available()) {
    // Register the character receiveds
    char c = BTSerial.read();
    
    // If we received  the end of line the controller message is complete
    if (c == '\n') {
      // Print the message
      // Serial.println(controllerMessage);

      // Example: 1024,256,1 or 1024,256,0
      // Find indexes of the two commas
      int index1 = controllerMessage.indexOf(',');              
      int index2 = controllerMessage.indexOf(',', index1 + 1);  
      // Extract substrings between commas
      int vrx = controllerMessage.substring(0, index1).toInt();
      int vry = controllerMessage.substring(index1 + 1, index2).toInt();
      int button = controllerMessage.substring(index2 + 1).toInt();

      // If button is in state 1, then enter in configuration mode
      if (button == 1) {
        // Listen to the phone
        PhoneSerial.listen();
        
        // Initialize the character variable
        char p;

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

        // Print phone message
        Serial.println(phoneMessage); 

        mode = phoneMessage;
      }

      // Easy mode: Only VRX is considered, performance are limited
      if (motorSpeed == 0){
        // Output a LOW signal to stop the motors
        pinMode(STOP_PIN, OUTPUT);
        digitalWrite(STOP_PIN, LOW);
      } else {
        // Set pinMode as INPUT so that it is an high impedance pin
        pinMode(STOP_PIN, INPUT);
      }

      // Since we want to use only the upper half of the joystick we have to map it properly
      // Decellerate
      if (vrx <= 502) {
        // Map joystick to PWM range
        int targetSpeed = map(vrx, 0, 502, 0, 127);  
        // Apply smoothing for gradual change in speed
        motorSpeed = ((1 - smoothingFactorDec) * motorSpeed) - (smoothingFactorDec * targetSpeed);
        // motorSpeed must be above 0
        if (motorSpeed < 0) {motorSpeed = 0;}
      }
      // Accellerate
      else {
        // Map joystick to PWM range
        int targetSpeed = map(vrx, 503, 1023, 0, 255);  
        // Apply smoothing for gradual change in speed
        motorSpeed = (smoothingFactorAcc * targetSpeed) + ((1 - smoothingFactorAcc) * motorSpeed);
      }
    
      // Print the motor speed
      Serial.println("Speed: " + String(motorSpeed) + ", STOP: " + String(digitalRead(STOP_PIN)));

      // Set motor speed
      analogWrite(SPEED_R_PIN, motorSpeed);
      analogWrite(SPEED_L_PIN, motorSpeed);

      // Prepare the receiver for a new command
      controllerMessage = "";

    // If we have not received a \n then the message is still not complete  
    } else {
      controllerMessage += c;
    }
  }
}