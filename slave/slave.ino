#include <SoftwareSerial.h>
#include <ezButton.h>

// Definitions
#define VRX_PIN  A0             // VRX pin
#define VRY_PIN  A1             // VRY pin
#define START_PIN 4             // Button
#define PROG_PIN 5              // Programming mode

// Libraries initialization
SoftwareSerial BTSerial(2, 3);  // RX, TX
ezButton button(START_PIN);     // Initialize pin button

// Variables
int xValue = 0;                 // To store value of the X axis
int yValue = 0;                 // To store value of the Y axis]
int prog = 0;                   // To store the progamming pin value
bool buttonState = false;       // Boolean to track the state of the button
int debounceDelay = 10;         // Debounce time

void setup() {
  // Initialize Programming mode pin
  pinMode(PROG_PIN, INPUT);

  // Initialize Serial communications
  Serial.begin(9600);
  BTSerial.begin(38400);

  // Set debounce time
  button.setDebounceTime(debounceDelay);

  // Arduino is ready for the loop
  Serial.println("Slave ready, sending data to master...");
}

void loop() {
  // Call the loop() function first
  button.loop(); 
  // Check if button is pressed, if so change the current state
  if (button.isPressed()) {
    buttonState = !buttonState;
  }
  // read analog X and Y analog values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);

  // Read the value of the programming pin
  prog = digitalRead(PROG_PIN);

  // Send data to the receiver
  // Example: 1024,256,1,0
  BTSerial.print(String(xValue) + "," + String(yValue) + "," + String(buttonState) + "," + String(prog) + "\n");
  Serial.print(String(xValue) + "," + String(yValue) + "," + String(buttonState) + "," + String(prog) + "\n");

  // Wait few milliseconds
  delay(100); 
}