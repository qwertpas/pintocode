// Pin definitions
const int leftSignalPins[] = {2, 3, 4};
const int rightSignalPins[] = {5, 6, 7};
const int numLeds = 3;

// Timing parameters
const int delayTime = 400; // Time in milliseconds between each LED lighting up
const int offTime = 400; // Time in milliseconds to keep LEDs off between sequences

// Control flags
bool leftSignalOn = false;
bool rightSignalOn = false;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);

  // Initialize the LED pins as outputs
  for (int i = 0; i < numLeds; i++) {
    pinMode(leftSignalPins[i], OUTPUT);
    pinMode(rightSignalPins[i], OUTPUT);
  }

  // Ensure all LEDs are off initially
  turnOffAllLeds();
}

void loop() {
  // Check if serial data is available
  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Determine which signal to trigger or turn off
    if (command == 'L' || command == 'l') {
      leftSignalOn = true;
      rightSignalOn = false;
    } else if (command == 'R' || command == 'r') {
      rightSignalOn = true;
      leftSignalOn = false;
    } else if (command == 'O' || command == 'o') {
      leftSignalOn = false;
      rightSignalOn = false;
      turnOffAllLeds();
    }
  }

  // Trigger the signals if they are on
  if (leftSignalOn) {
    triggerTurnSignal(leftSignalPins);
  } else if (rightSignalOn) {
    triggerTurnSignal(rightSignalPins);
  }
}

// Function to trigger the turn signal
void triggerTurnSignal(const int signalPins[]) {
  // Light up LEDs in sequence
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(signalPins[i], HIGH);
    delay(delayTime);
  }

  // Turn off all LEDs simultaneously
  turnOffAllLeds();

  // Keep LEDs off for a specified duration
  delay(offTime);
}

// Function to turn off all LEDs
void turnOffAllLeds() {
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(leftSignalPins[i], LOW);
    digitalWrite(rightSignalPins[i], LOW);
  }
}
