const int irSensorPin = 2;  // GPIO pin for HW-201 IR sensor

#define RELAY_PIN 3               // GPIO 3 for the relay
#define SWITCH_PIN 5              // GPIO 5 for the switch
#define HAND_DETECTION_TIME 60000  // 20 seconds for relay ON

bool relayState = true;          // True means relay is ON initially
bool handDetectionMode = false;  // Tracks if hand detection is active
unsigned long handDetectionStart = 0;
bool relayTimerActive = false;  // Tracks if relay is currently active
void setup() {
  Serial.begin(115200);

  // Setup pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Pullup resistor ensures default HIGH when not pressed
  pinMode(irSensorPin, INPUT);        // Set the IR sensor pin as input
 
}

void loop() {
  // Check if the switch is pressed
  bool switchState = digitalRead(SWITCH_PIN) == LOW;  // LOW means pressed

  if (!switchState) {
    // Switch not pressed, relay should stay ON
    relayState = true;
    handDetectionMode = false;
    relayTimerActive = false;
  } else {
    // Switch is pressed, enable hand detection
    handDetectionMode = true;  // Enable hand detection mode
  }

  // If in hand detection mode, check for hand presence
  if (handDetectionMode) {
    int sensorValue = digitalRead(irSensorPin);  // Read the IR sensor value

    if (sensorValue == LOW) {  // Hand detected
      //Serial.print(" Hand detected! ");
      relayState = true;    
      if (HAND_DETECTION_TIME>0){
      relayTimerActive = true;
      }          // Relay will be turned ON
             // Reset relay active status
      handDetectionStart = millis();  // Store the time when hand was detected
    }
    // Non-blocking relay control
    if (relayTimerActive) {
      Serial.print("Time Left: ");
      Serial.print(HAND_DETECTION_TIME - (millis() - handDetectionStart));
      if (millis() - handDetectionStart >= HAND_DETECTION_TIME) {
        // If the relay has been active for 20 seconds, turn it off
        relayState = false;
        relayTimerActive = false;  // Reset relay active status
        Serial.println("Relay turned OFF after 20 seconds.");
      }
    }else {
      if(sensorValue == LOW && relayTimerActive == false){
      // No hand detected, keep relay OFF
      relayState = true;  // Update relay state

      }
      else{
      // No hand detected, keep relay OFF
      relayState = false;  // Update relay state
      }

    }
  }

  // Update relay based on the current relay state
  updateRelay();

  // Debugging: print out the state of the system
  Serial.print(" Switch: ");
  Serial.print(switchState ? "Pressed " : "Not Pressed ");
  Serial.print(", Relay: ");
  Serial.print(relayState ? "ON " : "OFF ");
  Serial.print(", Hand Detect Mode: ");
  Serial.println(handDetectionMode ? "Active " : "Inactive ");
  //Serial.print("Hand Detection Mode: "); Serial.println(handDetectionMode ? "Active " : "Inactive ");
  delay(10);  // Add delay to reduce the number of prints
}

void updateRelay() {
  if (relayState) {
    digitalWrite(RELAY_PIN, LOW);  // Turn relay ON
    //Serial.println("Relay turned ON.");
  } else {
    digitalWrite(RELAY_PIN, HIGH);  // Turn relay OFF
                                    // Serial.println("Relay turned OFF.");
  }
}
