#include <Arduino.h>
#include <avr/sleep.h>

// Define pin assignments
#define WAKE_PIN_RISING 2   // Pin to wake up on rising edge
#define WAKE_PIN_FALLING 3  // Pin to wake up on falling edge
#define LED_PIN 13          // LED pin
#define PowerPin 10         // Power pin to reboot the system

// Global variable to store the last value of the pin
uint8_t lastPinValue = HIGH;

// Interrupt Service Routine (ISR) for waking up
void wakeUp() {
    // Empty ISR to wake up the microcontroller
}

void initializeLastPinValue(uint8_t pin) {
    uint8_t initialValue = digitalRead(pin);
    bool isStable = true;

    for (int i = 0; i < 10; i++) {
        delay(200); // Wait for 100ms
        uint8_t currentValue = digitalRead(pin);
        Serial.print("Initial Check ");
        Serial.print(i + 1);
        Serial.print(": Pin value = ");
        Serial.println(currentValue);

        if (currentValue != initialValue) {
            isStable = false;
            break;
        }
    }

    if (isStable) {
        lastPinValue = initialValue;
        Serial.print("Initial pin value stabilized as: ");
        Serial.println(lastPinValue);
    } else {
        Serial.println("Initial pin value is not stable. Defaulting to HIGH.");
        lastPinValue = HIGH; // Default value if unstable
    }
}

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure pin modes
    pinMode(WAKE_PIN_RISING, INPUT_PULLUP);
    pinMode(WAKE_PIN_FALLING, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Initialize LED to OFF
    pinMode(PowerPin, OUTPUT);   // Power pin to reboot the system
    digitalWrite(PowerPin, HIGH); // Initialize power pin to OFF

    // Determine the initial value of lastPinValue
    initializeLastPinValue(WAKE_PIN_RISING);

    // Attach interrupts to wake up the microcontroller
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_RISING), wakeUp, RISING);
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_FALLING), wakeUp, FALLING);
}

// Function to put the microcontroller into deep sleep
void goToSleep() {
    Serial.println("Going to sleep...");
    delay(50);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Deep sleep mode
    sleep_enable();

    // Enable interrupts to wake up
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_RISING), wakeUp, RISING);
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_FALLING), wakeUp, FALLING);

    sleep_mode(); // Enter sleep mode (execution stops here until wake-up)

    // Resumes here after wake-up
    sleep_disable();

    delay(50);
    detachInterrupt(digitalPinToInterrupt(WAKE_PIN_RISING));
    detachInterrupt(digitalPinToInterrupt(WAKE_PIN_FALLING));
    Serial.println("Woke up!");
}

// Function to check if a pin remains in a specific state for 500ms
void checkPinState(uint8_t rising_pin) {
    // Read the current state of the pin after waking up
    uint8_t expectedState = (lastPinValue == HIGH) ? LOW : HIGH;
    Serial.print("expectedState value = ");
    Serial.println(expectedState); // Print the current pin value
    // Check the pin value 10 times, each 100ms apart
    for (int i = 0; i < 10; i++) {
        delay(200); // Wait for 100ms
        uint8_t currentState = digitalRead(rising_pin);
        Serial.print("Check ");
        Serial.print(i + 1);
        Serial.print(": Pin value = ");
        Serial.println(currentState); // Print the current pin value

        if (currentState != expectedState) {
            // If the pin value changes, exit the function
            return;
        }
    }

    // If the pin value remained as the opposite of lastPinValue for 10 checks
    // Toggle the LED and apply changes to PowerPin
    digitalWrite(LED_PIN, HIGH);  // Turn LED ON
    digitalWrite(PowerPin, LOW);  // Turn PowerPin ON
    delay(500);                   // Wait for 500ms
    digitalWrite(LED_PIN, LOW);   // Turn LED OFF
    digitalWrite(PowerPin, HIGH); // Turn PowerPin OFF

    // Update lastPinValue to the current state
    lastPinValue = expectedState;
}

void loop() {
    // Print the current state of the pin before going to sleep
    uint8_t currentStateBeforeSleep = digitalRead(WAKE_PIN_RISING);
    Serial.print("Before sleep: Pin value = ");
    Serial.println(currentStateBeforeSleep);

    // Store the last pin value before going to sleep
    lastPinValue = currentStateBeforeSleep;

    // Enter deep sleep
    goToSleep();

    // Print the current state of the pin after waking up
    uint8_t currentStateAfterSleep = digitalRead(WAKE_PIN_RISING);
    Serial.print("After wake-up: Pin value = ");
    Serial.println(currentStateAfterSleep);

    // After waking up, check the state of each pin
    checkPinState(WAKE_PIN_RISING);
}




