#include <Arduino.h>
#include <avr/sleep.h>

// Define pin assignments
#define WAKE_PIN_RISING 2   // Pin to wake up on rising edge
#define WAKE_PIN_FALLING 3  // Pin to wake up on falling edge
#define LED_PIN 10          // LED pin

// Interrupt Service Routine (ISR) for waking up
void wakeUp() {
    // Empty ISR to wake up the microcontroller
}

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure pin modes
    pinMode(WAKE_PIN_RISING, INPUT_PULLUP);
    pinMode(WAKE_PIN_FALLING, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // Initialize LED to ON

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
    // Read the initial state of the pin
    uint8_t initialState = digitalRead(rising_pin);
    uint8_t expectedState = initialState;

    // If the pin is in the expected state
    // if (initialState == RisingExpectedState || initialState == FallingExpectedState) {
        // Wait for 500ms
    delay(1000);

    // Read the state of the pin again
    uint8_t finalState = digitalRead(rising_pin);

    // If the state remained unchanged
    if (finalState == expectedState) {
        // Toggle the LED based on the expected state
        // Turn power pin ON for 500ms to reboot the system
        digitalWrite(LED_PIN, LOW);  // Assuming LOW turns the LED ON
        delay(500);
        digitalWrite(LED_PIN, HIGH); // Turn LED OFF
    }
}

void loop() {
    // Enter deep sleep
    goToSleep();

    // After waking up, check the state of each pin
    checkPinState(WAKE_PIN_RISING);
}




