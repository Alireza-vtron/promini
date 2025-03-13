#include <Arduino.h>
#include <avr/sleep.h>

// Interrupt pins
#define WAKE_PIN_RISING 2   // Wake up on RISING edge
#define WAKE_PIN_FALLING 3  // Wake up on FALLING edge
#define LED_PIN 10          // LED to signal wake-up

void wakeUp() {
    // Interrupt Service Routine (does nothing, just wakes up)
}

void setup() {
    Serial.begin(115200);
    
    pinMode(WAKE_PIN_RISING, INPUT_PULLUP);
    pinMode(WAKE_PIN_FALLING, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED starts HIGH

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_RISING), wakeUp, RISING);
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_FALLING), wakeUp, FALLING);
}

void goToSleep() {
    Serial.println("Going to sleep...");

    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Deep sleep mode
    sleep_enable();

    // Enable interrupts to wake up
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_RISING), wakeUp, RISING);
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN_FALLING), wakeUp, FALLING);

    sleep_mode(); // Enter sleep mode (execution stops here until wake-up)

    // Resumes here after wake-up
    sleep_disable();
    detachInterrupt(digitalPinToInterrupt(WAKE_PIN_RISING));
    detachInterrupt(digitalPinToInterrupt(WAKE_PIN_FALLING));

    Serial.println("Woke up!");
    
    // Turn LED OFF for 500ms
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);

    // Go back to sleep
    goToSleep();
}

void loop() {
    goToSleep();  // Continuously enter sleep mode
}
