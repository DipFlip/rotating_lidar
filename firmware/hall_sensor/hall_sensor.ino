/**
 * Hall sensor firmware for ESP32-C3.
 *
 * Detects magnet passes on a rotating LiDAR mount and outputs structured
 * serial data for the ROS2 hall_sensor_node to parse.
 *
 * Output format: H,<count>,<millis>
 *   count  - cumulative number of magnet passes since boot
 *   millis - ESP32 uptime in milliseconds at detection time
 *
 * Debounce is applied to avoid double-counting on noisy transitions.
 */

#define HALL_PIN 2     // GPIO connected to hall sensor output
#define DEBOUNCE_MS 50 // Minimum time between triggers

volatile unsigned long lastTriggerMs = 0;
volatile int triggerCount = 0;
volatile bool newTrigger = false;

void IRAM_ATTR hallISR() {
  unsigned long now = millis();
  if (now - lastTriggerMs >= DEBOUNCE_MS) {
    lastTriggerMs = now;
    triggerCount++;
    newTrigger = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
  Serial.println("H,0,0");
}

void loop() {
  if (newTrigger) {
    newTrigger = false;
    // Output structured line: H,<count>,<millis>
    Serial.print("H,");
    Serial.print(triggerCount);
    Serial.print(",");
    Serial.println(lastTriggerMs);
  }
  delay(1);
}
