/**
 * Hall sensor firmware for ESP32-C3 — rotating LiDAR zero-reference.
 *
 * The hall sensor module has a digital output (DO) with built-in
 * comparator and LED indicator.  DO goes LOW when the magnet is
 * detected.  Connected to GPIO 4.
 *
 * Polls digitalRead at 10 kHz.  On each falling edge (HIGH->LOW),
 * outputs:  H,<count>,<millis>
 *
 * A heartbeat line is printed every second:  B,<pin_state>,<millis>
 *
 * Compile with:  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc
 */

#define HALL_PIN 4

// Minimum interval between triggers (ms) — debounce.
#define MIN_TRIGGER_INTERVAL_MS 200

// Heartbeat interval (ms)
#define HEARTBEAT_INTERVAL_MS 1000

static unsigned long trigger_count = 0;
static unsigned long last_trigger_ms = 0;
static unsigned long last_heartbeat_ms = 0;
static int last_pin = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT_PULLUP);
  delay(500);
  last_pin = digitalRead(HALL_PIN);
  Serial.println("H,0,0");
}

void loop() {
  int pin = digitalRead(HALL_PIN);
  unsigned long now = millis();

  // Detect falling edge (HIGH -> LOW = magnet arriving)
  if (pin == LOW && last_pin == HIGH) {
    if (now - last_trigger_ms >= MIN_TRIGGER_INTERVAL_MS) {
      trigger_count++;
      last_trigger_ms = now;
      Serial.print("H,");
      Serial.print(trigger_count);
      Serial.print(",");
      Serial.println(now);
    }
  }
  last_pin = pin;

  // Heartbeat
  if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_ms = now;
    Serial.print("B,");
    Serial.print(pin);
    Serial.print(",");
    Serial.println(now);
  }

  delayMicroseconds(100);  // 10 kHz polling
}
