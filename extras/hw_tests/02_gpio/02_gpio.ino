#include <Adafruit_TSL2585.h>

#define GPIO_PIN 3

Adafruit_TSL2585 tsl2585;

void haltWithFailure(const __FlashStringHelper *message);
void haltWithSuccess();

void setup() {
  Serial.begin(115200);
  // Wait for the Serial Monitor to open on native USB boards.
  // Remove this while (!Serial) loop to run without a USB connection.
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println("TSL2585 D3 GPIO input hardware test");

  pinMode(GPIO_PIN, INPUT);
  Serial.println("D3 will only pull low or release");

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println("Begin succeeded");

  Serial.println();
  if (!tsl2585.enableGPIOInput(true)) {
    haltWithFailure(F("Enabling the sensor GPIO input failed"));
  }
  Serial.println("Sensor GPIO input enabled");

  delay(10);
  if (!tsl2585.readGPIOInput()) {
    haltWithFailure(F("Sensor GPIO did not read high when D3 was released"));
  }
  Serial.println("PASS: sensor GPIO read high when D3 was released");

  digitalWrite(GPIO_PIN, LOW);
  pinMode(GPIO_PIN, OUTPUT);
  delay(10);
  if (tsl2585.readGPIOInput()) {
    haltWithFailure(F("Sensor GPIO did not read low when D3 pulled low"));
  }
  Serial.println("PASS: sensor GPIO read low when D3 pulled low");

  pinMode(GPIO_PIN, INPUT);
  delay(10);
  if (!tsl2585.readGPIOInput()) {
    haltWithFailure(F("Sensor GPIO did not return high when D3 was released"));
  }
  Serial.println("PASS: sensor GPIO returned high when D3 was released");

  haltWithSuccess();
}

void loop() {}

void haltWithFailure(const __FlashStringHelper *message) {
  pinMode(GPIO_PIN, INPUT);
  tsl2585.enableGPIOInput(false);
  Serial.print("FAIL: ");
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  pinMode(GPIO_PIN, INPUT);
  tsl2585.enableGPIOInput(false);
  Serial.println();
  Serial.println("ALL GPIO INPUT TESTS PASSED");
  while (true) {
    delay(100);
  }
}
