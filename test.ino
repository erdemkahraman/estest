uint8_t array[64];

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.write(array, sizeof(array));
  delay(1000);
}
