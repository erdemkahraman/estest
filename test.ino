uint8_t array[64];

void setup() {
  Serial.begin(115200);
  for(int i=0; i<sizeof(array); i++) {
    array[i] = i;
  }
}

void loop() {
  Serial.write(array, sizeof(array));
  delay(1000);
}
