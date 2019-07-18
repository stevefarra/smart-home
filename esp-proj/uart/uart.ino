void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {
    Serial.println(Serial.readStringUntil('\n'));
  }
} 
