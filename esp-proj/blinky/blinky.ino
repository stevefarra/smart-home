void setup() {
 pinMode(5, OUTPUT); // GPIO05, Digital Pin D1
}

void loop() {
 digitalWrite(5, HIGH);
 delay(900);
 digitalWrite(5, LOW);
 delay(500);
}
