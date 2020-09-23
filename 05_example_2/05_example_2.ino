#define PIN_LED 7
unsigned int count;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  count = 0;
  count += 1;
  delay(1000);
  count += 1;
}


void loop() {
   digitalWrite(PIN_LED, LOW);
   delay(1000);
   digitalWrite(PIN_LED, HIGH);
   delay(100);
   digitalWrite(PIN_LED, LOW);
   delay(100);
   digitalWrite(PIN_LED, HIGH);
   delay(100);
   digitalWrite(PIN_LED, LOW);
   delay(100);
   digitalWrite(PIN_LED, HIGH);
   delay(100);
   digitalWrite(PIN_LED, LOW);
   delay(100);
   digitalWrite(PIN_LED, HIGH);
   delay(100);
   digitalWrite(PIN_LED, LOW);
   delay(100);
   digitalWrite(PIN_LED, HIGH);
   delay(100);
   digitalWrite(PIN_LED, LOW);
   delay(100);
   digitalWrite(PIN_LED, HIGH);
   delay(100);
   digitalWrite(PIN_LED, LOW);
   delay(100);
   digitalWrite(PIN_LED, HIGH);
  while (1) {}
}
