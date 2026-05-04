// For the XIAO Sense, these are the pins:
// LED_RED   (P0.26)
// LED_GREEN (P0.30)
// LED_BLUE  (P0.06)

void setup() {
  // Initialize the digital pins as outputs.
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Turn all off initially (HIGH = OFF on this board)
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void loop() {
  // Blink Red
  digitalWrite(LED_RED, LOW);  // ON
  delay(500);
  digitalWrite(LED_RED, HIGH); // OFF
  delay(500);
  
  // Blink Blue
  digitalWrite(LED_BLUE, LOW);  // ON
  delay(500);
  digitalWrite(LED_BLUE, HIGH); // OFF
  delay(500);
}