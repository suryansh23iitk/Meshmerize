// Pin Definitions
#define AIN1 19   // Motor A input 1
#define BIN1 5  // Motor B input 1
#define AIN2 21   // Motor A input 2
#define BIN2 18  // Motor B input 2
#define PWMA 13   // PWM control for Motor A
#define PWMB 12 // PWM control for Motor B
#define STBY 4   // Standby control

void setup() {
  // Set motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // Set STBY pin as output and bring it HIGH to exit standby mode
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // Disable standby mode
}

void loop() {
  // Move Forward
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 50);  // Set speed to maximum
  analogWrite(PWMB, 50);  // Set speed to maximum
  delay(2000);  // Move forward for 2 seconds

  // // Stop
  // digitalWrite(AIN1, LOW);
  // digitalWrite(AIN2, LOW);
  // digitalWrite(BIN1, LOW);
  // digitalWrite(BIN2, LOW);
  // delay(1000);  // Stop for 1 second

}