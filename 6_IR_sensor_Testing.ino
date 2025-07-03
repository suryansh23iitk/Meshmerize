#define NUM_SENSORS 6
unsigned int sensorArray[NUM_SENSORS] = {34,25, 14, 35, 26, 27};
unsigned int sensorValues[NUM_SENSORS];



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < NUM_SENSORS; i++){
    sensorValues[i] = analogRead(sensorArray[i]);
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println("");
  delay(20);
}
