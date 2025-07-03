//Defing pins for motor driver
#define AIN1 19
#define BIN1 18
#define AIN2 21
#define BIN2 5
#define PWMA 13
#define PWMB 12
#define STBY 4

//Defing pin for the red LED
#define LED 2 

//Defining interrupt pins [These are for the corner most sensors]
#define INT0 39  //Interrupt pins
#define INT1 36
//Defining pins for push buttons
#define SW1 22 // For RHR to LHR
#define SW2 23 // For Final Run
//Defing pins for RGB lights
// #define R 13
#define G 33
#define B 32


//Defining the number of sensors used [apart from the corner most ones], their pin numbers and an array to store their values
const int numChannels = 6;
const int sensorArray[6] = {34, 25, 14, 35, 26, 27}; // Example pins for ESP32   // 2 and 3 Online , 1 and 4 for PID , 0 and 5 for turns
unsigned int sensorValues[numChannels];

//Defining threshold values for black and white detection
int thresholdl  = 1000; //low
int thresholdh  = 2000; //high
int threshold_avg = 1500; // average

// Defining constans for PID
float Kp = 0.04;       // 0.04 and 2 good for 100 speed
float Kd = 3.0;
float Ki = 0.1;
int  PIDvalue;
float error, previousError , P , I , D;

//Defining the max speed that can be attained by any one of the motor
int max_speed=200;
// Defining speeds of left and right motors
int lsp = max_speed;
int rsp = max_speed;

// Defining array to store the final path
char path[100];  // Assuming it won't be bigger than 100 turns :)
int path_length=0; // Storing the actual number of turns

// Defing some useful flags
int end = 0; // =1 when reached the end
int flag_final_run = 0; // =1 when the push button for final run is pressed
int flag_turn = 0; // =1 whenever a turn is detected by the corner most sensors
int flag_left_mode=0; // =1 when the push button to use the LHR is pressed

//Declaration of some funstiosn used in setup
void TurnAvailable();
void LefttModeActivator();

void setup() {
  // Setting the motor driver pins to output 
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY,OUTPUT);
  digitalWrite(STBY, HIGH); // Settign STBY to HIGH [IMP!!]

  // Settign the pins of rgb light to output
  pinMode(LED, OUTPUT);
  // pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  // Initializing them to be off
  // digitalWrite(R, LOW);
  digitalWrite(G, LOW);
  digitalWrite(B, LOW);
  // Initializing the push buttons as pull down [this depends on how you have set the circuit for the push button and may vary]
  pinMode(SW1, INPUT_PULLDOWN);
  pinMode(SW2, INPUT_PULLDOWN);


  Serial.begin(115200); // Initialize serial communication at 115200 baud

  
  //Attaching the Extreme sensor readings to interupt for Turns
  attachInterrupt(digitalPinToInterrupt(INT0), TurnAvailable, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT1), TurnAvailable, FALLING);
  //Attaching Interrupt for activating the LHR
  attachInterrupt(digitalPinToInterrupt(SW1), LefttModeActivator, RISING);
}

// Declaration of the remaining functions used 
void Turn();
void GetSensorData();
void linefollow();
void PID();
void motor1run(int motorSpeed);
void motor2run(int motorSpeed);
void Right();
void Left();
void simplify_path();

void loop()
{
  // Snippet when the end is reached
  if(end)
    {
      //Turn the Red LED HIGH
      digitalWrite(LED, HIGH);
      //Stop the  motors
      motor1run(0);
      motor2run(0);
      // Print the path [Just for debugging]
      for(int i=0;i<path_length;i++)
      {
        Serial.print(path[i]);
        Serial.print(" ");
      }
      Serial.println("");
      // Since end can be considered as turn [ make flag turn  = 0]
      flag_turn = 0;
      //Set the flag to 1 to begin the final run
      if(digitalRead(SW2))
      {
        flag_final_run = 1;
      }
      // counter to keep track of how many turns hae we taken
      int counter=0;
      // Snippet for the final run
      while(flag_final_run){
        //1st ly the set the red LED back to low
        digitalWrite(LED, LOW);
        // Keep following the line until a turn is detected
        linefollow();
        if(flag_turn)
        {
          // Degenrative braking to somewhat stop the motor from overshooting
          motor1run(-255);
          motor2run(-255);
          delay(20);
          // A bit of a pause to shed the effects of the already gained momentum speeding up the forward push.
          motor1run(0);
          motor2run(0);
          delay(10);
          // Pushing the bot forward enough to take effective turns
          motor1run(100);
          motor2run(100);
          delay(100);
          //  motor1run(0);
          // motor2run(0);
          // delay(100);
          //At any turn simply see which turn wwas stored in the final path and take it.
          if(path[counter]=='L')
          {
            //GetSensorData();
            //if(sensorValues[0] > 1000 && sensorValues[1] > 1000 && sensorValues[2] > 1000 && sensorValues[3] > 1000 && sensorValues[4] > 1000 && sensorValues[5] > 1000){
              Left();
           // }
            //else{
              //LeftWithForward();
            //}
            counter++;
          }
          else if(path[counter]=='R')
          {
            //GetSensorData();
            //if(sensorValues[0] > 1000 && sensorValues[1] > 1000 && sensorValues[2] > 1000 && sensorValues[3] > 1000 && sensorValues[4] > 1000 && sensorValues[5] > 1000){
              Right();
            //}
            //else{
             // RightWithForward();
            //}
            counter++;
          }
          else if(path[counter] == 'E'){
            motor1run(0);
            motor2run(0);
            break;
          }
          // Just a fail safe
          else
          {
            counter++;
            GetSensorData();
            linefollow();
          }
          //When a turn ends, set the flag to zero
          flag_turn = 0;
        }
      }
      // When final run ends set it's flag to zero
      flag_final_run=0;
    }
  
  //Snippet for the dry run
  if(!end){

    // Set the mode you are going to se
    if(flag_turn )
    {
      //Serial.println("turn");
      if(flag_left_mode){
        Turn_Left_Priority();
      }
      else{
        Turn_Right_Priority();
      }
       simplify_path();
    }
    // Keep following the line until any turn or backward detected [turn isn't visible here since it is called by the interrupts]
    linefollow();
    GetSensorData();

    // If all sensors are on black [It should be a U-turn] [A higher threshold to avoid fake U-turns]
    if(sensorValues[0] > 2500 && sensorValues[1] > 2500 && sensorValues[2] > 2500 && sensorValues[3] > 2500 && sensorValues[4] > 2500 && sensorValues[5] > 2500){
      delay(50);
      // Re checking to be completely sure that the U-turn truely exists 
      GetSensorData();
      if(sensorValues[0] > 2500 && sensorValues[1] > 2500 && sensorValues[2] > 2500 && sensorValues[3] > 2500 && sensorValues[4] > 2500 && sensorValues[5] > 2500)
      {
      //Serial.println("back");
      // Once sure, take the U-turn
      Backward();
      path[path_length]='B';
      path_length++;
      }
    } 
  }
}

// Snippet for taking turns in LHR mode
void Turn_Left_Priority(){
  // Declare the flags for each turn and initialize them to zero
  int left_available = 0;
  int forward_available = 0;
  int right_available = 0;
  // Degenerative braking to control the overshoot
  motor1run(-255);
  motor2run(-255);
  delay(10);
  // This exists twice since even with the degenerative braking, the bot still overshoots at times and hence we stop ahead of the turns. [a check at an appropriate delay ensures correct detection of each turn]
  //Check if Left turn exists
  if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    left_available = 1;
  }
  //Check if Right turn exists
  if(sensorValues[5] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    right_available = 1;
    //Serial.println("RGT");
  }
  delay(10); // braking takes place
  // A bit of a pause to shed the effects of the already gained momentum speeding up the forward push.
  motor1run(0);
  motor2run(0);
  delay(10);
  GetSensorData();
  // for(int i = 0; i < numChannels; i++){
  //   Serial.print(sensorValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");
   
  //Check if Left turn exists
  if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    left_available = 1;
  }
  //Check if Right turn exists
  if(sensorValues[5] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    right_available = 1;
    //Serial.println("RGT");
  }
  // Since it is left prioritized, if the bot goes at a turn in a slant direction, it might have detected a right turn but fails to detect a left. Hence just to be sure that the left exists, We are rechecking the left turn after a slight forward thrust.
   if(!left_available)
  {
    // A bit of a pause to shed the effects of the already gained momentum speeding up the forward push.
    motor1run(0);
    motor2run(0);
    delay(100);
    // The thrust is done with lsp and rsp [which is just opposite]. It is because as said the bot might come in a slant direction and this must be because the pid was governing it in that direction. Hence giving it a kind of reverse forward thrust will not only give it the forawrd thrust but also align it.
    motor1run(lsp);
    motor2run(rsp);
    delay(25);
    // motor1run(0);
    // motor2run(0);
    // delay(1000);
    // Finally rechecking if left exists
    GetSensorData();
    if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl))left_available = 1;
  }
  // A bit of a pause to shed the effects of the already gained momentum speeding up the forward push.
  motor1run(0);
  motor2run(0);
  delay(150);
  // Move forward enough to take turn and check forward
  motor1run(100);
  motor2run(100);
  delay(150);

  // Get sensor data to check for forward
  GetSensorData();
  // The condition for forward is kept so lenient since at times we saw the bot going slant and forward being missed 
  if((sensorValues[0]< thresholdl || sensorValues[1]< thresholdl || sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl || sensorValues[4]< thresholdl || sensorValues[5]< thresholdl)){
    forward_available = 1;
  }
  // If even after a forward thrust from a turn, all the sensors are white [It must be the end].
  if(sensorValues[0]<thresholdl && sensorValues[1]<thresholdl && sensorValues[2]<thresholdl && sensorValues[3]<thresholdl && sensorValues[4]<thresholdl && sensorValues[5]<thresholdl) //End
  {

    path[path_length]='E';
    path_length++;
    end=1;
    return;
  }

  // LEFT PRIORITIZED
  if(left_available){
  //  Serial.println("Left");
  // If forward does not exist take normal left
    if(sensorValues[0] > thresholdh && sensorValues[1] > thresholdh && sensorValues[2] > thresholdh && sensorValues[3] > thresholdh && sensorValues[4] > thresholdh && sensorValues[5] > thresholdh){
      Left();
    }
    // else take left with forward
    else{
      LeftWithForward();
    }
    path[path_length]='L';
    path_length++;
  }
  else if(forward_available){
    // Serial.println("Forward");
    path[path_length]='F';
    path_length++;
    
  }
  // Similarly for right
  else if(right_available){
    // Serial.println("Right");
    if(sensorValues[0] > thresholdh && sensorValues[1] > thresholdh && sensorValues[2] > thresholdh && sensorValues[3] > thresholdh && sensorValues[4] > thresholdh && sensorValues[5] > thresholdh){
      Right();
    }
    else{
      RightWithForward();
    }
    path[path_length]= 'R';
    path_length++;
  }
  flag_turn = 0;
  //digitalWrite(LED,LOW);
}

// Same as Turn_Left_Priority just that the priority is now given the Right.
void Turn_Right_Priority()
{
  // for(int i = 0; i < numChannels; i++){
  //   Serial.print(sensorValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");
  int left_available = 0;
  int forward_available = 0;
  int right_available = 0; 
  motor1run(-255);
  motor2run(-255);
  delay(10);
  //Check if Left turn exists
  if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    left_available = 1;
  }
  //Check if Right turn exists
  if(sensorValues[5] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    right_available = 1;
    //Serial.println("RGT");
  }
  delay(10);
  motor1run(0);
  motor2run(0);
  delay(10);
  GetSensorData();
  //Check if Left turn exists
  if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    left_available = 1;
  }
  //Check if Right turn exists
  if(sensorValues[5] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    right_available = 1;
    //Serial.println("RGT");
  }
   
   if(!right_available)
  {

    motor1run(0);
    motor2run(0);
    delay(100);
    motor1run(100);
    motor2run(100);
    delay(50);
    // motor1run(0);
    // motor2run(0);
    // delay(1000);
     GetSensorData();
    if(sensorValues[5] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
      right_available = 1;
    }
  }
  // Move forward enough to take turn and check forward
  motor1run(0);
  motor2run(0);
  delay(150);
  motor1run(100);
  motor2run(100);
  delay(150);
 

  // Get sensor data to check for forward
  GetSensorData();
  if(sensorValues[0]< thresholdl || sensorValues[1]< thresholdl || sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl || sensorValues[4]< thresholdl || sensorValues[5]< thresholdl){
    forward_available = 1;
  }

  if(sensorValues[0]<thresholdl && sensorValues[1]<thresholdl && sensorValues[2]<thresholdl && sensorValues[3]<thresholdl && sensorValues[4]<thresholdl && sensorValues[5]<thresholdl) //End
  {

    path[path_length]='E';
    path_length++;
    end=1;
    return;
  }

  // RIGHT PRIORITIZED
  if(right_available){
   // Serial.println("Left");
    if( sensorValues[1] > 2500 && sensorValues[2] > 2500 && sensorValues[3] > 2500 && sensorValues[4] > 2500 ){
      Right();
    }
    else{
      RightWithForward();
    }
    path[path_length]='R';
    path_length++;
  }
  else if(forward_available){
    //Serial.println("Forward");
    path[path_length]='F';
    path_length++;
    
  }
  else if(left_available){
    //Serial.println("Right");
    if(sensorValues[0] > 2500 && sensorValues[1] > 2500 && sensorValues[2] > 2500 && sensorValues[3] > 2500 && sensorValues[4] > 2500 && sensorValues[5] > 2500){
      Left();
    }
    else{
      LeftWithForward();
    }
    path[path_length]= 'L';
    path_length++;
  }
  flag_turn = 0;
  //digitalWrite(LED,LOW);
  
}

void Left(){
  // 1 sec stop for testing
  // motor1run(0);
  // motor2run(0);
  // delay(1000);
  //Glow CYAN colour for left turns
  digitalWrite(G, HIGH);
  digitalWrite(B, HIGH);
  //Keep rotating in this manner until all the while loops end
  motor1run(45);
  motor2run(-45);
  //1st'ly let the 3rd sensor [i.e. the center right sensor] go to black [i.e. keep rotating until it is on white] [This while loop will immediately end in case forward DNE]
  GetSensorData();
  while(sensorValues[3]<=threshold_avg){
    GetSensorData();
  }
  // Next let the 3rd sensor go to white [i.e. keep rotating until it is on black]
  GetSensorData();
  while(sensorValues[3]>threshold_avg){
    GetSensorData();
    for(int i = 0; i < numChannels; i++){
    // Serial.print(sensorValues[i]);
    // Serial.print(" ");
  }
  // Serial.println("");
  }
  //This should bring it almost on center
  //Then stop it for a while to get rid of the previous momentum
  motor1run(0);
  motor2run(0);
  delay(20);
  //This is to manage overshoot in the turn.
  //Rotate it in opposite manner
  motor1run(-45);
  motor2run(45);
  // Until the 2nd sensor gets back on white.
  while(sensorValues[2]>threshold_avg){
    GetSensorData();
  }
  //Once the turn is done.turn the rgb light off
  digitalWrite(G, LOW);
  digitalWrite(B, LOW);
}

void LeftWithForward(){
  // 1 sec stop for testing
  // motor1run(0);
  // motor2run(0);
  // delay(1000);

  //Glow CYAN colour for left turns
  digitalWrite(G, HIGH);
  digitalWrite(B, HIGH);
  //Keep rotating in this manner until all the while loops end
  motor1run(45);
  motor2run(-45);
  // The above logic of Left turn (in theory) works well in both the cases of forward existing and not but due to the bot reaching the turns slantly, [At times both the sensors 2 and 3 are on the opposite side of forward [i.e. both are on the right of forward]] in such a case, the left turn acts as a forward turn. Hence to get rid of the issue, this 2nd snippet of left turn exists.
  //Here we initiate the turn based on the rightmost sensor [since it is assured that the cornermost sensor shouldn't be incorrectly placed[if it is we are doomed anyways :(]]
  //Hence rotate until the rightmost sensor 1stly reaches white 
  GetSensorData();
  while(sensorValues[5]>threshold_avg){
    GetSensorData();
  }
  // Then until it reaches black
  while(sensorValues[5]<=threshold_avg){
    GetSensorData();
  }
  // This has assured that we are on the right side and reverting back to the previous logic, we bring he sensor 3 to white line
  GetSensorData();
  while(sensorValues[3]>threshold_avg){
    GetSensorData();
    // for(int i = 0; i < numChannels; i++){
    // Serial.print(sensorValues[i]);
    // Serial.print(" ");
  }
  //As explained above,this snippet handles the oevrshoot
  motor1run(0);
  motor2run(0);
  delay(20);
  motor1run(-45);
  motor2run(45);
  while(sensorValues[2]>threshold_avg){
    GetSensorData();
  }
  digitalWrite(G, LOW);
  digitalWrite(B, LOW);
  // Serial.println("");
}
// Same as that of left
void Right(){
  // 1 sec stop for testing
  // motor1run(0);
  // motor2run(0);
  // delay(1000);
  digitalWrite(G, HIGH);
  GetSensorData();
  motor1run(-45);
  motor2run(45);
  while(sensorValues[2]<=threshold_avg){
    GetSensorData();
  }

  GetSensorData();
  while(sensorValues[2]>threshold_avg){
    GetSensorData();
  }
  motor1run(0);
  motor2run(0);
  delay(20);
  motor1run(45);
  motor2run(-45);
  while(sensorValues[3]>threshold_avg){
    GetSensorData();
  }
  digitalWrite(G, LOW);
}

// Same as that of Left With Forward
void RightWithForward(){
  // 1 sec stop for testing
  // motor1run(0);
  // motor2run(0);
  // delay(1000);
  digitalWrite(G, HIGH);
  GetSensorData();
  motor1run(-45);
  motor2run(45);
  while(sensorValues[0]>threshold_avg){
    GetSensorData();
  }
  while(sensorValues[0]<=threshold_avg){
    GetSensorData();
  }

  GetSensorData();
  while(sensorValues[2]>threshold_avg){
    GetSensorData();
  }
  motor1run(0);
  motor2run(0);
  delay(20);
  motor1run(45);
  motor2run(-45);
  while(sensorValues[3]>threshold_avg){
    GetSensorData();
  }
  digitalWrite(G, LOW);
}

void Backward(){
  // 1 sec stop for testing
  // Degerative braking
  motor1run(-255);
  motor2run(-255);
  delay(20);
  //Getting rid of previous momentum
  motor1run(0);
  motor2run(0);
  delay(100);
  //Glow blue light for backward turns
  digitalWrite(B, HIGH);
  //same rsp, lsp logic to correctly align the bot
  motor1run(rsp);
  motor2run(lsp);
  delay(20);
  // then a forward thrust before actually taking the U-turn [In theory this is not required but doing this gives pid more time to correct any incorrect positioning of the bot]
  motor1run(100);
  motor2run(100);
  delay(80);
  //Finally take the U-turn [Just choosen to do it as a right turn [could have done it left as well]]
  GetSensorData();
  motor1run(-45);
  motor2run(45);
  // rotate until both the sensors have reached the center
  while(sensorValues[2]>threshold_avg && sensorValues[3]>threshold_avg){
    GetSensorData();
  }
  // In a similat manner, get rid of any overshoot that might have taken place
  motor1run(0);
  motor2run(0);
  delay(20);
  motor1run(45);
  motor2run(-45);
  while(sensorValues[3]>threshold_avg){
    GetSensorData();
  }
  //Although U-turn isn't a turn, but during the U-turn, one of the corner sensor crosses the white line, which calls the interrupt and sets the flag to 1. Since we know that this is a mistake, hence equating it back to zero
  flag_turn=0;
  //Turn off the rgb light
  digitalWrite(B, LOW);
}
//Simply analogRead's all the sensor's data
void GetSensorData(){
  for(int i = 0; i < numChannels; i++){
    sensorValues[i] = analogRead(sensorArray[i]);
    // Serial.print(sensorValues[i]);
    // Serial.print(" ");
  }
  // Serial.println("");
}
//Just get sensor data and based on it call PID()
void linefollow()
{
    GetSensorData();
    PID();
}
//This is the actual line PID logic
void PID()
{
  //Here (and above) all the quantities are defined a float since quantities such as 0.3, Kp, Kd (which are in decimal) are multiplied
  // The corner sensors aren't being used eventually since we want such a good PID which never deviates so much. [But it might be initially required it bot is swinging too much]
  // the weights of sensor 1 and 4 are kept less since they are on the brim of the white line [ the slightest of the deviation will significantly impact their values, hence calcualted the maximum erro (in our case 4000 (if one is on white and other is on black), then on ,multipling Kp and 0.3, we get a value around 123 (which isn't impulsive, but still good enough to get a good stable line following (obiously this requires trial and error))]
  // Kp needs to be kept small like 0.04, large values of Kp will make quickly take the error to it's maximum value and cause sharp deviatinos which will lead to overshoots and hence constant wigling.
  // Kd can be adjusted as required.
  // Ki might be used to get of the error that the sesnors have in themselves [like value of each sensor when tested, varies by 10- 30]
  float error = 0.0* sensorValues[0] + 0.3 * sensorValues[1] + sensorValues[2] - sensorValues[3] - 0.3 * sensorValues[4] - 0.0* sensorValues[5];
  // Set a basespeed from which the PIDvalue is either added or substracted
  int basespeed = 150;
  //Rest is the basic logic of PID line following
  P = error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Kd * D);
  previousError = error;
  
  lsp = basespeed - PIDvalue;
  rsp = basespeed + PIDvalue;
  
  if (lsp > max_speed) {
    lsp = max_speed;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > max_speed) {
    rsp = max_speed;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  //Finally moving motors with the speed calculated here
  motor1run(lsp);
  motor2run(rsp);
}


void motor1run(int motorSpeed) {
  // Making the function robust by elimating values (having magnitude) larger than 255
  motorSpeed = constrain(motorSpeed, -255, 255);
  //Move it forward if speed is +ve
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } 
  //Move it backward if speed is -ve
  else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } 
  // Else stop the motor
  else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  // Making the function robust by elimating values (having magnitude) larger than 255
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } 
  //Move it forward if speed is +ve
  else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } 
  //Move it backward if speed is -ve
  else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

// Interrupt function making the flag_turn = 1, whenever a turn is detected
void TurnAvailable(){
  flag_turn = 1;
  //digitalWrite(LED, HIGH);
}

// A holy snippet of code found on internet.
// The only thing we know is that it works :)
void simplify_path()
{
  // Path simplification. The strategy is that whenever we encounter a
  // sequence xBx, we can simplify it by cutting out the dead end.

  if (path_length < 3 || path[path_length - 2] != 'B')      // simplify the path only if the second-to-last turn was a 'B'
    return;
  int total_angle = 0;
  int m;
  // Get the angle as a number between 0 and 360 degrees.
  for (m = 1; m <= 3; m++)
  {
    switch (path[path_length - m])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
  // Replace all of those turns with a single one.
  total_angle = total_angle % 360;
  switch (total_angle)
  {
    case 0:
      path[path_length - 3] = 'F';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  path_length -= 2;
}

// Another interrupt function
void LefttModeActivator(){
  flag_left_mode = 1;
}
