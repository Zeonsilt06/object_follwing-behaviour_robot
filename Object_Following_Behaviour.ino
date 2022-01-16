//Pin for Motor
#define PWMA 6
#define PWMB 9
#define AIN2 7
#define AIN1 8
#define STBY 5
#define BIN1 10
#define BIN2 11

//Pin for Button
#define BUTTON_A 4
#define BUTTON_B 3

//pin for Ultrasonic Sensor
#define frontUS_echo 13
#define frontUS_trig 12
#define rightUS_echo 2
#define rightUS_trig 1

//==== Paramater for handling button A debounce ====//
const int DEBOUNCE_DELAY = 50; // the debounce time; increase if the output flickers
int lastSteadyState = LOW;       // the previous steady state from the input pin
int lastFlickerableState = LOW;  // the previous flickerable state from the input pin
int currentState;                // the current reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
bool buttonIsPressed = false; //flag for trigger when button is pressed

//==== Paramater for handling button B debounce ====//
const int DEBOUNCE_DELAY_B = 50; // the debounce time; increase if the output flickers
int lastSteadyState_B = LOW;       // the previous steady state from the input pin
int lastFlickerableState_B = LOW;  // the previous flickerable state from the input pin
int currentState_B;                // the current reading from the input pin
unsigned long lastDebounceTime_B = 0;  // the last time the output pin was toggled
bool buttonIsPressed_B = false; //flag for trigger when button is pressed

//=== Parameter for handling motors ===//
int motorBaseSpeed = 250; //base speed of both motors
int motorMinSpeed = 30; // minimum motors speed
int MOTOR_FACTOR = motorBaseSpeed /100; //motor factor to reduce speed
const int L_MOTOR_FACTOR = 1;
const int R_MOTOR_FACTOR = 1;
const int L_MOTOR_THRESHOLD = 80;
const int R_MOTOR_THRESHOLD = 80;

//=== Parameter for handling Ultrasonic Sensor ===//
long duration_Front, distance_Front;
long duration_Right, distance_Right;
unsigned long UScm;
unsigned long USpm;
unsigned long USperiod = 50;
const long MAX_DISTANCE = 100; //max measuring distance
const long STOP_DISTANCE = 5; //stop distance before crash
long DISTANCE_FACTOR = MAX_DISTANCE/100; //distance factor 

void setup() {
  pinMode(frontUS_echo, INPUT);
  pinMode(frontUS_trig, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  digitalWrite(STBY, HIGH);
  Serial.begin(9600);
}

void loop() {
  
  US_handler();
  Button_Handler();
  if (buttonIsPressed == true) Run_Forward(); //toggle to run
  else if (buttonIsPressed == false) Stop_Motor(); //toggle to stop
  
}

void Button_Handler(){
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_A);
   // If the switch/button changed, due to noise or pressing:
  if (currentState != lastFlickerableState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the the last flickerable state
    lastFlickerableState = currentState;
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (lastSteadyState == HIGH && currentState == LOW){
      Serial.println("Motor Running");
      buttonIsPressed = true;
    }
    // save the the last steady state
    lastSteadyState = currentState;
  }
    // read the state of the switch/button:
  currentState_B = digitalRead(BUTTON_B);
   // If the switch/button changed, due to noise or pressing:
  if (currentState_B != lastFlickerableState_B) {
    // reset the debouncing timer
    lastDebounceTime_B = millis();
    // save the the last flickerable state
    lastFlickerableState_B = currentState_B;
  }
  if ((millis() - lastDebounceTime_B) > DEBOUNCE_DELAY_B) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (lastSteadyState_B == HIGH && currentState_B == LOW){
      Serial.println("Motor Stopped");
      buttonIsPressed = false;
    }
    // save the the last steady state
    lastSteadyState_B = currentState_B;
  }
}

//function to handle motors
void Run_Forward(){
  int leftSpeed = motorBaseSpeed;
  int rightSpeed = motorBaseSpeed;

  //if the robot get close to object in frot of it, the motor will slow down according to magnitude bellow
  if (distance_Front <= MAX_DISTANCE){
    long magnitude = (long)(MAX_DISTANCE - distance_Front)/DISTANCE_FACTOR;
    leftSpeed = motorBaseSpeed - (magnitude*MOTOR_FACTOR);
    rightSpeed = motorBaseSpeed - (magnitude*MOTOR_FACTOR);
  }

  if(leftSpeed <= L_MOTOR_THRESHOLD){
    leftSpeed *= L_MOTOR_FACTOR;
  }
  if(rightSpeed <= R_MOTOR_THRESHOLD){
    rightSpeed *= R_MOTOR_FACTOR;
  }

  //if the speed is very close to minimum speed, motor will slow down to minimum speed, indicating the robot is very close to stop distance
  if(leftSpeed < motorMinSpeed)leftSpeed = motorMinSpeed;
  if(rightSpeed < motorMinSpeed)rightSpeed = motorMinSpeed;
  
  //stop when the robot in range of stop distance
  if(distance_Front <= STOP_DISTANCE)leftSpeed = 0;
  if(distance_Front <= STOP_DISTANCE)rightSpeed = 0;
  
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

//function to stop motors
void Stop_Motor(){
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

//function to measure distance using Ultrasonic sensor
void US_handler(){
  //non-blocking approach code
  UScm = millis();
  if(UScm > USpm + USperiod){
    //trigger pin procedure to emit Ultrsound wave
    digitalWrite(frontUS_trig, LOW);
    digitalWrite(rightUS_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(frontUS_trig, HIGH);
    digitalWrite(rightUS_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(frontUS_trig, LOW);
    digitalWrite(rightUS_trig, LOW);

    //calculate time and distance based on pulse that came back
    duration_Front = pulseIn(frontUS_echo,HIGH,38000);
    distance_Front = (duration_Front*0.034/2);
    duration_Right = pulseIn(rightUS_echo,HIGH,38000);
    distance_Right = (duration_Right*0.034/2);
    if (distance_Front > MAX_DISTANCE) distance_Front = MAX_DISTANCE;
    if (distance_Right > MAX_DISTANCE) distance_Right = MAX_DISTANCE;
  
    //print distance to debug
//    Serial.print("Object in front = ");
//    Serial.print(distance_Front);
//    Serial.print(" cm");
//    Serial.print("  |   Object in right = ");
//    Serial.print(distance_Right);
//    Serial.print(" cm");
//    Serial.println( );

    //update current miilis of US
    USpm = UScm;
  } 
}
