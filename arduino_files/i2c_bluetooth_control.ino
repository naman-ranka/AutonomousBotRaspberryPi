#include <Wire.h>
 
// LED on pin 13
const int ledPin = 13; 
char zero = 0;
boolean X = false;
boolean Y = false;
int X_val = 0;
int Y_val = 0;

const int enA =11;
const int in1 = 9;       // LEFT SIDE MOTOR
const int in2 =10;

const int enB = 6;
const int in3 = 7;       //RIGHT SIDE MOTOR
const int in4 =8;
int motorSpeedA = 0;
int motorSpeedB = 0;

int z ;
 
void setup() {
  // Join I2C bus as slave with address 8
  Wire.begin(0x8);
  Serial.begin(9600);
  // Call receiveEvent when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}
 
// Function that executes whenever data is received from master
void receiveEvent(int howMany) {
  
  int numOfBytes = Wire.available();
  Serial.println();
  Serial.println(numOfBytes);
  byte b = Wire.read(); 
  Serial.println();
  


  for(int i=0; i<numOfBytes-1; i++){
    
    char data = Wire.read();
    //int new1 = data - '0';
    Serial.print(data);
    if (data == 'X'){
      X = true;
      //Serial.print("inx");
      Y = false;
      z = 1;
      X_val = 0;   
      continue;
         
    }
    if (data == 'Y'){
      X = false;
      // Serial.print("outofx");
      Y = true;
      z = 1;
      Y_val = 0;
      continue;
    }
    if (X){
      
      X_val = X_val*10+ (data - '0');
      
    }
    if (Y){
      
      Y_val = Y_val*10+ (data - '0');
      
    }
  
  
    /*
    int data = Wire.read();
    Serial.print(data);
    */
    
    }
    Serial.println();
  Serial.print("X_val:");
  Serial.println(X_val);
  Serial.print("Y_val:");
  Serial.println(Y_val);
  Serial.println();


 int xAxis = X_val; // Read Joysticks X-axis
 int yAxis = -(Y_val-1023); // Read Joysticks Y-axis

 // Y-axis used for forward and backward control
 if (yAxis < 470) {
   // Set Motor A backward
   digitalWrite(in1, HIGH);
   digitalWrite(in2, LOW);
   // Set Motor B backward
   digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);
   // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
   motorSpeedA = map(yAxis, 470, 0, 0, 255);
   motorSpeedB = map(yAxis, 470, 0, 0, 255);
 }
 else if (yAxis > 550) {
   // Set Motor A forward
   digitalWrite(in1, LOW);
   digitalWrite(in2, HIGH);
   // Set Motor B forward
   digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH);
   // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
   motorSpeedA = map(yAxis, 550, 1023, 0, 255);
   motorSpeedB = map(yAxis, 550, 1023, 0, 255);
 }
 // If joystick stays in middle the motors are not moving
 else {
   motorSpeedA = 0;
   motorSpeedB = 0;
 }

 // X-axis used for left and right control
 if (xAxis < 470) {
   // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
   int xMapped = map(xAxis, 470, 0, 0, 255);
   // Move to left - decrease left motor speed, increase right motor speed
   motorSpeedA = motorSpeedA - xMapped;
   motorSpeedB = motorSpeedB + xMapped;
   // Confine the range from 0 to 255
   if (motorSpeedA < 0) {
     motorSpeedA = 0;
   }
   if (motorSpeedB > 255) {
     motorSpeedB = 255;
   }
 }
 if (xAxis > 550) {
   // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
   int xMapped = map(xAxis, 550, 1023, 0, 255);
   // Move right - decrease right motor speed, increase left motor speed
   motorSpeedA = motorSpeedA + xMapped;
   motorSpeedB = motorSpeedB - xMapped;
   // Confine the range from 0 to 255
   if (motorSpeedA > 255) {
     motorSpeedA = 255;
   }
   if (motorSpeedB < 0) {
     motorSpeedB = 0;
   }
 }
 // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
 if (motorSpeedA < 70) {
   motorSpeedA = 0;
 }
 if (motorSpeedB < 70) {
   motorSpeedB = 0;
 }
 analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
 analogWrite(enB, motorSpeedB); // Send PWM signal to motor B

Serial.println("Finish");

  
  /*
  Serial.println("Start");
  while (Wire.available()) { // loop through all but the last
    byte c = Wire.read(); // receive byte as a character
    //digitalWrite(ledPin, c);
    Serial.print(c);
  }
  Serial.println("Next");
*/
}
void loop() {
  delay(1);
}
