    #include <Wire.h>    

const int EnableL =11;
const int HighL = 9;       // LEFT SIDE MOTOR
const int LowL =10;

const int EnableR = 6;
const int HighR = 7;       //RIGHT SIDE MOTOR
const int LowR =8;

int data,c;

void setup() {
Wire.begin(0x8);
Serial.begin(9600);
Wire.onReceive(receiveEvent);

pinMode(EnableL, OUTPUT);      
pinMode(HighL, OUTPUT);
pinMode(LowL, OUTPUT);

pinMode(EnableR, OUTPUT);
pinMode(HighR, OUTPUT);
pinMode(LowR, OUTPUT);


}

void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    c = Wire.read(); // receive byte as a character
    
    if (c<250){
       data = (128-c)/2;
    }
    else{
      data = c;
      }
    Serial.println(c);
    
    
    /*
    if (data < 60 and data > 10)
        data = data;

    if (data >-64 and data < 10)
      data = data;
      */
  }
}

void Stop1()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  Serial.println("Stop");
  delay(5000);  
 
 
  
}

void Stop()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  
  
}

void Move()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  int l = 128-data*4;
  if (l>=255){
    l = 255;
  }
   if (l<=0){
    l = 0;
  }
  analogWrite(EnableL,l);
  Serial.print(l);
  Serial.println(" :EnaL");
  
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  int r = 128+(data*4)-1;
  if (r>=255){
    r = 255;
  }
   if (r<=0){
    r = 0;
  }
  analogWrite(EnableR,r);
  
  Serial.print(r);
  Serial.println(" :EnaR");
  
}

void UturnR()
{

 Stop();
  delay(1500);
  //Right
  
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,160);  
  delay(550);

  
  //Stop
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  delay(1000);

  
//Forward
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,130);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,130);


  delay(400);
  //Stop
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);


  
  delay(1000);
  
 //Right
  
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,160);  
  delay(550);

  
  //Stop
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  delay(1000);
}

void right_shift()
{
  Stop();

  delay(1500);

  //right
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,160); 

  
  delay(350);

  Stop();

  delay(500);
  

  //Forward

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,130);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,130);
  
  delay(700);

  Stop();

  delay(500);

 

  

  //Left

  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,160);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255); 

  delay(300);


  
  Stop();

  delay(500);

}

void left_shift()
{
  Stop();

  delay(1500);

  //left
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,160);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255); 

  delay(350);

  Stop();

  delay(500);

  //Forward
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,130);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,130);

  delay(600);

  Stop();

  delay(500);

  //right
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,160);

  delay(350);

  Stop();

  delay(500);

}




void loop()
{
//Serial.println("c ");
//Serial.print(c);
//Serial.println(" data");
//Serial.print(data);

if (data==64)
{
 Stop();
}
else if (data==255)
{
 Stop1();
}
else if(data ==254)
{
  UturnR();
  }

else if(data == 251)
{
  
  right_shift();
  //rightshift
  }

  else if (data == 252)
  {
    
    left_shift();
    //leftshift
    }
  
else
{
 Move();
}
}
