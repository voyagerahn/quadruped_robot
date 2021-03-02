#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
}

void loop() {
  if (Serial.available()) {
   
    String ist = Serial.readStringUntil('\n');
    int b=0;
    
    int comma = ist.indexOf(','); //콤마(,)를 찾아 번호를 매긴다
    int len = ist.length(); //문자의길이를 측정한다

    int x = ist.substring(0,comma).toInt(); //첫번째문자부터 콤마전까지의 수를 정수로 변환
    int y = ist.substring(comma+1,len).toInt(); //콤마이후의 수를 정수로 변환

// motor 1=7=11, motor 2=8
//FR LEG_오른발을 기준으로 잡음
    int MOTOR1 = constrain(map(x,0,180,190,432),190,432);      //HS-M7990TH mid 311   // moter1
    int MOTOR2 = constrain(map(y,180,0,190,432),190,432);      //HS-M7990TH mid 308.5 // moter2 

//FL LEG_왼발의 순서는 반대로 했음 코딩에따라서 다시 0~180으로 매핑 다시하던가 하삼
    int MOTOR4 = constrain(map(x,180,0,177,416),170,418);      //HS-M7990TH mid 311   // moter4 
    int MOTOR5 = constrain(map(y,0,180,190,429),190,429);      //HS-M7990TH mid 306.5 // moter5 

//BR LEG
    int MOTOR7 = constrain(map(x,0,180,193,431),190,432);      //HS-M7990TH mid 311   // moter7
    int MOTOR8 = constrain(map(y,180,0,190,427),190,427);      //HS-M7990TH mid 308.5 // moter8 

int MOTOR10;
//BR LEG    
  if(x<=90){
    MOTOR10 = constrain(map(x,90,0,312,420),312,420);      //HS-M7990TH mid 316   // moter10 
  }
  else{
    MOTOR10 = constrain(map(x,180,90,180,312),180,312);
  }
    int MOTOR11 = constrain(map(y,0,180,190,429),190,429);     //HS-M7990TH mid 311   // moter11 

// HIP
    int FR_HIP = constrain(map(b,-60,60,210,404),210,404);     //mg-996r  mid 307     //motor 0
    int BR_HIP = constrain(map(b,-60,60,198,416),198,416);     //mg-996r  mid 307     //motor 6
    int LEFT_HIP = constrain(map(b,60,-60,210,390),210,390);   //mg-996r  mid 300     //motor 3, 9
 

    //LEG MOTOR PWM
    pwm.setPWM(1,0,MOTOR1);   //motor1
    pwm.setPWM(2,0,MOTOR2);   //motor2
    pwm.setPWM(4,0,MOTOR4);   //motor4
    pwm.setPWM(5,0,MOTOR5);   //motor5
    
    pwm.setPWM(7,0,MOTOR7);   //motor7
    pwm.setPWM(8,0,MOTOR8);   //motor8
    pwm.setPWM(10,0,MOTOR10); //motor10
    pwm.setPWM(11,0,MOTOR11);  //motor11
  
    // HIP MOTOR PWM
    pwm.setPWM(0,0,FR_HIP);   //motor0
    pwm.setPWM(3,0,LEFT_HIP); //motor3
    pwm.setPWM(6,0,BR_HIP);   //motor6
    pwm.setPWM(9,0,LEFT_HIP); //motor9

    Serial.print('(');
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.println(')');

    
    delay(10);
  }
}
