#include <Servo.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <IMU.h> 

#define MAX_GAMMA 50
///VARIABLES SERVOMOTORES
#define MAX_SERVOS 12
#define MAX_PULSE 432
#define MIN_PULSE 188
Servo Servos[MAX_SERVOS];
cIMU    IMU;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMillis = 0;
const long interval = 20;
unsigned long loopTime;
unsigned long previousLooptime;
double t;

struct angles {
  double tetta;
  double alpha;
  double gamma;
};////vector
struct angles anglesFR;
struct angles anglesFL;
struct angles anglesBR;
struct angles anglesBL;
//VARIABLES PARA RECIVIR EL COMANDO
const byte numChars = 32;
char receivedChars[numChars];
int spaceCounter = 0;

boolean newData = false;

char a;
int pulse[12];
int batterieVoltage;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pwm.begin();
  pwm.setPWMFreq(50);
  IMU.begin();

  setPulse();
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  setPulse();
  moveServos(pulse);

  //IMUSetup();
  //connectServos(); //서보모터 I2C통신으로 연결 방법 다름


}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;
    t = float(currentMillis) / 1000;

    loopTime = currentMillis - previousLooptime;
    //Serial.print("<");
    //Serial.print(loopTime);
    //Serial.print("#");
    read_accel();
    read_axes(); 

    previousLooptime = currentMillis;

    recvWithStartEndMarkers();

    //batterieStatus();  // 배터리 표시 코드

    newData = false;
    moveServos(pulse);
  }
}

void setPulse() {         //set initialization
  for (int i = 0 ; i < 12; i++) {
    pulse[i] = 0;
  }
}

void moveServos(int *pulse) {

  for (int i = 0 ; i < 12; i++) {
    pwm.setPWM(i, 0, pulse[i]);
  }
}

void read_accel(){
 static uint32_t tTime[3];
  static uint32_t imu_time = 0;


  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];

  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

   //Serial.print(imu_time);
    Serial.print("<");
    Serial.print(IMU.accRaw[0]);    // ACC X
    Serial.print("#");
    Serial.print(IMU.accRaw[1]);    // ACC Y
    Serial.print("#");
    //Serial.print(IMU.accRaw[2]);    // ACC Z
    //Serial.println(">");


   }
}

void read_axes()
{
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;


  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();
    }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];

  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

  //Serial.print(imu_time);
   // Serial.print("<");
    Serial.print(IMU.rpy[0]);       //roll
    Serial.print("#");
    Serial.print(IMU.rpy[1]);       //pitch
    //Serial.print("#");
    //Serial.println(IMU.rpy[2]);     //yaw
    Serial.print(">");
  }


 }


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char spaceMarker = '#';

  char rc;

  while (Serial.available() > 0 && newData == false) {
    int angle;
    rc = Serial.read();
    //Serial.println("serial In");
    if (recvInProgress == true) {
      if (rc != endMarker && rc != spaceMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else if (rc == spaceMarker ) {
        receivedChars[ndx] = '\0';

        if (spaceCounter == 0) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[0]= constrain(map(angle, -60, 60, 220,414),220,414);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 1) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[1]= constrain(map(angle, 90, -90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 2) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[2]= constrain(map(angle, 90, -90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 3) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[3]= constrain(map(angle, -60, 60, 230,410),230,410);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 4) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[4]= constrain(map(angle, -90, 90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 5) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[5]= constrain(map(angle, -90, 90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 6) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[6]= constrain(map(angle, -60,60,208,436),208,436);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 7) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[7]= constrain(map(angle, 90, -90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 8) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[8]= constrain(map(angle, 90, -90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 9) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[9]= constrain(map(angle, -60,60,230,410),230,410);
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 10) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          if(angle>=0){
          pulse[10] = constrain(map(angle,0,90,312,420),312,420);      //HS-M7990TH mid 316   // moter10
             }
           else{
          pulse[10] = constrain(map(angle,-90,0,180,312),180,312);
             }
          spaceCounter++;
          ndx = 0;
        }
        else if (spaceCounter == 11) {
          //Serial.println(receivedChars);
          angle = atoi(receivedChars);
          pulse[11]= constrain(map(angle, -90, 90, 184,430),184,430);
          spaceCounter++;
          ndx = 0;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        //Serial.println(receivedChars);
        angle = atoi(receivedChars);
        recvInProgress = false;
        ndx = 0;
        spaceCounter = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}


void batterieStatus() {
  batterieVoltage = analogRead(A1);

  if (batterieVoltage >= 164) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(9, HIGH);
  }
  else if (batterieVoltage >= 154) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
  }
  else if (batterieVoltage >= 144) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
  }
  else if (batterieVoltage >= 134) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
  }
  else if (batterieVoltage >= 124) {
    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
  }
  else {
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
  }
}
