#include <Arduino.h>
// #include "Servo.h"

// Servo servo;

#define INA1 16
#define INB1 9
#define INA2 11
#define INB2 10
#define INA3 5
#define INB3 12
#define PWM1 4
#define PWM2 7
#define PWM3 6


void setup() {
  pinMode(INA1, 1);
  pinMode(INB1, 1);
  pinMode(PWM1, 1);
  pinMode(INA2, 1);
  pinMode(INB2, 1);
  pinMode(PWM2, 1);

  Serial3.begin(115200);
  Serial.begin(115200);
//   servo.attach(8);
}


void motor(int motor, int vel) {
  int in1 = 0, in2 = 0, pwmPin = 0;
  /*int speed1 = 0;
  speed1 = map(vel, -255, 255, 0, 255);*/
  if (motor == 1) {
    in1 = INA1;
    in2 = INB1;
    pwmPin = PWM1;
  }
  if (motor == 2) {
    in1 = INA2;
    in2 = INB2;
    pwmPin = PWM2;
  }
  if (motor == 3) {
    in1 = INA3;
    in2 = INB3;
    pwmPin = PWM3;
  }

  ///

  digitalWrite(in1, !(vel >= 0));
  digitalWrite(in2, (vel > 0));

  vel = abs(vel);
  analogWrite(pwmPin, min(vel, 255));
}   


double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int arr[] = { 0, 0, 0 };  //относится к функции ниже. А сама функция возвращает лишь ссылку на него

int* procesVs(int x, int y) {  //рассчёт НЕ конечных скоростей
  int v1 = 0;
  int v2 = 0;
  int v3 = 0;

  double alpha = 0;                         //потом задаётся
  double beta = 0;                          //120* - alpha
  int Vs = (dist(x, y, 0, 0) / 255) * 255;  //функция задания скорости порпорционально отклонению 
  if (Vs > 255) {
    Vs = 255;
  }
  int polozh;
  double anglePos = atan2(y, x);
  if (anglePos >= (-PI / 6) && anglePos <= (PI / 2)) {             // верх право
    polozh = 1;
    alpha = atan2(y, x) + (PI / 6);
  } else if (anglePos >= (-5 * PI / 6) && anglePos < (-PI / 6)) {  //низ
    polozh = 2;
    alpha = atan2(y, x) + (5 * PI / 6);
  } else if (anglePos > (PI / 2) && anglePos <= PI) {              //верх лево верх
    polozh = 3;
    alpha = atan2(y, x) - (PI / 2);
  } else {                                                         //верх лево низ
    polozh = 4;
    alpha = abs(-PI - atan2(y, x)) + (PI / 2);
  }
  beta = (TWO_PI / 3) - alpha;
  int Valpha = alpha / (TWO_PI / 3) * Vs;
  int Vbeta = beta / (TWO_PI / 3) * Vs;
  if (polozh == 1) {
    v3 = Vbeta;
    v1 = Valpha;
  } else if (polozh == 2) {
    v2 = Vbeta;
    v3 = Valpha;
  } else if (polozh == 3) {
    v1 = Vbeta;
    v2 = Valpha;
  } else {  //4 сектор, то есть polozh = 4;

    v1 = Vbeta;
    v2 = Valpha;
  }
  arr[0] = v1;
  arr[1] = v2;
  arr[2] = v3;
  return arr;
}

int vel1 = 0;
int vel2 = 0;
int vel3 = 0;

void findVs(int x, int y) {  //x, y -- координаты 
  int v1 = 0;
  int v2 = 0;
  int v3 = 0;
  int* Vplus = procesVs(x, y);  //скорости из реального положения 
  v1 += Vplus[0];
  v2 += Vplus[1];
  v3 += Vplus[2];
  int* Vminus = procesVs(-x, -y);  //скорости из противоположной точки, относительно реального положения 
  v1 -= Vminus[0];
  v2 -= Vminus[1];   
  v3 -= Vminus[2];
  
  //Serial.println(atan2(y, x));
  vel1 = v1;
  vel2 = v2;
  vel3 = v3;
  Serial.print(vel1);
  Serial.print("\t");
  Serial.print(-vel2);
  Serial.print("\t");
  Serial.print(-vel3);
  Serial.println("\t");
  motor(2, vel1);
  motor(3, -vel2);
  motor(1, vel3);
  // motor(2, 200);
  // motor(3, -200);
  // motor(1, 200);
  
  delay(1);
  // motor(1, 255);
  // motor(3, 255);
  // motor(2, 0);
}


void manipulator(byte x1, byte y1) {
  // Serial.print(x1);
  // Serial.print("\t");
  // Serial.println(y1);
  int x = map(x1, 0, 255, -255, 255);
  int y = map(y1, 0, 255, -255, 255);

  if (abs(x) > 80 || abs(y) > 80) {

    findVs(x, y);
    
  }

  else {
    motor(1, 0);
    motor(2, 0);
    motor(3, 0);
  }
  // Serial.println(vel1);
}


int v_vpered = 40;
int v_angle = 70;
float kp = 2;
float kd = 4;
static int volatile errOld = 0;

void azimut(int zahvat_x, int res_x) {
    
    int e = res_x - zahvat_x;
    int u = int(e*kp + (e - errOld)* kd);
    errOld = e;
    motor(1, v_vpered);
    motor(2, v_angle + u);
    motor(3, v_angle - u);
}

void loop() {
    int zahvat = 0;
    int res = 0;
    int i1 = 0;
    int i2 = 0;
    if (Serial3.available()) {
        i1 = Serial3.read();
        // Serial.println(tg);
    }
    if (Serial3.available()) {
        i2 = Serial3.read();
    }
    if (i1 != 0 && i2 != 0) { 
        if (i1%2 == 0) {
            res = i1;
            zahvat = i2;
        }
        else {
            res = i2;
            zahvat = i1;
        }
    }
    if (res == 0 || zahvat != 0) { 
        Serial.print(res);
        Serial.print("\t");
        Serial.println(zahvat);
    }
    
    
}