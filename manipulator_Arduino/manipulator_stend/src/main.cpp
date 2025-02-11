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


int v_vpered = 50;
int v_angle = 70;
float kp = 0.3;
float kd = 4;
static int volatile errOld = 0;
static int u = 0;

void azimut(int zahvat_x, int zahvat_y, int tag_x, int tag_y) {
    
    int e = tag_x - zahvat_x;
    u = int(e*kp + (e - errOld)* kd);
    u = constrain(u, 0, 30);
    errOld = e;
    // int v_vpered = v_angle - u;
    motor(1, v_vpered + u);
    motor(2, v_vpered - u - 15);
    motor(3, v_vpered - u - 25);
    // motor(1, -40);
    // motor(2, -40);
    // motor(3, -40);
}

struct Data
{ 
public:
  int16_t tag_x;    
  int16_t tag_x_old;    
  int16_t tag_y;
  int16_t zahvat_x;
  int16_t zahvat_y;
  int16_t pole_x;
  int16_t pole_y;
  bool CRC_Error;
};
Data data = {
  .tag_x      = 0,
  .tag_x_old  = 1000,
  .tag_y      = 0,
  .zahvat_x   = 0,
  .zahvat_y   = 0,
  .pole_x     = 0,
  .pole_y     = 0,
  .CRC_Error  = false
};

uint8_t crc8(uint8_t *pcBlock, unsigned int len)
{
    unsigned char crc = 0xFF;
    unsigned int i;

    while (len--)
    {
        crc ^= *pcBlock++;

        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

Data readInt()
{
  while(Serial3.read() != 0xFF);
  uint8_t res_arr[9] = {0};
  /*
  res_arr[0] = tag_lo;   
  res_arr[1] = tag_hi;   
  res_arr[2] = tag_y;
  res_arr[3] = zahvat_lo;
  res_arr[4] = zahvat_hi;
  res_arr[5] = zahvat_y;
  res_arr[6] = pole_lo;
  res_arr[7] = pole_hi;
  res_arr[8] = pole_y;
  */
  while(Serial3.available() < 10);
  for(int i = 0; i < 9; i++) {
    res_arr[i] = Serial3.read();
    // Serial.print("res_arr["); Serial.print(i); Serial.print("]: "); Serial.println(res_arr[i]);
  }
  uint8_t crc_in = Serial3.read();
  if (crc_in != crc8(res_arr, 9)) {
    // Serial.print(crc_in);
    // Serial.print("   ");
    // Serial.println(crc8(res_arr, 9));
    Serial.println("CRC error!!!");
    // delay(2000);
    data.CRC_Error = true;
    return data;
  }
  else {
    data.tag_x = (res_arr[1] << 8) | res_arr[0];
    data.tag_y = res_arr[2];
    data.zahvat_x = (res_arr[4] << 8) | res_arr[3];
    data.zahvat_y = res_arr[5];
    data.pole_x = (res_arr[7] << 8) | res_arr[6];
    data.pole_y = res_arr[8];
    data.CRC_Error = false;
    // Serial.println("CRC success!!!");
    return data;
  }
}

void testSerRead()
{
  Serial.println(Serial3.read());
}

void printData(const Data& d) {
  Serial.print("  tag_x: "); Serial.print(d.tag_x);
  Serial.print("  tag_x_old: "); Serial.print(d.tag_x_old);
  Serial.print("  tag_y: "); Serial.print(d.tag_y);
  Serial.print("  zahvat_x: "); Serial.print(d.zahvat_x);
  Serial.print("  zahvat_y: "); Serial.print(d.zahvat_y);
  Serial.print("  pole_x: "); Serial.print(d.pole_x);
  Serial.print("  pole_y: "); Serial.print(d.pole_y);
  Serial.print("  CRC_Error: "); Serial.print(d.CRC_Error ? "true " : "false ");
  Serial.print("  u: "); Serial.println(u);
}

void stop() {
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
}

void loop() {
    Data d = readInt();
    printData(d);
    if (true) {
      d.tag_x_old = d.tag_x;
    }
    if (!d.CRC_Error) {
      if (d.zahvat_x < d.tag_x_old)
      {
        d = readInt();  
        if (d.tag_x != 0) {
          d.tag_x_old = d.tag_x;
        }
        azimut(d.zahvat_x, d.zahvat_y, d.tag_x, d.tag_y);
        printData(d);
      }
      else {
        stop();
      }
      
    }
    // printData(d);
    delay(500);
    // testSerRead();
}