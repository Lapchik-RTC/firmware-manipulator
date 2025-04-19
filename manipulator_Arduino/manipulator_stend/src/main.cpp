#include <Arduino.h>
#include "Servo.h"

Servo servo;

#define INA1 11
#define INB1 10
#define INA2 5
#define INB2 12
#define INA3 9
#define INB3 16
#define PWM1 7
#define PWM2 6
#define PWM3 4
#define CS_PIN_1 A2
#define CS_PIN_2 A0
#define CS_PIN_3 A1

#define NACHALO 0
#define SHVATIL_BANKU 1
#define SDVIG_X 10
#define SDVIG_Y 20
// #define NACHALO 4

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
  delay(1);
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
  Serial.print(vel2);
  Serial.print("\t");
  Serial.print(vel3);
  Serial.println("\t");
    motor(1, vel1);
    motor(2, vel2);
    motor(3, vel3);
  // motor(2, 20);
  // motor(3, -20);
  // motor(1, -20);
  
  // delay(1);
  // motor(1, 255);
  // motor(3, 255);
  // motor(2, 0);
}

bool flaggg = 0;
void manipulator(int x1, int y1, bool vpered, bool nazad, bool hvatay) {
  // Serial.print(x1);
  // Serial.print("\t");
  // Serial.println(y1);
  int x = map(x1, -32768, 32767, -255, 255);
  int y = map(y1, -32768, 32767, -255, 255);
  if (abs(x) > 50 || abs(y) > 50) {

    findVs(-x, -y);
    
  }
  if (vpered) {
    motor(1, 180);
    motor(2, 200);
    motor(3, 200);
    delay(50);
  }
  if (nazad) {
    motor(1, -180);
    motor(2, -200);
    motor(3, -200);
    delay(50);
  }
  if (hvatay) {
    servo.write(135);
  }
  if (!hvatay) {
    servo.write(180);
  }

  if ((abs(x) < 50 && abs(y) < 50) && !vpered && !nazad && !hvatay) {
    motor(1, 0);
    motor(2, 0);
    motor(3, 0);
    delay(100);
  }
  // Serial.println(vel1);
}

struct Data
{ 
public:
  int16_t tag_x;    
  int16_t tag_y;
  int16_t zahvat_x;
  int16_t zahvat_y;
  int16_t pole_x;
  int16_t pole_y;
  bool CRC_Error;
};
Data data = {
  .tag_x      = 0,
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

bool flag = true;
int16_t tag_x_const = 0;
int16_t tag_y_const = 0;
int16_t pole_x_const = 0;
int16_t pole_y_const = 0;

static int pos = 0;

float filter(int cs) {  // Фильтрация нулевых значений и шумов
  int i = 0;
  bool flag_corrent = 0;
  int cspin = 0;
  int size = 10;
  float currentBuffer[size] = {0};
  int bufferIndex = 0;         
  float currentFiltered = 0;
  if(cs == 1) cspin = CS_PIN_1;
  if(cs == 2) cspin = CS_PIN_2;
  if(cs == 3) cspin = CS_PIN_3;
  // int rawValue = analogRead(cspin);
  // float voltage = (rawValue * 10.0) / 1023.0;
  // float current = voltage / 0.13;
  // float current = rawValue * 10;
  while(flag_corrent == 0) {
    float Value = (analogRead(cspin)*30) / (2.2*1023);
    if(Value > 0.01 && flag_corrent == 0) {
      currentBuffer[i] = Value;
      i++;
    }
    if (i == size - 1) {
      flag_corrent = 1;
      for (int j = 0; j < size; j++) {
        currentFiltered += currentBuffer[j];
      }
      currentFiltered = (currentFiltered / size)*100;
    }
  }
  return currentFiltered;
}

void printData(const Data& d) {
  Serial.print("  tag_x: "); Serial.print(d.tag_x);
  Serial.print("  tag_x_const: "); Serial.print(tag_x_const);
  Serial.print("  tag_y: "); Serial.print(tag_y_const);
  Serial.print("  zahvat_x: "); Serial.print(d.zahvat_x);
  Serial.print("  zahvat_y: "); Serial.print(d.zahvat_y);
  Serial.print("  pole_x: "); Serial.print(d.pole_x);
  Serial.print("  pole_y: "); Serial.print(d.pole_y);
  Serial.print("  CRC_Error: "); Serial.print(d.CRC_Error ? "true " : "false ");
  Serial.print("  teg_y_const: "); Serial.print(tag_y_const); 
  Serial.print("  pole_x_const: "); Serial.print(pole_x_const); 
  Serial.print("  u_x: "); Serial.println(filter(1));
}

void stop() {
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
  delay(100);
}

bool baze_values(Data& d) {
  if (d.tag_x != 0 && flag) {
    tag_x_const = d.tag_x;
    if (d.tag_y != 0 && flag) {
      tag_y_const = d.tag_y;
      if (d.pole_x != 0 && flag) {
        pole_x_const = d.pole_x;
        if (d.pole_y != 0 && flag) {
          pole_y_const = d.pole_y;
          flag = false;
          return true;
        }
      }
    }
  }
  return false;
}

int v_vpered = 90;
int v_vpered_levo = 140;
int err = 26;
int v_angle = 70;
float kp = 3;
float kd = 3;
static int volatile errOld = 0;
static int u = 0;

void azimut(int zahvat_x, int zahvat_y, int tag_x, int tag_y) {
    
    int e = tag_x - zahvat_x;
    u = int(e*kp + (e - errOld)* kd);
    u = constrain(u, 0, 100);
    errOld = e;
    // int v_vpered = v_angle - u;
    Serial.print(v_vpered - err);
    Serial.print("\t");
    Serial.print(v_vpered_levo + u);
    Serial.print("\t");
    Serial.print(v_vpered  - u);
    Serial.println("\t");
    // motor(1, v_vpered - err);
    // motor(2, v_vpered + u);
    // motor(3, v_vpered - u);
    motor(1, v_vpered - err);
    motor(2, v_vpered_levo + u);
    motor(3, v_vpered - u);
}

int condition = NACHALO;

float kp_x = 10;
float kd_x = 25;
static int volatile errOld_x = 0;
static int u_x = 0;

void  shag_x(int16_t x_zahvat, int16_t x_aim) {
  int e;
  if (condition == NACHALO) {
    if(x_zahvat > 160) {
      e = (x_aim - SDVIG_X) - x_zahvat;
    }
    else {
      e = (x_aim + SDVIG_X) - x_zahvat;
    }
      // u_x = int(e*kp_x + ((e - errOld_x)*kd_x));
      u_x = int(e*kp_x + (e - errOld_x)* kd_x);
      u_x = constrain(u_x, -80, 80);
      errOld_x = e;
  }
  else if (condition == SHVATIL_BANKU) {
    if(x_zahvat > 160) {
      e = (x_aim - SDVIG_X) - x_zahvat;
    }
    else {
      e = (x_aim + SDVIG_X) - x_zahvat;
    }
      // u_x = int(e*kp_x + ((e - errOld_x)*kd_x));
      u_x = int(e*kp_x + (e - errOld_x)* kd_x);
      u_x = constrain(u_x, -70, 70);
      errOld_x = e;
  }
  Serial.println(u_x);
  // Serial.print(0);
  // Serial.print("\t");
  // Serial.print(v_vpered_levo + u_x);
  // Serial.print("\t");
  // Serial.print(v_vpered - u_x);
  // Serial.println("\t");
  // motor(1, v_vpered - err);
  // motor(2, v_vpered + u);
  // motor(3, v_vpered - u);
                                      // if (u_x > 0) {
  // motor(1, 55);
  // motor(2, v_vpered_levo + u_x);
  // motor(3, v_vpered - u_x);
  motor(1, 21);
  motor(2, u_x);
  motor(3, -u_x);
  delay(130);
  motor(1, -21);
  motor(2, -u_x);
  motor(3, u_x);
  delay(10);
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
  delay(150);
  // }
  // else {
  // motor(1, 25);
  // motor(2, +u_x);
  // motor(3, -u_x);
  // delay(130);
  // motor(1, 0);
  // motor(2, 0);
  // motor(3, 0);
  // delay(270);
  // }
}


float kp_y = 2;
float kd_y = 3;
int volatile errOld_y = 0;
int u_y = 0;
int v_vpered_y = 50;
int max_v1 = 87;
int max_v2 = 115;

void shag_y(float y_zahvat, int16_t y_aim) {
  int v1_y;
  int v2_y;
  int v3_y;
  int k = 1;
  if(y_aim > int(y_zahvat)) k = -1;
                                                                                        //   y = 20 * (1 - (96 / azhvat_y))
  y_zahvat = constrain(y_zahvat, 50, 240);
  y_zahvat = y_zahvat - 80;
  v1_y = int(max_v1 * (y_zahvat/28));
  v2_y = int(max_v2 * (y_zahvat/30));
  v3_y = v2_y;

  v1_y = v1_y*k;
  v2_y = v2_y*k;
  v3_y = v3_y*k;
  Serial.print(v1_y);
  Serial.print("\t");
  Serial.print(v2_y);
  Serial.print("\t");
  Serial.print(v3_y);
  Serial.println("\t");
  motor(1, v1_y);
  motor(2, v2_y);
  motor(3, v3_y);
  delay(150);
  motor(1, -v1_y);
  motor(2, -v2_y);
  motor(3, -v3_y);
  delay(20);
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
  delay(150);
}

#define SERIAL1_RX_BUFFER_SIZE 256

struct GamePad {
    // Кнопки и D-Pad
    bool A, B, X, Y;
    bool DPad_Up, DPad_Down, DPad_Left, DPad_Right;
    
    // Стики
    int16_t LeftThumbX;
    int16_t LeftThumbY;
    int16_t RightThumbX;
    int16_t RightThumbY;
    
    // Дополнительные кнопки
    bool LeftThumbPress;
    bool RightThumbPress;
    bool LB;
    bool RB;
    bool LeftTrigger;
    bool RightTrigger;
    bool Start;
    bool Back;
    
    // Статус ошибки
    bool CRC_Error;
};

GamePad gamePad;

uint16_t crc16_ccitt(const uint8_t* gamePad, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)gamePad[i] << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

bool readPacket() {
    static uint8_t packet[13];
    static uint8_t index = 0;
    static unsigned long lastByteTime = 0;
    
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        // Serial.print("читаем");
        
        // Сброс при длительном простое
        if (millis() - lastByteTime > 50) index = 0;
        lastByteTime = millis();
        
        // Поиск стартового байта
        if (index == 0 && b != 0x0A) continue;
        
        packet[index++] = b;
        
        // Полный пакет
        if (index >= 13) {
            index = 0;
            // Serial.print(packet[1]);
            // Проверка CRC (байты 1-11)
            uint16_t receivedCrc = (packet[11] << 8) | packet[12];
            uint16_t calculatedCrc = crc16_ccitt(&packet[1], 10);
            
            if (receivedCrc != calculatedCrc) {
                gamePad.CRC_Error = true;
                return false;
            }
            
            // Распаковка данных
            gamePad.A          = packet[1] & 0x40;
            gamePad.B          = packet[1] & 0x80;
            gamePad.X          = packet[1] & 0x10;
            gamePad.Y          = packet[1] & 0x20;
            gamePad.DPad_Up    = packet[1] & 0x08;
            gamePad.DPad_Down  = packet[1] & 0x04;
            gamePad.DPad_Left  = packet[1] & 0x02;
            gamePad.DPad_Right = packet[1] & 0x01;
            
            gamePad.LeftThumbX  = (packet[2] << 8) | packet[3];
            gamePad.LeftThumbY  = (packet[4] << 8) | packet[5];
            gamePad.RightThumbX = (packet[6] << 8) | packet[7];
            gamePad.RightThumbY = (packet[8] << 8) | packet[9];
            
            gamePad.LeftThumbPress  = packet[10] & 0x80;
            gamePad.RightThumbPress = packet[10] & 0x40;
            gamePad.LB              = packet[10] & 0x20;
            gamePad.RB              = packet[10] & 0x10;
            // gamePad.LeftTrigger     = packet[10] & 0x08 ? 255 : 0;
            gamePad.LeftTrigger     = packet[10] & 0x08;
            gamePad.RightTrigger    = packet[10] & 0x04;
            gamePad.Start           = packet[10] & 0x02;
            gamePad.Back            = packet[10] & 0x01;
            
            gamePad.CRC_Error = false;
            return true;
        }
    }
    // Serial.print("NO");
    return false;
}

void printPacket() {
  // Serial.println("Valid packet:");
  if (readPacket()) {
    Serial.println("Valid packet:");
    Serial.print("LeftThumbX: "); Serial.print(gamePad.LeftThumbX);
    Serial.print(" LeftThumbY: "); Serial.print(gamePad.LeftThumbY);
    if(gamePad.LeftThumbPress) Serial.println("  Левый стик нажат! ");
    else Serial.println("  Левый НЕ стик нажат! ");

    Serial.print("RightThumbX: "); Serial.print(gamePad.RightThumbX);
    Serial.print(" RightThumbY: "); Serial.print(gamePad.RightThumbY);
    if(gamePad.RightThumbPress) Serial.println("  Правый стик нажат! ");
    else Serial.println("  Правый НЕ стик нажат! ");
    Serial.print("Buttons: ");
    if(gamePad.A) Serial.print("A ");
    if(gamePad.B) Serial.print("B ");
    if(gamePad.X) Serial.print("X ");
    if(gamePad.Y) Serial.print("Y; ");
    if(gamePad.DPad_Up) Serial.print("DPad_Up ");
    if(gamePad.DPad_Down) Serial.print("DPad_Down ");
    if(gamePad.DPad_Left) Serial.print("DPad_Left ");
    if(gamePad.DPad_Right) Serial.println("DPad_Right ");
  } else if(gamePad.CRC_Error) {
    Serial.println("CRC Error!");
  }
// delay(20);
}

uint64_t tim = 0;
uint64_t tim_old = 0;
bool state = 1, stButton = false;
// void readButton(bool knopka){
//   if(knopka){
//     // Serial.println("Button pressed");
//     if(!stButton) {
//       tim = millis();
//       if(tim - tim_old < 1000) {
//         state = 1;
//       }
//       else {
//         state = 0;
//       }
//       stButton = true;
//       tim_old = tim;
//     }
//   }
//   else if (stButton){
//       stButton = false;
//   }
//   delay(1000);
// }

void readButton(bool knopka){
  if(knopka){
    tim = millis();
    if(tim - tim_old < 3000) {
      state = 1;
    }
    else {
      state = 0;
    }
    tim_old = tim;
  }
  // else if (stButton){
  //     stButton = false;
  // }
  delay(3000);
}

uint64_t time_chek = 0;
void modeSelection() {
  time_chek = millis();
  while(time_chek > millis() - 3000) {
    readButton(gamePad.A);
    readPacket();
  }
  time_chek = millis();
} 

void setup() {
  pinMode(INA1, 1);
  pinMode(INB1, 1);
  pinMode(PWM1, 1);
  pinMode(INA2, 1);
  pinMode(INB2, 1);
  pinMode(PWM2, 1);
  pinMode(17, INPUT_PULLUP);
  pinMode(A0, INPUT);

  Serial3.begin(115200);
  Serial1.begin(19200);
  Serial.begin(115200);
  servo.attach(8);
  servo.write(110);
  delay(1000);

  memset(&gamePad, 0, sizeof(gamePad));
  // while(true);
  // while(!digitalRead(17));
  // while(digitalRead(17));
  // while(!digitalRead(17));
  // while(digitalRead(17));
}


uint64_t t1 = 0;
bool flag_2 = true;
bool s = 1;

// void loop() {
  // GamePad gamePad;
  // readPacket();
  // modeSelection();
  // Serial.println(state);
  // readButton(gamePad.A);
  // printPacket();
// }


void loop() {
  // while(1) {
  //   motor(1, -200);
  //   motor(2, -200);
  //   motor(3, -200);
  //   delay(100);
  // } 
  if(!s) {
    readPacket();
    printPacket();
    manipulator(gamePad.LeftThumbX, gamePad.LeftThumbY, gamePad.DPad_Up, gamePad.DPad_Down, gamePad.B);
    // modeSelection();
  }
  if (state) {
    pos = 2;
    Data d = readInt();
    printData(d);
    if(flag && state) {
      baze_values(d);
      d = readInt();
      Serial.println("vechno");
    }
    Serial.println("start");
    if (!d.CRC_Error && !flag && d.zahvat_x != 0  ) {
      if (pos == 0) {
        while (abs(tag_y_const - d.zahvat_y) > 50 && d.zahvat_y != 0 && flag_2) {
          if (abs(tag_y_const - d.zahvat_y) < 50) {
            flag_2 = false;
          }
          shag_y(d.zahvat_y, (tag_y_const));
          Serial.println("X");
          d = readInt();
          printData(d);
        }
        if (abs(tag_y_const - d.zahvat_y) > 15 && d.zahvat_y != 0) {
          shag_y(d.zahvat_y, (tag_y_const));
          Serial.println("Y_1");
          d = readInt();
          printData(d);
        }
        if (abs(tag_x_const - d.zahvat_x) > 10 && d.zahvat_x != 0) {
          shag_x(d.zahvat_x, (tag_x_const));
          Serial.println("X_1");
          d = readInt();
          printData(d);
        }
        if(abs(tag_y_const - d.zahvat_y) < 60 && d.zahvat_y != 0) {
          servo.write(180);
          Serial.println("servo OPEN");
          // motor(1, 80);
          // motor(2, 230);
          // motor(3, 125);
          // delay(3000);
          d = readInt();
          printData(d);
          shag_y(d.zahvat_y, (tag_y_const - 10));
        }
        d = readInt();
        printData(d);
        if (d.zahvat_y < (tag_y_const + 5) && d.zahvat_y != 0) {
          stop();
          motor(1, -200);
          motor(2, -200);
          motor(3, -200);
          delay(1000);
          servo.write(135);
          // motor(1, -150);
          // motor(2, 0);
          // motor(3, 0);
          // delay(600);
          Serial.println("servo CLOSE");
          stop();
          motor(1, -150);
          motor(2, -125);
          motor(3, -130);
          delay(3500);
          stop();
          motor(1, -200);
          motor(2, 180);
          motor(3, 150);
          delay(400);
          stop();
          // motor(1, 200);
          // motor(2, -150);
          // motor(3, -150);
          // delay(200);
          Serial.println("stop");
          d = readInt();
          printData(d);
          
          stop();
          // while (true)
          // {
          //   /* code */
          // }
          pos = 1;
          // motor(1, 40);
          // motor(2, -80);
          // motor(3, 120);
          // delay(300);
        }
      }
      else if (pos == 1) {
        d = readInt();
        printData(d);
        if (abs(d.zahvat_x - pole_x_const) > 18 && d.zahvat_x != 0) {
          shag_x(d.zahvat_x, (pole_x_const));
          Serial.println("X");

        }
        if ((abs(pole_y_const - d.zahvat_y) > 17 && d.zahvat_y != 0) || abs(d.zahvat_x - pole_x_const) > 22) {
          if (abs(pole_y_const - d.zahvat_y) > 10) { 
            shag_y(d.zahvat_y, pole_y_const);
            Serial.println("Y_1");
            // d = readInt();
            // printData(d);
          }
          // motor(1, 80);
            // motor(2, 230);
            // motor(3, 125);
            // delay(3000);
            // d = readInt();
            // printData(d);
          if (abs(d.zahvat_x - pole_x_const) > 20 && d.zahvat_x != 0) {
            shag_x(d.zahvat_x, (pole_x_const));
          }
        }
        else {
          servo.write(180);
          stop();
          Serial.println("servo OPEN");
          pos = 2;
          motor(1, -200);
          motor(2, -125);
          motor(3, -200);
          delay(3000);
          stop();
          // motor(1, -255);
          // motor(2, 0);
          // motor(3, 0);
          // delay(500);
          // stop();
          while(filter(1) < 512 && s) {
            d = readInt();
            printData(d);
            motor(1, -200);
            motor(2, -150);
            motor(3, -170);
            delay(6000);
            stop();
            s = false;
          }
          stop();
          servo.write(110);
        }
      }
    }
    // else {
    //   Serial.println("stopping");
    //   delay(100);
    // }
  }
  if (pos == 2) {
    
    modeSelection();
    Serial.println(state);
    pos = 0;
  }
}