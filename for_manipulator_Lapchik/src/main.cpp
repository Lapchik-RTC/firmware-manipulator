#include <Arduino.h>
#include "Servo.h"

Servo servo;

#define INA1 13
#define INB1 22
#define INA2 16
#define INB2 15
#define INA3 17
#define INB3 12
#define PWM1 4//5
#define PWM2 5
#define PWM3 2


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
  digitalWrite(in2, (vel >= 0));

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
  // Serial.print(vel1);
  // Serial.print("\t");
  // Serial.print(vel2);
  // Serial.print("\t");
  // Serial.print(vel3);
  // Serial.println("\t");
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
void manipulator(int x1, int y1) {
  // Serial.print(x1);
  // Serial.print("\t");
  // Serial.println(y1);
  int x = map(x1, -32768, 32767, -255, 255);
  int y = map(y1, -32768, 32767, -255, 255);
  // if (abs(x) > 50 || abs(y) > 50) {

    findVs(-x, -y);
    
  // }
  // Serial.println(vel1);
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
    uint8_t LeftTrigger;
    uint8_t RightTrigger;
    bool Start;
    bool Back;
    
    // Статус ошибки
    bool CRC_Error;
};

GamePad gamePad;

uint16_t  crc16_ccitt(const uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  
  for (int i = 0; i < length; ++i) {
      crc ^= static_cast<uint16_t>(data[i]) << 8;
      
      for (int j = 0; j < 8; ++j) {
          crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
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
      
      // Сброс при длительном простое
      if (millis() - lastByteTime > 50) index = 0;
      lastByteTime = millis();
      
      // Поиск стартового байта
      if (index == 0 && b != 0x0A) continue;
      
      packet[index++] = b;
      
      // Полный пакет
      if (index >= 15) {
          index = 0;
          // packet[0] = 0x0A;
          // Проверка CRC (байты 1-11)
          // uint16_t receivedCrc = (packet[12] << 8) | packet[13];
          uint16_t receivedCrc = (packet[11] << 8) | packet[12];
          uint16_t calculatedCrc = crc16_ccitt(&packet[1], 10);  // 12 байт данных в пакете
          
          if (receivedCrc != calculatedCrc) {
            Serial.println(receivedCrc); 
            Serial.println(packet[13]);
            gamePad.CRC_Error = true;
            return false;
          }
          
          // if (receivedCrc != 0) {
          //   Serial.println(receivedCrc); 
          //   Serial.println();
          //   gamePad.CRC_Error = true;
          //   return false;
          // }
          
          // Serial.println(receivedCrc); 
          // Serial.println(calculatedCrc);
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

          gamePad.LeftTrigger     = (packet[11] & 0xFF);
          gamePad.RightTrigger    = (packet[12]  & 0xFF);
          
          gamePad.CRC_Error = false;
          return true;
      }
  }
  return false;
}

void printTrigger() {
  if (readPacket()) {
    Serial.println(int(gamePad.LeftTrigger));
  }
}

void printPacket() {
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
delay(20);
}

int min_LeftThumbX = 4000, min_LeftThumbY = 4000;

bool nado_rabotat() {
  if (!gamePad.A && !gamePad.B && !gamePad.DPad_Up && !gamePad.DPad_Down && !gamePad.DPad_Left && !gamePad.DPad_Right && (abs(gamePad.LeftThumbX) < min_LeftThumbX) && (abs(gamePad.LeftThumbY) < min_LeftThumbY)) {  
    return false;
  }
  else {
    return true;
  }
}

void setup() {
  pinMode(INA1, 1);
  pinMode(INB1, 1);
  pinMode(PWM1, 1);
  pinMode(INA2, 1);
  pinMode(INB2, 1);
  pinMode(PWM2, 1);
  pinMode(INA3, 1);
  pinMode(INB3, 1);
  pinMode(PWM3, 1);
  // pinMode(17, INPUT_PULLUP);

  // Serial3.begin(19200);
  Serial1.begin(19200);
  Serial.begin(19200);
  servo.attach(46);
  servo.write(90);
  delay(1000);

  memset(&gamePad, 0, sizeof(gamePad));
  // while(true);
  // while(!digitalRead(17));
  // while(digitalRead(17));
  // while(!digitalRead(17));
  // while(digitalRead(17));
}


uint64_t t1 = 0;
bool flag = 0;
int pos = 90;

//нужная версия кода
void loop() {
  // printPacket();
  // printTrigger();
  readPacket();
  // Serial.println("Loop");
  // if (Serial1.available() > 0) {
  //   Serial.println("Serial1");
  //   Serial.write(Serial1.read());
  //   Serial.println("Serial1");
  // }
  printPacket();
  // printTrigger();
  
  // printPacket();
  // readPacket();
  // manipulator(32000, 32000);
  // motor(1, 200);
  // motor(2, -200);
  // motor(3, -200);

  if (!nado_rabotat()) {
    motor(1, 0);
    motor(2, 0);
    motor(3, 0);
    delay(2);
  }
  else {
    if(gamePad.DPad_Right) {
      if (pos < 110) {
        pos += 8;
        servo.write(pos);

        delay(80);
      }
    }
    if(gamePad.DPad_Left) {
      if (pos > 40) {
        pos -= 8;
        servo.write(pos);
        delay(80);
      }
    }
    if(gamePad.DPad_Up) {
        delay(100);
      }
    }
    if(gamePad.DPad_Left) {
      if (pos > 25) {
        pos -= 8;
        servo.write(pos);
        delay(100);
      }
    }
    if(gamePad.DPad_Left) {
      if (pos > 25) {
        pos -= 8;
        servo.write(pos);
        delay(100);
      }
    }
    if(gamePad.DPad_Down) {
      motor(1, 200);
      motor(2, 250);
      motor(3, 250);
    }
    if(gamePad.DPad_Down) {

    if(gamePad.DPad_Up) {

      motor(1, -250);
      motor(2, -200);
      motor(3, -200);
    }
    if(abs(gamePad.LeftThumbX) > min_LeftThumbX || abs(gamePad.LeftThumbY) > min_LeftThumbY) {
      manipulator(gamePad.LeftThumbX, gamePad.LeftThumbY);
    }
  }
  // delay(500);
  // delay(5);
  // delay(500);
}