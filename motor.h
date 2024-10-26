//************************************************************//
//      MOTOR.H
//  Заголовочный файл модуля управления драйверами двигателей.
//  Библиотека для работы с контроллером "Рябина2.0"
//
//  Разработчик: Александр Сафонов
//************************************************************//

#pragma once


#include <stdint.h>

#include <Arduino.h>


class Motor
{
  private:
    uint8_t pinSideA;
    uint8_t pinSideB;
    uint8_t pinPWM;
    uint8_t pinEnableFaultA;
    uint8_t pinEnableFaultB;
    uint8_t pinCurrentSense;
    void setDirectSpeed(int16_t speed);

    int16_t curSpeed;
    int16_t newSpeed;
    int16_t deltaSpeedMin;
    unsigned long deltaTime;
    unsigned long prevTime;
    unsigned long prevTimeCur;
    unsigned int currentMax;
    float curr_kP ;

    Motor (Motor & );
    Motor & operator= (Motor & );

  public:
    enum lock_fault { FAULT_NONE = 0, FAULT_A, FAULT_B, FAULT_BOTH };

//Инициализация максимальных значений ток и  параметров плавного разгона. Можно не вызывать Ю при желанииЮ тогда будут дефолтные параметры 
    void init(int16_t deltaSpeedMin,  unsigned long deltaTime, unsigned int currentMax, float curr_kP )
    {
      this->deltaSpeedMin = deltaSpeedMin;
      this->deltaTime =  deltaTime;
      this->currentMax = currentMax;
      this->curr_kP = curr_kP;
    }

    explicit Motor (uint8_t number);

    void drive (int16_t speed);
    void brake (void);

    void work(void);

    unsigned int getCurrent_mA();
    lock_fault isFault (void);
};

extern Motor motor1;
extern Motor motor2;



