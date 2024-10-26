//************************************************************//
//      MOTOR.CPP
//  Файл-реализация модуля управления драйверами двигателей.
//  Библиотека для работы с контроллером "Рябина2.0"
//
//  Разработчик: Александр Сафонов
//************************************************************//

#include "motor.h"


#define GAIN_Imot_ADC 60
Motor motor1(1);
Motor motor2(2);


Motor::Motor (uint8_t number)
{
  switch (number)
  {
    case (1):
      pinSideA = 53;
      pinSideB = 49;
      pinPWM = 4;
      pinEnableFaultA = 47;
      pinEnableFaultB = 48;
      pinCurrentSense = A0;
      break;

    case (2):
      pinSideA = 26;
      pinSideB = 29;
      pinPWM = 13;
      pinEnableFaultA = 27;
      pinEnableFaultB = 28;
      pinCurrentSense = A1;
      break;
  }

  deltaSpeedMin = 2 ;
  deltaTime = 100 ;
  currentMax = 10000;
  curr_kP  = 0.1 ;
  curSpeed =0 ;
  newSpeed=0;
  prevTime = 0;
  prevTimeCur = 0;

  pinMode(pinSideA, OUTPUT);
  pinMode(pinSideB, OUTPUT);
  digitalWrite(pinSideA, LOW);
  digitalWrite(pinSideA, LOW);

  pinMode(pinPWM, OUTPUT);
  analogWrite(pinPWM, 0);

  pinMode(pinEnableFaultA, INPUT);
  pinMode(pinEnableFaultB, INPUT);
  pinMode(pinCurrentSense, INPUT);
}

void Motor::drive (int16_t velocity)
{
  newSpeed = velocity;

}

void Motor::brake (void)
{
  digitalWrite(pinSideA, HIGH);
  digitalWrite(pinSideB, HIGH);
  analogWrite(pinPWM, 0);
   curSpeed = 0 ;
   newSpeed=0; 
}

unsigned int Motor::getCurrent_mA()
{
  uint16_t valueADC = analogRead(pinCurrentSense);
  return 1000 *  valueADC / GAIN_Imot_ADC;
}

Motor::lock_fault Motor::isFault (void)
{
  uint8_t is_a = FAULT_A * (uint8_t)digitalRead(pinEnableFaultA);
  uint8_t is_b = FAULT_B * (uint8_t)digitalRead(pinEnableFaultB);

  return (lock_fault)(is_a + is_b);
}

void Motor::setDirectSpeed(int16_t velocity)
{


  if (velocity > -2 &&  velocity < 2 )
  {
    brake();
  }
 else  if (velocity > 2)
  {
    digitalWrite(pinSideA, HIGH);
    digitalWrite(pinSideB, LOW);
  }
  else if (velocity < -2)
  {
    digitalWrite(pinSideA, LOW);
    digitalWrite(pinSideB, HIGH);
  }
  velocity = abs( velocity);
  analogWrite(pinPWM, velocity);
}

void Motor::work()
{

 
  if (millis() - prevTimeCur > deltaTime * 2)
  {
    prevTimeCur = millis();

    if (this->getCurrent_mA() > currentMax)
    {
      if (newSpeed < 0)
      {
        newSpeed +=    ( this->getCurrent_mA() - currentMax)  * curr_kP;
      }
      else if (newSpeed > 0)
        newSpeed -=   (this->getCurrent_mA() - currentMax) * curr_kP;
    }

  }
    //Serial1.print(pinPWM);
      //Serial1.print(' ');
       //Serial1.println(getCurrent_mA());

  if (newSpeed == curSpeed)
    return;

  if (millis() - prevTime > deltaTime)
  {

    prevTime = millis();
 
    int16_t deltaSpeed = newSpeed - curSpeed;
    if (abs(deltaSpeed) < deltaSpeedMin )
    {
      curSpeed += deltaSpeed;
      setDirectSpeed(curSpeed);
      return;
    }
    if (deltaSpeed < 0)
    {
      curSpeed -= deltaSpeedMin;
    }
    else
    {
      curSpeed += deltaSpeedMin;
    }

    
    
    setDirectSpeed(curSpeed);
  
  }
}


