#include <Arduino.h>
#include <ucode.h>
#include <QuickPID.h>
#include <Smoothed.h>
// #include <TimerInterrupt_Generic.h>
// #include <ISR_Timer_Generic.h>
// #define PROTOTYPE

#define ENABLE_PRINT

#define SERVO_LT 6
#define SERVO_LB 9
#define SERVO_RT 14
#define SERVO_RB 16

#define LRSMALLBIAS 1000
#define LRLARGEBIAS 500

#define CLOCKWISE 0
#define COUNTERCLOCKWISE 1

#define MAXTURNRPM 90
#define NORMALTURNRPM 70
#define MAXSTRAIGHTRPM 100
#define NORMALSTRAIGHTRPM 70

#define WHEEL_DIAMETER_cm 6
#define ONE_REVOLUTION_DISTANCE WHEEL_DIAMETER_cm *PI
#define ONE_RPM_IN_UCODE 4

#define USE_TIMER_1 true
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

// float Setpoint = 3500;
float Kp = 25.3;
float Ki = 0;
float Kd = 18;

bool plotEnable = false;

struct DISTANCE_UNIT
{
  float CM = 1;
  float INCH = 2.54;
  float MM = 0.1;
} DISTANCE_UNIT;

void FWD(int spd)
{
  setServoTurn(SERVO_LT, COUNTERCLOCKWISE, spd);
  setServoTurn(SERVO_LB, COUNTERCLOCKWISE, spd);
  setServoTurn(SERVO_RT, CLOCKWISE, spd);
  setServoTurn(SERVO_RB, CLOCKWISE, spd);
}

void BWD(int spd)
{
  setServoTurn(SERVO_LT, CLOCKWISE, spd);
  setServoTurn(SERVO_LB, CLOCKWISE, spd);
  setServoTurn(SERVO_RT, COUNTERCLOCKWISE, spd);
  setServoTurn(SERVO_RB, COUNTERCLOCKWISE, spd);
}

void LFT(int spd)
{
  setServoTurn(SERVO_LT, CLOCKWISE, spd);
  setServoTurn(SERVO_LB, CLOCKWISE, spd);
  setServoTurn(SERVO_RT, CLOCKWISE, spd);
  setServoTurn(SERVO_RB, CLOCKWISE, spd);
}

void RGT(int spd)
{
  setServoTurn(SERVO_LT, COUNTERCLOCKWISE, spd);
  setServoTurn(SERVO_LB, COUNTERCLOCKWISE, spd);
  setServoTurn(SERVO_RT, COUNTERCLOCKWISE, spd);
  setServoTurn(SERVO_RB, COUNTERCLOCKWISE, spd);
}

void STP()
{
  setServoStop(SERVO_LT);
  setServoStop(SERVO_LB);
  setServoStop(SERVO_RT);
  setServoStop(SERVO_RB);
}

void FWD_DISTANCE(int dist, float distance_multiplier, int RPM)
{
  RPM = constrain(RPM, 0, 60);
  float revolutions = (dist * distance_multiplier) / (ONE_REVOLUTION_DISTANCE); // all is in cm
  Serial.print("num revolutions:");
  Serial.println(revolutions);
  float runTime = (revolutions * 1000 / (RPM * 60));
  int speed = RPM * ONE_RPM_IN_UCODE;
  Serial.print("running at speed:");
  Serial.println(speed);
  Serial.print("With runtime:");
  Serial.println(runTime);
  FWD(speed);
  Serial.println("lmao");
  delay(runTime);
  STP();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.println("adasda");
  Initialization_Pruned();
  // Initialization();
  if (protocolRunState == false)
  {
  }
}

void loop()
{
  if (protocolRunState == false)
  {
#ifdef PROTOTYPE
    FWD_DISTANCE(10, DISTANCE_UNIT.CM, 100);
    delay(4000);
#endif

#ifndef PROTOTYPE
    int lasterror = 0;
    int P, I, D = 0;
    // LINE SENSOR VALUE, 1 MEANS ON LINE, 0 MEANS OFF LINE

    //  middle
    int linesens3 = readGrayValue(3, 0);
    // small lr
    int linesens2 = readGrayValue(2, 0);
    int linesens4 = readGrayValue(4, 0);
    // big lr
    int linesens1 = readGrayValue(1, 0);
    int linesens5 = readGrayValue(5, 0);

    // leftfavg.reading(linesens1 + linesens2);
    // rightavg.reading(linesens4 + linesens5);

    // int error = rightavg.getAvg() - leftfavg.getAvg(); // left decrements error, right increases

    bool leftbranch = linesens1 & linesens2;
    bool rightbranch = linesens4 & linesens5;

    int error = linesens4 * LRSMALLBIAS - linesens2 * LRSMALLBIAS;

    if (leftbranch & rightbranch)
    {
      LFT(80);
      Serial.println("middle");
    }
    else if (leftbranch)
    {
      LFT(80);
      Serial.println("lft");
      delay(2500);
      STP();
    }
    else if (rightbranch)
    {
      RGT(50);
      Serial.println("rgt");
      delay(2500);
      STP();
    }
    else
    {

      Serial.println("pid");

      P = error;

      I = I + error;

      D = error - lasterror;

      lasterror = error;

      int result = P * Kp + I * Ki + D * Kd;

      if (result > 0) // to right
      {
        LFT(constrain(result + NORMALTURNRPM, NORMALTURNRPM, MAXTURNRPM));
      }
      if (result < 0)
      {
        RGT(constrain(result + NORMALTURNRPM, NORMALTURNRPM, MAXTURNRPM));
      }
      if (result == 0)
      {
        FWD(MAXSTRAIGHTRPM);
      }
    }

#ifdef ENABLE_PRINT
    if (plotEnable)
    {
      Serial.print(">P:");
      Serial.println(P);
      Serial.print(">I:");
      Serial.println(I);
      Serial.print(">D:");
      Serial.println(D);
      Serial.print(">error:");
      Serial.println(error);
    }

    if (Serial.available())
    {
      STP();
      String incoming = Serial.readString();
      incoming.trim();
      char first = incoming.charAt(0);
      switch (first)
      {
      case 'p':
        Kp = incoming.substring(1).toFloat();
        Serial.print("Kp set to: ");
        Serial.println(Kp);
        break;
      case 'i':
        Ki = incoming.substring(1).toFloat();
        Serial.print("Ki set to: ");
        Serial.println(Ki);
        break;
      case 'd':
        Kd = incoming.substring(1).toFloat();
        Serial.print("Kd set to: ");
        Serial.println(Kd);
        break;
      case 'q':
        Serial.print("Kp set to: ");
        Serial.println(Kp);
        Serial.print("Ki set to: ");
        Serial.println(Ki);
        Serial.print("Kd set to: ");
        Serial.println(Kd);
        break;
      case 'm':
        plotEnable = !plotEnable;
        Serial.print("plotting: ");
        Serial.println(plotEnable);
        break;
      default:
        Serial.println("nuhuh");
        break;
      }
    }
#endif

#endif // PROTOTYPE
  }
}
