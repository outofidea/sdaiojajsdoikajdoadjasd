#ifndef UCODE_H
#define UCODE_H

#include <Arduino.h>
#include"SemiduplexSerial.h" 

//SupportedModulesDriveNeedsToBeIdddedHere
#include "FlexiTimer2.h"
#include "ArduinoJson.h"
#include"onBoardHW.h"
#include "NewTone.h"
#include"TransforRobot.h"
#include"uKitMotor.h"
#include"uKitSensor.h"
#include"uKitServo.h"
#include "ClickButton.h"
#include"uKitId.h"
#include "KalmanMPU6050.h"
#include "VisionDiscrim.h"
const char* versionNumber="v1.2.28";

bool flexiTimerFlag=true;
bool butonTimer=true;

String inputString = "";         //UsedToSaveReceipts
String strLoopCmd = "";          /*Loop data*/
bool blLoopFlag = false;         /*Judge whether to implement Loop data*/
uint8_t uiOnlineCheck = 0;
int iWIfiListSize = -1;
const char* snCode="";
bool newLineReceived = false; //TheEndOfThePreviousDataEndLog

char deciveSN[20]={0};



bool protocolRunState=true;
bool bWifiRun=false;
int timeTimes=0,timeFlag=0,buttonFlag=0;
OnBoardHW Sensor;
TransforRobot transfor_robot;
uKitMotor uKitMotor_inst;
uKitServo uKitServo_inst;
uKitSensor uKitSensor_inst;
uKitId uKitId_inst;
VisionDiscrim mVisionDiscrim;


#define SERIAL_READ_TIMEOUT 10  /*time out ms*/
//uKitServoApi

#define setServoTurn(id,dir,speed) uKitServo_inst.setServoTurn(id,dir,speed)//TheSteeringMachineWheelModeControl,IdIsTheRudderNumber,DirIsTheDirection (1Clockwise,0InverseTime),TheSpeedIsSpeed (05)
#define setServoAngles(ids, angles, times, length) uKitServo_inst.setServoAngles(ids, angles, times, length) // Control multiple steering gear modes, IDS is an array of the steering gear, Angles is an angle array (-118 ° ~ 118 °), TIMES is running time (300-5000)
#define setServoAnglesWait(ids, angles, times, length, wait) uKitServo_inst.setServoAnglesWait(ids, angles, times, length, wait) // Control multiple steering gear modes, IDS is an array of the steering gear, Angles is an angle array (-118 ° ~ 118 °), TIMES is running time (300-5000)
#define setServoAngle(id,angle,times) uKitServo_inst.setServoAngle(id,angle,times)// The steering gear mode, ID is the rudder number, Angle is the angle (-118 ° ~ 118 °), Times is running time (300-5000)
#define setServoStop(id) uKitServo_inst.setServoStop(id)//单个舵机停止函数
#define setServosStop(ids, length) uKitServo_inst.setServosStop(ids, length)//多个舵机停止函数
#define readServoAnglePD(id) uKitServo_inst.readServoAnglePD(id)// Read a single steering gear and return to the angle value of the steering gear (refund the power)
#define readServoOriginalAnglePD(id) uKitServo_inst.readServoOriginalAnglePD(id)// Read a single steering gear and return to the angle value of the steering gear (refund the power)
#define readServoAnglePD_M(read_id,num) uKitServo_inst.readServoAnglePD_M(read_id,num)// Read a single steering gear and return to the angle value of the steering gear (refund the power)
#define readServoAngleNPD(id) uKitServo_inst.readServoAngleNPD(id)// Read a single steering gear and return to the angle value of the steering gear (without the power reflection)
#define readServoOriginalAngleNPD(id) uKitServo_inst.readServoOriginalAngleNPD(id)// Read a single steering gear and return to the angle value of the steering gear (without the power reflection)
#define readServoAngleNPD_M(read_id,num) uKitServo_inst.readServoAngleNPD_M(read_id,num)// Read a single steering gear and return to the angle value of the steering gear (refund the power)
#define playMotion(id,action,times) uKitServo_inst.playMotion(id,action,times)//播放动作
//uKitSensorApi


#define setEyelightPetalu(id,a,b) uKitSensor_inst.setEyelightPetalu(id,a,b)
#define setEyelightSceneUntil(id,a,b) uKitSensor_inst.setEyelightSceneUntil(id,a,b)
#define setEyelightLookUntil(id,a,b,c,d,e) uKitSensor_inst.setEyelightLookUntil(id,a,b,c,d,e)
#define setUltrasonicRgbled(id,r,g,b) uKitSensor_inst.setUltrasonicRgbled(id,r,g,b)
#define setUltrasonicRgbledOff(id) uKitSensor_inst.setUltrasonicRgbledOff(id)
#define setAllSensorOff() uKitSensor_inst.setAllSensorOff()
#define readLightValue(id) uKitSensor_inst.readLightValue(id)
#define readSoundValue(id) uKitSensor_inst.readSoundValue(id)
#define readInfraredDistance(id) uKitSensor_inst.readInfraredDistance(id)// Ukit infrared sensor control function, return CM, (0-20) CM
#define Set_Infrared_Id(id) uKitSensor_inst.Set_Infrared_Id(id)//设置红外id
#define readColorRgb(id) uKitSensor_inst.readColorRgb(id)
#define readColor(id,color) uKitSensor_inst.readColor(id,color)
#define setColorOff(id) uKitSensor_inst.setColorOff(id)
#define setEyelightLook(id,face,times,red,green,blue) uKitSensor_inst.setEyelightLook(id,face,times,red,green,blue)// Ukit LED sensor.The ID is the ID number of LED, and FACE is the type of expression (0)
#define setEyelightScene(id,scene,times) uKitSensor_inst.setEyelightScene(id,scene,times)
#define setEyelightOff(id) uKitSensor_inst.setEyelightOff(id)
#define setEyelightAllPetals(id,red,green,blue) uKitSensor_inst.setEyelightAllPetals(id,red,green,blue)// Ukit LED sensor.The ID is the ID number of LED, and FACE is the type of expression (0)
#define setEyelightPetals(id,petalnum,petals) uKitSensor_inst.setEyelightPetals(id,petalnum,petals)
#define readHumitureValue(id,choice) uKitSensor_inst.readHumitureValue(id,choice)
#define readButtonValue(id) uKitSensor_inst.readButtonValue(id)//返回256无操作，返回257是单击，返回258是双击
#define readUltrasonicDistance(id) uKitSensor_inst.readUltrasonicDistance(id)
#define setNixieTubeFull(id,tpye,method,frequency,times,start,ends) uKitSensor_inst.setNixieTubeFull(id,tpye,method,frequency,times,start,ends)
#define setNixieTube(id,number) uKitSensor_inst.setNixieTube(id,number)
#define redvalue uKitSensor_inst.redvalue
#define greenvalue uKitSensor_inst.greenvalue
#define bluevalue uKitSensor_inst.bluevalue

//uKitMotorApi
#define setMotorTurn(id,pwmDuty) uKitMotor_inst.setMotorTurn(id,pwmDuty)
#define setMotorTurnAdj(id,speed,time) uKitMotor_inst.setMotorTurnAdj(id,speed,time)
#define readMotorSpeed(id) uKitMotor_inst.readMotorSpeed(id)
#define setMotorStop(id) uKitMotor_inst.setMotorStop(id)
#define setMotorsStop(ids, length) uKitMotor_inst.setMotorsStop(ids, length)
#define MotorSetID(id_old,id_new) uKitMotor_inst.MotorSetID(id_old,id_new)
#define MotorCheckID(id) uKitMotor_inst.MotorCheckID(id)
#define StopServo() uKitMotor_inst.StopServo()
//uKitId_inst
#define setDeciveId() uKitId_inst.setDeciveId()
#define getDeciveId() uKitId_inst.getDeciveId()
#define getCpuUUID() uKitId_inst.getCpuUUID()
//#defineGetVisionId()UKitIdGetVisionId()
//transformerRobotApi
#define Transformer_Car_Forward(a) transfor_robot.forward(a)     //TheCarForwardFunction,Speed05
#define Transformer_Car_TurnLeft(speed) transfor_robot.turnL(speed) //TurnLeftToTheLeft,Speed05
#define Transformer_Car_TurnRight(speed) transfor_robot.turnR(speed) //TurnRightByCar,Speed05
#define Transformer_Car_Back(speed) transfor_robot.back(speed)   //CarlaneRearResponseFunction,Speed05
#define Transformer_Car_Stop() transfor_robot.stops()           //小车停止函数
#define Transformer_Car_LineTracking_TurnLeft(speed) transfor_robot.turn_L(speed)
#define Transformer_Car_LineTracking_TurnRight(speed) transfor_robot.turn_R(speed)
#define Transformer_Car_LineTracking_TurnAround(speed) transfor_robot.turn_U(speed)
#define Transformer_Car_ForwardWithTime(speed, time) transfor_robot.one_step(speed, time)
#define motion_case(cases, times) transfor_robot.motion_case(cases, times)
#define motion_forward(times) transfor_robot.motion_forward(times)         //人形模式前进
#define motion_car(times) transfor_robot.motion_car(times)                 //保持车形态
#define motion_getup(times) transfor_robot.motion_getup(times)             //车变人
#define motion_sitdown(times) transfor_robot.motion_sitdown(times)         //人变车
#define motion_back(times) transfor_robot.motion_back(times)               //人形模式后退
#define motion_left_shift(times) transfor_robot.motion_left_shift(times)   //人形模式左移
#define motion_right_shift(times) transfor_robot.motion_right_shift(times) //人形模式右移
#define motion_turn_left(times) transfor_robot.motion_turn_left(times)     //人形模式左拐
#define motion_turn_right(times) transfor_robot.motion_turn_right(times)   //人形模式右拐
#define motion_omotion(times) transfor_robot.motion_omotion(times)         //保持人形态
#define motion_button(times) transfor_robot.motion_button(times)           //人形按键
#define motion_lift(times) transfor_robot.motion_lift(times)               // Let the robot raise the right hand. Times represents the number of times the action is.One step at a time.
#define motion_lift_hand(times) transfor_robot.motion_lift_hand(times)     // Let the robot raise their hands. Times represent the number of times of execution.One step at a time.
#define motion_head(times) transfor_robot.motion_head(times)               //拆引爆雷
#define motion_zero(times) transfor_robot.motion_zero(times)               //零状态

//sensor2Api
#define num1 Sensor.num1
#define num2 Sensor.num2
#define num3 Sensor.num3
#define num4 Sensor.num4
#define num5 Sensor.num5
#define getGrayAllValue() Sensor.getGrayAllValue()
#define readGrayValue(num,grayval) Sensor.readGrayValue(num,grayval)
#define setRgbledColor(red,green,blue) Sensor.setRgbledColor(red,green,blue)//板载rgb灯函数
#define setcolor(color) Sensor.setcolor(color)
#define readBatteryVoltage() Sensor.readBatteryVoltage()
#define readHcsr04Distance(jp) Sensor.readHcsr04Distance(jp) // Ultrasonic function.Return to CM, JP is the location. If the ultrasonic wave is connected to JP1, then JP is 1.
#define redPin Sensor.redPin
#define greenPin Sensor.greenPin
#define bluePin Sensor.bluePin
#define Button_pin Sensor.Button_pin
#define GrayscaleNum1 Sensor.GrayscaleNum1
#define GrayscaleNum2 Sensor.GrayscaleNum2
#define GrayscaleNum3 Sensor.GrayscaleNum3
#define GrayscaleNum4 Sensor.GrayscaleNum4
#define GrayscaleNum5 Sensor.GrayscaleNum5
#define IR_S Sensor.IR_S
#define buzzer_pin Sensor.buzzer_pin

/*vision module*/
#define VisionIdentity(iId, iType, iTimeOut) mVisionDiscrim.VisionIdentity(iId, iType, iTimeOut)
#define VisionEnable(iVal,iType) mVisionDiscrim.VisionEnable(iVal,iType)
#define VisionOffset(iId) mVisionDiscrim.VisionOffset(iId)

void protocol();
void serialEvent();
void protocolTest();
void BuildReqBacket(uint8_t uDev, uint8_t uMode, uint8_t uId, const char * ptrUuid, String & strData);
//clickButtonApi
ClickButton button1(Button_pin, HIGH, CLICKBTN_PULLUP);//设置按键

void stopDecives(){
  digitalWrite(redPin,HIGH);// en: main board rgb lamp, r interface set to high/cn: motherboard RGB light, R interface set to high -level output.
  digitalWrite(greenPin,HIGH);// en: main board rgb lamp, g interface set to high/cn: motherboard RGB light, G interface is set to high -level output.
  digitalWrite(bluePin,HIGH);// en: main board rgb lamp, b interface set to high/cn: motherboard RGB light, B interface set to high -level output.
  noNewTone(buzzer_pin);     
  setAllSensorOff();
  setMotorStop(0xff);
  StopServo();
  setUltrasonicRgbledOff(0x00);
  delay(2);
}

void flexiTimer2_func() {
  if(timeFlag==0){
     timeTimes+=1;
  }
  else{
    if(butonTimer==true){
   
    button1.Update();
    if(button1.clicks == 1){
      buttonFlag=1;   
    }
    if(button1.clicks == 2){
      buttonFlag=2;   
    }
    if(button1.clicks == -1){
      buttonFlag=-1;   
    }    
    }
    
  }
 
}
 void Initialization(){
  protocolRunState=true;
  Sensor.checkVersion();
  delay(5);
  button1.ClickButtons(Button_pin, HIGH, CLICKBTN_PULLUP);
  pinMode(redPin, OUTPUT); // en: main board rgb lamp, r interface set to output/cn: motherboard RGB light, R interface settings to output.
  pinMode(greenPin, OUTPUT);// en: main board rgb lamp, g interface set to output/cn: motherboard RGB light, G interface settings are set to output.
  pinMode(bluePin, OUTPUT);// en: main board rgb lamp, b interface set to output/cn: motherboard RGB light, B interface settings to output.
  digitalWrite(redPin,HIGH);// en: main board rgb lamp, r interface set to high/cn: motherboard RGB light, R interface set to high -level output.
  digitalWrite(greenPin,HIGH);// en: main board rgb lamp, g interface set to high/cn: motherboard RGB light, G interface is set to high -level output.
  digitalWrite(bluePin,HIGH);// en: main board rgb lamp, b interface set to high/cn: motherboard RGB light, B interface set to high -level output.
  pinMode(Button_pin,INPUT);
  pinMode(GrayscaleNum1, INPUT);  //左1的循迹传感器
  pinMode(GrayscaleNum2, INPUT);  //左2的循迹传感器
  pinMode(GrayscaleNum3, INPUT);  //中间的循迹传感器
  pinMode(GrayscaleNum4, INPUT);  //右2的循迹传感器
  pinMode(GrayscaleNum5, INPUT);  //右1的循迹传感器
  pinMode(IR_S,OUTPUT);
  pinMode(buzzer_pin,OUTPUT);

  delay(5);  //开机延时
  IMU::init();
  IMU::read();
  setAllSensorOff();
  setMotorStop(0xff);
  StopServo();
  setUltrasonicRgbledOff(0x00);
  Serial.begin(115200);// EN: Initialize the Serial Port
  //serialBegin(9600);
  delay(2);
  if(EEPROM.read(3)!=77){
    for(int i=0;i<21;i++){
      EEPROM.write(i,0);
    }
  }
  //Serial.begin (1000000) ;//en:Initialize the Serial Port (baud rate 115200)/cn: initialized serial port (Potter 115200)
  delay(2);
  flexiTimerFlag=true;
  const size_t capacity = JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(1);
  DynamicJsonDocument doc(capacity);
  JsonArray data = doc.createNestedArray("data");
  data.add(versionNumber);
  data.add(Sensor.Version);
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2);
  serializeMsgPack(doc, Serial);
  FlexiTimer2::set(20,flexiTimer2_func);
  FlexiTimer2::start(); 
 // getsdecive ();
  for(int i=0;i<7000;i++){
  serialEvent();
  protocol();
  delayMicroseconds(300);
  }
  
  
  


}


void Initialization_Pruned(){
   protocolRunState=true;
  Sensor.checkVersion();
  delay(5);
  button1.ClickButtons(Button_pin, HIGH, CLICKBTN_PULLUP);
  pinMode(redPin, OUTPUT); // en: main board rgb lamp, r interface set to output/cn: motherboard RGB light, R interface settings to output.
  pinMode(greenPin, OUTPUT);// en: main board rgb lamp, g interface set to output/cn: motherboard RGB light, G interface settings are set to output.
  pinMode(bluePin, OUTPUT);// en: main board rgb lamp, b interface set to output/cn: motherboard RGB light, B interface settings to output.
  digitalWrite(redPin,HIGH);// en: main board rgb lamp, r interface set to high/cn: motherboard RGB light, R interface set to high -level output.
  digitalWrite(greenPin,HIGH);// en: main board rgb lamp, g interface set to high/cn: motherboard RGB light, G interface is set to high -level output.
  digitalWrite(bluePin,HIGH);// en: main board rgb lamp, b interface set to high/cn: motherboard RGB light, B interface set to high -level output.
  pinMode(Button_pin,INPUT);
  pinMode(GrayscaleNum1, INPUT);  //左1的循迹传感器
  pinMode(GrayscaleNum2, INPUT);  //左2的循迹传感器
  pinMode(GrayscaleNum3, INPUT);  //中间的循迹传感器
  pinMode(GrayscaleNum4, INPUT);  //右2的循迹传感器
  pinMode(GrayscaleNum5, INPUT);  //右1的循迹传感器
  pinMode(IR_S,OUTPUT);
  pinMode(buzzer_pin,OUTPUT);
  delay(5);  //开机延时
  // Serial.println("Init IMUs");
  // IMU::init();
  // Serial.println("IMU Init done, reading");
  // IMU::read();
  Serial.println("IMU read done");
  Serial.println("Set sensor states");
  setAllSensorOff();
  setMotorStop(0xff);
  StopServo();
  setUltrasonicRgbledOff(0x00);
  Serial.begin(115200);// EN: Initialize the Serial Port
  //serialBegin(9600);
  Serial.println("read eeprom");
  delay(2);
  if(EEPROM.read(3)!=77){
    for(int i=0;i<21;i++){
      EEPROM.write(i,0);
    }
  }
  //Serial.begin (1000000) ;//en:Initialize the Serial Port (baud rate 115200)/cn: initialized serial port (Potter 115200)
  delay(2);
  protocolRunState = false;
  Serial.println(protocolRunState);
}

long ucode_random(long min, long max) {
  unsigned long seed_mills = micros();
  randomSeed(seed_mills);
  //SerialPrint("seed ");
  //SerialPrintln(seedMills);
  return random(min, max);
}

long ucode_random(unsigned long max) {
  return ucode_random(0, max);
}

void tone2(uint16_t frequency, unsigned long duration = 0)
{
  NewTone(buzzer_pin,frequency,duration);  
}
void noTone2(int pin)
{
  noNewTone(pin);  
}



void ProtocolParser(unsigned char device,unsigned char mode,unsigned char id,int* buf,const char* uuid,unsigned char* bin64,unsigned char *ids,int* par1,int* par2,int* par3,int* par4,int* par5,int* par6,int* par7,int* par8,int eyeTime, unsigned char idsLength){
  size_t capacity = JSON_ARRAY_SIZE(8) + JSON_OBJECT_SIZE(8)+100;
  unsigned char* rgbValue=NULL;
  int ret;
  VisionWifiStatus mVisionWifiStatus = {0};
  VisionIpInfo mVisionIpInfo = {0};
  VisionWifiList mVisionWifiList = {0};
  VisionWifiInfo mVisionWifiInfo = {0};
  DisplayQuality strDisplayQuality = {0};
  VisionHWInfo strVisionHWInfo = {0};
  VisionSWInfo strVisionSWInfo = {0};
  JsonObject JsDataItem;
  uint8_t uiBuf[20];
  DynamicJsonDocument root(capacity);
  root["device"]=device;
  root["mode"]=mode;
  root["id"]=id;
  root["code"]=1;
  JsonArray data = root.createNestedArray("data");
  root["uuid"]=uuid;
 
  switch(device){
    case 1: //舵机
      butonTimer=false;
      buttonFlag=0; 
      switch(mode){
        case 127: //轮模式
           setServoTurn(id,buf[0],buf[1]);     
           root["code"]=0;
           break;
        case 128: //角模式
           setServoAngle(id,buf[0],buf[1]);
           root["code"]=0;
           break;    
        case 129://读取角度
           root["code"]=0; 
           if(buf[0]==0){
            data.add(readServoOriginalAnglePD(id));  
           }
           else{
            data.add(readServoOriginalAngleNPD(id));
           }
           break;
        case 130: //停止舵机
           setServoStop(id);
           root["code"]=0;
           break;     
        case 131://多舵机轮模式
          uKitServo_inst.setServoTurns(ids,par1,par2, idsLength);
          root["code"]=0;
          break;
        case 132://多舵机角度模式
        {
          // Ukitzervo
          bool w = (par3[0] == 0 ? false : true);
          setServoAnglesWait(ids,par1,par2,idsLength,w);
          root["code"]=0;
        }
          break;
        case 133://多舵机停止
          setServosStop(ids, idsLength);
          root["code"]=0;
          break;                                             
      }
      break;
    case 2:    //电机
      butonTimer=false;
      buttonFlag=0; 
      switch(mode){
        case 127: //恒速转动
           setMotorTurnAdj(id,buf[0],0xffff);     
           root["code"]=0;
           break;
        case 128: //pwm转动
           setMotorTurn(id,buf[0]);
           root["code"]=0;
           break;    
        case 129://读取速度
           data.add(readMotorSpeed(id));  
           root["code"]=0; 
           break;
        case 130: //停止电机
           setMotorStop(id);
           root["code"]=0;
           break; 
        case 131: //多电机恒速模式
           uKitMotor_inst.setMotorTurnAdjs(ids,par1,idsLength);
           root["code"]=0;
           break;  
        case 132: //多电机pwm模式
           uKitMotor_inst.setMotorTurns(ids,par1,idsLength);
           root["code"]=0;
           break;
        case 133:
           setMotorsStop(ids, idsLength);
           root["code"]=0;
           break;                                               
      }
      break;
    case 3:    //眼灯
      butonTimer=false;  
      buttonFlag=0; 
      switch(mode){
        case 127: //亮起眼灯
           setEyelightAllPetals(id,buf[0],buf[1],buf[2]);    
           root["code"]=0;
           break;
        case 128: //眼灯表情
           setEyelightLook(id,buf[0],buf[4],buf[1],buf[2],buf[3]);
           root["code"]=0;
           break;    
        case 129://情景灯
           setEyelightScene(id,buf[0],buf[1]);
           root["code"]=0;   
           break;
        case 130: //关闭眼灯
           setEyelightOff(id);
           root["code"]=0;
           break;    
        case 131: //自定义眼灯
           setEyelightPetalu(id,8,buf);
           root["code"]=0;
           break;   
        case 132: //眼灯表情阻塞
           setEyelightLookUntil(id,buf[0],buf[4],buf[1],buf[2],buf[3]);
           root["code"]=0;
           break;  
        case 133: //情景灯阻塞
           setEyelightSceneUntil(id,buf[0],buf[1]);             
           root["code"]=0;
           break;             
        case 134: //多亮起眼灯
           for(int i=0;i<idsLength;i++){
            setEyelightAllPetals(ids[i],par1[i],par2[i],par3[i]);   
           } 
           root["code"]=0;   
           break; 
        case 135: //多眼灯表情
           for(int i=0;i<idsLength;i++){    
            setEyelightLook(ids[i],par1[i],par2[i],par3[i],par4[i],par5[i]);
           }
           root["code"]=0;
           break; 
        case 136://多情景灯
          for(int i=0;i<idsLength;i++){ 
           setEyelightScene(ids[i],par1[i],par2[i]);
          }
           root["code"]=0;   
           break;
        case 137: //多自定义眼灯
          for(int i=0;i<idsLength;i++){       
           uKitSensor_inst.setEyelightPetalus(ids[i],par1[i],par2[i],par3[i],par4[i],par5[i],par6[i],par7[i],par8[i],eyeTime);
          }
           root["code"]=0; 
           break;          
        case 138: //多眼灯表情阻塞
          for(int i=0;i<idsLength;i++){          
           setEyelightLookUntil(ids[i],par1[i],par2[i],par3[i],par4[i],par5[i]);
          }
           root["code"]=0;
           break;  
        case 139: //多情景灯阻塞
          for(int i=0;i<idsLength;i++){     
           setEyelightSceneUntil(ids[i],par1[i],par2[i]);    
          }         
           root["code"]=0;
           break;
        case 140: //关闭多眼灯
          for(int i=0;i<idsLength;i++){     
           setEyelightOff(ids[i]);    
          }         
           root["code"]=0;
           break;                                                                              
      }
      break;        
    case 4:    //传感器
      butonTimer=false;   
      buttonFlag=0; 
      switch(mode){
        case 127: //超声波
           data.add(readUltrasonicDistance(id)); 
           root["code"]=0; 
           break;
        case 128: //红外
           data.add(readInfraredDistance(id)); 
           root["code"]=0; 
           break;    
        case 129://按压
           data.add(readButtonValue(id));
           root["code"]=0; 
           break;
        case 130: //亮度
           data.add(readLightValue(id));   
           root["code"]=0; 
           break;
         case 131: //声音
           data.add(readSoundValue(id)); 
           root["code"]=0; 
           break;    
        case 132://温湿度
           if(buf[0]==0){
            data.add(readHumitureValue(id,'C'));
           }
           else if(buf[0]==1){
            data.add(readHumitureValue(id,'F'));
           }
           else if(buf[0]==2){
            data.add(readHumitureValue(id,'H'));
           }        
           root["code"]=0;       
           break;
        case 133: //颜色识别模式
           switch(buf[0]){
            case 0:
              data.add(readColor(id,"Red"));
              break;
            case 1:
              data.add(readColor(id,"Green")); 
              break;    
            case 2:
              data.add(readColor(id,"Blue")); 
              break;
            case 3:
              data.add(readColor(id,"Yellow")); 
              break;                            
            case 4:
              data.add(readColor(id,"Cyan")); 
              break;
            case 5:
              data.add(readColor(id,"Purple")); 
              break;     
             case 6:
              data.add(readColor(id,"Orange")); 
              break;
            case 7:
              data.add(readColor(id,"Black")); 
              break;    
            case 8:
              data.add(readColor(id,"White")); 
              break;
            case 9:
              data.add(readColor(id,"Gray"));     
              break;                                                   
           }
           root["code"]=0;            
           break;  
        case 134: //颜色RGB模式
           rgbValue=readColorRgb(id);
           data.add(rgbValue[0]);
           data.add(rgbValue[1]);
           data.add(rgbValue[2]);  
           root["code"]=0;                     
           break;    
        case 135: //ukit超声波灯光
           setUltrasonicRgbled(id,buf[0],buf[1],buf[2]);
           root["code"]=0;                     
           break;  
        case 136: //关闭ukit超声波灯光
           setUltrasonicRgbledOff(id);
           root["code"]=0;                      
           break;                                                                  
      }
      break;    
     case 5:    //板载蜂鸣器
      butonTimer=false;    
      buttonFlag=0; 
      switch(mode){
        case 127: //播放音调
           tone2(buf[0],buf[1]);
           root["code"]=0; 
           break;
        case 128: //播放频率
           tone2(buf[0],buf[1]); 
           root["code"]=0; 
           break;    
        case 129://结束声音
           noNewTone(buzzer_pin);     
           root["code"]=0; 
           break;                                           
      }
      break; 
     case 6:    //板载RGB
      butonTimer=false;  
      buttonFlag=0; 
      switch(mode){
        case 127: //RGB
           setRgbledColor(buf[0],buf[1],buf[2]);
           root["code"]=0; 
           break;
        case 128: //播放频率
           setRgbledColor(0,0,0);
           root["code"]=0; 
           break;                                            
      }
      break;   
     case 7: //电池电压
      butonTimer=false;  
      buttonFlag=0; 
      if(mode==127){     
        data.add(readBatteryVoltage());  
        root["code"]=0;                  
      }                                  
      break;    
    case 8: //巡线传感器
      butonTimer=false;      
      buttonFlag=0;  
      if(mode==127){            
        data.add(readGrayValue(id,buf[0]));   
        root["code"]=0;                
      }                                  
      break;  
    case 9: //陀螺仪
      butonTimer=false;     
      buttonFlag=0;   
      if(mode==127){  
        IMU::read();     
        data.add(IMU::getRoll());   
        data.add(IMU::getPitch());   
        data.add(IMU::getRawAccelX());
        data.add(IMU::getRawAccelY());
        data.add(IMU::getRawAccelZ());
        data.add(IMU::getRawGyroX()); 
        data.add(IMU::getRawGyroY());
        data.add(IMU::getRawGyroZ()); 
        root["code"]=0;           
      }                            
      break;
    case 10: //板载按键
      
      if(mode==127){               
        data.add(buttonFlag); 
        buttonFlag=0; 
        root["code"]=0;                            
      } 
                                
      break;
    case 11:    //ID相关
      switch(mode){
        case 130: //停止设备
          root["code"]=0; 
          stopDecives();
          break;          
        case 127: //修改ID
          uKitId_inst.setAllDeciveId(buf[0],buf[1],buf[2]);
          delay(220);                
          if(buf[2]==uKitId_inst.getAllDeciveId(buf[0],buf[2])){
            root["code"]=0;
          }
          else{
            root["code"]=1;
          }
          break;
        #if 0
        case 128: //获取ID
          switch(buf[0]){
            case 0:
              uKitId_inst.getDeciveIdJs(uuid);
              break;
            case 1:
              uKitId_inst.getServoIdJs(uuid);
              break;
            case 2:
              uKitId_inst.getMotorIdJs(uuid);
              break;              
            case 5:
              uKitId_inst.getEyeLightIdJs(uuid);
              break;              
          }
          root["code"]=0;         
          break;
        #endif   
        case 129: //检测固件
          root["code"]=0;  
          strcpy(deciveSN,uKitId_inst.read_String(0).c_str());
          data.add(0);   
          data.add(versionNumber);
          data.add(Sensor.Version);       
          data.add(deciveSN);
          break;   
        case 131: //获取版本号
          data.add(uKitSensor_inst.getSensorVersion(id, buf[0]));
          //Tone2(100,300);
          //DataAdd("9527");
          root["code"]=0;
          break;
        case 132: //进入升级
          if(uKitSensor_inst.setSensorUpdate(id,buf[0])==170){
            root["code"]=0; 
          }    
          break;    
        case 133: //正在升级
          if(uKitSensor_inst.setSensorUpdating(id,buf[1],buf[2],bin64,buf[0])==170){
            setRgbledColor(255,0,0);
            delay(10);
            setRgbledColor(0,0,0);
            root["code"]=0; 
          }  
          break;        
        case 134: //结束升级
          if(uKitSensor_inst.setSensorUpdated(id,buf[1],buf[0])==170){
            root["code"]=0; 
          }
          break;     
        case 135: //烧录SN
          uKitId_inst.writeString(0, snCode);  
          delay(5);
          //strcpy(deciveSn,uKitIdReadString(0)CStr());
          if(uKitId_inst.read_String(0)==snCode){
            root["code"]=0;     
            setRgbledColor(255,0,0);
            tone2(800,200);
            delay(200);
            noNewTone(buzzer_pin);
            setRgbledColor(0,0,0);
          }
          else{
            root["code"]=1; 
          }      
          break;    
        case 136: //烧录SN
          strcpy(deciveSN,uKitId_inst.read_String(0).c_str());
          root["sn"]=deciveSN;
          break;          
        case 137: //读取sn
          tone2(800,200);  
          delay(200); 
          strcpy(deciveSN,uKitId_inst.read_String(0).c_str());
          root["sn"]=deciveSN;
          break;      
                
      }
      break;
    /*vision wifi*/
    case 12:
      JsDataItem = data.createNestedObject();
      switch (mode)
      {
        /*wifi connect*/
        case 127:
            ret = mVisionDiscrim.WifiSsidSet(id, (const char *)par1, (const char *)par2, &mVisionWifiStatus);
            if(0 == ret)
            {
              JsDataItem["state"] = mVisionWifiStatus.mStatus;
              JsDataItem["disreason"] = mVisionWifiStatus.mDisReason;
              JsDataItem["ssid"] = mVisionWifiStatus.mSsid;
              JsDataItem["rssi"] = mVisionWifiStatus.mRssi;
              JsDataItem["authmode"] = mVisionWifiStatus.mAuthmode;
            }
          break;
        /*get status of wifi*/
        case 128:
            ret = mVisionDiscrim.WifiStatusGet(id, &mVisionWifiStatus);
            if(0 == ret)
            {
              JsDataItem["state"] = mVisionWifiStatus.mStatus;
              JsDataItem["disreason"] = mVisionWifiStatus.mDisReason;
              JsDataItem["ssid"] = mVisionWifiStatus.mSsid;
              JsDataItem["rssi"] = mVisionWifiStatus.mRssi;
              JsDataItem["authmode"] = mVisionWifiStatus.mAuthmode;
            }
          break;
        /*get ip of wifi*/
        case 129:
            ret = mVisionDiscrim.WifiIpGet(id, &mVisionIpInfo);
            if(0 == ret)
            {
              memset(uiBuf, 0, sizeof(uiBuf));
              //sprintf(uiBuf, "%d.%d.%d.%d", (mVisionIpInfoMIp)&0Xff, (mVisionIpInfoMIp>>8)&0Xff, (mVisionIpInfoMIp>>16)&0Xff, (mVisionIpInfoMIp>>24)&0Xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mIp)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mIp>>8)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mIp>>16)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d", (mVisionIpInfo.mIp>>24)&0xff);
              JsDataItem["ip"] = uiBuf;
              memset(uiBuf, 0, sizeof(uiBuf));
              
              //sprintf(uiBuf, "%d.%d.%d.%d", (mVisionIpInfoMMask)&0Xff, (mVisionIpInfoMMask>>8)&0Xff, (mVisionIpInfoMMask>>16)&0Xff, (mVisionIpInfoMMask>>24)&0Xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mMask)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mMask>>8)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mMask>>16)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d", (mVisionIpInfo.mMask>>24)&0xff);
              JsDataItem["mask"] = uiBuf;
              memset(uiBuf, 0, sizeof(uiBuf));
              
              //sprintf(uiBuf, "%d.%d.%d.%d", (mVisionIpInfoMGw)&0Xff, (mVisionIpInfoMGw>>8)&0Xff, (mVisionIpInfoMGw>>16)&0Xff, (mVisionIpInfoMGw>>24)&0Xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mGw)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mGw>>8)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d.", (mVisionIpInfo.mGw>>16)&0xff);
              sprintf(reinterpret_cast<char*>(uiBuf) + strlen(reinterpret_cast<char*> (uiBuf)), "%d", (mVisionIpInfo.mGw>>24)&0xff);
              JsDataItem["gw"] = uiBuf;
            }
          break;
        /*wifi on-off*/
        case 130:
            ret = mVisionDiscrim.WifiConnectSet(id, par1[0]);
          break;
        /*get list of wifi*/
        case 131:
            if(-1 == iWIfiListSize)
            {
              ret = mVisionDiscrim.WifiUpdate(id, &mVisionWifiList);
              if(0 == ret)
              {
                iWIfiListSize = mVisionWifiList.mNums;
                if(iWIfiListSize > 0)
                {
                  bWifiRun = true;
                  strLoopCmd = "";
                  //buildReqPacket(device,Model,Id,Uuid,StringLoopCmd);
                  
                  DynamicJsonDocument jsWifiRoot(64);
                  jsWifiRoot["device"] = device;
                  jsWifiRoot["mode"] = mode;
                  jsWifiRoot["id"] = id;
                  jsWifiRoot["uuid"] = uuid;
                  #ifdef DEBUG_PRINT_TAG
                    LOG_PRINTLN("Get list wifi:");
                    Serial2.begin(115200);
                    serializeJson(jsWifiRoot, Serial2);
                    Serial2.end();
                    LOG_PRINTLN("#");
                  #endif
                  serializeMsgPack(jsWifiRoot, strLoopCmd);
                }
                return;
              }
            }
            else
            {
              /*wifi info*/
              ret = mVisionDiscrim.WifiItem(id, &mVisionWifiInfo);
              if(0 == ret)
              {
                JsDataItem["total"] = mVisionWifiInfo.mNums;
                JsDataItem["index"] = mVisionWifiInfo.mIndex;
                JsDataItem["ssid"] = mVisionWifiInfo.mWifiSsid;
                JsDataItem["rssi"] = mVisionWifiInfo.mRssi;
                JsDataItem["authmod"] = mVisionWifiInfo.mEncryMod;
              }
              else
              {
                return;
              }
              
            }
          break;
        /*set quality of graphic display*/
        case 132:
            strDisplayQuality.mQuality = par1[0]&0xffff;
            ret = mVisionDiscrim.DisplayQualitySet(id, strDisplayQuality);
          break;
        /*get version of hardware */
        case 133:
            ret = mVisionDiscrim.HardWareVersionGet(id, &strVisionHWInfo);
            if(0 == ret)
            {
              JsDataItem["version"] = strVisionHWInfo.mVer;
            }
          break;
        /*get version of software*/
        case 134:
            ret = mVisionDiscrim.SoftWareVersionGet(id, &strVisionSWInfo);
            if(0 == ret)
            {
              JsDataItem["version"] = strVisionSWInfo.mVer;
            }
          break;
        default:
          ret = -1;
          break;
      }
      root["code"] = ret;
      break;
    default:
      root["id"]=id;
      root["code"]=1;
      root["uuid"]=uuid;
      break;
  }

 
  //Serial.println();
  //serializeJson(root,Serial);
  //serialPrintln();

  uint16_t bufferLength=measureMsgPack(root);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2);
  serializeMsgPack(root,Serial);
}

void serialEvent()
{
  int8_t uStatic = SERIAL_READ_TIMEOUT;
  char uReadByte;

  if(protocolRunState&&(!newLineReceived))
  {
    while(1)
    {
      while((Serial.available() <= 0)&&uStatic)
      {
        delay(1);
        uStatic--;
      }
      if(uStatic <= 0)
      {
        if(inputString.length()>0)
        {
          bWifiRun = false;
          newLineReceived = true;
        }   
        break;
      }
      uReadByte = Serial.read();
      inputString += uReadByte;
      // Serial.print("received:");
      //SerialPrintln(uReadByte);
      //SerialPrint("len:");
      //SerialPrint(inputStringLength());
      uStatic = SERIAL_READ_TIMEOUT;
    }
    return;
  }
}

void BuildReqBacket(uint8_t uDev, uint8_t uMode, uint8_t uId, const char * ptrUuid, String & strData)
{
  DynamicJsonDocument jsRoot(64);
  jsRoot["device"] = uDev;
  jsRoot["mode"] = uMode;
  jsRoot["id"] = uId;
  jsRoot["uuid"] = ptrUuid;
  serializeMsgPack(jsRoot, strData);
}

void protocol(){
    if ((newLineReceived||(bWifiRun))) { 
    if(flexiTimerFlag==false){
      FlexiTimer2::start();
      flexiTimerFlag=true;
    }
    
    const size_t capacity =JSON_ARRAY_SIZE(3) + JSON_ARRAY_SIZE(64) + JSON_OBJECT_SIZE(6) + 230;
    DynamicJsonDocument root(capacity);
    if(newLineReceived)
    {
      uint8_t uLen;
      uLen = inputString.charAt(1);

      String tmpMy;
      for(int i = 0; i < uLen; i++)
      {
        tmpMy += inputString[i+2];
      }
      inputString = "";
      if(uLen != tmpMy.length())
      {
        inputString = "";
        newLineReceived = false;
        return;
      }

      deserializeMsgPack(root, tmpMy);
      iWIfiListSize = -1;
    }
    else
    {
      iWIfiListSize--;
      if(iWIfiListSize < 0)
      {
        bWifiRun = false;
        iWIfiListSize = -1;
        return;
      }
      deserializeMsgPack(root, strLoopCmd);
    }

    int eyeTime=0,buf[9]={0},par1[18]={0},par2[18]={0},par3[10]={0},par4[10]={0},par5[10]={0},par6[10]={0},par7[10]={0},par8[10]={0};
    unsigned char bin64[64]={0},ids[18]={0};
    unsigned char device = root["device"];
    unsigned char mode = root["mode"];
    unsigned char id = root["id"];  
    JsonArray data = root["data"];
    for(int i=0;i<data.size();i++){
       buf[i]  = data[i];  
    }
    const char* uuid = root["uuid"];
    JsonArray qualId = root["ids"];
    unsigned char idsLength = qualId.size();
    
    switch(device){   
      case 1:  
          if(mode >= 131 && mode <= 133){  
            for(int i=0;i<qualId.size();i++){
              ids[i]  = qualId[i];
              //SerialPrint("===id:");
              //SerialPrintln(ids[i]);
              if (mode != 133) {
                par1[i]  = root["par1"][i];
                par2[i]  = root["par2"][i]; 
                //SerialPrint("===par1:");
                //SerialPrintln(par1[i]);
                //SerialPrint("===par2:");
                //SerialPrintln(par2[i]);
              }  
            }
            if (mode == 132) {
              par3[0]  = root["par3"][0];
            }
          }   
        break;
      case 2:
          if(mode >= 131 && mode <= 133){
            for(int i=0;i<qualId.size();i++){
              ids[i]  = qualId[i]; 
              if (mode != 133) {
                par1[i]  = root["par1"][i];
              }
            }
          }
        break;      
      case 3: 
        if(mode >= 132 && mode <= 140){ 
          for(int i=0;i<qualId.size();i++){
            ids[i]  = qualId[i];  
          }
          if (mode != 140) {
              for(int i=0;i<qualId.size();i++){
              par1[i]  = root["par1"][i];
              par2[i]  = root["par2"][i];  
            }
          }
          if(mode==134|| mode==135 ||mode==138 || mode==137){
          for(int i=0;i<qualId.size();i++){
            par3[i]  = root["par3"][i];  
          } 
          }
          if(mode==135 ||mode==138 || mode==137){
          for(int i=0;i<qualId.size();i++){
            par4[i]  = root["par4"][i]; 
            par5[i]  = root["par5"][i]; 
          }
          }
          if( mode==137){
          for(int i=0;i<qualId.size();i++){
            par6[i]  = root["par6"][i]; 
            par7[i]  = root["par7"][i]; 
            par8[i]  = root["par8"][i];  
          } 
          eyeTime=root["time"];
          }
        }        
        break;         
      case 10:
        butonTimer=true; 
        break;
      case 11:
        if(mode==135){
          snCode = root["sn"];
        }
        else if(mode==133){
          for(int i=0;i<64;i++){
            bin64[i] = root["bin64"][i];
          }
        
        }
        else if(128 == mode)
        {
          switch(buf[0])
          {
              case 0:
                uKitId_inst.getDeciveIdJs(uuid);
                break;
              case 1:
                uKitId_inst.getServoIdJs(uuid);
                break;
              case 2:
                uKitId_inst.getMotorIdJs(uuid);
                break;              
              case 5:
                uKitId_inst.getEyeLightIdJs(uuid);
                break;              
          }
        }
        break;
      /*vision wifi*/
      case 12:
        switch(mode)
        {
          /*set wifi*/
          case 127:
            /*get wifi id and passwd*/
            if((root["data"].size()>=1)&&(root["data"][0].is<char *>()))
            {
              char * ptrId = root["data"][0];
              if(ptrId&&(strlen(ptrId)<=70))
              {
                memcpy(par1, ptrId, strlen(ptrId));
              }
              if((root["data"].size()>=2)&&(root["data"][1].is<char *>()))
              {
                char * ptrPasswd = root["data"][1];
                if(ptrPasswd&&(strlen(ptrPasswd)<=70))
                {
                  memcpy(par2, ptrPasswd, strlen(ptrPasswd));
                }
              }
            }
            break;
          /*wifi on-off*/
          case 130:
            if((root["data"].size() >= 1)&&root["data"][0].is<int>())
            {
              int iType = root["data"][0];
              if(iType >= 0)
              {
                par1[0] = iType;
              }
            }
            break;
          /*set quality of graphic display*/
          case 132:
            if((root["data"].size() >= 1)&&root["data"][0].is<int>())
            {
                par1[0] = root["data"][0];
            }
        }
        break;
      
    }
    if(device!=11 || mode!=128)
    {
      ProtocolParser(device,mode,id,buf,uuid,bin64,ids,par1,par2,par3,par4,par5,par6,par7,par8,eyeTime, idsLength);
    }
    inputString = "";   // clear the string
    newLineReceived = false;
    timeFlag=1; 
    timeTimes=0;
    //flexiTimer2::stop();
    uuid="";
    snCode="";
    protocolRunState=true;   
    
  }
  else if(protocolRunState==false){
    FlexiTimer2::stop();
    flexiTimerFlag=false;
  }
  if(timeTimes>=100){
    protocolRunState=false;
    //flexiTimer2::stop();
    timeFlag=1;
    timeTimes=0;
  }

}
void consoleLog(unsigned char level,  const String msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);   
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  const char *buf=NULL;
  buf=msg.c_str();
  doc["msg"] = buf;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
  
}
void consoleLog(unsigned char level,  const char* msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);   
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  doc["msg"] = msg;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
  
}


void consoleLog(unsigned char level, const long msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  doc["msg"] = msg;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
  
}
void consoleLog(unsigned char level, const char msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  doc["msg"] = msg;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
  
}

void consoleLog(unsigned char level, const int msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  doc["msg"] = msg;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
  
}
void consoleLog(unsigned char level, const uint16_t msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  doc["msg"] = msg;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
}
void consoleLog(unsigned char level, const uint32_t msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  doc["level"] = level;
  doc["msg"] = msg;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  
}
void consoleLog(unsigned char level,const double msg){
  const size_t capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  String gy = "";
  const char *buf=NULL;
  long tmp = msg;   // integer part
  long yytmp = abs((msg -tmp))*100+0.5;  // The decimal part; from the decimal point, the third place is the four houses and five entry to the second place to the second place
  if(yytmp >= 100) {   //
   yytmp=0;  
   ++tmp;  //099Xyz.. +0005 ===>10Pqr
  } //ItIs099Xyz
  if(msg<0 &&tmp==0){
  gy += "-0"; // integer part
  gy += ".";  // Numeros, nonsense
  if(yytmp < 10) gy += "0";
  gy += yytmp;
  }
  else{
  gy += tmp; // integer part
  gy += ".";  // Numeros, nonsense
  if(yytmp < 10) gy += "0";
  gy += yytmp;
  }
  buf=gy.c_str();
  doc["level"] = level;
  doc["msg"] =buf;
  uint16_t bufferLength=measureMsgPack(doc);
  byte Buffer[2]={0};
  Buffer[0]=(bufferLength>>8)&0xff;
  Buffer[1]=(bufferLength)&0xff;
  Serial.write(Buffer,2); 
  serializeMsgPack(doc, Serial);
  

}

#endif /* UCODE_H */
