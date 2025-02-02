
#include"uKitServo.h"  



void uKitServo::setServoTurn(unsigned char id,int dir, int speed){
  unsigned char buf[4];
  int speeds=0;
  if(speed > 0)
  {
    speeds=map(speed,1,255,50,1000);
  }
    if(dir==0){
        buf[0]=0xFD;
        buf[1]=0x00;
        buf[2]=(speeds &0xFF00) >> 8;
        buf[3] = speeds & 0x00FF;          
    }     
    else if(dir==1){
        buf[0]=0xFE;
        buf[1]=0x00;
        buf[2]=(speeds &0xFF00) >> 8;
        buf[3] = speeds & 0x00FF;             
    };
   
  ubtServoActionProtocol(0xFA,id,0x01,buf);
  //TXD(0xFA,id,4,0x01,buf); 
  
}

void uKitServo::setServoTurns(unsigned char *id,int *dir, int *speed, unsigned char length){
  for(int i=0;i<length;i++){
    // Serial.print("setServoTurns id:");
    // Serial.println(id[i]);
    // Serial.print("dir:");
    // Serial.println(dir[i]);
    // Serial.print("speed:");
    // Serial.println(speed[i]);
    setServoTurn(id[i],dir[i],speed[i]);
  }
}


//id表示舵机号，angle表示角度（角度范围-118°~118°），time表示旋转所需时间（时间范围：300~5000）
void uKitServo::setServoAngle(unsigned char id,int angle,int times){
  unsigned char buf[4];
  buf[0]=angle+120;
  buf[1]=(times/20);
  buf[2]= 0; //((times/20) & 0xFF00) >> 8;
  buf[3]= 0; //(times/20) & 0x00FF;

  ubtServoActionProtocol(0xFA,id,0x01,buf);
  
}
void uKitServo::setServoAngles(unsigned char *id,int *angle,int *times, unsigned char length, bool wait){
    int maxTime = 0;
    for(int i=0;i<length;i++){
      // Serial.print("angle: ");
      // Serial.println(angle[i]);
      setServoAngle(id[i],angle[i],times[i]); 
      if (maxTime < times[i]) { maxTime = times[i]; }
    }
    // Serial.print("wait: ");
    // Serial.println(wait);
    if (wait) {
      // Serial.println("wait");
      delay(maxTime);
    }      
}

void uKitServo::setServoAnglesWait(unsigned char *id,int *angle,int *times, unsigned char length, bool wait){
    setServoAngles(id, angle, times, length, wait);
}


void uKitServo::setServoStop(unsigned char id){
  unsigned char buf[4]={0xFF,0,0,0}; 
  ubtServoProtocol(0xFA,id,0x01,buf);
}

void uKitServo::setServosStop(unsigned char *ids, unsigned char length) {
  for(int i=0;i<length;i++){
      setServoStop(ids[i]); 
    }
}

void uKitServo::setServoStiffness(unsigned char id,unsigned char stiffness){
  unsigned char tData[4];
  tData[0]=stiffness;
  tData[1]=0;
  tData[2]=0;
  tData[3]=0;
  
  TXD(0xFA,id,8,0x01,tData );
}

int uKitServo::readServoAnglePD(unsigned char id){//单个舵机回读(掉电回读）
  int tCmd=0,tRet=0;
  unsigned char aa[4]={0,0,0,0};
  tRet=ubtServoProtocol(0xFA,id,0x02,aa);
  if (tRet == 1000) { tRet = 0; }

  if(tRet == 0){ tCmd = 0; }
  else if(tRet == 1){ tCmd =- 120; }
  else{ tCmd = tRet-120; }
  
  delay(5);
    
  if(tCmd >= -120 && tCmd <= 120)
    return tCmd;
  else
    return 0;
}

void uKitServo::readServoAnglePD_M(unsigned char *read_id,char num)//舵机回读
{
    int setServoAngle=0;
    Serial.println("");
    Serial.print("----------");
    Serial.print(read_num++);
    Serial.println("----------");
    Serial.print("{");
    for(int i=0;i<num;i++)
    {
        setServoAngle=readServoAnglePD(read_id[i]);
        setServoAngle = setServoAngle - 120;
        delay(20);
        if(setServoAngle>=-118 && setServoAngle<=118)
        {
          Serial.print(setServoAngle);
          if(i<num-1)
            Serial.print(",");
        }
        
        else
        {
          Serial.print("id-");
          Serial.print(setServoAngle);
          Serial.print(":Out of range,");
        }
    }
      Serial.print("}");
}

// 该接口为
int uKitServo::readServoOriginalAnglePD(unsigned char id){//单个舵机回读(掉电回读）
  int tCmd=0,tRet=0;
  unsigned char aa[4]={0,0,0,0};
  tRet=ubtServoProtocol(0xFA,id,0x02,aa);
  return tRet;
}

int uKitServo::readServoAngleNPD(unsigned char id){//单个舵机回读(不掉电回读）
  int tCmd=0,tRet=0;
  unsigned char aa[4]={0,0,0,0};
  tRet=ubtServoProtocol(0xFA,id,0x03,aa);
  if (tRet == 1000) { tRet = 0; }

  if(tRet == 0){ tCmd = 0; }
  else if(tRet == 1){ tCmd =- 120; }
  else{ tCmd = tRet-120; }
  
  delay(5);
    
  if(tCmd >= -120 && tCmd <= 120)
    return tCmd;
  else
    return 0;
}

void uKitServo::readServoAngleNPD_M(unsigned char *read_id,char num)//舵机回读
{
    int setServoAngle=0;
    Serial.println("");
    Serial.print("----------");
    Serial.print(read_num++);
    Serial.println("----------");
    Serial.print("{");
    for(int i=0;i<num;i++)
    {
        setServoAngle=readServoAngleNPD(read_id[i]);
        delay(20);
        if(setServoAngle>=-118 && setServoAngle<=118)
        {
          Serial.print(setServoAngle);
          if(i<num-1)
            Serial.print(",");
        }
        
        else
        {
          Serial.print("id-");
          Serial.print(setServoAngle);
          Serial.print(":Out of range,");
        }
    }
      Serial.print("}");
}

int uKitServo::readServoOriginalAngleNPD(unsigned char id){//单个舵机回读(不掉电回读）
  int tCmd=0,tRet=0;
  unsigned char aa[4]={0,0,0,0};
  tRet=ubtServoProtocol(0xFA,id,0x03,aa);
  delay(5);
  return tRet;
}

void uKitServo::ServoRead(){
  unsigned char t=0;
  static unsigned char ServoId[18]={0},ServoIdRead[18]={0};
  static int start=0;
  if(start==0){
  Serial.print("当前读取的舵机ID：{");
  for(int i=1;i<=18;i++){
    ServoId[i]=getServoId(i);
    if(ServoId[i]!=0){   
      ServoIdRead[t]=ServoId[i];
      Serial.print(ServoIdRead[t]);
        Serial.print(",");
      ++t;
      
    }
  }
  Serial.println("}");
  start=1;
  }
  readServoAnglePD_M(ServoIdRead,t);
}

//sizeof(action), sizeof(id) 此处计算有问题
void uKitServo::playMotion(unsigned char *id,signed char **action,int *times){
  for(int i=0;i<sizeof(action)/sizeof(action[0]);i++){
    for(int t=0;t<sizeof(id)/sizeof(id[0]);t++){
      setServoAngle(id[t],action[i][t],500);
    }  
      delay(times[i]);
    }
  
}




void uKitServo::setServoTurn1M(unsigned char id,int dir, int speed){
  unsigned char buf[4];
  int speeds=0;
  speeds=map(speed,0,255,0,1000);
 
     
    if(dir==0){
        buf[0]=0xFD;
        buf[1]=0x00;
        buf[2]=(speeds &0xFF00) >> 8;
        buf[3] = speeds & 0x00FF;          
    }     
    else if(dir==1){
        buf[0]=0xFE;
        buf[1]=0x00;
        buf[2]=(speeds &0xFF00) >> 8;
        buf[3] = speeds & 0x00FF;             
    }
        
  ubtServoProtocol1M(0xFA,id,0x01,buf);
  //TXD(0xFA,id,4,0x01,buf); 
  
}



//id表示舵机号，angle表示角度（角度范围-118°~118°），time表示旋转所需时间（时间范围：300~5000）
void uKitServo::setServoAngle1M(unsigned char id,int angle,int times){
  unsigned char buf[4];
  buf[0]=angle+120;
  buf[1]=(times/20);
  buf[2]=((times/20) & 0xFF00) >> 8;
  buf[3]=(times/20) & 0x00FF;

  ubtServoProtocol1M(0xFA,id,0x01,buf);
  
}


void uKitServo::setServoStop1M(unsigned char id){
  unsigned char buf[4]={0xFF,0,0,0}; 
  ubtServoProtocol1M(0xFA,id,0x01,buf);
}

void uKitServo::setServoStiffness1M(unsigned char id,unsigned char stiffness){
  unsigned char tData[4];
  tData[0]=stiffness;
  tData[1]=0;
  tData[2]=0;
  tData[3]=0;
  
  TXD(0xFA,id,8,0x01,tData );
}

int uKitServo::readServoAnglePD1M(unsigned char id){//单个舵机回读(掉电回读）
  int tCmd=0,tRet=0;
  unsigned char aa[4]={0,0,0,0};
  tRet=ubtServoProtocol1M(0xFA,id,0x02,aa);
  if(tRet==0){
    tCmd=0;
  }
  else if(tRet==1){
    tCmd=-120;
  }
  else{
    tCmd=tRet-120;
  }
  
  delay(5);

    return tCmd;

}


int uKitServo::readServoAngleNPD1M(unsigned char id){//单个舵机回读(不掉电回读）
  int tCmd=0,tRet=0;
  unsigned char aa[4]={0,0,0,0};
  tRet=ubtServoProtocol1M(0xFA,id,0x03,aa);
  if(tRet==0){
    tCmd=0;
  }
  else if(tRet==1){
    tCmd=-120;
  }
  else{
    tCmd=tRet-120;
  }
 
  delay(5);
  return tCmd;
 
}


//sizeof(action), sizeof(id) 此处计算有问题
void uKitServo::playMotion1M(unsigned char *id,signed char **action,int *times){
  for(int i=0;i<sizeof(action)/sizeof(action[0]);i++){
    for(int t=0;t<sizeof(id)/sizeof(id[0]);t++){
      setServoAngle1M(id[t],action[i][t],500);
    }  
      delay(times[i]);
    }
  
}
