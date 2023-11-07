#include <avr/interrupt.h>
#include <EEPROM.h>
#include "SCServo.h"
#include "SoftwareSerial.h"

const int motorDirPin=8;
const int motorPWMPin=9;
const int brakePin1=11;
const int brakePin2=12;
char rx[30];
int temp[2];
float angle1; //target angle1
float angle0; //target angle0
float ainput; //angle input bt PWM
float ainput2; //angle input by analog volatge
float err=0;
float errp=0; //previous err
float errpp=0; //pre-previous err
double ierr=0; //err 적분
double derr=0; //err 미분
double output=0;
int DC1output=0;
bool flag=0;
bool flag2=0;
bool DC0flag=0;
bool DC1flag=0;
char DCnum;
volatile bool iflag=0; //interrupt flag
bool grad=1;
int p2v;
int i=0;
int j=0;
int a0c;
int a1c;
int a2c;
int a3c;
int a4c;
double kp, ki, kd; //PID 계수
double tn=0; //time next
double tp=0; //time previous
double ti=0; //time interval
SMSBL sm;

void doMotor(bool dir, int vel)
{ 
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, (dir)?(255-vel):vel);
}
void doBrake(bool on)
{ 
  digitalWrite(brakePin1,on);
  digitalWrite(brakePin2,0);
}

void acheck()
{
  unsigned long t;
  ainput2=analogRead(A3);
  t=pulseIn(7,HIGH,3000);
  ainput2=0.3515*ainput2;
  ainput=(t-3)/0.124*0.05;
}
void setup()
{
  pinMode(7, INPUT);
  TCCR1B = TCCR1B & B11111000 | B00000100; //기본 490Hz (001=30KHz, 010=4KHz, 011=490Hz, 100=122Hz, 101=30Hz)
  pinMode(motorDirPin, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  sm.pSerial = &Serial1;
  Serial.setTimeout(50);
  //Serial.println("원하는 각도입력"); //format : *num
  kp=3; kd=0.2; ki=0.5;
  a0c=int8_t(EEPROM.read(0));
  a1c=int8_t(EEPROM.read(1));
}


void loop() 
{
  if(Serial.available())
  {
    Serial.readBytes(rx,30);
    if(rx[0]=='*')
    {
      sscanf(rx,"*%c,%d,%d$",&DCnum,&temp[0],&temp[1]);
      if(DCnum=='2')
      {
        angle0=float(temp[0]);
        angle1=float(temp[1]);
        DC0flag=1;
        DC1flag=1;
        DCnum=0;
      }
      else if(DCnum=='0')
      {
        angle0=float(temp[0]);
        DC0flag=1;
        DCnum=0;
      }
      else if(DCnum=='1')
      {
        angle1=float(temp[1]);
        DC1flag=1;
        DCnum=0;
      }
      rx[0]='\0';
    }
    else if(rx[0]=='t') //cali data 받아서 저장하기
    {
      Serial.print('d');  //due로부터 데이터를 받았음을 신호로 보냄
      sscanf(rx,"t%d,%d,%d,%d,%d",&a0c,&a1c,&a2c,&a3c,&a4c);
      EEPROM.write(0,a0c);
      EEPROM.write(1,a1c);
      EEPROM.write(2,a2c);
      EEPROM.write(3,a3c);
      EEPROM.write(4,a4c);
      a0c=int8_t(EEPROM.read(0));
      a1c=int8_t(EEPROM.read(1));
      rx[0]='\0';
    }
    else if(rx[0]=='r') //cali data 보내기
    {
      a0c=int8_t(EEPROM.read(0));
      a1c=int8_t(EEPROM.read(1));
      a2c=int8_t(EEPROM.read(2));
      a3c=int8_t(EEPROM.read(3));
      a4c=int8_t(EEPROM.read(4));
      Serial.print("t");
      Serial.print(a0c);
      Serial.print(",");
      Serial.print(a1c);
      Serial.print(",");
      Serial.print(a2c);
      Serial.print(",");
      Serial.print(a3c);
      Serial.print(",");
      Serial.println(a4c);
      rx[0]='\0';
    }
    else if(rx[0]=='d'&&rx[1]=='r') //current DC angle read
    {
      acheck();
      unsigned char ERR;
      int pos;
      int ii=0;
      float posf;
      do
      {
        pos = sm.ReadPos(1, &ERR);
        ii++;
        delay(10);
      }
      while(ii<10 && ERR==1);
      posf=0.0879*float(pos);
      Serial.print('*');Serial.print(ainput);Serial.print(',');Serial.print(ainput2);Serial.print(',');Serial.print(posf);Serial.println('$');
      rx[0]='\0';
    }
    else
    {
      //Serial.println("잘못된 포맷");
      rx[0]='\0';
    }
  }

  if(DC1flag==1)
  {
    angle1=constrain(angle1,90+a1c,270+a1c);
    DC1output=map(angle1,0,360,0,4095);
    sm.WritePosEx(1, DC1output, 200, 100);
    DC1flag=0;
  }
  
  if(DC0flag==1)
  {
    i=0;
    j=0;
    iflag=0;
    angle0=constrain(angle0,20+a0c,340+a0c);
    tp=micros()/1000;
    do
    {
      doBrake(0);
      acheck();
      ainput=constrain(ainput,20+a0c,340+a0c);
      tn=micros()/1000;
      ti=(tn-tp);
      tp=micros()/1000;
      err=angle0-ainput;     //err 계산
      if(flag2==0)
      {
        grad=(err>0)? 1:0; //응답이 향하는 방향을 정하는 구문. (목표값보다 작은상태에서 상승하는중이면 grad=1, 반대의경우 0)
        flag2=1; //while loop중 단 한번만 작동하도록 flag2 변경
        if(abs(err)<40)
        {
          ki=ki*(40.00-abs(err)); //각도차가 작게 시작하는경우 ki가 비례하여증가
          j=1;  //40도 안쪽의 오차인경우 brake 역전압을 낮게 주기 위한 변수
        }
      }
      ierr += (double)err*ti*0.001;       //err 적분
      derr = (double(err-errp))/(ti*0.001); //err 미분  
      output = kp*(double)err+ ki*ierr + kd*derr; //제어량 계산
      p2v=ceil(constrain(abs(output),0,255)); //제어량 제한 및 output->duty ratio 변환 
      //p2v=map(p2v,0,255,30,255); //포화 및 역치를 가지도록 변환
      if(grad) {doMotor(LOW, 255);}
      else {doMotor(HIGH, 255);}
      errpp = errp;
      errp = err;
      delay(12);
      if(Serial.available())
      {
        break;
      }
      i++;
    }
    while(i<1000 && !(grad?(err<1 && errpp<2):(err>-1 && errpp>-2)) ); //조건해석: 1000번 loop를 돌거나, 오차및 전전오차가 수렴하는방향으로 각각 1도 및 2도 이내이면 break

    //DC OFF & brake ON
    doMotor(HIGH,0);
    doBrake(1);
    
    /*if(grad) {doMotor(HIGH, 200-j*100);} //역토크를 통해 brake 걸기 (과전류 주의)
    else {doMotor(LOW, 200-j*100);}
    delay(30);
    doMotor(HIGH, 0);*/
    Serial.print('D');
    //초기화
    
    ierr=0;
    DC0flag=0;
    flag2=0;
    iflag=0;
    ki=0.3;
    
  }
}
