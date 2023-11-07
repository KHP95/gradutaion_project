#include <BasicLinearAlgebra.h>
#include <Servo.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <SPI.h>
using namespace BLA;

//servo variables
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo gripper;


float angle0 = 90;
float angle1 = 90;
float angle2 = 121;
float angle3 = 120;
float angle4 = 55;
float angle5 = 90;
int gripperopen = 145;
int gripperclose = 90;
int ANGLE0;
int ANGLE1;
int ANGLE2;
int ANGLE3;
int ANGLE4;
int ANGLE5;

//Pathplan variables
double th1 = 0 * 0.0174533; //deg2rad
double th2 = 0 * 0.0174533;
double th3 = 90 * 0.0174533;
double th4 = 90 * 0.0174533;
//float thlog[4][100];
float xpos[2];
float ypos[2];
float zpos[2];
double l1 = 30;   //link length(centimeter)
double l2 = 20;
double l3 = 20;
bool flag = 0;
bool cflag = 0;   //constrain flag
//Pathplan matrices
BLA::Matrix<1, 3, Array<1, 3, double>> pos1 = {20, 0, 10}; //current end-effector position
BLA::Matrix<1, 3, Array<1, 3, double>> pos2; //target end-effector position
BLA::Matrix<1, 3, Array<1, 3, double>> np; //next position
BLA::Matrix<3, 1, Array<3, 1, double>> npd; //next position dot
BLA::Matrix<3, 4, Array<3, 4, double>> jab;
BLA::Matrix<3, 3, Array<3, 3, double>> jab2;
BLA::Matrix<4, 3, Array<4, 3, double>> pinvj;

//Communication variables
char rx[50];
char mainrx[3];
char dump[5];
RF24 radio(50, 51); //50:CE 51:CSN
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL}; //Radio adress

//Calibartion variables
char calrx[6];
int a0c = 0; //offset values
int a1c = 0;
int a2c = 0;
int a3c = 0;
int a4c = 0;
int a5c = 0;
int gpc = 0;

//Camera data struct
struct Camdata
{
  float x=40;
  float y=-20;
  float z=0;
  float roll;
  float pitch;
  float yaw;
} camdata;

//battery mount position
struct Battpos
{
  float xfull = 0;
  float yfull = -15;
  float zfull = 0;
  float xempty = 0;
  float yempty = -35;
  float zempty = 0;
} battpos;

//angle constraints
#define a0top 340
#define a0bottom 20
#define a1top 180
#define a1bottom 0
#define a2top 165
#define a2bottom 25
#define a3top 165
#define a3bottom 25
#define a4top 165
#define a4bottom 25
#define a5top 165
#define a5bottom 25

void Mission();   //mission function
void Pathplan(int i); //Pathplan function
void DCwrite(char DCnum, int DC0angle, int DC1angle); //DC PID control function
void Calibration();   //calibration function
void AngleConstrain(); //angle constraints check function
void Homeposition();  //DC & servos Homeposition()
void Manual();  //for manual contorl
void WaitDC();   //Wait for DC. if DC is at target angle, break
void PWMout(bool a4);  ////Convert th to angle and Send PWM to DC & servo


  void setup()
{
  //Open serial
  Serial.begin(115200); //Serial w/ computer
  Serial1.begin(115200); //serial w/ Uno
  Serial2.begin(115200); //Serial w/ Raspberry pi
  Serial.setTimeout(100);
  Serial1.setTimeout(100);
  Serial2.setTimeout(100);

  //Radio check
  radio.begin();
  radio.openWritingPipe(pipes[1]);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
  Serial.println(radio.isChipConnected() ? "nRF connected" : "nRF NOT connected");
  radio.printDetails();
  delay(200);

  //Reeceive calibration data from Uno
  Serial.println();
  Serial.println("Receiving data from Uno...");
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  Serial1.print("r");
  digitalWrite(33, LOW);
  int k = 0;
  do
  {
    if (Serial1.available())
    {
      Serial1.readBytes(rx, 50);
      sscanf(rx, "t%d,%d,%d,%d,%d,%d,%d$", &a0c, &a1c, &a2c, &a3c, &a4c, &a5c, &gpc);
      rx[0] = '\0';
      flag = 1;
      break;
    }
    delay(400);
    digitalWrite(33, HIGH);
    Serial1.print("r");
    digitalWrite(33, LOW);
    k++;
  }
  while (k < 5);
  if (flag == 1)
  {
    Serial.println("Data received.");
    Serial.print("a0c:"); Serial.print(a0c); Serial.print("  a1c:"); Serial.print(a1c); Serial.print("  a2c:"); Serial.print(a2c); Serial.print("  a3c:"); Serial.println(a3c);
    Serial.print("a4c:"); Serial.print(a4c); Serial.print("  a5c:"); Serial.print(a5c); Serial.print("  gpc:"); Serial.println(gpc);
    Serial.println();
    flag = 0;
  }
  else
  {
    Serial.println("Cannot receive data from Uno. starting w/o offsetvalues");
    Serial.println();
  }

  //attach & initiate servos
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  gripper.attach(6);
  Homeposition();
}

/*********************  MAIN LOOP  *******************/
void loop()
{
  Serial.readBytes(dump,5);
  Serial.println("Command -> 'm': mission, 'ma': maunual control, 'h': homeposition, 'ca': calibration, 'o': radio send open, 'c': radio send close");
  Serial.println();
  while (!Serial.available()) {}; //wait for Serial input
  mainrx[0] = '\0';
  mainrx[1] = '\0';
  mainrx[2] = '\0';

  //receive command
  if (Serial.available())
  {
    Serial.readBytes(mainrx, 3);
    if (mainrx[0] == 'm' && mainrx[1] == 'a')
    {
      Manual();
    }
    else if (mainrx[0] == 'm')
    {
      Mission();
    }
    else if (mainrx[0] == 'h')
    {
      Homeposition();
      WaitDC();
      Serial.println("Done");
    }
    else if (mainrx[0] == 'c' && mainrx[1] == 'a')
    {
      Calibration();
    }
    else if (mainrx[0] == 'o' || mainrx[0] == 'c')
    {
      radio.write(&mainrx, 1);
      Serial.println("Sending command to target rover.");
      Serial.println();
    }
    else
    {
      Serial.println("Syntax err");
    }
  }

}
/***************** MAIN LOOP END  ********************/



void Manual()
{
  Serial.println("Manual control mode. enter 1 for angle control, enter 2 for coordinate control.");
  Serial.println("Or enter e for exit");
  do
  {
    if (Serial.available())
    {
      rx[0] = Serial.read();
      if (rx[0] == '1' || rx[0] == '2')
      {
        break;
      }
      else if (rx[0] == 'e')
      {
        return;
      }
      else
      {
        Serial.println("Syntax err. please try again.");
      }
    }
  } while (1);
  Serial.readBytes(dump, 5);

  if (rx[0] == '1')
  {
    Serial.println("Angle control mode.");

    //Angle0 control//
    Serial.println("Set DC0 angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          sscanf(rx, "%f$", &angle0);
          AngleConstrain();
          DCwrite('0', ANGLE0, ANGLE1);
          Serial.print("Done."); Serial.print("angle0 = "); Serial.println(angle0);
        }
      }
    } while (1);

    //Angle1 control//
    Serial.println("Set DC1 angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          sscanf(rx, "%f$", &angle1);
          AngleConstrain();
          DCwrite('1', ANGLE0, ANGLE1);
          Serial.print("Done."); Serial.print("angle1 = "); Serial.println(angle1);
        }
      }
    } while (1);

    //Angle2 control//
    Serial.println("Set servo1 angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          sscanf(rx, "%f$", &angle2);
          AngleConstrain();
          servo1.write(ANGLE2);
          Serial.print("Done."); Serial.print("angle2 = "); Serial.println(angle2);
        }
      }
    } while (1);

    //Angle3 control//
    Serial.println("Set servo2 angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          sscanf(rx, "%f$", &angle3);
          AngleConstrain();
          servo2.write(ANGLE3);
          Serial.print("Done."); Serial.print("angle3 = "); Serial.println(angle3);
        }
      }
    } while (1);

    //Angle4 control//
    Serial.println("Set servo3 angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          sscanf(rx, "%f$", &angle4);
          AngleConstrain();
          servo3.write(ANGLE4);
          Serial.print("Done."); Serial.print("angle4 = "); Serial.println(angle4);
        }
      }
    } while (1);

    //Angle5 control//
    Serial.println("Set servo4 angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          sscanf(rx, "%f$", &angle5);
          AngleConstrain();
          servo4.write(ANGLE5);
          Serial.print("Done."); Serial.print("angle5 = "); Serial.println(angle5);
        }
      }
    } while (1);

    //Gripper control//
    Serial.println("Set Gripper angle : Enter angle$ (n : next step, e : exit)");
    do
    {
      if (Serial.available())
      {
        Serial.readBytes(rx, 6);
        if (rx[0] == 'n')
        {
          break;
        }
        else if (rx[0] == 'e')
        {
          return;
        }
        else
        {
          float gripangle;
          sscanf(rx, "%f$", &gripangle);
          gripangle = constrain(gripangle, 90, 145);
          gripper.write(gripangle + gpc);
          Serial.print("Done."); Serial.print("gripangle = "); Serial.println(gripangle);
        }
      }
    } while (1);

  }
  ///Coordinat control mode///
  else if (rx[0] == '2')
  {
    Serial.println("Coordinate control mode. Enter target positions (format:*xpos,ypos,zpos* ,(centimeter))");
    do {} while (!Serial.available());
    Serial.readBytes(rx, 30);
    sscanf(rx, "*%f,%f,%f*", &xpos[0], &ypos[0], &zpos[0]);
    cflag = 0;
    rx[0] = '\0';
    
    pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
    Pathplan(100);
    Serial.println("Moving position..");
    PWMout(1);
    WaitDC();
    pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
    Serial.println("Done");
    Serial.print("angle0"); Serial.print("\t\t"); Serial.print("angle1"); Serial.print("\t\t"); Serial.print("angle2"); Serial.print("\t\t"); Serial.println("angle3");
    Serial.print(angle0, 3); Serial.print("\t\t"); Serial.print(angle1, 3); Serial.print("\t\t"); Serial.print(angle2, 3); Serial.print("\t\t"); Serial.println(angle3, 3);

    Serial.print("th1"); Serial.print("\t\t"); Serial.print("th2"); Serial.print("\t\t"); Serial.print("th3"); Serial.print("\t\t"); Serial.println("th4");
    Serial.print(th1, 3); Serial.print("\t\t"); Serial.print(th2, 3); Serial.print("\t\t"); Serial.print(th3, 3); Serial.print("\t\t"); Serial.println(th4, 3);

    Serial.print("xpos"); Serial.print('\t'); Serial.print("ypos"); Serial.print('\t'); Serial.println("zpos");
    Serial.print(pos1(0, 0)); Serial.print('\t'); Serial.print(pos1(0, 1)); Serial.print('\t'); Serial.println(pos1(0, 2));
    delay(1000);
    Serial.println("Command Executed");
  }
}


void Mission()
{
  ///////////////////////////////////Phase 1///////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("OK Processing..");
  Serial2.begin(115200);
  Serial2.setTimeout(100);
  Serial.println("Opening battery casing");
  dump[0]='o';
  radio.write(dump, 1);
  delay(500);
  //Serial2.print('r');
  Serial.println("Receving data from camera..");
  /*do 
  {
    if(Serial.available())
    {
      dump[0]=Serial.read();
      if(dump[0]=='e'){return;} 
    }
  } while (!Serial2.available());
  do
  {
    rx[0]=Serial2.read();
  }while(rx[0]=='*');
  Serial2.readBytes(rx, 50);
  sscanf(rx, "%f,%f,%f;%f,%f,%f$", &camdata.x, &camdata.y, &camdata.z, &camdata.roll, &camdata.pitch, &camdata.yaw);
  rx[0] = '\0';*/
  Serial.println("Received camera data.");
  Serial.print("X"); Serial.print('\t'); Serial.print("Y"); Serial.print('\t'); Serial.print("Z"); Serial.print('\t'); Serial.print("Roll"); Serial.print('\t'); Serial.print("Pitch"); Serial.print('\t'); Serial.println("Yaw");
  Serial.print(camdata.x);; Serial.print('\t'); Serial.print(camdata.y);; Serial.print('\t'); Serial.print(camdata.z);; Serial.print('\t'); Serial.print(camdata.roll);; Serial.print('\t'); Serial.print(camdata.pitch);; Serial.print('\t'); Serial.println(camdata.yaw);
  Serial.println("Press Enter if nothing wrong");
  do {} while (!Serial.available());
  Serial.readBytes(dump, 5);
  Serial.println("Proceeding..");



  ///////////////////////////////////Phase 2-1/////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Moving to target position...");
  //Calculate target position using camdata
  xpos[0] = camdata.x - 5.0;
  ypos[0] = camdata.y;
  zpos[0] = camdata.z + 35.0;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  //call Pathplan function
  Pathplan(100);
  //Convert th to angle and Send PWM to DC & servo
  PWMout(1);
  //Wait for DC. if DC is at target angle, proceed.
  WaitDC();
  //Now the gripper is above target position.
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  Serial.println("Done. moving down & grip..");
  zpos[0] = zpos[0] - 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(1);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);
  gripper.write(gripperclose + gpc);
  delay(500);


  Serial.println("Done gripping. Moving up..");
  zpos[0] = zpos[0] + 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(1);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  ///////////////////////////////////Phase 2-2/////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Moving to our empty battery mount..");
  xpos[0] = battpos.xempty;
  ypos[0] = battpos.yempty;
  zpos[0] = battpos.zempty+35;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(0);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  Serial.println("Moving down & release..");
  zpos[0] = zpos[0] - 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(0);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);
  gripper.write(gripperopen + gpc);
  delay(500);


  Serial.println("Done releasing. moving up..");
  zpos[0] = zpos[0] + 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(0);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  ///////////////////////////////////Phase 2-3/////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Moving to our charged battery mount..");
  xpos[0] = battpos.xfull;
  ypos[0] = battpos.yfull;
  zpos[0] = battpos.zfull+35;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(0);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  Serial.println("Moving down & grip..");
  zpos[0] = zpos[0] - 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(0);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);
  gripper.write(gripperclose + gpc);
  delay(500);


  Serial.println("Done gripping. moving up..");
  zpos[0] = zpos[0] + 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(0);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  ///////////////////////////////////Phase 2-4/////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Moving to target position...");
  xpos[0] = camdata.x - 5.0;
  ypos[0] = camdata.y;
  zpos[0] = camdata.z + 35.0;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(1);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);


  Serial.println("Moving down & release..");
  zpos[0] = zpos[0] - 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(1);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);
  gripper.write(gripperopen + gpc);
  delay(500);


  Serial.println("Done releasing. Moving up..");
  zpos[0] = zpos[0] + 15;
  pos2(0, 0) = xpos[0]; pos2(0, 1) = ypos[0]; pos2(0, 2) = zpos[0];
  Pathplan(100);
  PWMout(1);
  WaitDC();
  pos1(0, 0) = xpos[0]; pos1(0, 1) = ypos[0]; pos1(0, 2) = zpos[0];
  delay(1000);

  ///////////////////////////////////Phase 3///////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Mission done. Moving to Homeposition() & close battery casing..");
  Homeposition();
  WaitDC();
  delay(500);
  dump[0]='c';
  radio.write(dump, 1);
  Serial2.end();
  delay(1000);
}


//타겟로버에서 작업시 a5=1, 교체로버에서 작업시 a5=0
void PWMout(bool a5)
{
  //Convert th to angle
  angle0 = 360 - float(th1 * 57.2958) - 270;
  angle1 = (360 - float(th2 * 57.2958)) - 270;
  angle2 = 2 * (th3 * 57.2958 - 37) + 25;
  angle3 = th4 * 57.2958+25;

  //Servo3 should move inverse to DC0, Servo4 should move inverse to DC1+servo1+servo2
  angle4 = 180.00 - 57.2958 * (th2 + th3 + th4) + 55.00;
  angle5 = (a5) ? ((-th1 * 57.2958 - camdata.yaw) + 90.00) : (90);

  //Constrain angles.
  AngleConstrain();

  //PWM signal out
  DCwrite('2', ANGLE0, ANGLE1);
  servo1.write(ANGLE2);
  servo2.write(ANGLE3);
  servo3.write(ANGLE4);
  servo4.write(ANGLE5);
}

void Homeposition()
{
  DCwrite('2', 90 + a0c, 90 + a1c);
  servo1.write(121+ + a2c);
  servo2.write(120 + a3c);
  servo3.write(55 + a4c);
  servo4.write(90 + a5c);
  gripper.write(gripperopen + gpc);
}

void WaitDC()
{
  char val = 0;
  int k = 0;
  byte counter = 0;
  Serial1.readBytes(dump,5);
  while (k < 800 && counter < 2)
  {
    if (Serial1.available())
    {
      val = Serial1.read();
      Serial.print(val);
    }
    if (val == 'D')
    {
      val = 0;
      counter++;
    }
    k++;
    delay(10);
  }
  delay(1000);
}

void Pathplan(int i)
{
  int tf = 10;  //final value
  //int ii = 0; //iteration for thlog
  double k1[4], k2[4], k3[4], k4[4]; //RK4 variables
  double th1tmp, th2tmp, th3tmp, th4tmp;
  double t = 0; //initial iterating value
  double tmpt;
  double h;     //increment
  h = tf / (double)i; //calculate increment

  //calculating Jacobain by RK4
  do
  {
    //Jacobian position
    /////////////////////RK1//////////////////////
    tmpt = t;
    th1tmp = th1;
    th2tmp = th2;
    th3tmp = th3;
    th4tmp = th4;

    np = (pos2 - pos1) * 6 / (pow(tf, 5)) * (pow(t, 5)) - (pos2 - pos1) * 15 / (pow(tf, 4)) * (pow(t, 4)) + (pos2 - pos1) * 10 / (pow(tf, 3)) * (pow(t, 3)) + pos1;
    npd = (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 5)) * (pow(tmpt, 4)) - (~pos2 * 60 - ~pos1 * 60) / (pow(tf, 4)) * (pow(tmpt, 3)) + (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 3)) * (pow(tmpt, 2));
    jab <<  -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp),
        (l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp),
        0, -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l3 * sin(th2tmp + th3tmp + th4tmp));

    //pseudo inverse of jacobian
    jab2 = jab * ~jab;
    jab2(0, 0) = jab2(0, 0) + 0.001;
    jab2(1, 1) = jab2(1, 1) + 0.001;
    jab2(2, 2) = jab2(2, 2) + 0.001;
    Invert(jab2);
    pinvj = ~jab * jab2;

    BLA::Matrix<4, 1, Array<4, 1, double>> W = pinvj * npd;

    k1[0] = W(0, 0);
    k1[1] = W(1, 0);
    k1[2] = W(2, 0);
    k1[3] = W(3, 0);
    /////////////////////RK1 end//////////////////////

    /////////////////////RK2//////////////////////////
    tmpt = t + h / 2;
    th1tmp = th1tmp + k1[0] / 2 * h;
    th2tmp = th2tmp + k1[1] / 2 * h;
    th3tmp = th3tmp + k1[2] / 2 * h;
    th4tmp = th4tmp + k1[3] / 2 * h;

    npd = (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 5)) * (pow(tmpt, 4)) - (~pos2 * 60 - ~pos1 * 60) / (pow(tf, 4)) * (pow(tmpt, 3)) + (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 3)) * (pow(tmpt, 2));
    jab <<  -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp),
        (l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp),
        0, -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l3 * sin(th2tmp + th3tmp + th4tmp));

    //pseudo inverse of jacobian
    jab2 = jab * ~jab;
    jab2(0, 0) = jab2(0, 0) + 0.001;
    jab2(1, 1) = jab2(1, 1) + 0.001;
    jab2(2, 2) = jab2(2, 2) + 0.001;
    Invert(jab2);
    pinvj = ~jab * jab2;

    W = pinvj * npd;

    k2[0] = W(0, 0);
    k2[1] = W(1, 0);
    k2[2] = W(2, 0);
    k2[3] = W(3, 0);
    /////////////////////RK2 end//////////////////////

    /////////////////////RK3//////////////////////////
    tmpt = t + h / 2;
    th1tmp = th1tmp + k2[0] / 2 * h;
    th2tmp = th2tmp + k2[1] / 2 * h;
    th3tmp = th3tmp + k2[2] / 2 * h;
    th4tmp = th4tmp + k2[3] / 2 * h;

    npd = (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 5)) * (pow(tmpt, 4)) - (~pos2 * 60 - ~pos1 * 60) / (pow(tf, 4)) * (pow(tmpt, 3)) + (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 3)) * (pow(tmpt, 2));
    jab <<  -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp),
        (l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp),
        0, -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l3 * sin(th2tmp + th3tmp + th4tmp));

    //pseudo inverse of jacobian
    jab2 = jab * ~jab;
    jab2(0, 0) = jab2(0, 0) + 0.001;
    jab2(1, 1) = jab2(1, 1) + 0.001;
    jab2(2, 2) = jab2(2, 2) + 0.001;
    Invert(jab2);
    pinvj = ~jab * jab2;

    W = pinvj * npd;

    k3[0] = W(0, 0);
    k3[1] = W(1, 0);
    k3[2] = W(2, 0);
    k3[3] = W(3, 0);
    /////////////////////RK3 end//////////////////////

    /////////////////////RK4//////////////////////////
    tmpt = t + h;
    th1tmp = th1tmp + k3[0] * h;
    th2tmp = th2tmp + k3[1] * h;
    th3tmp = th3tmp + k3[2] * h;
    th4tmp = th4tmp + k3[3] * h;

    npd = (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 5)) * (pow(tmpt, 4)) - (~pos2 * 60 - ~pos1 * 60) / (pow(tf, 4)) * (pow(tmpt, 3)) + (~pos2 * 30 - ~pos1 * 30) / (pow(tf, 3)) * (pow(tmpt, 2));
    jab <<  -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*cos(th1tmp),
        (l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp))*cos(th1tmp), (l1 * cos(th2tmp) + l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l2 * cos(th2tmp + th3tmp) + l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp), (l3 * cos(th2tmp + th3tmp + th4tmp))*sin(th1tmp),
        0, -(l1 * sin(th2tmp) + l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l2 * sin(th2tmp + th3tmp) + l3 * sin(th2tmp + th3tmp + th4tmp)), -(l3 * sin(th2tmp + th3tmp + th4tmp));

    //pseudo inverse of jacobian
    jab2 = jab * ~jab;
    jab2(0, 0) = jab2(0, 0) + 0.001;
    jab2(1, 1) = jab2(1, 1) + 0.001;
    jab2(2, 2) = jab2(2, 2) + 0.001;
    Invert(jab2);
    pinvj = ~jab * jab2;

    W = pinvj * npd;

    k4[0] = W(0, 0);
    k4[1] = W(1, 0);
    k4[2] = W(2, 0);
    k4[3] = W(3, 0);
    /////////////////////RK4 end//////////////////////

    W(0, 0) = (1.00 / 6) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    W(1, 0) = (1.00 / 6) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    W(2, 0) = (1.00 / 6) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    W(3, 0) = (1.00 / 6) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);

    th1 = th1 + W(0, 0) * h;
    th2 = th2 + W(1, 0) * h;
    th3 = th3 + W(2, 0) * h;
    th4 = th4 + W(3, 0) * h;
    t = t + h;
  }
  while (t < tf);
}

void AngleConstrain()
{
  ANGLE0 = int(angle0) + a0c;
  ANGLE1 = int(angle1) + a1c;
  ANGLE2 = int(angle2) + a2c;
  ANGLE3 = int(angle3) + a3c;
  ANGLE4 = int(angle4) + a4c;
  ANGLE5 = int(angle5) + a5c;
  
  ANGLE0 = constrain(ANGLE0, a0bottom, a0top);
  ANGLE1 = constrain(ANGLE1, a1bottom, a1top);
  ANGLE2 = constrain(ANGLE2, a2bottom, a2top);
  ANGLE3 = constrain(ANGLE3, a3bottom, a3top);
  ANGLE4 = constrain(ANGLE4, a4bottom, a4top);
  ANGLE5 = constrain(ANGLE5, a5bottom, a5top);
}

void DCwrite(char DCnum, int DC0angle, int DC1angle)
{
  DC0angle = DC0angle + 90; //for singleturn encoder, +90deg (to prevent overflow at 0deg)
  DC1angle = DC1angle + 90;
  Serial1.print('*'); Serial1.print(DCnum); Serial1.print(','); Serial1.print(DC0angle); Serial1.print(','); Serial1.print(DC1angle); Serial1.println('$');
  
  delay(100);
  //rx[0]='\0';
  //Serial1.readBytes(rx,50);
  //Serial.print(rx);
}

void Calibration()
{
  mainrx[0] = '\0';
  calrx[0] = '\0';
  Serial.println("Entering calibration mode..");
  Homeposition();
  angle2=121;
  angle3=120;
  angle4=35;
  angle5=90;
  gripper.write(gripperclose);
  WaitDC();
  delay(500);
  Serial.println();
  Serial.println("DC & servos are at Homeposition");
calstart:
  Serial.println("Choose a DC or Servo to calibrate. Enter 0~6 (6=gripper)");
  Serial.println("For save, enter 's' / For exit, enter 'e'");
  Serial.println();
  do {} while (!Serial.available());
  Serial.readBytes(calrx, 2);
  //Exit & syntax check
  if (calrx[0] == 'e')
  {
    calrx[0] = '\0';
    Homeposition();
    Serial.println("Exiting calibration mode");
    Serial.println();
    return;
  }
  else if (calrx[0] == 's')
  {
    calrx[0] = '\0';
    Serial.println("Saving offsetvalues and sending to Uno..");
    digitalWrite(33, HIGH);
    Serial1.print("t"); Serial1.print(a0c); Serial1.print(","); Serial1.print(a1c); Serial1.print(","); Serial1.print(a2c); Serial1.print(","); Serial1.print(a3c); Serial1.print(","); Serial1.print(a4c); Serial1.print(","); Serial1.print(a5c); Serial1.print(","); Serial1.print(gpc); Serial1.print('$');
    digitalWrite(33, LOW);
    delay(500);
    Homeposition();
    if (Serial1.available())
    {
      char sig = Serial1.read();
      if (sig == 'd')
      {
        Serial.println("Done.");
        Serial.println();
      }
      else
      {
        Serial.println("Cannot send massage to Uno. Check Uno or wiring");
        Serial.println();
      }
    }
    else
    {
      Serial.println("Cannot send massage to Uno. Check Uno or wiring");
      Serial.println();
    }
    return;
  }
  else if (calrx[0] != '0' && calrx[0] != '1' && calrx[0] != '2' && calrx[0] != '3' && calrx[0] != '4' && calrx[0] != '5' && calrx[0] != '6')
  {
    Serial.println("Syntax err!");
    calrx[0] = '\0';
    goto calstart;
  }
  switch (calrx[0])
  {
    case '0':
      Serial.println("Calibrating DC0(angle0). enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal0:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting DC0 cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &a0c);
        if (a0c > 90 || a0c < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal0;
        }
        AngleConstrain();
        DCwrite('0', ANGLE0, ANGLE1);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(a0c); Serial.println(")");
      }
      while (1);
      break;
    case '1':
      Serial.println("Calibrating servo1. enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal1:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting servo1 cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &a1c);
        if (a1c > 90 || a1c < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal1;
        }
        AngleConstrain();
        DCwrite('1', ANGLE0, ANGLE1);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(a1c); Serial.println(")");

      }
      while (1);
      break;
    case '2':
      Serial.println("Calibrating servo2. enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal2:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting servo2 cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &a2c);
        if (a2c > 90 || a2c < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal2;
        }
        AngleConstrain();
        servo1.write(ANGLE2);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(a2c); Serial.println(")");
      }
      while (1);
      break;
    case '3':
      Serial.println("Calibrating servo3. enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal3:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting servo3 cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &a3c);
        if (a3c > 90 || a3c < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal3;
        }
        AngleConstrain();
        servo2.write(ANGLE3);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(a3c); Serial.println(")");
      }
      while (1);
      break;
    case '4':
      Serial.println("Calibrating servo4. enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal4:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting servo4 cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &a4c);
        if (a4c > 90 || a4c < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal4;
        }
        AngleConstrain();
        servo3.write(ANGLE4);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(a4c); Serial.println(")");

      }
      while (1);
      break;
    case '5':
      Serial.println("Calibrating servo5. enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal5:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting servo5 cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &a5c);
        if (a5c > 90 || a5c < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal5;
        }
        AngleConstrain();
        servo4.write(ANGLE5);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(a5c); Serial.println(")");
      }
      while (1);
      break;
    case '6':
      Serial.println("Calibrating gripper. enter offsetvalue. (format: xx$)(range:+-90deg)");
      Serial.println("If calibration is done, enter 'e'");
cal6:
      do
      {
        calrx[0] = '\0';
        do {} while (!Serial.available());
        Serial.readBytes(calrx, 5);
        if (calrx[0] == 'e')
        {
          Serial.println("Exiting gripper cal");
          Serial.println();
          goto calstart;
        }
        sscanf(calrx, "%d$", &gpc);
        if (gpc > 90 || gpc < -90)
        {
          Serial.println("Range or syntax err. please retry");
          Serial.println();
          goto cal6;
        }
        AngleConstrain();

        gripper.write(gripperclose + gpc);
        Serial.print("Done."); Serial.print(" (offsetval : "); Serial.print(gpc); Serial.println(")");

      }
      while (1);
      break;
    default:
      Serial.println("Syntax err! please retry");
      goto calstart;
      break;
  }
}
