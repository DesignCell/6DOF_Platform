
  #include <math.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  #define SERVOMIN  550 // this is the 'minimum' pulse length count (150 out of 4096)
  #define SERVOMAX  150 // this is the 'maximum' pulse length count (600 out of 4096)

  //STMPE610 Touch Screen Controller
  #include <SPI.h>
  #include <Wire.h>
  #include "Adafruit_STMPE610.h"
  Adafruit_STMPE610 touch = Adafruit_STMPE610();
  
  long int delta[6];    // 0=X 1=Y 2=Z 3=X:X 4=Y:Y 5=Z:Z
  int off[6];         // Servo angle home offset to parallel with linkage
  int joy[4];         // Joystick Values 0=x1 1=y1 2=x2 3=y2
  long int XYZ_Base[6][3] = {
    { -9986, 74329, 0},
    {  9986, 74329, 0},
    { 69364,-28516, 0},
    { 59378,-45813, 0},
    {-59387,-45813, 0},
    {-69364,-28516, 0}
  };                         // Plateform - Base
  long int Z_Eff = 135000;   // Plateform - Effector Z Home
  long int XYZ_Eff[6][3] = {
    { -59378, 45813,  Z_Eff },
    {  59378, 45813,  Z_Eff },
    {  69364, 28516,  Z_Eff },   
    {   9986,-74329,  Z_Eff },
    {  -9986,-74329,  Z_Eff },
    { -69364, 28516,  Z_Eff }
  };                        // Plateform - Effector
  long int XYZ_Disp[6][3];       // Plateform - Effector Displaced
  long int XYZ_XX[6][3];         // Plateform - Effector Displaced X:X
  long int XYZ_YY[6][3];         // Plateform - Effector Displaced Y:Y
  long int XYZ_ZZ[6][3];         // Plateform - Effector Displaced Z:Z
  long int XYZ_Act[6][3];        // Actuation XYZ
  long int Act[6];               // Actuation Length
  double pos[6];                 // Servo Angle Output X100
  long int x_limit = 35000;      // MinMax X Limit X1000
  long int y_limit = 30000;      // MinMax Y Limit X1000
  long int z_limit = 20000;      // MinMax Z Limit X1000
  int xx_limit = 16000;          // MinMax XX Limit X1000
  int yy_limit = 16000;          // MinMax YY Limit X1000
  int zz_limit = 25000;          // MinMax ZZ Limit X1000
  long int ServoLink = 50000;    // Servo Linkage Length
  long int ServoArm = 25000;     // Servo Arm Length
  long int ActLimitMax = 162000; // Servo Actuation Limit Max
  long int ActLimitMin = 120000; // Servo Actuation Limit Min
  long int ActExt = 92030;       // Servo Extension
  int scnt = 0;             // Servo cnt
  int xyz = 0;              // XYZ loop cnt
 
  long int xE; // X Error
  long int yE; // Y Error
  long int ixE; // X Integral Error
  long int iyE; // Y Integral Error  
  int xS; // X Sign
  int yS; // Y Sign
  int xSp; // X Sign Previous
  int ySp; // Y Sign Previous
  long int dxE; // X Derivative Delta Error
  long int dyE; // Y Derivative Delta Error
  long int xEL; // X Error Last
  long int yEL; // Y Error Last
  double kp = .28799; // PID:P .15
  double ki = .03369; // PID:I 0.0015
  double kd =0;  // PID:D M2.2
    int ccnt =0;
  int dcnt = 1;            // loop delay 10ms=100Hz
  unsigned long pMillis = 0;
  unsigned long interval = 750000;
  unsigned long PID_Millis;
  
void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  Serial.begin(230400);
  
  touch.begin(0x44);
   
//Home Position
  delta[0] = 0;     //Xmm
  delta[1] = 0;     //Ymm
  delta[2] = 0;     //Zmm
  delta[3] = 0;     //X:X
  delta[4] = 0;     //Y:Y
  delta[5] = 0;     //Z:Z
  
//Servo Home Offset
  off[0] = 3;  //1
  off[1] = 5;  //2
  off[2] = 0;  //3
  off[3] = 0;  //4
  off[4] = 0;  //5
  off[5] = -8;  //6


}

void loop() {

  //=== Read Pots ====================================================================================================================
  //==================================================================================================================================
  unsigned long cMillis = micros();
  if (cMillis - pMillis > interval){
    pMillis = cMillis;
      //kp = map(analogRead(A0), 0,1024, 0.0000,  1000.0000)/1000.0000;
      ki = map(analogRead(A1), 0, 1024, 0.0000,  50000.00000)/100000.000000;
      kd = map(analogRead(A2), 0, 1024, 0.0000, 10000.0000)/1000.0000;
  }

  //=== Read X,Y,Z Offsets ===========================================================================================================
  //==================================================================================================================================
  delta[0] = 0; // map(analogRead(X),0,1024,-x_limit,x_limit)+Xdead;   // Xmm x1000
  delta[1] = 0; // map(analogRead(Y),1024,0,-y_limit,y_limit)+Ydead;    // Ymm x1000
  delta[2] = 0; // map(joy[1],1024,0,-z_limit,z_limit)+79;      // Zmm x1000
    
  //=== Read Screen Error=============================================================================================================
  //==================================================================================================================================

  //XY from STMPE610 Touch Screen Controler
  uint16_t x, y;
  uint8_t z;
  if (touch.touched()) {
    // read x & y & z;
    while (! touch.bufferEmpty()){
      touch.readData(&x, &y, &z);
      xE = map(y,0, 4048,-75000, 75000)-3000; //Reads X axis touch position
      yE = map(x, 0, 4048, -45000, 45000)+3000; //Reads Y axis touch position
    }
    touch.writeRegister8(STMPE_INT_STA, 0xFF);
  }
      else {
        ccnt=ccnt+1;
        if (ccnt>100) {
          ccnt=0;
          xE=0;
          yE=0;
          ixE=0;
          iyE=0;
          dxE=0;
          dyE=0;
        }
    }

  delta[5] = 0;                                              // Z:Z x1000

  //=== PID Control ==================================================================================================================
  //==================================================================================================================================
  
  
  // Integral
  ixE = ixE + ((cMillis-pMillis)*((xEL+xE)/2));
  iyE = iyE + ((cMillis-pMillis)*((yEL+yE)/2));
  //Anti-Windup
//  if (ixE <(-xx_limit)) ixE=-xx_limit;
//  if (ixE >xx_limit) ixE=xx_limit;
//  if (iyE <(-yy_limit)) iyE=-yy_limit;
//  if (iyE >y_limit) iyE=y_limit;

  // Derivative
  dxE = (xE - xEL)/(cMillis-pMillis);
  dyE = (yE - yEL)/(cMillis-pMillis);
  xEL = xE; // Save as Last
  yEL = yE; // Save as Last

  delta[3] = -(yE*kp + iyE*ki + dyE*kd); //X:X
  if (delta[3] <(-xx_limit)) delta[3]=-xx_limit;
  if (delta[3] >xx_limit) delta[3]=xx_limit;
  delta[4] = -(xE*kp + ixE*ki + dxE*kd);
  if (delta[4] <-yy_limit) delta[4]=-yy_limit;
  if (delta[4] >yy_limit) delta[4]=yy_limit;

//  delta[0] = -delta[4];
//  delta[1] = -delta[3];
 
  //=== Effector Displaced X:X =======================================================================================================
  //==================================================================================================================================
  for (scnt =0; scnt <=5; scnt++){
    XYZ_XX[scnt][0] = 0;  // Plateform - Effector Displaced X:X
  }
  for (scnt =0; scnt <=5; scnt++){
    XYZ_XX[scnt][1] = -(XYZ_Eff[scnt][1]-XYZ_Eff[scnt][1]*cos(radians(delta[3]/1000)));  // Plateform - Effector Displaced X:X
  }
  for (scnt =0; scnt <=5; scnt++){
    XYZ_XX[scnt][2] = -XYZ_Eff[scnt][1]*sin(radians(delta[3]/1000))  ;  // Plateform - Effector Displaced X:X
  }

  //=== Effector Displaced Y:Y =======================================================================================================
  //==================================================================================================================================
  for (scnt =0; scnt <=5; scnt++){
    XYZ_YY[scnt][0] = -(XYZ_Eff[scnt][0]-XYZ_Eff[scnt][0]*cos(radians((delta[4]-1500)/1000)));  // Plateform - Effector Displaced Y:Y
  }
  for (scnt =0; scnt <=5; scnt++){
    XYZ_YY[scnt][1] = 0;  // Plateform - Effector Displaced Y:Y
  }
  for (scnt =0; scnt <=5; scnt++){
    XYZ_YY[scnt][2] = -XYZ_Eff[scnt][0]*sin(radians((delta[4]-1500)/1000))  ;  // Plateform - Effector Displaced Y:Y
  }

  //=== Effector Displaced Z:Z =======================================================================================================
  //==================================================================================================================================
  XYZ_ZZ[0][0] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[1][0] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[2][0] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[3][0] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[4][0] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[5][0] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[0][1] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[1][1] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[2][1] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[3][1] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[4][1] = 0  ;  // Plateform - Effector Displaced Z:Z
  XYZ_ZZ[5][1] = 0  ;  // Plateform - Effector Displaced Z:Z
  for (scnt =0; scnt <=5; scnt++){
    XYZ_ZZ[scnt][2] = 0;  // Plateform - Effector Displaced Z:Z
  }

  //=== Plateform - Effector Displaced ===============================================================================================
  //==================================================================================================================================
  for (scnt =0; scnt <= 5; scnt++){
    for (xyz = 0; xyz <=2; xyz++){
      XYZ_Disp[scnt][xyz] = (XYZ_Eff[scnt][xyz] + delta[xyz] + XYZ_XX[scnt][xyz] + XYZ_YY[scnt][xyz] + XYZ_ZZ[scnt][xyz]);
    }
  }
  
  //=== Plateform - Actuation XYZ ====================================================================================================
  //==================================================================================================================================
  for (scnt =0; scnt <= 5; scnt++){
    for (xyz = 0; xyz <=2; xyz++){
      XYZ_Act[scnt][xyz] = abs(XYZ_Base[scnt][xyz] - XYZ_Disp[scnt][xyz]); 
    }
  }
  
  //=== Actuation Length =============================================================================================================
  //==================================================================================================================================
  for (scnt =0; scnt <= 5; scnt++){
    Act[scnt] = sqrt(pow(XYZ_Act[scnt][0],2)+pow(XYZ_Act[scnt][1],2)+pow(XYZ_Act[scnt][2],2));
    if (Act[scnt] < ActLimitMin){
      Act[scnt] = ActLimitMin; 
    }
    if (Act[scnt] > ActLimitMax){
      Act[scnt] = ActLimitMax; 
    }
  }

  //=== Servo Position ===============================================================================================================
  //==================================================================================================================================
  for (scnt =0; scnt <= 5; scnt++){
    pos[scnt] = 180.00-degrees(acos((pow((Act[scnt]-ActExt)/1000,2)+pow(ServoArm/1000,2)-pow(ServoLink/1000,2))/(2*(Act[scnt]-ActExt)/1000*ServoArm/1000)));
  }

  //=== PWM Position  ================================================================================================================
  //==================================================================================================================================
  pwm.setPWM(0,0,map(pos[0]+off[0],0,180,SERVOMIN, SERVOMAX));  //1
  pwm.setPWM(1,0,map(pos[1]+off[1],0,180,SERVOMIN, SERVOMAX));  //2
  pwm.setPWM(2,0,map(pos[2]+off[2],0,180,SERVOMIN, SERVOMAX));  //3
  pwm.setPWM(3,0,map(pos[3]+off[3],0,180,SERVOMIN, SERVOMAX));  //4
  pwm.setPWM(4,0,map(pos[4]+off[4],0,180,SERVOMIN, SERVOMAX));  //5
  pwm.setPWM(5,0,map(pos[5]+off[5],0,180,SERVOMIN, SERVOMAX));  //6
 
  //=== Telemetry ====================================================================================================================
  //==================================================================================================================================
  Serial.print(xE);Serial.print(","); //0
  Serial.print(yE);Serial.print(","); //1
  Serial.print(-xE*kp);Serial.print(","); //2
  Serial.print(-yE*kp);Serial.print(","); //3
  Serial.print(-ixE*ki);Serial.print(","); //4
  Serial.print(-iyE*ki);Serial.print(","); //5
  Serial.print(-dxE*kd);Serial.print(","); //6
  Serial.print(-dyE*kd);Serial.print(","); //7
  Serial.print(kp,4);Serial.print(","); //8
  Serial.print(ki,6);Serial.print(","); //9
  Serial.print(kd,4);Serial.print(","); //10
  Serial.print(delta[3]);Serial.print(","); //11
  Serial.println(delta[4]); //12

  delay(dcnt); 
 }







