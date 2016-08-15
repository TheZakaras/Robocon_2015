  #include <SabertoothSimplified.h>
#include <SoftwareSerial.h>

SabertoothSimplified ST1(Serial1);
SabertoothSimplified ST2(Serial2);
SoftwareSerial BTSerial(10, 11); // RX | TX    //The software serial buffer can hold 64 bytes

volatile byte a[21];
int contInt = 1;
volatile int straycount=0;//Interrupt from controller containing array    // Interrupt 0 is on DIGITAL PIN 2!
int state;
int brake = -1;
int Lx_new;
int Ly_new;
//new variables
int x;  //Joystick X
int y;  //Joystick Y
int t;  //Turning L2,R2
boolean line;  //line present or absent
volatile byte c[21]; //ideal array


/****************************BOT MOTION********************/
#define brakingButton triangle
#define BotCW R2
#define BotCCW L2
#define Speed_toggle up_button

int Speed_toggle_flag = 0;
float spd=1;
float x_1;
float y_1;
int deadz=46;
float V_x;  //Sideways Velocity
float V_y;  //Forward Velocity
float V_t;  //Turning velocity
float rot=0.2;  //Rotation Factor for turning while line following
//drive motors
float m1=0;
float m2=0;
float m3=0;
float m4=0;
float divisor;  //divisor  for normalizing
//int m1in1=46; //for hercules motor driver
//int m1in2=47;
//int m2in1=48;
//int m2in2=49;
//int m3in1=50;
//int m3in2=51;
//int m4in1=52;
//int m4in2=53;
int pwm1=6;
int pwm2=7;
int pwm3=5;
int pwm4=4;
int M1_Centre = 124;
int M2_Centre = 143;
int M3_Centre = 130;
int M4_Centre = 130;
/*********************************************************/

/********************************SERVING****************************/
/*#define Serve square1
volatile int servecount = 1;
int servemotor1a = 24;
int servemotor1b = 22;
int servemotor2a = 26;
int servemotor2b = 28;
int servemotor3a = 30;
int servemotor3b = 32;
int servemotor4a = 34;
int servemotor4b = 36;
int servemotor5a = 38;
int servemotor5b = 40;
int servemotor6a = 42;
int servemotor6b = 44;*/
//int piston_solfor_racquet = 23; //racquet Swing Piston Solenoids already defined in RACQUET SWING
//int piston_solbac_racquet = 25;
/*******************************************************************/

/*******************************Racquet SWING************************/
#define RacquetSwing R1
int piston_solfor_racquet = 23; //racquet Swing Piston Solenoids
int piston_solbac_racquet = 29;

#define RacquetSwing2 L1
int piston_solfor_racquet2 = 25; //racquet Swing Piston Solenoids
int piston_solbac_racquet2 = 27;
/*******************************************************************/

/**************************FLAGS************************************/
#define AllReset square1
int Flag_Brake = 0;
int Flag_Served = 0;
int Flag_Racquet_Swung = 0;
int Flag_Racquet_Swung2 = 0;
/*******************************************************************/

/*********************Assigning PS Buttons*****************************************************/
volatile int start1 = 0;
volatile int select = 0;
volatile int L3 = 0;
volatile int R3 = 0;
volatile int triangle = 0;
volatile int square1 = 0;
volatile int circle = 0;
volatile int cross = 0;
volatile int L2 = 0;
volatile int R2 = 0;
volatile int L1 = 0;
volatile int R1 = 0;
volatile int Lx = 127;
volatile int Ly = 127;
volatile int Rx = 127;
volatile int Ry = 127;
//volatile int up_pressure;
//volatile int down_pressure;
//volatile int left_pressure;
//volatile int right_pressure;
volatile int up_button = 0;
volatile int down_button = 0;
volatile int left_button = 0;
volatile int right_button = 0;
/****************************************************************/

/******************************************************************************************************************/

void setup(){
  Serial.begin(38400);
  Serial1.begin(38400);
  Serial2.begin(38400);
  //SabertoothTXPinSerial.begin(9600);
  BTSerial.begin(38400);
  
  attachInterrupt(contInt, receiveArray, CHANGE);
  
  
  /******BLUETOOTH MODULE***************/
  pinMode(8, INPUT);
  state = digitalRead(8);  //Unpaired = 0, Paired = 1
  
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);        //To be pulled high for AT Mode
  /*************************************/
  
  /*************************IDEAL ARRAY**********************/
  c[0] = 60;
  c[1] = 0;
  c[2] = 0;
  c[3] = 0;
  c[4] = 0;
  c[5] = 0;
  c[6] = 0;
  c[7] = 0;
  c[8] = 0;
  c[9] = 0;
  c[10] = 0;
  c[11] = 0;
  c[12] = 0;
  c[13] = 127;
  c[14] = 127;
  c[15] = 127;
  c[16] = 127;
  //volatile int up_pressure;
  //volatile int down_pressure;
  //volatile int left_pressure;
  //volatile int right_pressure;
  c[17] = 0;
  c[18] = 0;
  c[19] = 0;
  c[20] = 0;
  /**********************************************************/
  
  /****************************BOT MOTION********************/
//  pinMode(m1in1,OUTPUT);
//  pinMode(m1in2,OUTPUT);
//  pinMode(m2in1,OUTPUT);
//  pinMode(m2in2,OUTPUT);
//  pinMode(m3in1,OUTPUT);
//  pinMode(m3in2,OUTPUT);
//  pinMode(m4in1,OUTPUT);
//  pinMode(m4in2,OUTPUT);

  //**********SABERTOOTH************************//

  //*********Analog Mode*************//
//  pinMode(pwm1,OUTPUT);
//  pinMode(pwm2,OUTPUT);
//  pinMode(pwm3,OUTPUT);
//  pinMode(pwm4,OUTPUT);
//  analogWrite(+1,M1_Centre);
//  analogWrite(pwm2,M2_Centre);
//  analogWrite(pwm3,M3_Centre);
//  analogWrite(pwm4,M4_Centre);
  //********************************//

  //*********Serial Mode*************//
  //To prevent random shorting of wires at output of RC circuit
  pinMode(pwm1,INPUT);
  pinMode(pwm2,INPUT);
  pinMode(pwm3,INPUT);
  pinMode(pwm4,INPUT);
  //********************************//
  
  //*******************************************//
  /**********************************************************/

  
  /**********************************SERVING****************************/
  /*pinMode(A0,INPUT);//IR Sensor
  pinMode(servemotor1a,OUTPUT);//Motor 1
  pinMode(servemotor1b,OUTPUT);//Motor 1
  pinMode(servemotor2a,OUTPUT);//Motor 2
  pinMode(servemotor2b,OUTPUT);//Motor 2
  pinMode(servemotor3a,OUTPUT);//Motor 3
  pinMode(servemotor3b,OUTPUT);//Motor 3
  pinMode(servemotor4a,OUTPUT);//Motor 4
  pinMode(servemotor4b,OUTPUT);//Motor 4
  pinMode(servemotor5a,OUTPUT);//Motor 5
  pinMode(servemotor5b,OUTPUT);//Motor 5
  pinMode(servemotor6a,OUTPUT);//Motor 6
  pinMode(servemotor6b,OUTPUT);//Motor 6
  
  digitalWrite(servemotor1a,LOW);//Motor 1
  digitalWrite(servemotor1b,LOW);//Motor 1
  digitalWrite(servemotor2a,LOW);//Motor 2
  digitalWrite(servemotor2b,LOW);//Motor 2
  digitalWrite(servemotor3a,LOW);//Motor 3
  digitalWrite(servemotor3b,LOW);//Motor 3
  digitalWrite(servemotor4a,LOW);//Motor 4
  digitalWrite(servemotor4b,LOW);//Motor 4
  digitalWrite(servemotor5a,LOW);//Motor 5
  digitalWrite(servemotor5b,LOW);//Motor 5
  digitalWrite(servemotor6a,LOW);//Motor 6
  digitalWrite(servemotor6b,LOW);//Motor 6*/
  /*********************************************************************/
  
  /***********************************RACQUET SWING**********************/
  pinMode(piston_solfor_racquet, OUTPUT);
  digitalWrite(piston_solfor_racquet, LOW);
  pinMode(piston_solbac_racquet, OUTPUT);
  digitalWrite(piston_solbac_racquet, LOW);
  
  pinMode(piston_solfor_racquet2, OUTPUT);
  digitalWrite(piston_solfor_racquet2, LOW);
  pinMode(piston_solbac_racquet2, OUTPUT);
  digitalWrite(piston_solbac_racquet2, LOW);
  /*********************************************************************/
  
    
  //LED for indicating start
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  while(!start1)
  {
    BTSerial.write('s');
    attachInterrupt(contInt, receiveArray, CHANGE);
    delay(150);
    Serial.println(start1);
  }
  digitalWrite(13, HIGH); //for hercules motor
}

/******************************************************************************************************************/

void loop() {
  
  state = digitalRead(8);
  if(!state)    //if bluetooth module not paired, then update array with ideal values
  {
    for(int k=0;k<=20;k++)
    {
      a[k]=c[k];
    }
  }
  
  BTSerial.write('s');
  attachInterrupt(contInt, receiveArray, CHANGE);
  
  int start_time = millis();
  for(int i=0; i<=20; i++)
  {
    Serial.print(a[i]);
    Serial.print(" ");
  }
//  start1 = a[1];
//  select = a[2];
//  L3 = a[3];
//  R3 = a[4];
//  triangle = a[5];
//  square1 = a[6];
//  circle = a[7];
//  cross = a[8];
//  L2 = a[9];
//  R2 = a[10];
//  L1 = a[11];
//  R1 = a[12];
//  Lx = a[13];
//  Ly = a[14];
//  Rx = a[15];
//  Ry = a[16];
////  up_pressure = a[17];
////  down_pressure = a[18];
////  left_pressure = a[19];
////  right_pressure = a[20];
//  up_button = a[17];
//  down_button = a[18];
//  left_button = a[19];
//  right_button = a[20];
  
  /*if(start1 == 1)
    Serial.print("    start1 pressed");
  if(select == 1)
    Serial.print("    Select pressed");
  if(L3 == 1)
    Serial.print("    L3 pressed");
  if(R3 == 1)
    Serial.print("    R3 pressed");
  if(L2 == 1)
    Serial.print("    L2 pressed");
  if(R2 == 1)
    Serial.print("    R2 pressed");
  if(L1 == 1)
    Serial.print("    L1 pressed");
  if(R1 == 1)
    Serial.print("    R1 pressed");
  if(triangle == 1)
    Serial.print("    Triangle pressed");        
  if(circle == 1)
    Serial.print("    Circle pressed");
  if(cross == 1)
    Serial.print("    X pressed");
  if(square1 == 1)
    Serial.print("    square1 pressed");     
  Serial.println();
  //if(L2 || R2) 
  //Serial.print("Stick Values:");
  //Serial.print(Ly); //Left stick, Y axis. Other options: LX, RY, RX  
  //Serial.print(",");
  //Serial.print(Lx); 
  //Serial.print(",");
  //Serial.print(Ry); 
  //Serial.print(",");
  //Serial.println(Rx); 
  

  /*Serial.print("Lx: "); //Left stick, Y axis. Other options: LX, RY, RX  
  Serial.print(Lx);
  Serial.print("    Ly: "); //Left stick, Y axis. Other options: LX, RY, RX  
  Serial.print(Ly);
  Serial.println();*/
  

  /*******************************BOT MOTION*********************************/
  Serial.print("Rx:");
  Serial.print(Rx);
  Serial.print("  Ly:");
  Serial.print(Ly);
  x=Rx;
  y=Ly;
  if(BotCW==1)
    t=1;
  else if(BotCCW==1)
    t=-1;
  else t=0;
  
  if(Speed_toggle==1)
    spd = (float)1/2;
  else
    spd = (float)1/6;
    
  Serial.print("  x:");
  Serial.print(x);
  Serial.print("  y:");
  Serial.print(y);
  if(x>=0 && x<(128-deadz/2))
  {
    x_1=(x-(127.5-deadz/2))/(127.5-deadz/2);
  }
  else if(x>128+deadz/2 && x<=256)
  {
    x_1=(x-(127.5+deadz/2))/(127.5-deadz/2);
  }
  else
  {
    x_1 = 0;
  }

  if(y >= 0 && y <(128-deadz/2))
  {
    y_1=-((y-(127.5-deadz/2)) / (127.5-deadz/2));
  }
  else if(y > 128+deadz/2 && y<=256)
  {
    y_1=-((y -(127.5+deadz/2)) / (127.5-deadz/2));
  }
  else
  {
    y_1 = 0;
  }

  if(x_1>=0)  
    V_x = (pow(11,x_1)-1);
  else
    V_x = -pow(11,-x_1)+1;

  if(y_1>=0)
    V_y = pow(11,y_1)-1;
  else
    V_y = -pow(11,-y_1)+1;

  V_t = t;

//  if(ps2x.Button(PSB_PAD_LEFT))  //Change the drive if curve selected
//  {
//    if(mode != 2 || mode !=0)
//    V_x=(abs(x_1)/x_1)*10;
//    V_t= -V_x*rot;  //rot is the multiplication factor which decides difference in absolute values of left and right wheels  
//  }
//m1(Serial1,2);m2(Serial1,1);m3(Serial2,2),m4(Serial2,1)
//***********Analog Mode********//
//  m1 = (V_x - V_y + 1*V_t)*12.7*2*48/25;  //Subtract |V_t| from absolute value of linear velocity
//  m2 = (V_x + V_y - 1*V_t)*12.7*2*38/25;
//  m4 = (-V_x + V_y + 1*V_t)*12.7*2; //Add |V_t| to absolute value of linear velocity
//  m3 = (-V_x - V_y - 1*V_t)*12.7*2;
//******************************//
  Serial.print("  V_x:");
  Serial.print(V_x);
  Serial.print("  V_y:");
  Serial.print(V_y);
  Serial.print("  V_t:");
  Serial.print(V_t);
  
//***********Serial Mode********//
  m1 = (V_x - V_y - 5.8*V_t)*12.7*2;  //Subtract |V_t| from absolute value of linear velocity
  m2 = (-V_x + V_y - 5.8*V_t)*12.7*2;
  m4 = (V_x + V_y + 5.8*V_t)*12.7*2; //Add |V_t| to absolute value of linear velocity
  m3 = (-V_x - V_y + 5.8*V_t)*12.7*2;
//******************************//
  
  if(abs(m1)>127 || abs(m2)>127 || abs(m3)>127 || abs(m4)>127)  //Normalizing the values to counter the overflow
  {
    divisor= max(abs(m1),abs(m2));
    divisor= max(abs(divisor),abs(m3));
    divisor= max(abs(divisor),abs(m4));
    //  divisor= divisor;
    m1= (float)m1*127/divisor;
    m2= (float)m2*127/divisor;
    m4= (float)m4*127/divisor;
    m3= (float)m3*127/divisor;
  }
  
  m1=(float)m1*(float)spd;  //Speed Adjustment
  m2=(float)m2*(float)spd;
  m3=(float)m3*(float)spd;
  m4=(float)m4*(float)spd;
    
  if((!Flag_Brake)&&(brakingButton==1))
  {
    brake = -1*brake;    //brake
    Flag_Brake = 1;
  }
  Serial.print("  Brake:");
  Serial.println(brake);
    
  if(brake!=1)
  {
    
    /*******VOLTAGE EQUALIZING OFFSET***/
    if((int)m2 < 0)
    {
      m2 = (float)((m2*17)/15.2);
    }
    else
    {
      m2 = (float)((m2*17)/15.8);
    }
    if((int)m1 < 0)
    {
      m1 = (float)((m1*17)/14.8);
    }
    else
    {
      m1 = (float)(m1);
    }    
    //Serial 1 Motors are getting lower voltage than Serial 2 Sabers
    m2 = (float)(m2*19.1/18.8);
    m1 = (float)(m1*19.1/18.8);
    if((int)m4 > 0)    //for  sabertooth only!!!
    {
      m4 = (float)((m4*17)/15.1);
    }
    else
    {
      m4 = (float)((m4*17)/15.4);
    }
    if((int)m3 > 0)    //for  sabertooth only!!!
    {
      m3 = (float)((m3*17)/14);
    }
    else
    {
      m3 = (float)((m3*17)/16);
    }
    /***********************************/    
    
    //**********SERIAL MODE**********//
    ST2.motor(2,(int)-m3);
    ST2.motor(1,(int)m4);
    ST1.motor(1,(int)-m2);
    ST1.motor(2,(int)m1);
    
    Serial.print(m1);  
    Serial.print("    ");
    Serial.print((int)m2);
    Serial.print("    ");
    Serial.print((int)m3);
    Serial.print("    ");
    Serial.print((int)m4);
    Serial.print("    ");
    Serial.println();   
    //********************************//
    
    //**********ANALOG MODE**********//
//    analogWrite(pwm1,M1_Centre+(m1*0.5));
//    Serial.print((int)M1_Centre+(m1*0.5));
//    Serial.print("    ");
//    
//    analogWrite(pwm2,M2_Centre+(m2*0.5));
//    Serial.print((int)M2_Centre+(m2*0.5));
//    Serial.print("    ");
//    
//    analogWrite(pwm3,M3_Centre-(m3*0.5));
//    Serial.print((int)M3_Centre-(m3*0.5));
//    Serial.print("    ");
//    
//    analogWrite(pwm4,M4_Centre-(m4*0.5));
//    Serial.print((int)M4_Centre-(m4*0.5));
//    Serial.print("    ");
//    Serial.println();    
    //********************************//
  }
  else
  {
    //SERIAL MODE
    ST2.motor(2,0);
    ST2.motor(1,0);
    ST1.motor(1,0);
    ST1.motor(2,0);
    
    //ANALOG MODE
//    analogWrite(pwm1,M1_Centre);
//    analogWrite(pwm2,M2_Centre);
//    analogWrite(pwm3,M3_Centre);
//    analogWrite(pwm4,M4_Centre);
  }
  /**************************************************************************/
  
  /**********************************SERVING*********************************/
  /*if((!Flag_Served) && (Serve == 1))
  {
    Flag_Served = 1;
    servemotor(servecount++);
  }*/
  /**************************************************************************/
  
  /************************************RACQUET SWING*************************/
  if((!Flag_Racquet_Swung) && (RacquetSwing==1))
  {
    Flag_Racquet_Swung = 1;
    swingracquet();
  }
  
  if((!Flag_Racquet_Swung2) && (RacquetSwing2==1))
  {
    Flag_Racquet_Swung2 = 1;
    swingracquet2();
  }
  /************************************************************************/
  
  if(Flag_Brake || Flag_Served || Flag_Racquet_Swung || Flag_Racquet_Swung2)
  {
    Serial.println("Flag High, press Square");
  }
  
  if(AllReset == 1)
  {         
    Flag_Brake = 0;
    Flag_Served = 0;
    Flag_Racquet_Swung = 0;
    Flag_Racquet_Swung2 = 0;
  }
  
  Serial.println();
  int time_elapsed = millis() - start_time;
  if(time_elapsed < 150)
  delay(150-time_elapsed);  //loop should execute for atleast 150 ms to avoid messing up received data
}

/******************************************************************************************************************/

/**************INTERRUPT ROUTINE FOR RECEIVING ARRAY FROM BLUETOOTH**********/
void receiveArray()
{
  detachInterrupt(contInt);
  if(BTSerial.available())
  {
    a[0] = BTSerial.read();
    if(a[0] == 60)
    {
      //Serial.println("Rec.");
      for(int i=1; i<=20; i++)
      { 
        a[i] = BTSerial.read();
        if(a[i]==255)
        {
          straycount++;
        }
        if (straycount>=4)
        {
          for(int j=1;j<=20;j++)
          {
            a[j]=c[j];
          }
          straycount = 0;
          break;
        }
      }
      straycount = 0;
    }
    BTSerial.flush();  //clear the buffer
  }
  
  start1 = a[1];
  select = a[2];
  L3 = a[3];
  R3 = a[4];
  triangle = a[5];
  square1 = a[6];
  circle = a[7];
  cross = a[8];
  L2 = a[9];
  R2 = a[10];
  L1 = a[11];
  R1 = a[12];
  Lx = a[13];
  Ly = a[14];
  Rx = a[15];
  Ry = a[16];
//  up_pressure = a[17];
//  down_pressure = a[18];
//  left_pressure = a[19];
//  right_pressure = a[20];
  up_button = a[17];
  down_button = a[18];
  left_button = a[19];
  right_button = a[20];
  //attachInterrupt(contInt, receiveArray, CHANGE);
}

/*****************************SERVING************************/
/*void servemotor(int servecount)
{
  //delay(100);
  digitalWrite(servemotor1b,HIGH);  //Motor 1: move outward
 
  while(analogRead(A0)>100)
  {
    Serial.println("Sensing IR");
  }  //sensing shuttle
  digitalWrite(servemotor1b,LOW);       //Motor 1: stop
  Serial.println("Serving");
  delay(175);
  swingracquet();                  //hitting shuttle
  
  digitalWrite(servemotor1a,HIGH);//Motor 1: move inward
  delay(750);
  digitalWrite(servemotor1a,LOW);
  
  if (servecount==1)//Motor 2
  {
    digitalWrite(servemotor2a,HIGH);
    delay(750);
    digitalWrite(servemotor2a,LOW);
    Serial.println("Serve count 1");
  }
  else if (servecount==2)//Motor 3
  {
    digitalWrite(servemotor3a,HIGH);
    delay(750);
    digitalWrite(servemotor3a,LOW);
  }
  else if (servecount==3)//Motor 4
  {
    digitalWrite(servemotor4a,HIGH);
    delay(750);
    digitalWrite(servemotor4a,LOW);
  }
  else if (servecount==4)//Motor 5
  {
    digitalWrite(servemotor5a,HIGH);
    delay(750);
    digitalWrite(servemotor5a,LOW);
  }
  else if (servecount==5)//Motor 5
  {
    digitalWrite(servemotor6a,HIGH);
    delay(750);
    digitalWrite(servemotor6a,LOW);
  }
}*/

void swingracquet()
{
  digitalWrite(piston_solfor_racquet,HIGH);
  //digitalWrite(piston_solbac_racquet,LOW);
  delay(500);
  digitalWrite(piston_solfor_racquet,LOW);
  //digitalWrite(piston_solbac_racquet,LOW);
  delay(50);
  //digitalWrite(piston_solfor_racquet,LOW);
  digitalWrite(piston_solbac_racquet,HIGH);
  delay(500);
  //digitalWrite(piston_solfor_racquet,LOW);
  digitalWrite(piston_solbac_racquet,LOW);
//  delay(50);
}

void swingracquet2()
{
  digitalWrite(piston_solfor_racquet2,HIGH);
  //digitalWrite(piston_solbac_racquet,LOW);
  delay(500);
  digitalWrite(piston_solfor_racquet2,LOW);
  //digitalWrite(piston_solbac_racquet,LOW);
  delay(50);
  //digitalWrite(piston_solfor_racquet,LOW);
  digitalWrite(piston_solbac_racquet2,HIGH);
  delay(500);
  //digitalWrite(piston_solfor_racquet,LOW);
  digitalWrite(piston_solbac_racquet2,LOW);
//  delay(50);
}
