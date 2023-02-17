//--------------------------------------------------------------------
//------------------------ VIDI 324 ----------------------------------
//------------ kod VIDI X AI Machine Vision Sensor Robot -------------
//------------------- napisan za potrebe članka u --------------------
//----------------- časopisu VIDI 324 - ožujak 2023. -----------------
//--------------------------------------------------------------------
//
// Kod radi s VIDI project X pločicom o kojoj možete saznati na linku
// https://vidilab.com/vidi-project-x
//
//-------------------------------------------------------------------
// napisao: Hrvoje Šomođi, Vidi - 17.02.2023.
//-------------------------------------------------------------------
//
// Nedostaju li vam niže spomenuti libraryji
// instalirajte ih s priloženih linkova
// za instalaciju biblioteka
// http://librarymanager/All#SparkFun_VL53L1X
// http://librarymanager/All#Adafruit_ILI9341
//
// HUSKYLENS.h library - https://github.com/HuskyLens/HUSKYLENSArduino

#define UART // if true then serial output

#if defined(UART)
// UART output
#else
// No UART output
#endif

#include <analogWrite.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
// Nemate li SparkFun_VL53L1X library, kliknite na link za instalaciju:
// http://librarymanager/All#SparkFun_VL53L1X

#include "HUSKYLENS.h"

HUSKYLENS huskylens;

int ID1 = 1;

int Direction = 0;

void printResult(HUSKYLENSResult result);

//fedinirajmo I2C komunikacijske pinove
#define I2C_SDA 33
#define I2C_SCL 32
//10:24:34.350 -> I2C device found at address 0x29 - Distance Laser
//10:24:34.350 -> I2C device found at address 0x32 - AI Cam

#include "Adafruit_ILI9341.h"
#include "Adafruit_GFX.h"
#include <SPI.h>
//Nemate li library, kliknite na link za instalaciju:
// http://librarymanager/All#Adafruit_ILI9341
// Obavezno kliknite na "Install All" gumb

// ILI9341 TFT LCD deklaracija spajanja zaslona
#define TFT_CS   5
#define TFT_DC  21

Adafruit_ILI9341 TFT = Adafruit_ILI9341(TFT_CS, TFT_DC);

int myWidth;
int myHeight;
int j = 0;

// Desni Motor
const int IN1 = 14;
const int IN2 = 13;

// Lijevi Motor
const int IN3 = 22;
const int IN4 = 12;

// Treći motor
const int IN5 = 27;
const int IN6 = 2;

int mspeed = 150;
int  Duration = 0;

int Last_Direction = 0;
double xComponent;
double yComponent;
double length;
double angle;

double speed;
double rotation;
double r;
double left;
double right;
double d = 0.15; //Whell distance

void Motor_Init() {
  pinMode(IN2, INPUT_PULLUP);
  pinMode(IN4, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
}

// Test motora korištenjem For petlje
void Test(int Duration) {
  // Ostani u For petlji 100 ciklusa
  // int Duration = 100;
#if defined(UART)
  Serial.println("Test Start");
#endif
  Stop();
  for ( j = 0; ++j <= Duration; ) {
    digitalWrite(IN1, 1);
#if defined(UART)
    Serial.println("IN1");
#endif
  }
  Stop();
  for ( j = 0; ++j <= Duration; ) {
    digitalWrite(IN2, 1);
#if defined(UART)
    Serial.println("IN2");
#endif
  }
  Stop();
  for ( j = 0; ++j <= Duration; ) {
    digitalWrite(IN3, 1);
#if defined(UART)
    Serial.println("IN3");
#endif
  }
  Stop();
  for ( j = 0; ++j <= Duration; ) {
    digitalWrite(IN4, 1);
#if defined(UART)
    Serial.println("IN4");
#endif
  }
  Stop();
  for ( j = 0; ++j <= Duration; ) {
    digitalWrite(IN5, 1);
#if defined(UART)
    Serial.println("IN5");
#endif
  }
  Stop();
  for ( j = 0; ++j <= Duration; ) {
    digitalWrite(IN6, 1);
#if defined(UART)
    Serial.println("IN6");
#endif
  }
  Stop();
#if defined(UART)
  Serial.println("Test End");
#endif
}

// Test motora uz pomoć pauze
void DTest(int Duration) {
  // int Duration = 100;
  // Ostani u pauzi 100 milisekundi
#if defined(UART)
  Serial.println("Test Start");
#endif
  Stop();
  //Desno rikverc
  digitalWrite(IN1, 1);
#if defined(UART)
  Serial.println("IN1");
#endif
  delay(Duration);
  Stop();
  // Lijevo naprijed
  digitalWrite(IN2, 1);
#if defined(UART)
  Serial.println("IN2");
#endif
  delay(Duration);
  Stop();
  // Lijevo rikverc
  digitalWrite(IN3, 1);
#if defined(UART)
  Serial.println("IN3");
#endif
  delay(Duration);
  Stop();
  // Desno naprijed
  digitalWrite(IN4, 1);
#if defined(UART)
  Serial.println("IN4");
#endif
  delay(Duration);
  Stop();
  digitalWrite(IN5, 1);
#if defined(UART)
  Serial.println("IN5");
#endif
  delay(Duration);
  Stop();
  digitalWrite(IN6, 1);
#if defined(UART)
  Serial.println("IN6");
#endif
  delay(Duration);
  Stop();
#if defined(UART)
  Serial.println("Test End");
#endif
}

// Stop function
void Stop() {
#if defined(UART)
  Serial.println("Stop");
#endif
  /*
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 1);
    digitalWrite(IN5, 1);
    digitalWrite(IN6, 1);
  */  
  digitalWrite(IN1, LOW); //IN1=0
  digitalWrite(IN2, LOW); //IN2=1
  digitalWrite(IN3, LOW); //IN3=0
  digitalWrite(IN4, LOW); //IN4=1
  digitalWrite(IN5, LOW); //IN5=0
  digitalWrite(IN6, LOW); //IN6=0
#if defined(UART)
  Serial.println(String() + F("IN1=") + digitalRead(IN1));
  Serial.println(String() + F("IN2=") + digitalRead(IN2));
  Serial.println(String() + F("IN3=") + digitalRead(IN3));
  Serial.println(String() + F("IN4=") + digitalRead(IN4));
  Serial.println(String() + F("IN5=") + digitalRead(IN5));
  Serial.println(String() + F("IN6=") + digitalRead(IN6));
#endif
  delay(Duration);
}

// Forward function
void Forward() {
#if defined(UART)
  Serial.println("Forward");
#endif
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, 1);
}

// Back function
void Backwards() {
#if defined(UART)
  Serial.println("Back");
#endif
  digitalWrite(IN1, 1);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, LOW);
}

// Turn  left
void Turn_Left() {
#if defined(UART)
  Serial.println("Left");
#endif
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 1);
}

// Turn  right
void Turn_Right() {
#if defined(UART)
  Serial.println("Right");
#endif
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Turn  left
void Slow_Left(int m_speed) {
#if defined(UART)
  Serial.println("Slow Left");
#endif
  analogWrite(IN1, m_speed);
  analogWrite(IN2, m_speed);
  analogWrite(IN3, 255 - m_speed);
  analogWrite(IN4, 255 - m_speed);
}

// Turn  right
void Slow_Right(int m_speed) {
#if defined(UART)
  Serial.println("Slow Right");
#endif
  analogWrite(IN1, 255 - m_speed);
  analogWrite(IN2, 255 - m_speed);
  analogWrite(IN3, m_speed);
  analogWrite(IN4, m_speed);
}

// Turn  left
void Forward_Left(int m_speed) {
#if defined(UART)
  Serial.println("Forward Left");
#endif
  digitalWrite(IN1, LOW);
  analogWrite(IN2, 255 - m_speed);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, 255);
}

// Turn  right
void Forward_Right(int m_speed) {
#if defined(UART)
  Serial.println("Forward Right");
#endif
  digitalWrite(IN1, LOW);
  analogWrite(IN2, 255);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, 255 - m_speed);
}

void Motor_Up() {
#if defined(UART)
  Serial.println("Motor Up");
#endif
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, 1);
}

void Motor_Down() {
#if defined(UART)
  Serial.println("Motor Down");
#endif
  digitalWrite(IN5, 1);
  digitalWrite(IN6, LOW);
}

void Pico_distance()
{
  for ( j = 0; ++j <= 100; ) //
  {
    Turn_Left();
  }
  for ( j = 0; ++j <= 100; ) //
  {
    Turn_Right();
  }
}

void setup(void)
{
  Motor_Init();
  Stop();
  TFT.begin();                   // inicijalizacuija zaslona
  TFT.setRotation(3);            // postavi orijentaciju
  myWidth  = TFT.width() ;       // ekran je širok?
  myHeight = TFT.height();       // ekran je visok?
  TFT.fillScreen(ILI9341_BLACK); // obojaj zaslon u crno
  TFT.setTextColor(ILI9341_RED); // postavljamo boju teksta u zelenu
  TFT.setTextSize(3);
  TFT.setCursor(0, 170);
  TFT.println("      VIDI X");
  TFT.println(" Autonomni robot ");
  TFT.setTextColor(ILI9341_BLUE);
  TFT.print("==##=========##==");
  TFT.setTextColor(ILI9341_GREEN); // postavljamo boju teksta u zelenu
  TFT.setTextSize(1);
  TFT.setCursor(0, 0);
#if defined(UART)
  Serial.begin(115200);                 //Inicijalizacija serijske komunikacije
#endif
  Wire.begin(I2C_SDA, I2C_SCL); //Inicijalizacija I2C veze
  while (!huskylens.begin(Wire))
  {
#if defined(UART)
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
#endif
    delay( 100 );
  }
#if defined(UART)
  Serial.println(F("Switching the algorithm to line tracking!"));
#endif
  //huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING); //Switch the algorithm to object tracking.
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
}

void AI_kamera() {
  if (!huskylens.request()) {
#if defined(UART)
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
#endif
  }
  else if (!huskylens.isLearned()) {
#if defined(UART)
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
#endif
  }
  else if (!huskylens.available()) {
#if defined(UART)
    Serial.println(F("No block or arrow appears on the screen!"));
#endif
  }
  else
  {
#if defined(UART)
    Serial.println(F("###########"));
#endif
    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();
#if defined(UART)
      printResult(result);
#endif
      Direction = result.xOrigin - result.xTarget;
#if defined(UART)
      Serial.println(String() + Direction);
#endif
    }
  }
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  }
  else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  }
  else {
    Serial.println("Object unknown!");
  }
}

void loop(void)
{
  Stop();
  delay(1000);

  // AI_kamera();
  // Block:xCenter=154,yCenter=221,width=33,height=33,ID=1
  // Arrow:xOrigin=280,yOrigin=66,xTarget=280,yTarget=32,ID=1
  // Arrow:xOrigin=200,yOrigin=238,xTarget=192,yTarget=0,ID=1
  // 337,173

  int32_t error;
  if (!huskylens.request(ID1)) {
    Stop();
#if defined(UART)
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
#endif
    Direction = 0;
  }
  else if (!huskylens.isLearned()) {
    Stop();
#if defined(UART)
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
#endif
    Direction = 0;
  }
  else if (!huskylens.available()) {
#if defined(UART)
    Serial.println(F("No block or arrow appears on the screen!"));
#endif
    if ( Last_Direction == 2 ) {
      Turn_Right();
    }
    else if ( Last_Direction == 1 ) {
      Turn_Left();
    }
    else {
      Stop();
    }
  }
  else
  {
    HUSKYLENSResult result = huskylens.read();
    Direction = result.xOrigin - result.xTarget;
    // result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
    // result.xCenter+ F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);

    //Calculate the x and y components of the vector by subtracting the target coordinates from the origin coordinates:
    xComponent = result.xTarget - result.xOrigin;
    yComponent = result.yTarget - result.yOrigin;

    //Calculate the length of the vector using the Pythagorean theorem:
    length = sqrt((xComponent * xComponent) + (yComponent * yComponent));

    //Calculate the angle of the vector in radians using the inverse tangent function:
    angle = atan(yComponent / xComponent);

    //Calculate the forward speed of the robot as the magnitude of the vector, which is the length you calculated previously:
    speed = length / 10;

    //Determine the distance between the wheels of the robot, which we will call "d".
    //Calculate the radius of the turn that the robot needs to make, which we will call "r". This is equal to d/2 times the tangent of the rotation angle:
    r = ( d / 2.00 ) * tan(angle);

    //Calculate the left and right wheel speeds using the following formulas:
    if ( r == 0 ) {
      left = 0;
      right = 0;
    }
    else
    {
      left = speed * (1 - d / (2 * r));
      right = speed * (1 + d / (2 * r));
    }

#if defined(UART)
    Serial.print(String() + F("xComponent = ") + xComponent);
    Serial.print(String() + F(", yComponent = ") + yComponent);
    Serial.print(String() + F(", length = ") + length);
    Serial.print(String() + F(", angle = ") + angle);
    Serial.println( String(", r = ") + String(r) );
    Serial.print(String() + F("left_speed = ") + left);
    Serial.println(String() + F(",  right_speed = ") + right);
    printResult(result);
#endif

    if (result.yOrigin < 238) {
      Forward();
    }
    else if (length < 100) {
      Forward();
    }
    else if ( ((Direction == 0)) ) {
      Forward();
    }
    else if ( (Direction > 0) && (Direction < 150) ) {
      Forward_Left( int(right - left) );
      Last_Direction = 1;
    }
    else if ( (Direction < 0) && (Direction > -150) ) {
      Forward_Right( int(left - right) );
      Last_Direction = 2;
    }
    else if ( Direction <= -150 ) {
      Turn_Right();
      Last_Direction = 1;
    }
    else if ( Direction >= 150 ) {
      Turn_Left();
      Last_Direction = 2;
    }
    else {
#if defined(UART)
      Serial.println("Else STOP!");
#endif
      Stop();
      delay(Duration);
    }
  }

  // TEST
  //Turn_Right(); Stop();
  //Turn_Left(); delay(Duration);
  //Forward(); delay(Duration0);
  //Back(); delay(Duration);
  //Slow_Left(100); delay(Duration);
  // Turn  right
  //Slow_Right(100); delay(Duration0);
  //Forward_Left(50); delay(Duration);
  //Forward_Right(50); delay(Duration);
  //vozi_senzorom_udaljenosti();
  // Stop(); // Zaustavi sve motore
  // delay(Duration); // Odmori zbog testiranja
  // Možemo krenuti ispočetka mjeriti udaljenosti
  // DTest(Duration); // Koristili smo za testiranje spojeva
  // Mala_udaljenost();
  /*
    for (int o = 0; o < 255; o = o + 1) { //
      Serial.println(String() + "Right o = " + o);
      //Forward_Right(o);
      analogWrite(IN4, o);
      delay(100);
    }

    for (int o = 0; o < 255; o = o + 1) { //
      Serial.println(String() + "Left o = " + o);
      //Forward_Left(o);
      analogWrite(IN4, o);
      delay(100);
    }
  */
}
