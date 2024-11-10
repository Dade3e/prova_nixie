#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

#define A 12
#define B 10
#define C 9
#define D 11
#define MODE 8
#define SET 7
#define SHOW 6

int stato = 0;

int digits [10][4] {
  {0,0,0,0},
  {0,0,0,1},
  {0,0,1,0},
  {0,0,1,1},
  {0,1,0,0},
  {0,1,0,1},
  {0,1,1,0},
  {0,1,1,1},
  {1,0,0,0},
  {1,0,0,1}};

void setup() {

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(MODE, INPUT_PULLUP);
  pinMode(SET, INPUT_PULLUP);
  pinMode(SHOW, INPUT_PULLUP);
  //prova_nixie();

}
int H;
int M;
void loop() {
  if(stato == 0){
    tmElements_t tm;
    if(digitalRead(MODE) == LOW ){
      stato = 1;
      if (RTC.read(tm)) {
        H = tm.Hour;
        M = tm.Minute;
      }else{
        H = 0;
        M = 0;
      }
      while(digitalRead(MODE) == LOW);
    }

    if (RTC.read(tm)) {
      int h = tm.Hour;
      printDigit(h/10);
      delay(500);
      spento();
      delay(250);

      printDigit(h%10);
      delay(500);
      spento();
      delay(250);

      delay(250);
      int m = tm.Minute;
      printDigit(m/10);
      delay(500);
      spento();
      delay(250);

      printDigit(m%10);
      delay(500);
      spento();
      delay(250);
    }
    else{
      printDigit(0);
      delay(1000);
      spento();
    }
    delay(3000);
  }

  if(stato == 1){
    if(digitalRead(MODE) == LOW){
      stato = 2;
      while(digitalRead(MODE) == LOW);
    }
    if(digitalRead(SET) == LOW){
      H += 1;
      if(H > 23)
        H = 0;
      while(digitalRead(SET) == LOW);
    }
    printDigit(H/10);
    delay(500);
    spento();
    delay(250);

    printDigit(H%10);
    delay(500);
    spento();
    delay(250);

    delay(500);
  }

  if(stato == 2){
    if(digitalRead(MODE) == LOW){
      stato = 3;
      while(digitalRead(MODE) == LOW);
    }
    if(digitalRead(SET) == LOW){
      M += 1;
      if(M > 59)
        M = 0;
      while(digitalRead(SET) == LOW);
    }
    printDigit(M/10);
    delay(500);
    spento();
    delay(250);

    printDigit(M%10);
    delay(500);
    spento();
    delay(250);

    delay(500);
  }

  if(stato == 3){
    tmElements_t tm;
    tm.Hour = H;
    tm.Minute = M;
    tm.Second = 0;
    tm.Day = 20;
    tm.Month = 11;
    tm.Year = CalendarYrToTm(2024);
    RTC.write(tm);
    stato = 0;
  }
  
}

void printDigit(int number){
  digitalWrite(A,digits[number][3]);
  digitalWrite(B,digits[number][2]);
  digitalWrite(C,digits[number][1]);
  digitalWrite(D,digits[number][0]);
}

void spento(){
  digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,HIGH);
  digitalWrite(D,HIGH);
}

void prova_nixie() {
 for(int i=0;i<10;i++) {
  digitalWrite(A,digits[i][3]);
  digitalWrite(B,digits[i][2]);
  digitalWrite(C,digits[i][1]);
  digitalWrite(D,digits[i][0]);
  delay(200);
 }
}