#include "LowPower.h"
#include <RTClib.h>
// #include <Wire.h>

RTC_DS3231 rtc;


#define A 9
#define B 7
#define C 6
#define D 8

#define POT A0
#define CHECKBAT A1
#define BATEN A2
#define BAT0 digitalWrite(BATEN, LOW)
#define BAT1 digitalWrite(BATEN, HIGH)
#define ENPOT 10
#define POT0 digitalWrite(ENPOT, LOW)
#define POT1 digitalWrite(ENPOT, HIGH)
#define IR 11
#define ENIR 12
#define IR0 digitalWrite(ENIR, LOW)
#define IR1 digitalWrite(ENIR, HIGH)
#define ENNixie 4
#define NIXIE0 digitalWrite(ENNixie, LOW)
#define NIXIE1 digitalWrite(ENNixie, HIGH)
#define ENMotor 5
#define MOTOR0 digitalWrite(ENMotor, LOW)
#define MOTOR1 digitalWrite(ENMotor, HIGH)


#define wakeUpPin 2

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 3

int stato = 0;
void wakeUp(){
    stato = 1;
}

void onAlarm() {
    stato = 2;
}

int limits [7] {
  56,
  242,
  384,
  526,
  672,
  840,
  1020};

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

void setup()
{

  Serial.begin(115200);
    // Configure wake up pin as input.
    // This will consumes few uA of current.
    pinMode(wakeUpPin, INPUT);
    pinMode(A, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(C, OUTPUT);
    pinMode(D, OUTPUT);
    pinMode(ENIR, OUTPUT);
    pinMode(IR, INPUT);
    pinMode(ENPOT, OUTPUT);
    pinMode(POT, INPUT);
    pinMode(BATEN, OUTPUT);
    pinMode(CHECKBAT, INPUT);
    pinMode(ENMotor, OUTPUT);

    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1) delay(10);
    }

    if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    //aggiorno data e ora ogni compilazione
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //we don't need the 32K Pin, so disable it
    rtc.disable32K();

    // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    //
    rtc.clearAlarm(2);

    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    // schedule an alarm 10 seconds in the future
    /*if(!rtc.setAlarm1(
            rtc.now() + TimeSpan(0,0,1,0),
            DS3231_A1_Hour// this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen in 60 seconds!");
    }*/
    /*rtc.clearAlarm(1);
    if(!rtc.setAlarm1(
            DateTime(2024, 11, 21, 7, 26, 0), DS3231_A1_Hour
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm set!");
    }*/
    //testSveglia();

    NIXIE0;
    spento();
    
}

void loop() 
{
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(0, wakeUp, LOW);
    
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0);

    Serial.print("STATO = ");
    Serial.println(stato);
    

    //INTERRUPT TOUCH
    if(stato == 1){
      //spengo nixie
      NIXIE0;
      spento();
      int pvalue = checkPot();
      Serial.print("POT ");
      Serial.println(pvalue);
      if(pvalue < limits[0]){  //stato sveglia normale
        Serial.println("CHECK ORA");
        DateTime now = rtc.now();
        mostraOra(now);
        testSveglia();
      }
      else if(pvalue >= limits[0] && pvalue < limits[1]){ //sveglia disattiva
        Serial.println("CHECK ORA NON SUONA");
        DateTime now = rtc.now();
        mostraOra(now);
      }
      else if(pvalue >= limits[1] && pvalue < limits[2]){ //load tape
        Serial.println("LOAD TAPE");
        delay(1000);
        loadTape();
      }


      else if(pvalue >= limits[2] && pvalue < limits[3]){ //carillon
        Serial.println("CARILLON");
        IR1;
        delay(1);
        int IRValue = digitalRead(IR);
        IR0;
        if(IRValue == 1){
          MOTOR1;
        }else{
          MOTOR0;
        }
      }

      
      else if(pvalue >= limits[3] && pvalue < limits[4]){ //battery check
        Serial.println("BATTERY CHECK");
        BAT1;
        delay(50);
        int sensorValue = analogRead(CHECKBAT);
        BAT0;
        int battVal = (sensorValue - 200)/3;
        if(battVal > 9 ) battVal = 9;
        if(battVal < 0 ) battVal = 0;

        Serial.print(sensorValue);
        Serial.print(" ");
        Serial.println(battVal);
        
        NIXIE1;
        delay(100);
        
        printDigit(battVal);
        delay(3000);
        
        NIXIE0;
        spento();

        delay(100);
      }


      else if(pvalue >= limits[4] && pvalue < limits[5]){ //set sveglia
        Serial.println("SET SVEGLIA");
        DateTime alarm = rtc.getAlarm1();
        bool setAlarm = false;
        //Solo mostra la sveglia impostata
        if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
        while(digitalRead(wakeUpPin) == HIGH && setAlarm == false){
          int pvalue = checkPot();
          if(pvalue <= limits[4] || pvalue > limits[5]) // se muovo il cursore, setto la sveglia
            setAlarm = true;
          mostraOra(alarm);
          delay(500);
        }
        if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
        delay(1000);
        if(setAlarm == true){
          int h = 0;
          int m = 0;
          if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
          NIXIE1;
          while(digitalRead(wakeUpPin) == HIGH){
            int pvalue = checkPot();
            h = map(pvalue, 0, 1023, 0, 23);
            
            printDigit(h/10);
            delay(350);
            //spento();
            //delay(100);

            printDigit(h%10);
            delay(350);

            spento();
            delay(300);
            Serial.print("H: ");
            Serial.println(h);          
          }
          if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
          delay(1000);
          while(digitalRead(wakeUpPin) == HIGH){
            int pvalue = checkPot();
            m = map(pvalue, 0, 1023, 0, 59);
            printDigit(m/10);
            delay(350);
            //spento();
            //delay(100);

            printDigit(m%10);
            delay(350);

            spento();
            delay(300);
            Serial.print("M: ");
            Serial.println(m);  
          }
          NIXIE0;
          if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
          delay(100);
          DateTime now = rtc.now();
          if(!rtc.setAlarm1(
                  DateTime(now.year(), now.month(), now.day(), h, m, 0), DS3231_A1_Hour
          )) {
              Serial.println("Error, alarm wasn't set!");
          }else {
              Serial.println("Alarm set!");
          }
          alarm = rtc.getAlarm1();
          mostraOra(alarm);
          //testSveglia();
        }
      }


      else if(pvalue >= limits[5] && pvalue < limits[6]){ //set orologio
        Serial.println("SET OROLOGIO");
        int h = 0;
        int m = 0;
        if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
        NIXIE1;
        while(digitalRead(wakeUpPin) == HIGH){
          int pvalue = checkPot();
          h = map(pvalue, 0, 1023, 0, 23);
          
          printDigit(h/10);
          delay(350);
          //spento();
          //delay(100);

          printDigit(h%10);
          delay(350);

          spento();
          delay(300);
          Serial.print("H: ");
          Serial.println(h);          
        }
        if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
        delay(1000);
        while(digitalRead(wakeUpPin) == HIGH){
          int pvalue = checkPot();
          m = map(pvalue, 0, 1023, 0, 59);
          printDigit(m/10);
          delay(350);
          //spento();
          //delay(100);

          printDigit(m%10);
          delay(350);

          spento();
          delay(300);
          Serial.print("M: ");
          Serial.println(m);  
        }
        NIXIE0;
        if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
        delay(100);
        DateTime now = rtc.now();
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), h, m, 0));
      }else{
        Serial.println("NOTHING");
        delay(100);
        //nothing
      }
      stato = 0;
    }



    //EVENTO SVEGLIA
    if(stato == 2){
      Serial.print("DRIIIIN ");
      // print current time
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.println(date);
      if (rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        Serial.println("Alarm cleared");
      }
      int pvalue = checkPot();

      if(pvalue < limits[0]){  //stato sveglia normale
        //SUONAAA
        loadTape();
        IR1;
        delay(1);
        int IRValue = digitalRead(IR);
        IR0;
        int counter = 0;
        MOTOR1;
        while(IRValue == 1 && counter < 3600 && digitalRead(wakeUpPin) == HIGH){  //20 = 1 sec, 3 min = 20 * 60 * 3
          IR1;
          delay(1);
          IRValue = digitalRead(IR);
          IR0;
          delay(50);
          counter += 1;
          Serial.print(IRValue);
          Serial.print(" ");
          Serial.print(counter);
          Serial.print(" ");
          Serial.println(digitalRead(wakeUpPin));
        }
        MOTOR0;
        if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
      }
      
      stato = 0;
    }
    Serial.println("END LOOP");
    delay(1);
}

void loadTape(){
  
  if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
  IR1;
  delay(1);
  int IRValue = digitalRead(IR);
  Serial.println(IRValue);
  IR0;
  int counter = 0;
  MOTOR1;
  while(IRValue == 0 && counter < 200 && digitalRead(wakeUpPin) == HIGH){ //circa 10 secondi
    IR1;
    delay(1);
    IRValue = digitalRead(IR);
    IR0;
    delay(50);
    counter += 1;
    Serial.print(counter);
    Serial.print(" ");
    Serial.println(IRValue);
  }
  MOTOR0;
  if(digitalRead(wakeUpPin) == LOW){while(digitalRead(wakeUpPin) == LOW){delay(50);}}
  delay(100);
}

void mostraOra(DateTime now){
  spento();
  NIXIE1;
  delay(500);
  int h = now.hour();
  
  printDigit(h/10);
  delay(50);
  printDigit(h/10);
  delay(450);
  spento();
  delay(100);

  printDigit(h%10);
  delay(500);
  spento();
  delay(150);

  delay(250);
  int m = now.minute();
  printDigit(m/10);
  delay(500);
  spento();
  delay(100);

  printDigit(m%10);
  delay(500);
  spento();
  delay(50);
  
  NIXIE0;
  Serial.print("ORA MOSTRATA: ");
  Serial.print(h);
  Serial.print(" ");
  Serial.println(m);
}

int checkPot(){
  POT1;
  delay(1);
  int value = analogRead(POT);
  POT0;
  return value;
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
  NIXIE1;
  for(int i=9;i>=0;i--) {
    digitalWrite(A,digits[i][3]);
    digitalWrite(B,digits[i][2]);
    digitalWrite(C,digits[i][1]);
    digitalWrite(D,digits[i][0]);
    delay(200);
  }
  NIXIE0;
}
void testSveglia() {
  // print current time
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);
  Serial.print(date);

  // the stored alarm value + mode
  DateTime alarm1 = rtc.getAlarm1();
  Ds3231Alarm1Mode alarm1mode = rtc.getAlarm1Mode();
  char alarm1Date[12] = "DD hh:mm:ss";
  alarm1.toString(alarm1Date);
  Serial.print(" [Alarm1: ");
  Serial.print(alarm1Date);
  Serial.print(", Mode: ");
  switch (alarm1mode) {
    case DS3231_A1_PerSecond: Serial.print("PerSecond"); break;
    case DS3231_A1_Second: Serial.print("Second"); break;
    case DS3231_A1_Minute: Serial.print("Minute"); break;
    case DS3231_A1_Hour: Serial.print("Hour"); break;
    case DS3231_A1_Date: Serial.print("Date"); break;
    case DS3231_A1_Day: Serial.print("Day"); break;
  }
  // the value at SQW-Pin (because of pullup 1 means no alarm)
  Serial.print("] SQW: ");
  Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));

  // whether a alarm fired
  Serial.print(" Fired: ");
  Serial.print(rtc.alarmFired(1));

  // Serial.print(" Alarm2: ");
  // Serial.println(rtc.alarmFired(2));
  // control register values (see https://datasheets.maximintegrated.com/en/ds/DS3231.pdf page 13)
  // Serial.print(" Control: 0b");
  // Serial.println(read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), BIN);

  // resetting SQW and alarm 1 flag
  // using setAlarm1, the next alarm could now be configurated
  if (rtc.alarmFired(1)) {
      rtc.clearAlarm(1);
      Serial.print(" - Alarm cleared");
  }
  Serial.println();

}


