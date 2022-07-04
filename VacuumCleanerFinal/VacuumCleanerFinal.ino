#define ESP32_RTOS

#include "OTA.h"
#include <CheapStepper.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "SoundData.h"
#include "XT_DAC_Audio.h"
#include "MusicDefinitions.h"

//AUDIO
int8_t PROGMEM beepsound1[] = {NOTE_C6, SCORE_END};
int8_t PROGMEM beepsound2[] = {NOTE_C5, NOTE_C6, SCORE_END};

XT_Wav_Class stc(starttoclean);     // create an object of type XT_Wav_Class that is used by
// the dac audio class (below), passing wav data as parameter.

XT_DAC_Audio_Class DacAudio(25, 0);
XT_MusicScore_Class beep1(beepsound1, TEMPO_ALLEGRO, INSTRUMENT_PIANO);
XT_MusicScore_Class beep2(beepsound2, TEMPO_ALLEGRO, INSTRUMENT_PIANO);
XT_Wav_Class automode(am);
XT_Wav_Class manualmode(mm);
XT_Wav_Class alarmsound(doorlockalarmsounddata);

//PIN DEFINITION
#define MR1 23
#define MR2 26
#define MR3 27
#define MR4 32
#define ML1 2
#define ML2 12
#define ML3 16
#define ML4 19
#define TS1 5
#define TS2 34
#define TS3 35
#define TS4 33
#define TS5 39
#define BSL 14
#define BSR 13
#define IRL 21
#define LED 4
#define DACSWITCH 22
#define BATTERY 36


//CREDENTIALS
const char* mySSID = "cghs";
const char* myPASSWORD = "vatsanabha1972";
const char* auth = "2nDQIWU8N_T681AVyPzKPueY4lOPou5-";

//BLDC MOTOR CONTROL
#define BLDC_PWM_COUNT_LOW 13
#define BLDC_PWM_COUNT_HIGH 24
#define START_BLDC_PWM_COUNT 1
#define BLDC_PWM_TIMER_WIDTH 8


//MOTOR PINS
CheapStepper stepperl(ML1, ML2, ML3, ML4);
CheapStepper stepperr(MR1, MR2, MR3, MR4);

//PWM VARIABLES
int pwmfreq = 50000, pwmfreqbldc = 50;
short int impeller = 7, brushmotor = 0, led = 1;
short int pwmresolution = 8;


//PARAMETRIC VARIABLES
float currentangle = 0, x = 1, y = 1, tiredist = 8.57, tiredia = 2, numberofsteps = 4096, lastrotationdeg = 0, tempprevrotdeg = -1, tiredistinsteps = 5571.05;
long mainstepcount = 0;
boolean moveClockwise = true;
int stepnumberl = 0, stepnumberr = 0;

// IR LED VARIABLES
short int irarray[5] = {1, 1, 1, 1, 1}, ircodearray[5], cornerdetect = 0, cornerthreshold = 20;
int irbuffsize = 8, irbitoffset = 2000;
bool leddata = 0, irread = 0, irenable = 1, discardcode = 0, prevircodeval = 0;
bool irstring[5][8];
char lcode[] = "11001001", rcode[] = "11010001", mcode[] = "11011001";

// WIRELESS CONTROL
short int reql = 0, reqr = 0, opmode = 2, battery = 0, prevopmode = 2, impellerval = 0, brushval = 0;
bool rstmotor = 1, docked = 1, setupcomplete = 0, online = false, doorlockalarm = false;
//WidgetLCD lcd(V6);

//TIMERS
unsigned long infoupdate = 0, currcore1 = 0, currcore0 = 0;


//TOUCH VARIABLES
short int threshold = 45, prevtval = 0;
bool disabletouch=0;


// FUNCTIONS
void rotate(float deg, bool recordpos = 1);
void moveforward(long steps, bool recordpos = 1);
void fixrotatel(long steps, bool recordpos = 1);
void fixrotater(long steps, bool recordpos = 1);
void changemode(short int m);
void playsound(short int x = 0, int vol = 100);
void ledfade(short int delt = 1, bool fadein = 1, bool fadeout = 1);


TaskHandle_t Task1;



BLYNK_WRITE(V0)
{
  if (param.asInt() == 1)
  {
    reql = 1;
  }
  else
  {
    motoroff();
    reql = 0;
  }
  docked = 0;
}
BLYNK_WRITE(V2)
{
  if (param.asInt() == 1)
  {
    reql = 2;
  }
  else
  {
    motoroff();
    reql = 0;
  }
  docked = 0;
}

BLYNK_WRITE(V1)
{
  if (param.asInt() == 1)
  {
    reqr = 2;
  }
  else
  {
    motoroff();
    reqr = 0;
  }
  docked = 0;
}

BLYNK_WRITE(V3)
{
  if (param.asInt() == 1)
  {
    reqr = 1;
  }
  else
  {
    motoroff();
    reqr = 0;
  }
  docked = 0;
}

BLYNK_WRITE(V4)
{
  impellerval = param.asInt();
  if (impellerval == 0)
  {
    ledcWrite(impeller, BLDC_PWM_COUNT_LOW);
  }

  else
  {
    if (battery >= 0)
    {
      ledcWrite(impeller, map(impellerval, 1, 10, BLDC_PWM_COUNT_LOW+START_BLDC_PWM_COUNT, BLDC_PWM_COUNT_HIGH));
    }
    else
    {
      impellerval = 0;
      ledcWrite(impeller, BLDC_PWM_COUNT_LOW);
    }
  }
}

BLYNK_WRITE(V5)
{
  brushval = param.asInt();
  if (brushval == 0)
  {
    ledcWrite(brushmotor, 0);
  }

  else
  {
    if (battery >= 0)
    {
      ledcWrite(brushmotor, map(brushval, 1, 10, 60, 255));

    }
    else
    {
      brushval = 0;
      ledcWrite(brushmotor, 0);
    }
  }
}

BLYNK_WRITE(V7)
{

  changemode(param.asInt());

}

BLYNK_WRITE(V8)
{
  doorlockalarm = param.asInt();
  
}

void setup() {
  Serial.begin(115200);
  //stepperl.setRpm(15);
  //stepperr.setRpm(15);
  ledcSetup(impeller, pwmfreqbldc, BLDC_PWM_TIMER_WIDTH);
  ledcAttachPin(17, impeller);
  ledcSetup(brushmotor, pwmfreq, pwmresolution);
  ledcAttachPin(18, brushmotor);
  ledcSetup(led, pwmfreq, pwmresolution);
  ledcAttachPin(LED, led);
  ledcWrite(impeller, BLDC_PWM_COUNT_LOW);
  ledcWrite(brushmotor, 0);
  ledcWrite(led, 0);
  
  pinMode(TS1, INPUT);
  pinMode(BSL, INPUT_PULLUP);
  pinMode(BSR, INPUT_PULLUP);
  pinMode(TS2, INPUT);
  pinMode(TS3, INPUT);
  pinMode(TS4, INPUT);
  pinMode(TS5, INPUT);

  pinMode(IRL, OUTPUT);
  
  pinMode(DACSWITCH, OUTPUT);
  
  digitalWrite(DACSWITCH, 0);
  digitalWrite(IRL, 1);

  
  setupOTA("Vacuum Cleaner", mySSID, myPASSWORD);
  
  digitalWrite(IRL, 0);
  
  if (WiFi.waitForConnectResult() == WL_CONNECTED)
  {
    Serial.println("CONNECTED");
    online = true;
  }

  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
  unsigned long temptimer = millis();
  while (!setupcomplete)
  {
    ledfade(10, 1, 1);
    if(millis()-temptimer>60000ul)
    {
      ESP.restart();
    }
  }
  playsound(0);

}

void Task1code( void * parameter)
{
  if (online)
  {
    Blynk.begin(auth, mySSID, myPASSWORD);
    Blynk.virtualWrite(V4, 0);
    Blynk.virtualWrite(V5, 0);
    Blynk.virtualWrite(V7, opmode);
  }
  threshold = touchRead(T3) - 3;
  setupcomplete = 1;

  for (;;)
  {
    irsensor();
    menu();
    blynkupdate(millis());
    powermanagement();
    delay(200);
  }

}


void loop()
{
  //currcore1 = millis();

  if (prevopmode != opmode)
  {
    prevopmode = opmode;
    if (opmode == 1 && calcdist(1, 1) < 5000)
    {
      docked = 0;
      moveforward(-5000);
      rotate(180);
    }
    motoroff();
  }
  
  if (opmode == 1)
  {
    //stepperl.step(1);
    //stepperr.step(0);
    fstepl(1);
    fstepr(0);
    mainstepcount+=2;
    delay(2);
    avoidobstacle();
  }

  if (opmode == 2)
  {
    wirelesscontrol();
  }

  if (opmode == 3)
  {
    disabletouch = 1;
    getpos();
    zerostepval();
    ledfade(3, 1, 0);
    if (calcdist(1, 1) > 15000)
    {
      returnhome(-15000, 1);
    }
    followir();
    changemode(2);
    docked = 1;
    x = 1;
    y = 1;
    currentangle = 0;
    ledfade(10, 0, 1);
    disabletouch = 0;
  }

  if(doorlockalarm && online)
  {
    digitalWrite(DACSWITCH, 1);
    disabletouch = 1;
    while(doorlockalarm)
    {
      DacAudio.FillBuffer();
      if(alarmsound.Playing==false)
      {
        DacAudio.Play(&alarmsound);
      }
    }
    disabletouch = 0;
    digitalWrite(DACSWITCH, 0);
  }

}

void changemode(short int m)
{
  if (prevopmode == m)
  {
    return;
  }

  opmode = m;
  impellerval = 0;
  brushval = 0;
  switch (m)
  {
    case 1: irenable = 1;
      impellerval = 2;
      brushval = 3;
      break;

    case 2: irenable = 0;
      reql = 0;
      reqr = 0;
      break;

    case 3:
      break;

  }
  digitalWrite(IRL, 0);
  if (impellerval == 0)
  {
    ledcWrite(impeller, BLDC_PWM_COUNT_LOW);
  }
  else
  {
    ledcWrite(impeller, map(impellerval, 1, 10, BLDC_PWM_COUNT_LOW+START_BLDC_PWM_COUNT, BLDC_PWM_COUNT_HIGH));
  }

  if (brushval == 0)
  {
    ledcWrite(brushmotor, 0);
  }
  else
  {
    ledcWrite(brushmotor, map(brushval, 1, 10, 60, 255));
  }

}

void findmiddle()
{
  float ang1 = 0;
  short int i = 0;
  while (ircodearray[2] != 2)
  {
    irdatarx();
    rotate(1);
    if (ircodearray[2] == 0)
    {
      i++;
      if (i == 10)
      {
        i = 0;
        break;
      }
    }
  }
  rotate(-3);
  ang1 += 3;
  i = 0;
  while (ircodearray[2] != 1)
  {
    irdatarx();
    rotate(-1);
    ang1++;
    if (ircodearray[2] == 0)
    {
      i++;
      if (i == 10)
      {
        i = 0;
        break;
      }
    }
  }
  rotate(ang1 / 2);

}

void followir()
{
  irenable = 0;
  digitalWrite(IRL, 0);
  if (docked)
  {
    return;
  }
  short int a[4] = {0};
  while (digitalRead(BSL) == 0 && digitalRead(BSR) == 0)
  {
    irdatarx();

    if (discardcode == 1)
    {
      continue;
    }

    switch (ircodearray[2])
    {
      case 0: a[0]++;
        if (lastrotationdeg > 0)
        {
          rotate(2);
        }
        else
        {
          rotate(-2);
        }
        moveforward(200);

        if (a[0] == 220)
        {

          returnhome(-5000, 1);
        }
        if (a[0] > 440)
        {
          playsound(3);
          return;
        }
        break;

      case 3: a[3]++;
        if (a[3] == 30)
        {
          findmiddle();
        }
        moveforward(100);
        break;

      case 1: a[1]++;
        fixrotater(200);
        break;

      case 2: a[2]++;
        fixrotatel(-200);
        break;
    }

    switch (ircodearray[3])
    {
      case 1:
        rotate(30);
        moveforward(2000);
        rotate(-60);
        moveforward(500);
        break;

      case 2:
        rotate(-120);
        moveforward(2000);
        rotate(60);
        moveforward(500);
        break;

      case 3:
        rotate(-27);
        moveforward(500);
        break;

    }

    switch (ircodearray[0])
    {
      case 2:
        rotate(-10);
        moveforward(2000);
        rotate(60);
        moveforward(500);
        break;

      case 1:
        rotate(160);
        moveforward(2000);
        rotate(-60);
        moveforward(500);
        break;

      case 3:
        rotate(80);
        moveforward(500);
        break;

    }

    switch (ircodearray[1])
    {
      case 2:
        rotate(-30);
        moveforward(2000);
        rotate(60);
        moveforward(500);
        break;

      case 1:
        rotate(120);
        moveforward(2000);
        rotate(-60);
        moveforward(500);
        break;

      case 3:
        rotate(27);
        moveforward(500);
        break;

    }

    switch (ircodearray[4])
    {
      case 1:
        rotate(10);
        moveforward(2000);
        rotate(-60);
        moveforward(500);
        break;

      case 2:
        rotate(-160);
        moveforward(2000);
        rotate(60);
        moveforward(500);
        break;

      case 3:
        rotate(-80);
        moveforward(500);
        break;

    }
    continue;
    if (ircodearray[3] != 0)
    {
      rotate(-27);
      moveforward(500);
    }
    if (ircodearray[1] != 0)
    {
      rotate(27);
      moveforward(500);
    }

    if (ircodearray[0] != 0)
    {
      rotate(80);
      moveforward(500);
    }

    if (ircodearray[4] != 0)
    {
      rotate(-80);
      moveforward(500);
    }

  }
  moveforward(-100);
  playsound(0);
}


void powermanagement()
{
  if (battery <= 25)
  {
    if(impellerval!=0)
    {
      impellerval = 0;
      ledcWrite(impeller, BLDC_PWM_COUNT_LOW);
      
    }
    if(brushval!=0)
    {
      brushval = 0;
      ledcWrite(brushmotor, 0);
    }
    
    if (opmode != 2 && opmode != 3)
    {
      changemode(3);
    }
  }

}

void blynkupdate(unsigned long t)
{
  battery = map(analogRead(BATTERY), 2579, 3869, 0, 100);
  if (online && t - infoupdate > 5000ul)
  {
    infoupdate = t;
    Blynk.virtualWrite(V6, battery);
    Blynk.virtualWrite(V4, impellerval);
    Blynk.virtualWrite(V5, brushval);
    Blynk.virtualWrite(V7, opmode);
    //Blynk.virtualWrite(V8, prevtval);
    //Blynk.virtualWrite(V9, (float)y*0.00153);
    //Blynk.virtualWrite(V10, currentangle);

  }
  
  Blynk.run();

}

void returnhome(long p, long q)
{
//  irenable = 0;
//  digitalWrite(IRL, 0);
  rotate(findangle(p, q), 0);
  Serial.print("VALUE OF X AND Y IS: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  long reqsteps = calcdist(p, q);
  cornerdetect = 0;
  tempprevrotdeg = -1;
  while (1)
  {
    if (mainstepcount >= reqsteps)
    {
      x = p;
      y = q;
      motoroff();
      zerostepval();
      changemode(2);
      break;
    }
    fstepl(1);
    fstepr(0);
    mainstepcount+=2;
    delay(2);
    if(mainstepcount>3000)
    {
      cornerdetect = 0;
    }
    if (digitalRead(BSL))
    {
      getpos();
      zerostepval();
      moveforward(-500);
      float angle = currentangle;
      if(tempprevrotdeg >0)
      {
        cornerdetect++;
        
      }
      if(cornerdetect>4)
      {
        cornerdetect = 0;
        rotate(180);
      }
      else
      {
        rotate(-30);
      }
      tempprevrotdeg = lastrotationdeg;
      while (abs(currentangle - angle) > 5)
      {
        leftfollow();
      }
      rotate(findangle(p, q));
      reqsteps = calcdist(p, q);
    }

    if (digitalRead(BSR))
    {
      getpos();
      zerostepval();
      moveforward(-500);
      float angle = currentangle;
      if(tempprevrotdeg<0)
      {
        cornerdetect++;
        
      }
      if(cornerdetect>4)
      {
        cornerdetect = 0;
        rotate(180);
      }
      else
      {
        rotate(30);
      }
      tempprevrotdeg = lastrotationdeg;
      while (abs(currentangle - angle) > 5)
      {
        rightfollow();
      }
      rotate(findangle(p, q));
      reqsteps = calcdist(p, q);
    }

  }

}

void leftfollow()
{
  switch(readirdata())
  {
    case -1: rotate(1);
    break;
    
    case 10000: rotate(-1);
    break;

    case 1000: rotate(-10);
    break;

    case 100: rotate(-90);
    break;

    case 11000: rotate(-15);
    break;

    case 10100: rotate(-90);
    break;

    case 1100: rotate(-90);
    break;

    case 11100: rotate(-90);
    break;
  }
  
  if (digitalRead(BSL))
  {
    moveforward(-500);
    if(tempprevrotdeg>0)
    {
      cornerdetect++;
      
    }
    if(cornerdetect>4)
    {
      cornerdetect = 0;
      rotate(180);
    }
    else
    {
      rotate(-30);
    }
    tempprevrotdeg = lastrotationdeg;
  }
  if (digitalRead(BSR))
  {
    moveforward(-500);
    if(tempprevrotdeg<0)
    {
      cornerdetect++;
      
    }
    if(cornerdetect>4)
    {
      cornerdetect = 0;
      rotate(180);
    }
    else
    {
      rotate(30);
    }
    tempprevrotdeg = lastrotationdeg;
  }
  moveforward(100);

}

void rightfollow()
{
  switch(readirdata())
  {
    case -1: rotate(-1);
    break;
    
    case 1: rotate(1);
    break;

    case 10: rotate(10);
    break;

    case 100: rotate(90);
    break;

    case 11: rotate(15);
    break;

    case 110: rotate(90);
    break;

    case 111: rotate(90);
    break;

    case 101: rotate(90);
    break;
  }
  if (digitalRead(BSL))
  {
    moveforward(-500);
    if(tempprevrotdeg>0)
    {
      cornerdetect++;
      
    }
    if(cornerdetect>4)
    {
      cornerdetect = 0;
      rotate(180);
    }
    else
    {
      rotate(-30);
    }
    tempprevrotdeg = lastrotationdeg;
  }
  if (digitalRead(BSR))
  {
    moveforward(-500);
    if(tempprevrotdeg<0)
    {
      cornerdetect++;
      
    }
    if(cornerdetect>4)
    {
      cornerdetect = 0;
      rotate(180);
    }
    else
    {
      rotate(30);
    }
    tempprevrotdeg = lastrotationdeg;
  }
  moveforward(100);

}


void ledfade(short int delt, bool fadein, bool fadeout)
{
  short int brightness = 0;
  if (fadein)
  {
    for (brightness = 0 ; brightness < 255; brightness++)
    {
      ledcWrite(led, brightness);
      delay(delt);

    }
    digitalWrite(LED , HIGH);
  }

  if (fadeout)
  {
    brightness = 256;
    while (brightness != 0)
    {
      brightness--;
      ledcWrite(led, brightness);
      delay(delt);
    }
  }
}

void playsound(short int x, int vol)
{
  if(disabletouch)
  {
    return;
  }
  digitalWrite(DACSWITCH, 1);
  DacAudio.DacVolume = vol;

  if (x == 0)
  {
    DacAudio.Play(&beep1);
    while (beep1.Playing)
    {
      DacAudio.FillBuffer();
    }

  }

  if (x == 1)
  {
    DacAudio.Play(&stc);
    while (stc.Playing)
    {
      DacAudio.FillBuffer();
    }

  }

  if (x == 2)
  {
    DacAudio.Play(&automode);
    while (automode.Playing)
    {
      DacAudio.FillBuffer();
    }

  }

  if (x == 3)
  {
    DacAudio.Play(&beep2);
    while (beep2.Playing)
    {
      DacAudio.FillBuffer();
    }

  }

  if (x == 4)
  {
    DacAudio.Play(&manualmode);
    while (manualmode.Playing)
    {
      DacAudio.FillBuffer();
    }
  }

  digitalWrite(DACSWITCH, 0);
}


void menu()
{
  if(disabletouch)
  {
    return;
  }
  if (touchRead(T3) < threshold)
  {

    unsigned long ttimer = millis();
    while (touchRead(T3) <= threshold)
    {
      delay(10);
      if (millis() - ttimer > 50ul)
      {
        break;
      }
    }
    unsigned long curr = millis() - ttimer;
    if (curr < 50ul)
    {
      return;
    }

    else
    {
      playsound(0);
      if (opmode == 2)
      {
        playsound(2);
        changemode(1);

      }
      else
      {
        playsound(4);
        changemode(2);
      }
      prevtval = touchRead(T3);
      if (prevtval >= 30)
      {
        threshold = prevtval - 2;
      }
      ledfade(1, 1, 1);
    }
  }
}


void wirelesscontrol()
{
  short int st = reql * 3 + reqr;
  if (st == 0)
  {
    if (rstmotor)
    {
      motoroff();
      rstmotor = 0;
    }
  }
  else
  {
    rstmotor = 1;
  }
  switch (st)
  {
    case 5: moveforward(1);
      break;

    case 7: moveforward(-1);
      break;

    case 4: rotate(-1);
      break;

    case 8: rotate(1);
      break;

    case 2: fixrotater(1);
      break;

    case 1: fixrotater(-1);
      break;

    case 3: fixrotatel(-1);
      break;

    case 6: fixrotatel(1);
      break;

  }
}

void fixrotatel(long steps, bool recordpos)
{
  if (steps == 0)
  {
    return;
  }
  if (recordpos)
  {
    getpos();
  }
  zerostepval();
  float anglemoved = (float)(steps / tiredistinsteps) * (180 / PI);
  float circlex, circley;
  circlex = x + ((tiredistinsteps / 2) * cos(((currentangle + 270) * PI) / 180));
  circley = y + ((tiredistinsteps / 2) * sin(((currentangle + 270) * PI) / 180));
  currentangle += anglemoved;
  fixangle();
  x = circlex + (tiredistinsteps / 2) * cos(((currentangle + 90) * PI) / 180);
  y = circley + (tiredistinsteps / 2) * sin(((currentangle + 90) * PI) / 180);

  if (steps < 0)
  {
    moveClockwise = 1;
  }
  else
  {
    moveClockwise = 0;
  }
  long setVal = abs(steps);
  while (1)
  {
    stepperl.step(moveClockwise);
    mainstepcount++;
    if (mainstepcount >= setVal)
    {
      break;
    }
  }
  zerostepval();
  //motoroff();

}

void fixrotater(long steps, bool recordpos)
{
  if (steps == 0)
  {
    return;
  }
  if (recordpos)
  {
    getpos();
  }
  zerostepval();
  float anglemoved = (float)(steps / tiredistinsteps) * (180 / PI);
  float circlex, circley;
  circlex = x + ((tiredistinsteps / 2) * cos(((currentangle + 90) * PI) / 180));
  circley = y + ((tiredistinsteps / 2) * sin(((currentangle + 90) * PI) / 180));
  currentangle += anglemoved;
  fixangle();
  x = circlex + (tiredistinsteps / 2) * cos(((currentangle + 270) * PI) / 180);
  y = circley + (tiredistinsteps / 2) * sin(((currentangle + 270) * PI) / 180);
  if (steps < 0)
  {
    moveClockwise = 1;
  }
  else
  {
    moveClockwise = 0;
  }
  long setVal = abs(steps);
  while (1)
  {
    stepperr.step(moveClockwise);
    mainstepcount++;
    if (mainstepcount >= setVal)
    {
      break;
    }
  }
  zerostepval();
  //motoroff();

}


void irtxcode(char* x)
{
  int i = 0;
  for (i = 0; i < strlen(x); i++)
  {

    if (x[i] == '0')
    {
      digitalWrite(IRL, LOW);
      delayMicroseconds(irbitoffset * 2);

    }
    else
    {
      digitalWrite(IRL, HIGH);
      delayMicroseconds(irbitoffset * 2);

    }
    digitalWrite(IRL, LOW);
    delayMicroseconds(irbitoffset * 2);
  }
}

void fstepl(bool dir)
{
  if (dir) {
    switch (stepnumberl) {
      case 0:
        digitalWrite(ML1, HIGH);
        digitalWrite(ML2, LOW);
        digitalWrite(ML3, LOW);
        digitalWrite(ML4, LOW);
        break;
      case 1:
        digitalWrite(ML1, LOW);
        digitalWrite(ML2, HIGH);
        digitalWrite(ML3, LOW);
        digitalWrite(ML4, LOW);
        break;
      case 2:
        digitalWrite(ML1, LOW);
        digitalWrite(ML2, LOW);
        digitalWrite(ML3, HIGH);
        digitalWrite(ML4, LOW);
        break;
      case 3:
        digitalWrite(ML1, LOW);
        digitalWrite(ML2, LOW);
        digitalWrite(ML3, LOW);
        digitalWrite(ML4, HIGH);
        break;
    }
  } else {
    switch (stepnumberl) {
      case 0:
        digitalWrite(ML1, LOW);
        digitalWrite(ML2, LOW);
        digitalWrite(ML3, LOW);
        digitalWrite(ML4, HIGH);
        break;
      case 1:
        digitalWrite(ML1, LOW);
        digitalWrite(ML2, LOW);
        digitalWrite(ML3, HIGH);
        digitalWrite(ML4, LOW);
        break;
      case 2:
        digitalWrite(ML1, LOW);
        digitalWrite(ML2, HIGH);
        digitalWrite(ML3, LOW);
        digitalWrite(ML4, LOW);
        break;
      case 3:
        digitalWrite(ML1, HIGH);
        digitalWrite(ML2, LOW);
        digitalWrite(ML3, LOW);
        digitalWrite(ML4, LOW);


    }
  }
  stepnumberl++;
  if (stepnumberl > 3) {
    stepnumberl = 0;
  }


}

void fstepr(bool dir)
{
  if (dir) {
    switch (stepnumberr) {
      case 0:
        digitalWrite(MR1, HIGH);
        digitalWrite(MR2, LOW);
        digitalWrite(MR3, LOW);
        digitalWrite(MR4, LOW);
        break;
      case 1:
        digitalWrite(MR1, LOW);
        digitalWrite(MR2, HIGH);
        digitalWrite(MR3, LOW);
        digitalWrite(MR4, LOW);
        break;
      case 2:
        digitalWrite(MR1, LOW);
        digitalWrite(MR2, LOW);
        digitalWrite(MR3, HIGH);
        digitalWrite(MR4, LOW);
        break;
      case 3:
        digitalWrite(MR1, LOW);
        digitalWrite(MR2, LOW);
        digitalWrite(MR3, LOW);
        digitalWrite(MR4, HIGH);
        break;
    }
  } else {
    switch (stepnumberr) {
      case 0:
        digitalWrite(MR1, LOW);
        digitalWrite(MR2, LOW);
        digitalWrite(MR3, LOW);
        digitalWrite(MR4, HIGH);
        break;
      case 1:
        digitalWrite(MR1, LOW);
        digitalWrite(MR2, LOW);
        digitalWrite(MR3, HIGH);
        digitalWrite(MR4, LOW);
        break;
      case 2:
        digitalWrite(MR1, LOW);
        digitalWrite(MR2, HIGH);
        digitalWrite(MR3, LOW);
        digitalWrite(MR4, LOW);
        break;
      case 3:
        digitalWrite(MR1, HIGH);
        digitalWrite(MR2, LOW);
        digitalWrite(MR3, LOW);
        digitalWrite(MR4, LOW);


    }
  }
  stepnumberr++;
  if (stepnumberr > 3)
  {
    stepnumberr = 0;
  }


}

void avoidobstacle()
{
  if (digitalRead(BSL))
  {
    moveforward(-500);
    if (lastrotationdeg == 30)
    {
      rotate(30);
    }
    else
    {
      rotate(-30);
    }
  }
  if (digitalRead(BSR))
  {
    moveforward(-500);
    if (lastrotationdeg == -30)
    {
      rotate(-30);
    }
    else
    {
      rotate(30);
    }
  }
  int val = readirdata();
  float prevrot=0;
  if(val!=0)
  {
    prevrot = lastrotationdeg;
  }
  
  switch (val)
  {
    case 10000: rotate(-1);
      break;
    case 10001: rotate(180);
      break;
    case 11000: rotate(-5);
      break;
    case 1000: rotate(-5);
      break;
    case 1100: rotate(-30);
      break;
    case 10100: rotate(-30);
      break;
    case 100: rotate(lastrotationdeg);
      break;
    case 1: rotate(1);
      break;
    case 10: rotate(5);
      break;
    case 11: rotate(5);
      break;
    case 110: rotate(30);
      break;
    case 101: rotate(30);
      break;
    case 1001: rotate(-10);
      break;
    case 10010: rotate(10);
      break;
    case 1010: rotate(180);
      break;

  }
  if(val!=0 && prevrot!=lastrotationdeg)
  {
    cornerdetect++;
  }
  if(cornerdetect>cornerthreshold)
  {
    cornerdetect = 0;
    rotate(180);
  }

}


int readirdata()
{
  if (leddata)
  {
    irread = 1;
  }
  else
  {
    if(irread==1)
    {
      irread = 0;
    }
    return -1;
  }
  int i = 0, irout = 0;
  for (i = 0; i < 5; i++)
  {
    irout += ((int)!((bool)irarray[i])) * power(10, 4 - i);
  }
  irread = 0;
  return irout;
}


int power(int num1, int p)
{
  int i = 0, val = num1;
  if (p == 0)
  {
    return 1;
  }
  for (i = 0; i < p - 1; i++)
  {
    num1 = num1 * val;

  }
  return num1;
}

void updateir()
{
  int i = 0;
  for (i = 0; i < 5; i++)
  {
    if (irarray[i] == 0)
    {
      leddata = 1;
      return;
    }
  }
  leddata = 0;
}


void takeirinput()
{
  irarray[0] += digitalRead(TS1);
  irarray[1] += digitalRead(TS2);
  irarray[2] += digitalRead(TS3);
  irarray[3] += digitalRead(TS4);
  irarray[4] += digitalRead(TS5);
}

void externalirinput()
{
  irarray[0] = !digitalRead(TS1);
  irarray[1] = !digitalRead(TS2);
  irarray[2] = !digitalRead(TS3);
  irarray[3] = !digitalRead(TS4);
  irarray[4] = !digitalRead(TS5);

}

void irdatarx()
{
  int i = 0;
  unsigned long t = millis();
  while (digitalRead(TS1) == 1 && digitalRead(TS2) == 1 && digitalRead(TS3) == 1 && digitalRead(TS4) == 1 && digitalRead(TS5) == 1)
  {
    if (millis() - t > 100ul)
    {
      for (i = 0; i < 5; i++)
      {
        ircodearray[i] = 0;
      }
      return;
    }
    delayMicroseconds(10);
  }
  delayMicroseconds(irbitoffset);
  for (i = 0; i < irbuffsize; i++)
  {
    delayMicroseconds(irbitoffset * 4);
    irstring[0][i] = !digitalRead(TS1);
    irstring[1][i] = !digitalRead(TS2);
    irstring[2][i] = !digitalRead(TS3);
    irstring[3][i] = !digitalRead(TS4);
    irstring[4][i] = !digitalRead(TS5);
  }
  //  int j=0;
  //  Serial.println();
  //  for(i=0;i<5;i++)
  //  {
  //    for(j=0;j<irbuffsize;j++)
  //    {
  //      Serial.print(irstring[i][j]);
  //    }
  //    Serial.println();
  //  }
  checkirdata();
  for (i = 0; i < 5; i++)
  {
    Serial.print(ircodearray[i]);
  }
  Serial.println();
}

void checkirdata()
{
  int i = 0;
  bool val = 0;
  for (i = 0; i < 5; i++)
  {
    if (ircmp(irstring[i], mcode))
    {
      ircodearray[i] = 3;
      val = 1;
      continue;
    }
    if (ircmp(irstring[i], lcode))
    {
      ircodearray[i] = 2;
      val = 1;
      continue;
    }
    if (ircmp(irstring[i], rcode))
    {
      ircodearray[i] = 1;
      val = 1;
      continue;
    }
    ircodearray[i] = 0;

  }
  if (val == 0 && prevircodeval != 0)
  {
    discardcode = 1;
  }
  else
  {
    discardcode = 0;
  }
  prevircodeval = val;


}

bool ircmp(bool x[], char* y)
{
  int i = 0, match = 0;
  for (i = 0; i < irbuffsize; i++)
  {
    if (x[i] == 1 && y[i] == '1')
    {
      match++;
    }
    if (x[i] == 0 && y[i] == '0')
    {
      match++;
    }

  }
  if (match == irbuffsize)
  {
    //Serial.println("MATCH FOUND");
    return 1;
  }
  else
  {
    return 0;
  }
}


void printirdata()
{
  int i = 0;
  for (i = 0; i < 4; i++)
  {
    Serial.print(irarray[i]);
  }
  Serial.println(irarray[4]);
}

void irsensor()
{
  if (irenable == 0)
  {
    return;
  }
  while (irread)
  {
    delayMicroseconds(100);
  }
  leddata = 0;
  externalirinput();

  digitalWrite(IRL, 1);
  delay(2);

  takeirinput();
  updateir();
  //printirdata();
  digitalWrite(IRL, 0);

}


void motoroff()
{
  digitalWrite(ML1, LOW);
  digitalWrite(ML2, LOW);
  digitalWrite(ML3, LOW);
  digitalWrite(ML4, LOW);
  digitalWrite(MR1, LOW);
  digitalWrite(MR2, LOW);
  digitalWrite(MR3, LOW);
  digitalWrite(MR4, LOW);
}

float findangle(long p, long q)
{
  float num = (float)(q - y);
  float den = (float)(p - x);
  if (den == 0 && num == 0)
  {
    return 0;
  }
  float val = num / den;
  val = (180 * atan(val)) / PI;
  if (p >= x)
  {
    if (q < y)
    {
      val += 360;
    }
  }
  else
  {
    val += 180;
  }

  val -= currentangle;
  if (val < -180)
  {
    val += 360;
  }
  Serial.print(val);
  return val;
}

long calcdist(float p , float q)
{
  float num = (float)q - y;
  float den = (float)p - x;
  long steps = (long) sqrt(num * num + den * den);
  Serial.println(steps);
  return steps;
}


void moveto(long p, long q)
{
  Serial.println("MOVETOFUNCTION");
  rotate(findangle(p, q), 0);
  long steps = calcdist(p, q);
  Serial.print("NUMBER OF STEPS ARE ");
  Serial.println(steps);
  moveforward(steps, 0);
  motoroff();
  x = p;
  y = q;
  Serial.print("VALUE OF X AND Y IS: ");
  Serial.print(x);
  Serial.print("   ");
  Serial.println(y);
  Serial.println("END");


}

void moveforward(long steps, bool recordpos)
{
  //long steps = (long)((numberofsteps/(3.14*tiredia))* distx);
  if (steps == 0)
  {
    return;
  }
  if (steps < 0)
  {
    moveClockwise = 0;
  }
  else
  {
    moveClockwise = 1;
  }
  if (recordpos)
  {
    getpos();
  }
  zerostepval();
  long setVal = abs(steps);
  while (1)
  {
    stepperl.step(moveClockwise);
    stepperr.step(!moveClockwise);
    mainstepcount++;
    if (mainstepcount == setVal)
    {
      break;
    }
  }
  //motoroff();
  if (moveClockwise == 0)
  {
    mainstepcount = mainstepcount * -1;
  }
  getpos();
  zerostepval();
}

void getpos()
{
  if (mainstepcount == 0)
  {
    return;
  }

  x += (float)((mainstepcount * cos((currentangle * PI) / 180)));
  y += (float)((mainstepcount * sin((currentangle * PI) / 180)));

}

void zerostepval()
{
  mainstepcount = 0;
}

void rotate(float deg, bool recordpos)
{
  lastrotationdeg = deg;
  long rotationsteps = deg * ((tiredist * numberofsteps) / (360 * tiredia));
  Serial.println(rotationsteps);

  if (rotationsteps == 0)
  {
    return;
  }
  if (recordpos)
  {
    getpos();
  }
  zerostepval();

  if (rotationsteps < 0)
  {
    moveClockwise = true;


  }
  else
  {
    moveClockwise = false;

  }

  long setVal = abs(rotationsteps);
  while (1)
  {
    stepperl.step(moveClockwise);
    stepperr.step(moveClockwise);
    mainstepcount++;
    if (mainstepcount == setVal)
    {
      break;
    }

  }
  zerostepval();

  currentangle += deg;
  fixangle();
  Serial.println(currentangle);

}

void fixangle()
{
  //Serial.println(currentangle);
  if (currentangle == 360)
  {
    currentangle = 0;
    return ;
  }
  if (currentangle > 360)
  {
    currentangle -= 360;
    return ;
  }
  if (currentangle < 0)
  {
    currentangle += 360;
    return ;
  }

}

void readsens()
{
  Serial.print(digitalRead(TS1));
  Serial.print(digitalRead(TS2));
  Serial.print(digitalRead(TS3));
  Serial.print(digitalRead(TS4));
  Serial.print(digitalRead(TS5));
  Serial.print(digitalRead(BSL));
  Serial.print(digitalRead(BSR));
  Serial.print(" ");
  Serial.print(touchRead(T3));
  Serial.print(" ");
  Serial.println(analogRead(36));
}


void stepperfunc()
{
#ifdef defined(ESP32_RTOS) && defined(ESP32)
#else // If you do not use FreeRTOS, you have to regulary call the handle method.
  ArduinoOTA.handle();
#endif

  // Your code here
  for (int s = 0; s < 4096 * 10; s++) {
    // this will loop 4096 times
    // 4096 steps = full rotation using default values
    /* Note:
       you could alternatively use 4076 steps...
       if you think your 28BYJ-48 stepper's internal gear ratio is 63.68395:1 (measured) rather than 64:1 (advertised)
       for more info, see: http://forum.arduino.cc/index.php?topic=71964.15)
    */

    // let's move one "step" (of the 4096 per full rotation)

    stepperr.step(moveClockwise);
    stepperl.step(!moveClockwise);
    /* the direction is based on moveClockwise boolean:
       true for clockwise, false for counter-clockwise
       -- you could also say stepper.stepCW(); or stepper.stepCCW();
    */

    // now let's get the current step position of motor

    int nStep = stepperl.getStep();

    // and if it's divisible by 64...

    if (nStep % 64 == 0) {

      // let's print the position to the console

      Serial.print("current step position: "); Serial.print(nStep);
      Serial.println();

    }
  }

  // now we've moved 4096 steps

  // let's wait one second

  delay(1000);

  // and switch directions before starting loop() again

  moveClockwise = !moveClockwise;

}
