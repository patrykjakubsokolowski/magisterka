#define PD A0
#define ON 1
#define OFF 0

const int LD_READ = A1;  //dziala
const int SDA_DIS = A2; 
const int SLC_DIS = A3; 
const int SDA_DAC = A4; 
const int SLC_DAC = A5; 

const int TRG = 2;       //dopytac o pomiar synchroniczny 
const int ENABLE = 3;     //gotowe
const int LD_SET = 4;    // chyba do usuniecia, LD_SET to OUT z DAC
const int P_BUTTON1 = 7;  //dziala
const int P_BUTTON2 = 8;  //dziala
const int MOD = 9;       //dziala
const int SLIDE = 10;     //dziala
const int BUTTON_MOD = 11;  //dziala
const int BUTTON_EN = 12;   //dziala

volatile byte state = LOW;
volatile byte mod_en = LOW;
volatile byte laser_en = LOW;

String inputString = "";         
bool stringComplete = false;
String x = "1"; 

int button_mod = 0;
int last_button_mod;
int button_en = 0;
int last_button_en;
int button3 = 0;
int last_button3;
int push_button1 = 0;
int last_push_button1;
int push_button2 = 0;
int last_push_button2;
int freq_or_duty = 0;
int val;
float mV;
int freq = 1;
float duty = 0.5;

unsigned int ocr;
unsigned int icr;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(50);
  attachInterrupt(digitalPinToInterrupt(TRG), trigger, CHANGE);
  pinset();
}

void loop() {
  buttons();
  ld_read();
  if (stringComplete) {
    inputString = "";
    stringComplete = false;
    delay(100);
  } 
}

void enable(byte laser_en) {   //gotowe
  if(laser_en == 1) {
    digitalWrite(ENABLE, HIGH);
  } else {
    digitalWrite(ENABLE, LOW);
  }
}

//kod wyzwalany triggerem - dziala, do celowo ma wlaczac laser
void trigger() { // działa
  state = !state;
  //digitalWrite(6, state); //działa
}

//wlaczenia modulacji przyciskiem - dziala
void mod(byte mod_en) {
  if(mod_en == 0) {
     analogWrite(MOD, 0);
  }
  if(mod_en == 1) {
     pwm(freq, duty);
  }
}

//ustawianie wartosci modulacji pwm
void pwm(int freq, float duty) {
  icr = (16000000/(256*freq))-1;
  ocr = icr*duty;
  TCCR1A = (1<<WGM11)|(1<<COM1A1);  //preskaler 256
  TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS12); 
  ICR1 = icr;  //czestotliwosc   16Mhz/(256 * 1Hz) - 1 = 62499
  OCR1A = ocr;  //wypelnienie 50%   62499 - 100%  6249 - 19% 31249 - 50%
}

//ustawienia przycisków - jak przyjdzie to jeszcze dodać wyswietlacz po i2c na analogowych a2 i a3
// oraz dac na tez po i2c na a4 i a5 nadal wolny 5 i 6 chwilowo dodane tam diody do debugowania
void pinset() {
  pinMode(TRG, INPUT_PULLUP);
  pinMode(3, OUTPUT);  //yellow //enable
  pinMode(4, OUTPUT);  //green  //ld_set raczej do usuniecia
  pinMode(5, OUTPUT);  //white  //przycisk do zmniejszenia wartosci
  pinMode(6, OUTPUT);  //red    //przycisk do zwiekszenia wartosci
  pinMode(P_BUTTON1, INPUT_PULLUP);
  pinMode(P_BUTTON2, INPUT_PULLUP);
  pinMode(MOD, OUTPUT);
  pinMode(SLIDE, INPUT_PULLUP);
  pinMode(BUTTON_MOD, INPUT_PULLUP);
  pinMode(BUTTON_EN, INPUT_PULLUP);
}

//odczyt przycisków - dziala 1 i 2, trzeci do testów
void buttons() {
  button_mod = digitalRead(BUTTON_MOD);
  button_en = digitalRead(BUTTON_EN);
  button3 = digitalRead(SLIDE);
  push_button1 = digitalRead(P_BUTTON1);
  push_button2 = digitalRead(P_BUTTON2);
  
  if (button_mod != last_button_mod) {
    if(button_mod == LOW) {
      mod_en = 0;
    } 
    if(button_mod == HIGH) {
      mod_en = 1;
    } 
    mod(mod_en);
  }
  last_button_mod = button_mod;

  if (button_en != last_button_en) {
    if(button_en == LOW) {
      laser_en = 0;
    }
    if(button_en == HIGH) {
     laser_en = 1;
    }
    enable(laser_en);
  }
  last_button_en = button_en;


 if (push_button1 != last_push_button1) {
    if(push_button1 == LOW) {
      digitalWrite(4, HIGH);
      if(freq_or_duty == 1) {
        if(freq >=2) {
          freq--;
          Serial.println(freq);
        }
      }
      if(freq_or_duty == 0) {
        if(duty >= 0.2) {
          duty -= 0.1;
          Serial.println(duty);
        }
      }
    }
    if(push_button1 == HIGH) {
      digitalWrite(4, LOW);
    }
  }
  last_push_button1 = push_button1;

  
 if (push_button2 != last_push_button2) {
    if(push_button2 == LOW) {
      digitalWrite(6, HIGH);
      if(freq_or_duty == 1) {
        if(freq < 10) {
          freq++;
          Serial.println(freq);
        }
      }
      if(freq_or_duty == 0) {
        if(duty < 0.9) {
          duty += 0.1;
          Serial.println(duty);
        }
      }
    }
    if(push_button2 == HIGH) {
      digitalWrite(6, LOW);
    }
  }
  last_push_button2 = push_button2;
  
  if(button3 != last_button3) {
    if(button3 == LOW) {  // wybor zmiany f czy duty
      freq_or_duty = 0;
    }
    if(button3 == HIGH) {
      freq_or_duty = 1;
    }
  } 
  last_button3 = button3;

  mod(mod_en);
}

//sprawdzenia napiecia z fotodiody, zabezpieczenie - dziala
void ld_read() {
  val = analogRead(LD_READ);
  mV = val * (5.0/1024.0);
  //Serial.println(mV);
  if(mV >= 0.9) {  // zabezpieczenie mocy
    digitalWrite(ENABLE, LOW);  //sprawdzic low czy high to wartosci standardowe
  } 
}

//osbiór z andora - dziala
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
    if(inputString == "2") { 
    digitalWrite(5, HIGH);
    }
    if(inputString == "3") {
    digitalWrite(4, HIGH);
    }
  }
}
