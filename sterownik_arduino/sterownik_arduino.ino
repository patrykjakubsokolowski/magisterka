#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PD A0
#define ON 1
#define OFF 0
#define MCP4725_ADDR 0x60

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int LD_READ = A1;  //dziala
const int DAC_SDA = A4; //dziala
const int DAC_SLC = A5; //dziala

const int YELLOW = 3;     
const int GREEN = 4;   
const int WHITE = 5;    
const int RED = 6; 

const int TRG = 2;       //dopytac o pomiar synchroniczny 
const int ENABLE = 3;     //gotowe, ustawienie 1 daje wartosci standowdowe
const int LD_SET = 4;    // chyba do usuniecia, LD_SET to OUT z DAC
const int P_BUTTON1 = 7;  //dziala
const int P_BUTTON2 = 8;  //dziala
const int MOD = 9;       //dziala
const int SLIDE = 10;     //dziala zmiana Freq lub Duty
const int BUTTON_MOD = 11;  //dziala
const int BUTTON_EN = 12;   //dziala

volatile byte state = LOW;
volatile byte mod_en = LOW;
volatile byte laser_en = LOW;

String inputString = "";         
bool stringComplete = false;
String x = "1"; 

int button_mod = 0; //0 - mozna zmieniac moc, brak mod, 1 zmienia sie f i d, jest modulacja
int last_button_mod;
int button_en = 0;  //0 - nie dziala ld_set, ustawia sie standardowa wartosc, 1 - dziala ld_set
int last_button_en;
int button_f_d = 0;
int last_button3;
int push_button1 = 0;
int last_push_button1;
int push_button2 = 0;
int last_push_button2;
int freq_or_duty = 0; // 1 to f, 0 to d
int ld_safe = 0; //1 bezpieczny można odpalać
int val;
float mV = 0;
float mV_dac = 500;
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
  Wire.begin();
  lcd.init();
  lcd.begin(16,2);   // Inicjalizacja LCD 2x16
  lcd.backlight(); // zalaczenie podwietlenia 
  lcd_w();
}

void loop() {
  buttons();          //sprawdz przyciski
  ld_read();          //sprawdzenie napiecia z fotodiody
  enable(laser_en);   //
  if (stringComplete) {
    inputString = "";
    stringComplete = false;
    delay(100);
  } 
}

//odczyt przycisków - dziala 1 i 2, trzeci do testów
void buttons() {
  button_mod = digitalRead(BUTTON_MOD);
  button_en = digitalRead(BUTTON_EN);
  button_f_d = digitalRead(SLIDE);
  push_button1 = digitalRead(P_BUTTON1);
  push_button2 = digitalRead(P_BUTTON2);
  
  if (button_mod != last_button_mod) { //wlaczenie modulacji
    if(button_mod == LOW) {
      mod_en = 0;
    } 
    if(button_mod == HIGH) {
      mod_en = 1;
    } 
    mod(mod_en);
  }
  last_button_mod = button_mod;

  if (button_en != last_button_en) {  //wlaczenie en
    if(button_en == LOW) {
      laser_en = 0;
    }
    if(button_en == HIGH) {
     laser_en = 1;
    }
  }
  last_button_en = button_en;


 if (push_button1 != last_push_button1) { //przycisk w dol
    if(push_button1 == LOW) {
      digitalWrite(GREEN, HIGH);
      if(mod_en == 1) {
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
      } else {
        if(button_en && mV_dac > 50) mV_dac -= 50;
      }
    }
    if(push_button1 == HIGH) {
      digitalWrite(GREEN, LOW);
    }
  }
  last_push_button1 = push_button1;

 if (push_button2 != last_push_button2) { //przycisk w gore
    if(push_button2 == LOW) {
      digitalWrite(RED, HIGH);
      if(mod_en == 1) {
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
      } else {
        if(button_en && mV_dac <= 950 ) mV_dac += 50;
      }
    }
    if(push_button2 == HIGH) {
      digitalWrite(RED, LOW);
    }
  }
  last_push_button2 = push_button2;
  
  if(button_f_d != last_button3) {
    if(button_f_d == LOW) {  // wybor zmiany f czy duty
      freq_or_duty = 0;
    }
    if(button_f_d == HIGH) {  //zmiana f
      freq_or_duty = 1;
    }
  } 
  last_button3 = button_f_d;

  if(laser_en == 1) {
     mod(mod_en);
     dac(mV_dac);
  }
}

//sprawdzenia napiecia z fotodiody, zabezpieczenie - dziala
void ld_read() {
  val = analogRead(LD_READ); //aktualnie podlaczone wyjscie z dac, ale bedzie z przetwornika prad nap
  mV = val * (5.0/1024.0);
  //Serial.println(mV); //dodać wyświetlenie na oled
  if(mV >= 0.9) {  // zabezpieczenie mocy
    ld_safe = 0;  //sprawdzic low czy high to wartosci standardowe
  } else {
    ld_safe = 1;
  }
}

void enable(byte laser_en) {   //gotowe
  if(laser_en && ld_safe) { //jesli napiecie jest w zakresie bezpieczenstwa i dziala ld_set
    digitalWrite(ENABLE, LOW);
  } else {
    digitalWrite(ENABLE, HIGH); //standardowe ustawienia
    dac(300);// do usuniecia laser sam ustawia wartosc standardowa
  }
}

void dac(int mV) {
Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write((mV) >> 4);        // the 8 most significant bits...
  Wire.write((mV & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
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

void lcd_w() {
  lcd.setCursor(0,0); // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
  lcd.print("P:325mW  MOD ON");
  delay(500);
  lcd.setCursor(0,1); //Ustawienie kursora w pozycji 0,0 (drugi wiersz, pierwsza kolumna)
  lcd.print("F:15Hz   D:50%");
}

//ustawienia przycisków - jak przyjdzie to jeszcze dodać wyswietlacz po i2c na analogowych a2 i a3
// oraz dac na tez po i2c na a4 i a5 nadal wolny 5 i 6 chwilowo dodane tam diody do debugowania
void pinset() {
  pinMode(TRG, INPUT_PULLUP);
  pinMode(YELLOW, OUTPUT);  //yellow //enable
  pinMode(GREEN, OUTPUT);  //green  //ld_set raczej do usuniecia
  pinMode(WHITE, OUTPUT);  //white  //przycisk do zmniejszenia wartosci
  pinMode(RED, OUTPUT);  //red    //przycisk do zwiekszenia wartosci
  pinMode(P_BUTTON1, INPUT_PULLUP);
  pinMode(P_BUTTON2, INPUT_PULLUP);
  pinMode(MOD, OUTPUT);
  pinMode(SLIDE, INPUT_PULLUP);
  pinMode(BUTTON_MOD, INPUT_PULLUP);
  pinMode(BUTTON_EN, INPUT_PULLUP);
}



//osbiór z andora - dziala
void serialEvent() {        //komunikacja z andorem
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
    if(inputString == "2") { 
    digitalWrite(WHITE, HIGH);
    }
    if(inputString == "3") {
    digitalWrite(GREEN, HIGH);
    }
  }
}
