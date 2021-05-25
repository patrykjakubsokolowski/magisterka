#include <Wire.h>
#include <LiquidCrystal_I2C.h>
  //#include <PID_v2.h>

#define PD A0
#define ON 1
#define OFF 0
#define MCP4725_ADDR 0x60

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int DAC_READ = A0;  //dziala
const int LD_READ = A1;  //dziala
const int DAC_SDA = A4; //dziala
const int DAC_SLC = A5; //dziala

const int GREEN = 3;
const int YELLOW = 4;         
const int RED = 5; 

const int TRG = 2;       //dopytac o pomiar synchroniczny 
const int ENABLE = 3;     //gotowe, ustawienie 1 daje wartosci standowdowe
const int LD_SET = 4;    // chyba do usuniecia, LD_SET to OUT z DAC
const int P_BUTTON1 = 7;  //dziala
const int P_BUTTON2 = 8;  //dziala
const int MOD = 9;       //dziala
const int F_D = 10;     //dziala zmiana Freq lub Duty
const int BUTTON_MOD = 11;  //dziala
const int BUTTON_EN = 12;   //dziala

volatile byte mod_en = LOW;
volatile byte laser_en = LOW;
volatile byte ld_safe = LOW; //1 bezpieczny można odpalać

String inputString = "";         
bool stringComplete = false;
String x = "1"; 

int button_mod = 0; //0 - mozna zmieniac moc, brak mod, 1 zmienia sie f i d, jest modulacja
int last_button_mod;
int button_en = 0;  //0 - nie dziala ld_set, ustawia sie standardowa wartosc, 1 - dziala ld_set
int last_button_en;
int button_f_d = 0;
int last_button_f_d;
int button_trg = 0;
int last_button_trg;
int push_button1 = 0;
int last_push_button1;
int push_button2 = 0;
int last_push_button2;

int freq_or_duty = 0; // 1 to f, 0 to d
int val;
int mV = 0;
int dac_mV = 0;
int freq = 1;
float duty = 0.5;

static char tekst1[16];
static char tekst2[50];
static unsigned int moc[13][3] = {{200,10,100},{230,20,200},{300,70,600},{325,115,800},{350,170,900},{420,170,1250},{450,235,1350},{500,245,1500},{550,295,1700},{600,315,2200},{600,350,2400}}; //{300,275},{350,314},{397,125},{423,146}};
//dac, wyswietlacz, fotodioda
unsigned int moc_i = 1;
unsigned int moc_pomocnicza = 0;
int odejmowanie = 0;
unsigned int i = 0;

unsigned int ocr;
unsigned int icr;

  //double Kp = 2, Ki = 5, Kd = 1;
  //PID_v2 myPID(Kp, Ki, Kd, PID::Direct);
  //double input = 0;
  //double output = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(50);
  attachInterrupt(digitalPinToInterrupt(TRG), trigger, RISING);
  pinset();
  Wire.begin();
    //myPID.Start(analogRead(LD_READ),0,100);                   // input, current, setpoint
    //myPID.SetOutputLimits(0,400);
  lcd.init();
  lcd.begin(16,2);   // Inicjalizacja LCD 2x16
  lcd.backlight(); // zalaczenie podwietlenia 
  lcd_w();
  dac(50); 
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
  button_f_d = digitalRead(F_D);
  button_trg = digitalRead(TRG);
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
      moc_pomocnicza = moc[moc_i][0];
      dac(moc[moc_i][0]);
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
        if(moc_i > 0) moc_i -= 1;
      }
    }
    if(push_button1 == HIGH) {
    }
  }
  last_push_button1 = push_button1;

 if (push_button2 != last_push_button2) { //przycisk w gore
    if(push_button2 == LOW) {
      moc_pomocnicza = moc[moc_i][0];
      dac(moc[moc_i][0]);
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
        if(moc_i < 13 ) moc_i += 1;
      }
    }
    if(push_button2 == HIGH) {
    }
  }
  last_push_button2 = push_button2;
  
  if(button_f_d != last_button_f_d) {
    if(button_f_d == LOW) {  // wybor zmiany f czy duty
      freq_or_duty = 0;
    }
    if(button_f_d == HIGH) {  //zmiana f
      freq_or_duty = 1;
    }
  } 
  last_button_f_d = button_f_d;

   if(button_trg != last_button_trg) {
    if(button_trg == LOW) {  // wybor zmiany f czy duty
      //digitalWrite(GREEN, HIGH);
    }
    if(button_trg == HIGH) {  //zmiana f
      //digitalWrite(GREEN, HIGH);
    }

  } 
  last_button_trg = button_trg;

  mod(mod_en);
  if(laser_en == 1) {
     
    // if(abs(moc[moc_i][2] - mV) > 50) {
      // dac(moc[moc_i][0]);
      // Serial.print("X");
    // }
  }
  lcd_w();
}

//sprawdzenia napiecia z fotodiody, zabezpieczenie - dziala
void ld_read() {
  val = analogRead(LD_READ); //aktualnie podlaczone wyjscie z dac, ale bedzie z przetwornika prad nap
  mV = val * (5000.0/1024.0);
  val = analogRead(DAC_READ);
  dac_mV = val * (5000.0/1024.0);
  sprintf(tekst2, "PD:%3i MB:%3i DAC:%3i %3i S%2i en:%2i m:%2i", mV, moc[moc_i][2], moc_pomocnicza, dac_mV, ld_safe, laser_en, mod_en);
  Serial.println(tekst2); //--------------------------------------------------------------------------------
  //Serial.println(mV); //dodać wyświetlenie na oled
  if(dac_mV >= 800) {  // zabezpieczenie mocy
    ld_safe = 0;  //sprawdzic low czy high to wartosci standardowe
  } else {
    ld_safe = 1;
  }
  i++;
  
  if(laser_en == 1 and i%10 == 0 and mod_en == 0) {
    //moc_pomocnicza = moc[moc_i][0];
    odejmowanie = moc[moc_i][2] - mV;
    if(moc_pomocnicza < 800) {
      if(odejmowanie > 300 and moc_pomocnicza > 20) {
        moc_pomocnicza += 20;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -300 and moc_pomocnicza > 20) {
        moc_pomocnicza -= 20;
        dac(moc_pomocnicza);
      }
      if(odejmowanie > 100 and moc_pomocnicza > 1) {
        moc_pomocnicza += 5;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -100 and moc_pomocnicza > 1) {
        moc_pomocnicza -= 5;
        dac(moc_pomocnicza);
      }
      if(odejmowanie > 10 and moc_pomocnicza > 1) {
        moc_pomocnicza += 1;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -10 and moc_pomocnicza > 1) {
        moc_pomocnicza -= 1;
        dac(moc_pomocnicza);
      }
      i = 0;
    }
  }
}

void enable(byte laser_en) {   //gotowe
  if(laser_en and ld_safe) { //jesli napiecie jest w zakresie bezpieczenstwa i dziala ld_set
    digitalWrite(ENABLE, HIGH);
  } else {
    digitalWrite(ENABLE, LOW); //standardowe ustawienia
    dac(100);
  }
}

void dac(int mV) {
if(moc_pomocnicza < 800 and mV > 0) { 
Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write((mV) >> 4);        // the 8 most significant bits...
  Wire.write((mV & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
  } else {
    dac(200);
  }
}

//kod wyzwalany triggerem - dziala, do celowo ma wlaczac laser
void trigger() { // działa
  //laser_en = 1;
}
//wlaczenia modulacji przyciskiem - dziala
void mod(byte mod_en) {
  if(mod_en == 0) {
     digitalWrite(MOD,HIGH);
     digitalWrite(YELLOW, LOW);
  }
  if(mod_en == 1) {
     digitalWrite(YELLOW, HIGH);
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
  int duty_int = duty*100;
  lcd.setCursor(0,0); // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
  sprintf(tekst1, "P:%3imW ", moc[moc_i][1]);
  lcd.print(tekst1);
  lcd.setCursor(9,0);
  if(button_mod == 0) {
      lcd.print("MOD OFF");
  } else {
      lcd.print("MOD ON ");
  }
  lcd.setCursor(0,1); //Ustawienie kursora w pozycji 0,0 (drugi wiersz, pierwsza kolumna)
  sprintf(tekst1, "F:%2ikH   D:%2i", freq, duty_int);
  lcd.print(tekst1);
  lcd.setCursor(13,1); //Ustawienie kursora w pozycji 0,0 (drugi wiersz, pierwsza kolumna)
  lcd.print("%");
}

//ustawienia przycisków - jak przyjdzie to jeszcze dodać wyswietlacz po i2c na analogowych a2 i a3
// oraz dac na tez po i2c na a4 i a5 nadal wolny 5 i 6 chwilowo dodane tam diody do debugowania
void pinset() {
  pinMode(TRG, INPUT_PULLUP);
  pinMode(YELLOW, OUTPUT);  //yellow //mod
  pinMode(GREEN, OUTPUT);  //green  //trig
  pinMode(RED, OUTPUT);  //red    //enable
  pinMode(P_BUTTON1, INPUT_PULLUP);
  pinMode(P_BUTTON2, INPUT_PULLUP);
  pinMode(MOD, OUTPUT);
  pinMode(F_D, INPUT_PULLUP);
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
    digitalWrite(GREEN, HIGH);
    }
    if(inputString == "3") {
    digitalWrite(GREEN, LOW);
    }
  }
}
