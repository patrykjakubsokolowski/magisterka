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

const int RED = 3;
const int YELLOW = 4;         
const int GREEN = 5; 

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
static unsigned int moc[22][3] = {{200,5,90},{215,10,135},{240,20,202},{250,30,265},{255,40,361},{260,50,431},{270,60,500},{290,70,570},{300,80,650},{320,90,725},{330,100,785},{330,110,840},{345,120,908},{360,130,925},{380,140,1010},{400,150,1090},{420,175,1300},{450,200,1445},{470,225,1590},{490,250,1733},{530,275,1978},{540,300,2110}}; //{300,275},{350,314},{397,125},{423,146}};
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
  attachInterrupt(digitalPinToInterrupt(TRG), trigger, CHANGE);
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
    //delay(100);
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
            //Serial.println(freq);
          }
        }
        if(freq_or_duty == 0) {
          if(duty >= 0.2) {
            duty -= 0.1;
            //Serial.println(duty);
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
        if(moc_i < 21 ) moc_i += 1;
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
    if(button_trg == LOW) {  
      //digitalWrite(GREEN, HIGH);
    }
    if(button_trg == HIGH) { 
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
  
  if(laser_en == 1  and i%5 == 0 and mod_en == 0) { //tu było i%2
    //moc_pomocnicza = moc[moc_i][0];
    odejmowanie = moc[moc_i][2] - mV;
    if(moc_pomocnicza < 800) {
      if(odejmowanie > 300 and moc_pomocnicza > 1) {
        moc_pomocnicza += 50;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -300 and moc_pomocnicza > 51) {
        moc_pomocnicza -= 50;
        dac(moc_pomocnicza);
      }
      if(odejmowanie > 100 and moc_pomocnicza > 1) {
        moc_pomocnicza += 10;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -100 and moc_pomocnicza > 11) {
        moc_pomocnicza -= 10;
        dac(moc_pomocnicza);
      }
      if(odejmowanie > 40 and moc_pomocnicza > 1) {
        moc_pomocnicza += 2;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -40 and moc_pomocnicza > 3) {
        moc_pomocnicza -= 2;
        dac(moc_pomocnicza);
      }
      if(odejmowanie > 5 and moc_pomocnicza > 1) {
        moc_pomocnicza += 1;
        dac(moc_pomocnicza);
      } 
      if(odejmowanie < -5 and moc_pomocnicza > 1) {
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
  laser_en = !laser_en;
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
    if(inputString == "5") { 
    moc_i = 0;
    }
    else if(inputString == "10") {
    moc_i = 1;
    digitalWrite(GREEN, LOW);
    }
    else if(inputString == "20") {
    moc_i = 2; 
    digitalWrite(GREEN, HIGH);
    }
    else if(inputString == "30") { 
    moc_i = 3;
    }
    else if(inputString == "40") { 
    moc_i = 4;
    }
    else if(inputString == "50") { 
    moc_i = 5;
    }
    else if(inputString == "60") { 
    moc_i = 6;
    }
    else if(inputString == "70") { 
    moc_i = 7;
    }
    else if(inputString == "80") { 
    moc_i = 8;
    }
    else if(inputString == "90") { 
    moc_i = 9;
    }
    else if(inputString == "100") { 
    moc_i = 10;
    }
    else if(inputString == "110") { 
    moc_i = 11;
    }
    else if(inputString == "120") { 
    moc_i = 12;
    }
    else if(inputString == "130") { 
    moc_i = 13;
    }
    else if(inputString == "140") { 
    moc_i = 14;
    }
    else if(inputString == "150") { 
    moc_i = 15;
    }
    else if(inputString == "175") { 
    moc_i = 16;
    }
    else if(inputString == "200") { 
    moc_i = 17;
    }
    else if(inputString == "MOD") { 
     mod_en = !mod_en;
    }
    else if(inputString == "EN") { 
     laser_en = !laser_en;
    }
  }
}
