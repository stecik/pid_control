#include <Servo.h>
Servo myservo;

// čidla jsou číslované od středu
int cidl1 = A2;
int cidl2 = A3;
int cidp1 = A0;
int cidp2 = A1;

#define mp1 8
// pwm pin
#define mp2 6
#define ml1 7
// pwm pin
#define ml2 5

int pTrig = 2;
int pEcho = 3;
float errorOld;
float Isuma;
int vzdalenost;
float error;
int pid_value;
int cidla[6][6];
int cidla_norma[5];

// koeficienty pro pid - objetí překážky
float KpO = 10, KiO = 0, KdO = 0;

// koeficienty pro pid - jízda po čáře
// float Kp = 4, Ki = 0.02, Kd = 10;
float Kp = 4, Ki = 0.001, Kd = 5;

int max_rychlost = 220, start_rychlost = 160;
int hodnota_cerne = 200;

// váhy čidel
int c1_vaha = 1, c2_vaha = 2;

// automatické načtení max a min hodnot pro každé čidlo
void auto_nacteni() {
  digitalWrite(mp1, LOW);
  analogWrite(mp2, 200);
  digitalWrite(ml1, HIGH);
  analogWrite(ml2, 55);
  cidla[0][0] = cidl1;
  cidla[0][1] = cidl2;
  cidla[0][2] = cidp1;
  cidla[0][3] = cidp2;


  // nastavení min a max hodnot
  for (int i = 0; i < 4; i++) {
    cidla[2][i] = 1023;
    cidla[3][i] = 0;
  }


  while (millis() < 900 or analogRead(cidl1) < hodnota_cerne) {
    for (int i = 0; i < 4; i++) {
      int hodnota_cidla = analogRead(cidla[0][i]);
      cidla[1][i] = hodnota_cidla;
      if (hodnota_cidla < cidla[2][i]) {
        cidla[2][i] = hodnota_cidla;
      } else if (hodnota_cidla > cidla[3][i]) {
        cidla[3][i] = hodnota_cidla;
      }
    }
  }
  digitalWrite(ml1, LOW);
  analogWrite(ml2, LOW);
  digitalWrite(mp1, LOW);
  analogWrite(mp2, LOW);
}

void setup() {
  myservo.write(180);
  myservo.attach(9);
  // motory
  pinMode(mp1, OUTPUT);
  pinMode(mp2, OUTPUT);
  pinMode(ml1, OUTPUT);
  pinMode(ml2, OUTPUT);

  // čidla
  pinMode(cidl1, INPUT);
  pinMode(cidl2, INPUT);
  pinMode(cidp1, INPUT);
  pinMode(cidp2, INPUT);

  // ultrazvuk
  pinMode(pTrig, OUTPUT);
  pinMode(pEcho, INPUT);

  // ostatní
  Serial.begin(9600);
  errorOld = 0;
  Isuma = 0;
  auto_nacteni();
  delay(3000);
}

// ovládání ultrazvuku
int dalkomer() {
  digitalWrite(pTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(pTrig, LOW);
  long odezva = pulseIn(pEcho, HIGH, 5000);
  long vzdalenost = int(odezva / 58.31);
  if (vzdalenost == 0) {
    vzdalenost = 35;
  }
  return vzdalenost;
}

// výpočet pid hodnoty
int pid(float error, float Kp, float Ki, float Kd) {
  // P-složka
  float Pkorekce = Kp * error;

  // I-složka

  if ((Isuma * error) < 0) {
    Isuma = 0;
  } else {
    Isuma += error;
  }

  float Ikorekce = Isuma * Ki;

  // D-složka
  float Dkorekce = Kd * (error - errorOld);
  errorOld = error;

  //PID-regulace
  int pid = int(Pkorekce + Ikorekce + Dkorekce);

  return pid;
}

// normalizace čidel
int normalizace(int cidlo, int c_min, int c_max) {
  int cid = analogRead(cidlo);
  int c_norma = int(((cid - c_min) * 100.0) / (c_max - c_min));
  c_norma = constrain(c_norma, 0, 100);
  return c_norma;
}

void Step(bool dir = true, int del = 2, int steps = 2048) {
  int step_number = 1;
  if (dir == false) {
    step_number = 4;
  }

  for (int i = 0; i < steps; i++) {
    if (step_number == 1) {
      digitalWrite(STEPPER_PIN_1, HIGH);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, LOW);
    }
    if (step_number == 2) {
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, HIGH);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, LOW);
    }
    if (step_number == 3) {
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, HIGH);
      digitalWrite(STEPPER_PIN_4, LOW);
    }
    if (step_number == 4) {
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, HIGH);
    }

    if (dir == true) {
      step_number++;
      if (step_number > 4) {
        step_number = 0;
      }
    } else {
      step_number--;
      if (step_number < 0) {
        step_number = 4;
      }
    }
    delay(del);
  }
}

// ovládání motorů pro pid
void motory(int smer, int vychozi_rychlost, int max_rychlost) {
  int ml = vychozi_rychlost - smer;
  int mp = vychozi_rychlost + smer;
  if (max_rychlost > 255) {
    max_rychlost = 255;
  }

  // Levý motor
  if (ml >= 0) {
    if (ml > max_rychlost) {
      ml = max_rychlost;
    }
    digitalWrite(ml1, LOW);
  } else {
    if (ml < -max_rychlost) {
      ml = -max_rychlost;
    }
    digitalWrite(ml1, HIGH);
    ml = abs(ml);
    ml = max_rychlost - ml;
  }
  // Pravý motor
  if (mp >= 0) {
    if (mp > max_rychlost) {
      mp = max_rychlost;
    }
    digitalWrite(mp1, LOW);
  } else {
    if (mp < -max_rychlost) {
      mp = -max_rychlost;
    }
    digitalWrite(mp1, HIGH);
    ml = abs(mp);
    ml = max_rychlost - mp;
  }
  analogWrite(mp2, mp);
  analogWrite(ml2, ml);
}

void loop() {

  // Normalizace čidel

  for (int i = 0; i < 4; i++) {
    cidla_norma[i] = normalizace(cidla[0][i], cidla[2][i], cidla[3][i]);
  }

  // Vážený průměr čidel
  float cl_prumer = (cidla_norma[0] * c1_vaha + cidla_norma[1] * c2_vaha) / (c1_vaha + c2_vaha);
  float cp_prumer = (cidla_norma[2] * c1_vaha + cidla_norma[3] * c2_vaha) / (c1_vaha + c2_vaha);

  // pid regulace jízdy po čáře
  error = (cl_prumer - cp_prumer);

  pid_value = pid(error, Kp, Ki, Kd);
  motory(pid_value, start_rychlost, max_rychlost);

  // pid regulace objetí překážky
  
  vzdalenost = int((dalkomer() + dalkomer() + dalkomer()) / 3);
  if (vzdalenost < 20 and vzdalenost != 0) {
    motory(0, 0, 0);
    // vynuluj hodnoty pid

    Isuma = 0;
    errorOld = 0;
    motory(0, 0, 0);
    // otoč ultrazvuk
    myservo.write(90);
    // Step(true, 2, 650);
    // otoč se doleva
    digitalWrite(mp1, LOW);
    analogWrite(mp2, 255);
    digitalWrite(ml1, HIGH);
    analogWrite(ml2, 0);
    delay(300);
    motory(0, 0, 0);
    delay(1000);

    // pid regulace pro otočení
    while (analogRead(cidl1) < hodnota_cerne || analogRead(cidl2) < hodnota_cerne || analogRead(cidp1) < hodnota_cerne || analogRead(cidp2) < hodnota_cerne) {
      vzdalenost = int((dalkomer() + dalkomer() + dalkomer()) / 3);
      error = 20 - vzdalenost;
      pid_value = pid(error, KpO, KiO, KdO);
      motory(pid_value, 200, 255);
    }
    motory(0, 0, 0);
    // otoč se doleva
    digitalWrite(mp1, LOW);
    analogWrite(mp2, 100);
    digitalWrite(ml1, HIGH);
    analogWrite(ml2, 155);
    delay(300);
    motory(0, 0, 0);
    myservo.write(180);
    // Step(false, 2, 650);
  }
}