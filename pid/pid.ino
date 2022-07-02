// čidla jsou číslované od středu
#define cidl1 A5
#define cidl2 A6
#define cidl3 A7
#define cidp1 A0
#define cidp2 A2
#define cidp3 A4

#define mp1 9
// pwm
#define mp2 11 
#define ml1 12
// pwm
#define ml2 10 

#define STEPPER_PIN_1 4
#define STEPPER_PIN_2 6
#define STEPPER_PIN_3 7
#define STEPPER_PIN_4 8

int pTrig = 2;
int pEcho = 3;
int errorOld;
int Isuma;
int vzdalenost;
int error;
int pid_value;

// koeficienty pro pid - objetí překážky
float KpO = 10, KiO = 0, KdO = 20;

// koeficienty pro pid - jízda po čáře
float Kp = 4, Ki = 0.005, Kd = 20;

// proměnné pro normalizaci
int cl1_norma, cl2_norma, cl3_norma;
int cp1_norma, cp2_norma, cp3_norma;

// proměnné pro automatické načtení
int cl1_min = 1023, cl1_max = 0, cl2_min = 1023, cl2_max = 0, cl3_min = 1023, cl3_max = 0;
int cp1_min = 1023, cp1_max = 0, cp2_min = 1023, cp2_max = 0, cp3_min = 1023, cp3_max = 0;
int auto_p1, auto_p2, auto_p3, auto_l1, auto_l2, auto_l3;

// váhy čidel
int c1_vaha = 1, c2_vaha = 2, c3_vaha = 4;

// automatické načtení max a min hodnot pro každé čidlo
void auto_nacteni() {
  
  digitalWrite(mp1, LOW);
  analogWrite(mp2, 100);
  digitalWrite(ml1, HIGH);
  analogWrite(ml2, 155);
  while (millis() < 1200 or analogRead(cidl1) < 200) {
    auto_l1 = analogRead(cidl1);
    auto_l2 = analogRead(cidl2);
    auto_l3 = analogRead(cidl3);
    auto_p1 = analogRead(cidp1);
    auto_p2 = analogRead(cidp2);
    auto_p3 = analogRead(cidp3);

    // Načtení hodnot max, min
    // levé čidlo
    if (auto_l1 > cl1_max) {
      cl1_max = auto_l1;
    }
    else if (auto_l1 < cl1_min) {
      cl1_min = auto_l1;
    }
    if (auto_l2 > cl2_max) {
      cl2_max = auto_l2;
    }
    else if (auto_l2 < cl2_min) {
      cl2_min = auto_l2;
    }
    if (auto_l3 > cl3_max) {
      cl3_max = auto_l3;
    }
    else if (auto_l3 < cl3_min) {
      cl3_min = auto_l3;
    }

    // pravé čidlo
    if (auto_p1 > cp1_max) {
      cp1_max = auto_p1;
    }
    else if (auto_p1 < cp1_min) {
      cp1_min = auto_p1;
    }
    if (auto_p2 > cp2_max) {
      cp2_max = auto_p2;
    }
    else if (auto_p2 < cp2_min) {
      cp2_min = auto_p2;
    }
    if (auto_p3 > cp3_max) {
      cp3_max = auto_p3;
    }
    else if (auto_p3 < cp3_min) {
      cp3_min = auto_p3;
    }
  }
  digitalWrite(ml1, LOW);
  analogWrite(ml2, LOW);
  digitalWrite(mp1, LOW);
  analogWrite(mp2, LOW);
}

void setup() {
  // motory
  pinMode(mp1, OUTPUT);
  pinMode(mp2, OUTPUT);
  pinMode(ml1, OUTPUT);
  pinMode(ml2, OUTPUT);

  // čidla
  pinMode(cidl1, INPUT);
  pinMode(cidl2, INPUT);
  pinMode(cidl3, INPUT);
  pinMode(cidp1, INPUT);
  pinMode(cidp2, INPUT);
  pinMode(cidp3, INPUT);

  // ultrazvuk
  pinMode(pTrig, OUTPUT);
  pinMode(pEcho, INPUT);

  // krokový motorek
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);

  // ostatní
  Serial.begin(9600);
  errorOld = 0;
  Isuma = 0;
  auto_nacteni();
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
  Serial.println(vzdalenost);
  return vzdalenost;
}

// výpočet pid hodnoty
int pid(int error, float Kp, float Ki, float Kd) {
  // P-složka
  int Pkorekce = int(Kp * error);

  // I-složka
  if (Isuma * error < 0) {
    Isuma = 0;
  } else {
    Isuma += error;
  }
  int Ikorekce = int(Isuma * Ki);

  // D-složka
  int Dkorekce = int(Kd * (error - errorOld));
  errorOld = error;

  //PID-regulace
  int pid = Pkorekce + Ikorekce + Dkorekce;

  return pid;
}

// normalizace čidel
int normalizace(int cidlo, int c_min, int c_max) {
  int cid = analogRead(cidlo);
  int c_norma = int(((cid - c_min) * 100.0) / (c_max - c_min));
  c_norma = constrain(c_norma, 0, 100);
  return c_norma;
}

// ovládání motorů pro pid
void motory(int smer, int rychlost = 100) {
  int ml = rychlost - smer;
  int mp = rychlost + smer;

  // Levý motor
  if (ml >= 0) {
    if (ml > rychlost) {
      ml = rychlost;
    }
    digitalWrite(ml1, LOW);
  } else {
    if (ml < -rychlost) {
      ml = -rychlost;
    }
    digitalWrite(ml1, HIGH);
    ml = abs(ml);
    ml = rychlost - ml;
  }

  // Pravý motor
  if (mp >= 0) {
    if (mp > rychlost) {
      mp = rychlost;
    }
    digitalWrite(mp1, LOW);
  } else {
    if (mp < -rychlost) {
      mp = -rychlost;
    }
    digitalWrite(mp1, HIGH);
    mp = abs(mp);
    mp = rychlost - mp;
  }

  analogWrite(mp2, mp);
  analogWrite(ml2, ml);
}

// ovládání krokového motorku
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

void loop() {

  // Normalizace čidel
  cl1_norma = normalizace(cidl1, cl1_min, cl1_max);
  cl2_norma = normalizace(cidl2, cl2_min, cl2_max);
  cl3_norma = normalizace(cidl3, cl3_min, cl3_max);
  cp1_norma = normalizace(cidp1, cp1_min, cp1_max);
  cp2_norma = normalizace(cidp2, cp2_min, cp2_max);
  cp3_norma = normalizace(cidp3, cp3_min, cp3_max);

  // Vážený průměr čidel
  int cl_prumer = int((cl1_norma * -c1_vaha + cl2_norma * -c2_vaha + cl3_norma * -c3_vaha) / (-c1_vaha + -c2_vaha + -c3_vaha));
  int cp_prumer = int((cp1_norma * c1_vaha + cp2_norma * c2_vaha + cp3_norma * c3_vaha) / (c1_vaha + c2_vaha + c3_vaha));

  // pid regulace jízdy po čáře
  error = int((cl_prumer - cp_prumer) / 2);
  pid_value = pid(error, Kp, Ki, Kd);
  motory(pid_value, 100);

  // pid regulace objetí překážky
  vzdalenost = dalkomer();
  if (vzdalenost < 15 and vzdalenost != 0) {
    // vynuluj hodnoty pid
    Isuma = 0;
    errorOld = 0;
    motory(0, 0);
    // otoč ultrazvuk
    Step(true, 2, 650);
    // otoč se doleva
    digitalWrite(mp1, LOW);
    analogWrite(mp2, 100);
    digitalWrite(ml1, HIGH);
    analogWrite(ml2, 155);
    delay(400);
    motory(0, 0);
    // pid regulace pro objetí překážky
    while (analogRead(cidl1) < 200) {
      vzdalenost = dalkomer();
      error = 20 - vzdalenost;
      pid_value = pid(error, KpO, KiO, KdO);
      motory(pid_value, 200);
    }
    motory(0, 0);
    // otoč se doleva
    digitalWrite(mp1, LOW);
    analogWrite(mp2, 100);
    digitalWrite(ml1, HIGH);
    analogWrite(ml2, 155);
    delay(300);
    motory(0, 0);
    Step(false, 2, 650);
  }

}
