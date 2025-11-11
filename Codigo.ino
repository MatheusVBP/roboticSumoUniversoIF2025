#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define PEMFF 16
#define PEMFT 17
#define PEMBF 18
#define PEMBT 19
#define PDMFF 21
#define PDMFT 23
#define PDMBF 4
#define PDMBT 13

#define IRESQ 32
#define IRDIR 33

#define XSHUTsESQ 22
#define XSHUTsCENT 27
#define XSHUTsDIR 14

Adafruit_VL53L0X sESQ = Adafruit_VL53L0X();
Adafruit_VL53L0X sDIR = Adafruit_VL53L0X();
Adafruit_VL53L0X sCENT = Adafruit_VL53L0X();

float erro = 0;
float erroAnterior = 0;
float somaErro = 0;

int contadorAlvo = 0;
const int LIMITE_CONTADOR = 10;

const int DISTANCIA_PROXIMA = 50;
const int DISTANCIA_ATAQUE_PID = 300;
const int DISTANCIA_MINIMA_VALIDA = 10;

void setupVL53L0X() {
  pinMode(XSHUTsESQ, OUTPUT);
  pinMode(XSHUTsCENT, OUTPUT);
  pinMode(XSHUTsDIR, OUTPUT);

  digitalWrite(XSHUTsESQ, LOW);
  digitalWrite(XSHUTsCENT, LOW);
  digitalWrite(XSHUTsDIR, LOW);
  delay(10);

  digitalWrite(XSHUTsESQ, HIGH);
  delay(10);
  if (!sESQ.begin(0x30, &Wire)) {
    Serial.println("Falha ao iniciar sensor ESQUERDO");
  } else {
    //sESQ.setMeasurementTimingBudgetMicroSeconds(50000);
    Serial.println("Sensor ESQUERDO iniciado");
  }

  digitalWrite(XSHUTsCENT, HIGH);
  delay(10);
  if (!sCENT.begin(0x31, &Wire)) {
    Serial.println("Falha ao iniciar sensor CENTRAL");
  } else {
    Serial.println("Sensor CENTRAL iniciado");
  }

  digitalWrite(XSHUTsDIR, HIGH);
  delay(10);
  if (!sDIR.begin(0x32, &Wire)) {
    Serial.println("Falha ao iniciar sensor DIREITO");
  } else {
    Serial.println("Sensor DIREITO iniciado");
  }
}

void moverESQ(int vel1, int vel2) {
  ledcWrite(PEMFF, vel1);
  ledcWrite(PEMFT, vel2);
  ledcWrite(PEMBF, vel1);
  ledcWrite(PEMBT, vel2);
}

void moverDIR(int vel1, int vel2) {
  ledcWrite(PDMFF, vel1);
  ledcWrite(PDMFT, vel2);
  ledcWrite(PDMBF, vel1);
  ledcWrite(PDMBT, vel2);
}

void parar() {
  moverESQ(0, 0);
  moverDIR(0, 0);
}

void calcularVelocidadesPID(int distLeft, int distRight, int baseSpeed, int &velEsq, int &velDir) {
  erro = distLeft - distRight;
  erro = constrain(erro, -300, 300);

  somaErro += erro;
  float derivada = erro - erroAnterior;

  float Kp = 2.5;
  float Ki = 0.0;
  float Kd = 0.45;

  float correcao = Kp * erro + Ki * somaErro + Kd * derivada;
  erroAnterior = erro;

  correcao = constrain(correcao, -100, 100);

  velEsq = baseSpeed + correcao;
  velDir = baseSpeed - correcao;

  velEsq = constrain(velEsq, 0, 255);
  velDir = constrain(velDir, 0, 255);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(25, 26);
  setupVL53L0X();

  pinMode(IRDIR, INPUT);
  pinMode(IRESQ, INPUT);

  pinMode(PEMFF, OUTPUT);
  pinMode(PEMFT, OUTPUT);
  pinMode(PEMBF, OUTPUT);
  pinMode(PEMBT, OUTPUT);
  pinMode(PDMFF, OUTPUT);
  pinMode(PDMFT, OUTPUT);
  pinMode(PDMBF, OUTPUT);
  pinMode(PDMBT, OUTPUT);

  ledcAttach(PEMFF, 5000, 8);
  ledcAttach(PEMFT, 5000, 8);
  ledcAttach(PEMBF, 5000, 8);
  ledcAttach(PEMBT, 5000, 8);
  ledcAttach(PDMFF, 5000, 8);
  ledcAttach(PDMFT, 5000, 8);
  ledcAttach(PDMBF, 5000, 8);
  ledcAttach(PDMBT, 5000, 8);

  Serial.println("Sensores Ligados");

  moverESQ(255, 0);
  moverDIR(0, 255);
  delay(600);
}

void evitarBorda(bool bordaEsq, bool bordaDir) {
  unsigned long tempoInicial = millis();

  const long DURACAO_RE = 600;
  const long DURACAO_GIRO = 1500;
  const long DURACAO_TOTAL_MANOBRA = DURACAO_RE + DURACAO_GIRO;

  while (millis() - tempoInicial < DURACAO_TOTAL_MANOBRA) {
    unsigned long tempoAtual = millis() - tempoInicial;

    VL53L0X_RangingMeasurementData_t measureLeft, measureCenter, measureRight;
    sESQ.rangingTest(&measureLeft, false);
    sCENT.rangingTest(&measureCenter, false);
    sDIR.rangingTest(&measureRight, false);

    int distLeft = measureLeft.RangeMilliMeter;
    int distCenter = measureCenter.RangeMilliMeter;
    int distRight = measureRight.RangeMilliMeter;

    if ((distCenter < DISTANCIA_ATAQUE_PID && distCenter > DISTANCIA_MINIMA_VALIDA) || (distLeft < DISTANCIA_PROXIMA && distLeft > DISTANCIA_MINIMA_VALIDA) || (distRight < DISTANCIA_PROXIMA && distRight > DISTANCIA_MINIMA_VALIDA)) {
      parar();
      return;
    }

    int velEsqPID, velDirPID;
    calcularVelocidadesPID(distLeft, distRight, 255, velEsqPID, velDirPID);

    if (tempoAtual < DURACAO_RE) {
      moverESQ(0, velEsqPID);
      moverDIR(0, velDirPID);
    } else {
      if (bordaEsq && !bordaDir) {
        moverESQ(velEsqPID, 0);
        moverDIR(0, velDirPID);
      } else {
        moverESQ(0, velEsqPID);
        moverDIR(velDirPID, 0);
      }
    }

    delay(5);
  }

  parar();
}

void loop() {
  VL53L0X_RangingMeasurementData_t measureLeft, measureCenter, measureRight;
  sESQ.rangingTest(&measureLeft, false);
  sCENT.rangingTest(&measureCenter, false);
  sDIR.rangingTest(&measureRight, false);

  int distLeft = measureLeft.RangeMilliMeter;
  int distCenter = measureCenter.RangeMilliMeter;
  int distRight = measureRight.RangeMilliMeter;

/*
  if (measureLeft.RangeStatus != 4) {
    Serial.print("Distância Esquerda: ");
    Serial.print(distLeft);
    Serial.println(" mm");
  } else {
    Serial.println("Leitura inválida ou fora do alcance");
  }
  if (measureCenter.RangeStatus != 4) {
    Serial.print("Distância Centro: ");
    Serial.print(distCenter);
    Serial.println(" mm");
  } else {
    Serial.println("Leitura inválida ou fora do alcance");
  }
  if (measureCenter.RangeStatus != 4) {
    Serial.print("Distância Direita: ");
    Serial.print(distRight);
    Serial.println(" mm");
  } else {
    Serial.println("Leitura inválida ou fora do alcance");
  }
*/

  bool eBrancoESQ = (digitalRead(IRESQ) == 1);
  bool eBrancoDIR = (digitalRead(IRDIR) == 1);

  if (eBrancoESQ || eBrancoDIR) {
    contadorAlvo = 0;
    evitarBorda(eBrancoESQ, eBrancoDIR);
  }

  else if ((distLeft < DISTANCIA_PROXIMA && distLeft > DISTANCIA_MINIMA_VALIDA) && (distCenter < DISTANCIA_PROXIMA && distCenter > DISTANCIA_MINIMA_VALIDA) && (distRight < DISTANCIA_PROXIMA && distRight > DISTANCIA_MINIMA_VALIDA)) {
    moverESQ(255, 0);
    moverDIR(255, 0);
  }

  else if ((distCenter < DISTANCIA_ATAQUE_PID && distCenter > DISTANCIA_MINIMA_VALIDA) || (distLeft > 50 && distLeft < DISTANCIA_ATAQUE_PID) || (distRight > 50 && distRight < DISTANCIA_ATAQUE_PID) || (contadorAlvo > 0)) {

    if ((distCenter < DISTANCIA_ATAQUE_PID && distCenter > DISTANCIA_MINIMA_VALIDA) || (distLeft > 50 && distLeft < DISTANCIA_ATAQUE_PID) || (distRight > 50 && distRight < DISTANCIA_ATAQUE_PID)) {
      contadorAlvo = LIMITE_CONTADOR;
    } else {
      contadorAlvo--;
    }

    int velEsq, velDir;
    calcularVelocidadesPID(distLeft, distRight, 200, velEsq, velDir);
    moverESQ(velEsq, 0);
    moverDIR(velDir, 0);
  } else {
    moverESQ(200, 0);
    moverDIR(0, 160);
  }
}
