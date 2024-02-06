#include <Arduino.h>

float Sensibilidad = 0.139;  // Sensibilidad en V/A para nuestro sensor
float offset = 0.100;  // Equivale a la amplitud del ruido

float get_corriente();  // Declaración de la función

#define RELE1_PIN 26  // Pin para el rele1
#define RELE2_PIN 27  // Pin para el rele2

#define CICLOS_TOTALES_RELE1 3
#define DURACION_RELE1 250  // 1 segundo
#define INTERVALO_ENTRE_CICLOS_RELE1 30000  // 30 segundos en milisegundos

#define CICLOS_TOTALES_RELE2 8
#define DURACION_RELE2 500  // 500 milisegundos
#define INTERVALO_ENTRE_CICLOS_RELE2 240000  // 4 minutos en milisegundos

#define TIEMPO_TOTAL_ESPERA 43200000  // 12 horas en milisegundos

int cicloActualRele1 = 0;
int cicloActualRele2 = 0;
unsigned long tiempoInicio;

void setup() {
  pinMode(RELE1_PIN, OUTPUT);
  pinMode(RELE2_PIN, OUTPUT);

  // Inicialmente, apagar ambos relés
  digitalWrite(RELE1_PIN, HIGH);
  digitalWrite(RELE2_PIN, HIGH);

  Serial.begin(9600);

  // Guardar el tiempo de inicio
  tiempoInicio = millis();
}

void loop() {
  unsigned long tiempoActual = millis();

  if (cicloActualRele1 < CICLOS_TOTALES_RELE1) {
    // Ciclo activando el rele1
    digitalWrite(RELE1_PIN, LOW);

    // Medir corriente y mostrar resultados
    float corriente = get_corriente();
    Serial.print("Corriente: ");
    Serial.print(corriente, 3);
    Serial.println("A");

    delay(DURACION_RELE1);
    digitalWrite(RELE1_PIN, HIGH);

    // Esperar el intervalo entre ciclos
    delay(INTERVALO_ENTRE_CICLOS_RELE1);

    cicloActualRele1++;
  } else if (cicloActualRele2 < CICLOS_TOTALES_RELE2) {
    // Ciclo activando el rele2
    digitalWrite(RELE2_PIN, LOW);
    delay(DURACION_RELE2);
    digitalWrite(RELE2_PIN, HIGH);

    // Medir corriente y mostrar resultados
    float corriente = get_corriente();
    Serial.print("Corriente: ");
    Serial.print(corriente, 3);
    Serial.println("A");

    // Esperar el intervalo entre ciclos
    delay(INTERVALO_ENTRE_CICLOS_RELE2);

    cicloActualRele2++;
  } else {
    // Apagar ambos relés
    digitalWrite(RELE1_PIN, HIGH);
    digitalWrite(RELE2_PIN, HIGH);

    // Esperar hasta que se reinicie el sistema o realice alguna otra acción
    if (tiempoActual - tiempoInicio >= TIEMPO_TOTAL_ESPERA) {
      // Reiniciar el conteo de ciclos y el tiempo de inicio
      cicloActualRele1 = 0;
      cicloActualRele2 = 0;
      tiempoInicio = tiempoActual;
    }
  }
}

float get_corriente() {
  float voltajeSensor;
  float corriente = 0;
  long tiempo = millis();
  float Imax = 0;
  float Imin = 0;
  while (millis() - tiempo < 500)  // realizamos mediciones durante 0.5 segundos
  {
    voltajeSensor = analogRead(A0) * (5.0 / 1023.0);  // lectura del sensor
    corriente = 0.9 * corriente + 0.1 * ((voltajeSensor - 2.527) / Sensibilidad);  // Ecuación  para obtener la corriente
    if (corriente > Imax) Imax = corriente;
    if (corriente < Imin) Imin = corriente;
  }
  return (((Imax - Imin) / 2) - offset);
}
