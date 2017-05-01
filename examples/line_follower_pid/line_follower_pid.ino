#include <dreamster.h>
#include <PID_v1.h>
#include "helpers.h"

// serial config
const bool USAR_SERIAL = true;
const long SERIAL_BPS = 115200;

// determina si sigue una línea blanca (true) o una línea negra (false)
const bool LINEA_BLANCA = false;

Dreamster robot;
Leds leds;

const int CANTIDAD_DE_SENSORES = 2;
int sensores[CANTIDAD_DE_SENSORES];
int minimos_sensores[CANTIDAD_DE_SENSORES];
int maximos_sensores[CANTIDAD_DE_SENSORES];
float coeficientes_sensores[CANTIDAD_DE_SENSORES];
const int IZQ = 0;
const int DER = 1;

double k_p = 50, k_i = 0, k_d = 1;
double setpoint, input, output;
PID motores_pid(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);

void setup() {
  
  if (USAR_SERIAL) {
    Serial.begin(SERIAL_BPS);
  }  
  
  // inicialización de sensores y valores de calibración
  for (int i = 0; i < CANTIDAD_DE_SENSORES; i++) {
    sensores[i] = 0;
    minimos_sensores[i] = 0;
    maximos_sensores[i] = 1023;
    coeficientes_sensores[i] = 1.0;
  }
  
  // inicialización de PID, en modo automático, computando valores cada 1 ms,
  // y escalando los valores de salida entre 0 y 200 (100 por motor)
  input = 1024;
  setpoint = 1024;
  motores_pid.SetMode(AUTOMATIC);
  motores_pid.SetSampleTime(1); // ms
  motores_pid.SetOutputLimits(0, 200);

}

void loop() {
  delay(200);
  calibrar();
  delay(1000);
  seguirLinea();
}

void calibrar() {
int valor_sensor_linea = 1023;
int valor_sensor_piso = 0;
int ultimo_tiempo_sensor;
int contador_sensor = 0;
  
  leds.redOn();

  // reseteo calibración
  for (int i = 0; i < CANTIDAD_DE_SENSORES; i++) {
    sensores[i] = 0;
    minimos_sensores[i] = 1023;
    maximos_sensores[i] = 0;
    coeficientes_sensores[i] = 0.0;
  }
  
  // giro a la izquierda
  robot.move(-100, 100);
  
  ultimo_tiempo_sensor = millis();
  // giro por más o menos 2 vueltas en el lugar
  while (1) {
    // leo los sensores, y guardo los mínimos y los máximos
    obtenerSensores();
    for (int i = 0; i < CANTIDAD_DE_SENSORES; i++) {
      if (sensores[i] < minimos_sensores[i]) {
        minimos_sensores[i] = sensores[i];
        valor_sensor_piso = sensores[i];
      }
      if (sensores[i] > maximos_sensores[i]) {
        maximos_sensores[i] = sensores[i];
        valor_sensor_linea = sensores[i];
      }
    }
    
    // si todavía no encontré la línea, sigo esperando
    if (abs(valor_sensor_piso - valor_sensor_linea) < 100) {
      continue;
    }
    
    // si pasaron al menos 100 ms desde la última vez,
    // y si encontré la línea, 
    // entonces sumo 1 al contador
    if (millis() - ultimo_tiempo_sensor > 250 && sensores[IZQ] >= valor_sensor_linea - 50) {
      contador_sensor++;
      ultimo_tiempo_sensor = millis();
      if (USAR_SERIAL) {
        Serial.print("sensor\n");
      }
    }
    // cuando encontré la línea 2 veces, quito la inercia de los motores
    // y salgo
    if (contador_sensor == 2) {
      robot.move(100, -100);
      delay(10);
      break;
    }
  }
  // calculo los coeficientes de calibración
  for (int i = 0; i < CANTIDAD_DE_SENSORES; i++) {
    coeficientes_sensores[i] = 1023.0 / (float)(maximos_sensores[i] - minimos_sensores[i]);
  }
  
  robot.move(0, 0);
  
  leds.redOff();
  
}

void seguirLinea() {
  const int VALOR_SENSOR_LINEA = 900;
  const int VALOR_SENSOR_PISO = 256;
  const int VELOCIDAD_AVANZAR = 75;
  const int VELOCIDAD_FRENAR = -10;
  int sensores_linea = 0;
  int motor_izq = 0; 
  int motor_der = 0;
  unsigned int tiempo_debug = 0;
  unsigned int ultimo_tiempo_debug = 0;
  int ultimo_lado_sensor = IZQ;
  char modo = 'P';
  
  while (1) {
    obtenerSensoresCalibrados();
    sensores_linea = ((long)sensores[DER] * 2048) / ((long)sensores[IZQ] + (long)sensores[DER]);
    input = (double)sensores_linea;
    // llamo a PID siempre, aunque después no use sus valores,
    // para que no acumule valores en el tiempo sin ser usado
    motores_pid.Compute();
    
    if (sensores[IZQ] >= VALOR_SENSOR_LINEA - 50) {
      ultimo_lado_sensor = IZQ;
    } else if (sensores[DER] >= VALOR_SENSOR_LINEA - 50) {
      ultimo_lado_sensor = DER;
    }
    if (sensores[IZQ] <= VALOR_SENSOR_PISO + 50 && sensores[DER] <= VALOR_SENSOR_PISO + 50) {
      modo = 'A';
      
      if (ultimo_lado_sensor == IZQ) {
        leds.redOn();
        motor_izq = VELOCIDAD_FRENAR;
        motor_der = VELOCIDAD_AVANZAR;
      } else if (ultimo_lado_sensor == DER) {
        leds.blueOn();
        motor_izq = VELOCIDAD_AVANZAR;
        motor_der = VELOCIDAD_FRENAR;
      }
    } else {
      modo = 'P';
      leds.redOff();
      leds.blueOff();
      
      // 200 => ir a la izquierda
      //   0 => ir a la derecha
      if (output < 100) {
        motor_izq = 100 - output;
        motor_der = 0;
      } else if (output >= 100) {
        motor_izq = 0;
        motor_der = output / 2;
      }
    }
    robot.move(motor_izq, motor_der);
    
    if (USAR_SERIAL) {
      debug("%c ", modo);
      debug("T: %.4u ", tiempo_debug);
      debug("L: %.4i ", (int)sensores_linea);
      debug("I: %.4i ", sensores[IZQ]);
      debug("D: %.4i ", sensores[DER]);
      debug("VP: %.4i ", VALOR_SENSOR_PISO);
      debug("VL: %.4i ", VALOR_SENSOR_LINEA);
      Serial.print("\n");
      tiempo_debug = micros() - ultimo_tiempo_debug;
      ultimo_tiempo_debug = micros();
    }
  }

}

inline void obtenerSensores() {
  uint16_t left;
  uint16_t right;
  robot.read_ir(left, right);
  if (LINEA_BLANCA) {
    sensores[0] = left;
    sensores[1] = right;
  } else {
    sensores[0] = 1023 - left;
    sensores[1] = 1023 - right;
  }
}

inline void obtenerSensoresCalibrados() {
  int valor = 0;
  float valor_float = 0.0;
  obtenerSensores();
  for (int i = 0; i < CANTIDAD_DE_SENSORES; i++) {
    valor_float = sensores[i] - minimos_sensores[i];
    valor_float = valor_float * coeficientes_sensores[i];
    valor = (int)valor_float;
    if (valor > 1023) {
      valor = 1023;
    } else if (valor < 0) {
      valor = 0;
    }
    sensores[i] = valor;
  }
}
