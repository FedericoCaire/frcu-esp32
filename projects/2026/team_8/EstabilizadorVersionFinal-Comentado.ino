/*
 * ============================================================================
 *  ESTABILIZADOR DE CELULAR (GIMBAL DE 2 EJES) - ESP32 + MPU6050 + 2 SERVOS
 * ============================================================================
 *
 *  ¿QUÉ HACE ESTE PROGRAMA?
 *  Mantiene nivelado un soporte de celular aunque la base se incline.
 *  Un sensor MPU6050 (acelerómetro + giroscopio) mide la inclinación y dos
 *  servomotores se mueven en sentido contrario para compensarla.
 *
 *  EJES:
 *    - Eje X  -> "roll"  (inclinación lateral, como un balanceo de lado a lado)
 *    - Eje Y  -> "pitch" (inclinación de adelante hacia atrás)
 *
 *  ¿CÓMO FUNCIONA, EN RESUMEN?
 *  1) Al arrancar, se calibra el giroscopio (se mide su error en reposo).
 *  2) En cada vuelta del loop():
 *       a) Se leen el acelerómetro y el giroscopio.
 *       b) Se estima el ángulo actual combinando ambos sensores
 *          (un "filtro complementario" simplificado, ver sección 5).
 *       c) Con ese ángulo, un control integral mueve los servos hasta que
 *          el ángulo estimado vuelve a ser 0 (es decir, la posición está nivelada).
 *       d) Cada 1 segundo se imprimen los datos por el monitor serie.
 *
 *  CUESTIONES A TENER EN CUENTA
 *    - El GIROSCOPIO mide velocidad de giro (°/s). Es muy preciso para
 *      movimientos rápidos, pero al integrarlo (sumar en el tiempo) se va
 *      acumulando un error llamado "deriva" (el ángulo se va corriendo solo).
 *    - El ACELERÓMETRO mide la gravedad, así que da un ángulo absoluto que
 *      no deriva (es decir no tiene error), pero es muy ruidoso cuando el aparato se mueve o vibra.
 *    - Se usa el giroscopio mientras hay movimiento, y el acelerómetro
 *      para corregir la deriva cuando el aparato está quieto.
 *
 *  CONEXIONES:
 *    - MPU6050: SDA -> pin 18, SCL -> pin 19 (bus I2C)
 *    - Servo X (roll)  -> pin 16
 *    - Servo Y (pitch) -> pin 17
 *    - Joystick: VRx -> pin 4, VRy -> pin 15, SW (botón) -> pin 22,
 *                +5V -> pin 21 (el código lo pone en 3.3 V; ¡NUNCA a los 5 V de los servos!),
 *                GND -> pin 23 (el código lo pone en 0 V)
 *    - Luz testigo: el LED azul que ya trae la placa (pin 2). No lleva cables.
 *    - Botón extra: una pata -> pin 5, la otra -> GND común.
 *
 *  BOTÓN DEL JOYSTICK (presionar el stick, el "botón R3"):
 *    - Cambia entre modo AUTOMÁTICO y MANUAL.
 *
 *  BOTÓN EXTRA (pin 5):
 *    - TOQUE CORTO: cambia la velocidad (lenta -> media -> rápida -> lenta...)
 *      del modo manual Y de la corrección automática. El LED parpadea 1, 2 o 3 veces.
 *    - MANTENER 2 SEGUNDOS: vuelve a la posición nivelada del arranque y pasa
 *      a modo AUTOMÁTICO. El LED parpadea 4 veces rápido. No hace falta reiniciar.
 *
 *  MODOS:
 *    - AUTOMÁTICO (LED apagado): el gimbal se estabiliza solo y mantiene el
 *      ángulo de referencia. Al encender, la referencia es "nivelado".
 *    - MANUAL (LED prendido): la estabilización se pausa. Inclinar el stick
 *      mueve los servos; al soltarlo, se quedan donde están.
 *      Al volver a AUTOMÁTICO, el ángulo en que quedó pasa a ser la nueva
 *      referencia. Para volver a "nivelado", mantener 2 s el botón extra.
 *
 *  ETAPA 2: todo lo que agregó el Grupo 16 sobre el proyecto original del
 *  Grupo 8 está marcado con [ETAPA 2] en los comentarios.
 */

// ============================================================================
//  LIBRERÍAS
// ============================================================================
#include <Adafruit_MPU6050.h>  // Manejo del sensor MPU6050
#include <Adafruit_Sensor.h>   // Define el tipo "sensors_event_t" usado para leer datos
#include <Wire.h>              // Comunicación I2C (la usa el MPU6050)
#include <ESP32Servo.h>        // Control de servos en el ESP32

// ============================================================================
//  OBJETOS GLOBALES
// ============================================================================
Adafruit_MPU6050 mpu;  // Representa al sensor
Servo servoX;          // Servo que compensa el roll  (eje X)
Servo servoY;          // Servo que compensa el pitch (eje Y)

// ============================================================================
//  ÁNGULO ESTIMADO (resultado del filtro)
//  Es el "mejor cálculo" de cuánto está inclinado el aparato, en grados.
//  0° significa nivelado.
// ============================================================================
float anguloEstimadoX = 0;
float anguloEstimadoY = 0;

// ============================================================================
//  POSICIÓN CENTRAL DE LOS SERVOS
//  Valores iniciales que usamos para centrar el mpu y que quede nivelado.
//  Ángulo (0-180) al que debe estar cada servo para que el soporte quede
//  nivelado. Son valores propios de CÓMO QUEDÓ ARMADO este montaje físico;
//  si se rearma la estructura habrá que reajustarlos.
// ============================================================================
const float CENTRO_SERVO_X = 104.0;
const float CENTRO_SERVO_Y = 134.0;

// ============================================================================
//  TEMPORIZADORES
//  millis() devuelve los milisegundos transcurridos desde que se encendió
//  la placa. Se guardan marcas de tiempo para medir intervalos.
// ============================================================================
unsigned long tiempoAnterior = 0;  // Instante de la vuelta anterior del loop (para calcular dt)
unsigned long tiempoSerial = 0;    // Última vez que se imprimió por el monitor serie

unsigned long tiempoQuietoX = 0;   // Último momento en que se detectó movimiento en X
unsigned long tiempoQuietoY = 0;   // Último momento en que se detectó movimiento en Y

// ============================================================================
//  PARÁMETROS DEL FILTRO
// ============================================================================
const float UMBRAL_GYRO = 1.0;              // Constante que se usa básicamente para que el dispositivo no se reacomode ante una pequeña turbulencia. Por debajo de esto se considera que NO hay movimiento
const unsigned long TIEMPO_ESPERA = 500;    // milisegundos que debe estar quieto antes de corregir la deriva
const float CORRECCION_DERIVA = 0.02;       // Cuánto "pesa" el acelerómetro en cada corrección (2%).
                                            // Es chico a propósito: la corrección es lenta y suave.

// ============================================================================
//  OFFSETS DEL GIROSCOPIO
//  Un giroscopio nunca marca exactamente 0 en reposo; tiene un pequeño error
//  constante. Se mide al inicio (en función: "calibrarGyro" que está un poco más abajo) y se resta en cada lectura.
// ============================================================================
float offsetGyroX = 0;
float offsetGyroY = 0;
float offsetGyroZ = 0;

// ============================================================================
//  CONTROL DE SERVOS
// ============================================================================
float servoDeltaX = 0;   // Compensación acumulada del servo X (grados respecto al centro)
float servoDeltaY = 0;   // Compensación acumulada del servo Y (grados respecto al centro)

// [ETAPA 2] Ganancia integral: qué tan rápido se corrige en cada ciclo, una por cada
// nivel de velocidad (lenta, media, rápida). Se elige con el botón extra.
// Más alto = reacciona más rápido, pero puede oscilar (el soporte "tiembla"
// o se pasa de largo). El valor original del proyecto era 0.15.
const float KI_POR_NIVEL[] = {0.15, 0.30, 0.50};
const float ZONA_MUERTA = 0.5;  // Errores menores a este valor (en grados) se ignoran,
                                // así los servos no tiemblan por ruido mínimo.
const float LIMITE_DELTA = 85;  // Máxima compensación permitida desde el centro (protege el mecanismo).

// Cuando el pitch (eje Y) es muy grande, el cálculo del roll (eje x) con acelerómetro
// se vuelve poco confiable, así que en ese caso no se usa para corregir el roll.
// Por esto definimos la constante LIMITE_PUTCH_CORR como un límite a considerar.
const float LIMITE_PITCH_CORR = 50.0;

// ============================================================================
// [ETAPA 2]  JOYSTICK Y MODO MANUAL
//  Se usan pines de la fila de la placa que tiene lugar libre en la protoboard.
//  Los pines 4 y 15 son del ADC2: funcionan bien porque no se usa WiFi
//  (con WiFi encendido, el ADC2 deja de leer).
//
//  El joystick se alimenta desde dos pines comunes del ESP32 (uno en HIGH = 3.3 V
//  y otro en LOW = 0 V). Es válido porque consume menos de 1 mA, y así no se
//  comparte nada con los 5 V de los servos.
// ============================================================================
const int PIN_JOY_X = 4;      // VRx: mueve el servo X (roll)
const int PIN_JOY_Y = 15;     // VRy: mueve el servo Y (pitch)
const int PIN_JOY_BOTON = 22; // SW: al presionar el stick, se conecta a GND
const int PIN_JOY_VCC = 21;   // +5V del joystick (en realidad recibe 3.3 V)
const int PIN_JOY_GND = 23;   // GND del joystick
const int PIN_LED = 2;        // LED azul de la placa: luz testigo del modo
const int PIN_BOTON_EXTRA = 5; // Botón extra: velocidad (corto) y volver a nivelado (largo)

const int ZONA_MUERTA_JOY = 200;        // Lecturas (de 0 a 4095) alrededor del centro que se ignoran,
                                        // porque el stick suelto nunca da exactamente el mismo valor.
// [ETAPA 2] ORIENTACIÓN DEL JOYSTICK
// El joystick está montado con los pines de conexión hacia ARRIBA. En esa
// posición, sin intercambiar, mover el stick arriba/abajo inclinaba el
// soporte hacia los costados (y al revés): los ejes quedaban cruzados.
// Por eso se intercambian. Si se monta en otra posición, probar con false.
const bool INTERCAMBIAR_EJES_JOY = true;
const int INVERTIR_JOY_X = -1;          // -1: izquierda/derecha estaba al revés (probado en el gimbal)
const int INVERTIR_JOY_Y = -1;          // -1: arriba/abajo estaba al revés (probado en el gimbal)

// Velocidad de los servos en modo manual con el stick a fondo, en grados
// por segundo. Se elige con toques cortos del botón extra (lenta, media,
// rápida); el mismo nivel elige también KI_POR_NIVEL para el modo automático.
const float VELOCIDADES[] = {45.0, 90.0, 180.0};
const int CANTIDAD_VELOCIDADES = 3;
int nivelVelocidad = 1;  // Arranca en velocidad media

const unsigned long ANTIRREBOTE = 50;             // ms: ignora los "rebotes" eléctricos del botón
const unsigned long TIEMPO_PRESION_LARGA = 2000;  // ms que hay que mantener el botón extra para volver a nivelado
const unsigned long DURACION_PARPADEO = 150;      // ms que dura cada encendido/apagado del LED

int centroJoyX = 2048;  // Valor del stick suelto; se mide en calibrarJoystick()
int centroJoyY = 2048;

// Si el centro medido cae fuera de este rango, el joystick no esta bien
// conectado (un cable suelto lee 0 o 4095). En ese caso se ignora el stick.
const int CENTRO_JOY_MIN = 800;
const int CENTRO_JOY_MAX = 3300;
bool joystickOk = true;

bool modoManual = false;

// Ángulo que el modo automático intenta mantener. Arranca en 0 (nivelado) y
// se reemplaza por el ángulo actual cada vez que se vuelve de modo manual.
float referenciaX = 0;
float referenciaY = 0;

// Estado del botón del joystick (ver revisarBotonJoystick())
bool botonJoyPresionado = false;
unsigned long tiempoCambioBotonJoy = 0;

// Estado del botón extra (ver revisarBotonExtra())
bool botonExtraPresionado = false;
bool presionLargaHecha = false;
unsigned long tiempoCambioBotonExtra = 0;
unsigned long tiempoInicioPresion = 0;

// Parpadeos pendientes del LED (cada encendido o apagado cuenta como uno)
int parpadeosRestantes = 0;
unsigned long tiempoParpadeo = 0;

// ============================================================================
// [ETAPA 2]  FUNCIÓN: cambiarModo()
//  Pasa de automático a manual o al revés. Al volver a automático, el ángulo
//  en el que quedó el soporte pasa a ser la nueva referencia a mantener.
// ============================================================================
void cambiarModo() {
  modoManual = !modoManual;

  if (!modoManual) {
    referenciaX = anguloEstimadoX;
    referenciaY = anguloEstimadoY;
  }
}

// ============================================================================
// [ETAPA 2]  FUNCIÓN: cambiarVelocidad()
//  Pasa a la velocidad siguiente (y de la última vuelve a la primera).
//  El LED parpadea 1, 2 o 3 veces para indicar cuál quedó elegida.
// ============================================================================
void cambiarVelocidad() {
  nivelVelocidad = (nivelVelocidad + 1) % CANTIDAD_VELOCIDADES;

  parpadeosRestantes = 2 * (nivelVelocidad + 1);
  tiempoParpadeo = millis();
}

// ============================================================================
// [ETAPA 2]  FUNCIÓN: volverANivelado()
//  Vuelve a la referencia del arranque (0 = nivelado) y pasa a modo
//  automático, que se encarga de llevar el soporte de nuevo a la posición
//  derecha. El LED parpadea 4 veces para confirmarlo.
// ============================================================================
void volverANivelado() {
  referenciaX = 0;
  referenciaY = 0;
  modoManual = false;

  parpadeosRestantes = 8;
  tiempoParpadeo = millis();

  Serial.println(">>> Volviendo a nivelado");
}

// ============================================================================
// [ETAPA 2]  FUNCIÓN: revisarBotonJoystick()
//  Se llama en cada vuelta del loop(). Cada vez que se presiona el stick,
//  cambia de modo. Se usa lectura en el loop (y no una interrupción) igual
//  que con el botón extra, así los dos funcionan de la misma forma.
// ============================================================================
void revisarBotonJoystick() {
  bool lectura = (digitalRead(PIN_JOY_BOTON) == LOW);  // LOW = presionado

  // Solo se acepta un cambio si pasó el tiempo de antirrebote
  if (lectura != botonJoyPresionado && millis() - tiempoCambioBotonJoy >= ANTIRREBOTE) {
    botonJoyPresionado = lectura;
    tiempoCambioBotonJoy = millis();

    if (botonJoyPresionado) cambiarModo();
  }
}

// ============================================================================
// [ETAPA 2]  FUNCIÓN: revisarBotonExtra()
//  Se llama en cada vuelta del loop(). Distingue dos tipos de presión:
//    - Corta: se suelta antes de TIEMPO_PRESION_LARGA -> cambia la velocidad.
//    - Larga: se mantiene TIEMPO_PRESION_LARGA -> vuelve a nivelado
//      (y al soltarlo NO cambia la velocidad).
//  Para saber si la presión fue corta o larga hay que medir cuánto tiempo
//  se mantuvo, por eso la acción corta se hace recién al soltar.
// ============================================================================
void revisarBotonExtra() {
  bool lectura = (digitalRead(PIN_BOTON_EXTRA) == LOW);  // LOW = presionado

  if (lectura != botonExtraPresionado && millis() - tiempoCambioBotonExtra >= ANTIRREBOTE) {
    botonExtraPresionado = lectura;
    tiempoCambioBotonExtra = millis();

    if (botonExtraPresionado) {
      tiempoInicioPresion = millis();
      presionLargaHecha = false;
    } else if (!presionLargaHecha) {
      cambiarVelocidad();  // Se soltó rápido: presión corta
    }
  }

  // Si se mantiene apretado el tiempo suficiente: presión larga
  if (botonExtraPresionado && !presionLargaHecha &&
      millis() - tiempoInicioPresion >= TIEMPO_PRESION_LARGA) {
    presionLargaHecha = true;
    volverANivelado();
  }
}

// ============================================================================
// [ETAPA 2]  FUNCIÓN: actualizarLed()
//  Si hay parpadeos pendientes, los hace sin frenar el programa (no usa
//  delay). Si no, el LED muestra el modo: prendido = manual, apagado = automático.
// ============================================================================
void actualizarLed() {
  if (parpadeosRestantes > 0) {
    if (millis() - tiempoParpadeo >= DURACION_PARPADEO) {
      parpadeosRestantes--;
      tiempoParpadeo = millis();
    }
    // Impar = prendido, par = apagado (arranca apagado para que se note)
    digitalWrite(PIN_LED, parpadeosRestantes % 2 == 1 ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED, modoManual ? HIGH : LOW);
  }
}

// ============================================================================
// [ETAPA 2]  FUNCIÓN: leerEjeJoystick()
//  Convierte la lectura de un eje (0 a 4095) en un valor de -1 a +1
//  (-1 = stick a fondo para un lado, +1 = para el otro). Dentro de la zona
//  muerta da 0.
// ============================================================================
float leerEjeJoystick(int pin, int centro) {
  if (!joystickOk) return 0;  // Joystick mal conectado: el stick no mueve nada

  int desvio = analogRead(pin) - centro;

  if (abs(desvio) < ZONA_MUERTA_JOY) return 0;

  // Se escala por separado cada lado, porque el centro real no suele ser 2048
  if (desvio > 0) return (desvio - ZONA_MUERTA_JOY) / float(4095 - centro - ZONA_MUERTA_JOY);
  return (desvio + ZONA_MUERTA_JOY) / float(centro - ZONA_MUERTA_JOY);
}


// ============================================================================
//  [ETAPA 2] FUNCION: calibrarJoystick()
//  Con el stick SUELTO, promedia 32 lecturas de cada eje para saber cual es
//  su centro real (una sola lectura tiene ruido). Si el valor no es razonable,
//  avisa y deja el joystick deshabilitado en vez de mover los servos solos.
// ============================================================================
void calibrarJoystick() {
  const int muestras = 32;
  long sumaX = 0;
  long sumaY = 0;

  for (int i = 0; i < muestras; i++) {
    sumaX += analogRead(PIN_JOY_X);
    sumaY += analogRead(PIN_JOY_Y);
    delay(2);
  }

  centroJoyX = sumaX / muestras;
  centroJoyY = sumaY / muestras;

  Serial.print("Centro joystick X: ");
  Serial.print(centroJoyX);
  Serial.print(" | Y: ");
  Serial.println(centroJoyY);

  joystickOk = centroJoyX > CENTRO_JOY_MIN && centroJoyX < CENTRO_JOY_MAX &&
               centroJoyY > CENTRO_JOY_MIN && centroJoyY < CENTRO_JOY_MAX;

  if (!joystickOk) {
    Serial.println("¡ATENCION! Joystick no detectado: revisar los cables.");
    Serial.println("El stick queda deshabilitado (la estabilizacion funciona igual).");
  }
}

// ============================================================================
//  FUNCIÓN: calibrarGyro()
//  Se ejecuta una sola vez al inicio. Con el sensor COMPLETAMENTE QUIETO toma
//  500 lecturas del giroscopio y calcula el promedio de cada eje. Ese promedio
//  es el error de fábrica/temperatura ("offset") que luego se resta.
// ============================================================================
void calibrarGyro() {

  const int muestras = 500;  // Cantidad de lecturas a promediar

  float sumaX = 0;
  float sumaY = 0;
  float sumaZ = 0;

  Serial.println("================================");
  Serial.println("CALIBRANDO GIROSCOPIO");
  Serial.println("NO MUEVAS EL MPU6050");
  Serial.println("================================");

  delay(2000);  // Margen para soltar el aparato y que quede quieto

  // Se acumulan las lecturas
  for (int i = 0; i < muestras; i++) {

    sensors_event_t gyro;
    mpu.getGyroSensor()->getEvent(&gyro);

    // El sensor entrega radianes/segundo; se convierte a grados/segundo
    float gyroX = gyro.gyro.x * 180.0 / PI;
    float gyroY = gyro.gyro.y * 180.0 / PI;
    float gyroZ = gyro.gyro.z * 180.0 / PI;

    sumaX += gyroX;
    sumaY += gyroY;
    sumaZ += gyroZ;

    delay(5);  // Pausa corta entre lecturas (500 muestras x 5 ms = ~2.5 s)
  }

  // Promedio = error constante del giroscopio en reposo
  offsetGyroX = sumaX / muestras;
  offsetGyroY = sumaY / muestras;
  offsetGyroZ = sumaZ / muestras;

  Serial.println("Calibracion terminada.");

  Serial.print("Offset X: ");
  Serial.println(offsetGyroX);

  Serial.print("Offset Y: ");
  Serial.println(offsetGyroY);

  Serial.print("Offset Z: ");
  Serial.println(offsetGyroZ);

  Serial.println("================================");
}

// ============================================================================
//  SETUP: se ejecuta una vez al encender o reiniciar la placa
// ============================================================================
void setup() {
  Serial.begin(115200);  // Inicia el monitor serie (configurarlo a 115200 baudios)

  // Inicia el bus I2C con SDA en el pin 18 y SCL en el pin 19
  Wire.begin(18, 19);

  // Se asigna cada servo a su pin
  servoX.attach(16);
  servoY.attach(17);

  // Los servos arrancan en su posición central (soporte nivelado)
  servoX.write(CENTRO_SERVO_X);
  servoY.write(CENTRO_SERVO_Y);
  delay(2000);  // Tiempo para que dejen el dispositivo recto y quieto para que tome correctamente las lecturas del mpu

  // Se intenta conectar con el MPU6050; si no responde, reintenta cada 1 s
  // (el programa queda "esperando" acá hasta que el sensor esté conectado)
  while (!mpu.begin()) {
    Serial.println("MPU6050 not connected!");
    delay(1000);
  }
  delay(2000);

  Serial.println("MPU6050 ready!, calibrando gyro...");
  delay(1000);

  calibrarGyro();  // Mide el error del giroscopio (el aparato debe estar quieto)

  delay(2000);
  Serial.println("Supuestamente calibrado");

  // Pausa para que el sensor se estabilice antes de empezar
  delay(1000);

  // Se hace una lectura inicial del acelerómetro.
  // (El dato no se guarda ni se usa más adelante; el ángulo estimado
  //  arranca en 0, es decir, se asume que el aparato inicia nivelado.)
  sensors_event_t event;
  mpu.getAccelerometerSensor()->getEvent(&event);

  Serial.println("Posicion inicial guardada.");

  // Joystick: primero se le da energía (pin 21 = 3.3 V, pin 23 = 0 V)
  pinMode(PIN_JOY_VCC, OUTPUT);
  digitalWrite(PIN_JOY_VCC, HIGH);
  pinMode(PIN_JOY_GND, OUTPUT);
  digitalWrite(PIN_JOY_GND, LOW);
  delay(50);  // Margen para que la tensión se estabilice antes de leer

  // El botón usa la pull-up interna (suelto = HIGH, presionado = LOW)
  pinMode(PIN_JOY_BOTON, INPUT_PULLUP);
  pinMode(PIN_BOTON_EXTRA, INPUT_PULLUP);  // Mismo esquema que el botón del joystick

  // Luz testigo: arranca apagada (modo automático)
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Se mide el centro del stick (NO TOCAR EL JOYSTICK durante el arranque)
  calibrarJoystick();

  // Se inicializan los temporizadores justo antes de entrar al loop
  tiempoAnterior = millis();
  tiempoQuietoX = millis();
  tiempoQuietoY = millis();
}

// ============================================================================
//  LOOP: se repite constantemente mientras la placa esté encendida
// ============================================================================
void loop() {

  // ==========================================================================
  // 1. LEER SENSORES
  //    "event" guarda los datos del acelerómetro y "gyro" los del giroscopio.
  // ==========================================================================

  sensors_event_t event;
  sensors_event_t gyro;

  mpu.getAccelerometerSensor()->getEvent(&event);
  mpu.getGyroSensor()->getEvent(&gyro);


  // ==========================================================================
  // 2. CALCULAR TIEMPO TRANSCURRIDO (dt)
  //    Para integrar la velocidad de giro hace falta saber cuánto tiempo pasó
  //    desde la vuelta anterior.
  // ==========================================================================

  unsigned long tiempoActual = millis();

  float dt = (tiempoActual - tiempoAnterior) / 1000.0;

  tiempoAnterior = tiempoActual;


  // ==========================================================================
  // 3. GIROSCOPIO
  //    El MPU entrega rad/s; se pasa a grados/s y se le resta el offset
  //    calculado en la calibración para eliminar el error en reposo.
  // ==========================================================================

  float gyroX = gyro.gyro.x * 180.0 / PI;
  float gyroY = gyro.gyro.y * 180.0 / PI;

  gyroX -= offsetGyroX;
  gyroY -= offsetGyroY;


  // ==========================================================================
  // 4. ÁNGULO SEGÚN EL ACELERÓMETRO
  //    Usando la dirección de la gravedad se calcula la inclinación con
  //    trigonometría (atan2). El resultado sale en radianes y se pasa a grados.
  //    Este ángulo es "absoluto" (no deriva) pero es ruidoso si hay vibración.
  // ==========================================================================

  // Roll: rotación alrededor de X (corresponde a gyroX)
  float anguloAcelerometroX = atan2(
    event.acceleration.y,
    event.acceleration.z
  ) * 180.0 / PI;

  // Pitch: rotación alrededor de Y (corresponde a gyroY)
  float anguloAcelerometroY = atan2(
    -event.acceleration.x,
    sqrt(
      event.acceleration.y * event.acceleration.y +
      event.acceleration.z * event.acceleration.z
    )
  ) * 180.0 / PI;


  // ==========================================================================
  // 5. FILTRO (combina giroscopio + acelerómetro)
  //
  //    Para cada eje se distinguen dos situaciones:
  //
  //    A) HAY MOVIMIENTO (el giroscopio supera el umbral):
  //       Se confía en el giroscopio: el ángulo se actualiza sumando
  //       velocidad x tiempo. Además se reinicia el contador de "quietud".
  //
  //    B) ESTÁ QUIETO:
  //       Si lleva quieto más de TIEMPO_ESPERA, se mezcla un poquito del
  //       ángulo del acelerómetro con el ángulo estimado. Así se corrige
  //       lentamente la deriva del giroscopio sin meter ruido del acelerómetro.
  //       La fórmula es un promedio ponderado: 98% estimado + 2% acelerómetro.
  // ==========================================================================

  // ---------- EJE X (roll) ----------

  if (abs(gyroX) > UMBRAL_GYRO) {

    // Estamos moviendo el MPU: integramos la velocidad de giro
    anguloEstimadoX += gyroX * dt;

    // Reiniciamos el contador de quietud
    tiempoQuietoX = millis();

  } else {

    // El MPU está prácticamente quieto en este eje.
    // Se corrige solo si:
    //  - el pitch no es muy grande (con pitch alto, el roll calculado con el
    //    acelerómetro deja de ser confiable), y
    //  - pasaron al menos 500 ms sin movimiento.
    if (fabs(anguloEstimadoY) < LIMITE_PITCH_CORR &&
        (millis() - tiempoQuietoX >= TIEMPO_ESPERA)) {

      // Corrección lenta de la deriva
      anguloEstimadoX =
        anguloEstimadoX * (1.0 - CORRECCION_DERIVA) +
        anguloAcelerometroX * CORRECCION_DERIVA;
    }
  }


  // ---------- EJE Y (pitch) ----------
  // Misma lógica que el eje X, pero sin la restricción del límite de pitch.

  if (abs(gyroY) > UMBRAL_GYRO) {

    anguloEstimadoY += gyroY * dt;

    tiempoQuietoY = millis();

  } else {

    if (millis() - tiempoQuietoY >= TIEMPO_ESPERA) {

      anguloEstimadoY =
        anguloEstimadoY * (1.0 - CORRECCION_DERIVA) +
        anguloAcelerometroY * CORRECCION_DERIVA;
    }
  }


  // ==========================================================================
  // 6. SERVOS (control integral)
  //
  //    El ángulo estimado es el "error": cuánto está inclinado el aparato
  //    respecto a la posición nivelada (0°). En cada ciclo se suma una parte
  //    de ese error (KI * error) a la compensación acumulada del servo.
  //    Mientras haya error, la compensación sigue creciendo; cuando el servo
  //    corrigió lo suficiente, el error llega a 0 y la compensación se frena.
  //    Por eso se llama control "integral".
  //
  //    En modo automático, el "error" se mide respecto a la REFERENCIA
  //    (al encender es 0 = nivelado; después, el ángulo en que se dejó el
  //    soporte al salir del modo manual).
  //
  //    En MODO MANUAL no se estabiliza: el joystick mueve los servos a una
  //    velocidad proporcional a cuánto se inclina el stick. Al soltarlo,
  //    se quedan donde están. El filtro (sección 5) sigue funcionando igual,
  //    así el ángulo estimado está al día cuando se vuelve al modo automático.
  // ==========================================================================

  // [ETAPA 2] Botones, LED y modo manual / referencia
  revisarBotonJoystick();
  revisarBotonExtra();
  actualizarLed();

  if (modoManual) {

    float velocidad = VELOCIDADES[nivelVelocidad];  // grados por segundo
    float lecturaVRx = leerEjeJoystick(PIN_JOY_X, centroJoyX);
    float lecturaVRy = leerEjeJoystick(PIN_JOY_Y, centroJoyY);

    // Con el joystick girado, VRx mueve el servo Y y VRy mueve el servo X
    float movimientoX = INTERCAMBIAR_EJES_JOY ? lecturaVRy : lecturaVRx;
    float movimientoY = INTERCAMBIAR_EJES_JOY ? lecturaVRx : lecturaVRy;

    servoDeltaX += INVERTIR_JOY_X * movimientoX * velocidad * dt;
    servoDeltaY += INVERTIR_JOY_Y * movimientoY * velocidad * dt;

  } else {

    float errorX = anguloEstimadoX - referenciaX;
    float errorY = anguloEstimadoY - referenciaY;

    // Solo se acumula si el error supera la zona muerta (evita temblor por ruido)
    float ki = KI_POR_NIVEL[nivelVelocidad];
    if (fabs(errorX) > ZONA_MUERTA) servoDeltaX += ki * errorX;
    if (fabs(errorY) > ZONA_MUERTA) servoDeltaY += ki * errorY;
  }

  // Se limita la compensación para no forzar el mecanismo
  servoDeltaX = constrain(servoDeltaX, -LIMITE_DELTA, LIMITE_DELTA);
  servoDeltaY = constrain(servoDeltaY, -LIMITE_DELTA, LIMITE_DELTA);

  // Posición final de cada servo = centro +/- compensación.
  // En Y se RESTA porque ese servo está montado en sentido invertido.
  int posicionServoX = round(CENTRO_SERVO_X + servoDeltaX);
  int posicionServoY = round(CENTRO_SERVO_Y - servoDeltaY);

  // Los servos solo aceptan ángulos de 0 a 180
  posicionServoX = constrain(posicionServoX, 0, 180);
  posicionServoY = constrain(posicionServoY, 0, 180);

  // Se envía la orden a los servos
  servoX.write(posicionServoX);
  servoY.write(posicionServoY);


  // ==========================================================================
  // 7. MONITOR SERIE (solo para depuración)
  //    Una vez por segundo se imprimen los valores principales, útiles para
  //    verificar el funcionamiento y ajustar parámetros.
  //    (Se hace cada 1 s, y no en cada vuelta, para no ralentizar el control.)
  // ==========================================================================

  if (millis() - tiempoSerial >= 1000) {

    Serial.println("-----------------------------");

    Serial.println(modoManual ? "MODO: MANUAL (joystick)" : "MODO: AUTOMATICO (estabilizando)");

    const char* nombresVelocidad[] = {"LENTA", "MEDIA", "RAPIDA"};
    Serial.print("Velocidad manual: ");
    Serial.print(nombresVelocidad[nivelVelocidad]);
    Serial.print(" | Referencia X: ");
    Serial.print(referenciaX);
    Serial.print("° | Y: ");
    Serial.print(referenciaY);
    Serial.println("°");

    // Ángulos calculados solo con el acelerómetro
    Serial.print("Acelerometro X: ");
    Serial.print(anguloAcelerometroX);
    Serial.print("° | Y: ");
    Serial.print(anguloAcelerometroY);
    Serial.println("°");

    // Velocidad de giro medida (ya con el offset restado)
    Serial.print("Gyro X: ");
    Serial.print(gyroX);
    Serial.print(" °/s | Gyro Y: ");
    Serial.print(gyroY);
    Serial.println(" °/s");

    // Ángulo final estimado por el filtro
    Serial.print("Angulo estimado X: ");
    Serial.print(anguloEstimadoX);
    Serial.print("° | Y: ");
    Serial.print(anguloEstimadoY);
    Serial.println("°");

    // Posición actual enviada a cada servo
    Serial.print("Servo X: ");
    Serial.print(posicionServoX);
    Serial.print("° | Servo Y: ");
    Serial.print(posicionServoY);
    Serial.println("°");

    tiempoSerial = millis();  // Se marca el momento de esta impresión
  }
}
