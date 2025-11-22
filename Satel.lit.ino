#include <DHT.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// CONFIGURACIÓN DE PINES Y SENSORES
#define DHTPIN 2
#define DHTTYPE DHT11
#define TRIG_PIN 4
#define ECHO_PIN 5
#define SERVO_PIN 9

// DECLARACIÓN DE OBJETOS
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial mySerial(10, 11);
Servo radarServo;

// VARIABLES GLOBALES - RADAR
int angulo = 0;
int pasoBarrido = 2;
unsigned long lastRadarMillis = 0;
const unsigned long radarInterval = 100;
long distanciaActual = -1;
bool direccionDerecha = true;

// VARIABLES GLOBALES - SISTEMA
bool enviarDatos = true;
unsigned long lastSendMillis = 0;
const unsigned long sendInterval = 2000;

// VARIABLES GLOBALES - TEMPERATURA
float tempBuffer[10];
int bufferIndex = 0;
bool bufferLleno = false;
float valorLimite = 25.0;
bool calculoEnSatelite = true;

// VARIABLES GLOBALES - ALARMA
float ultimasMedias[3];
int mediaIndex = 0;
bool alarmaActivada = false;

// VARIABLES GLOBALES - TIMEOUT
bool esperandoTimeout = false;
unsigned long nextTimeoutHT = 0;

// FUNCIONES DEL RADAR - MEJORADA
long medirDistancia() {
  // Limpiar el trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Activar el pulso de trigger por 10 microsegundos
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Medir la duración del pulso en el pin ECHO
  unsigned long duracion = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout de 30ms
  
  // Calcular distancia (en cm)
  if (duracion == 0) {
    return -1; // Timeout - no se recibió eco
  }
  
  long distancia = duracion * 0.0343 / 2;
  
  // Filtrar valores fuera de rango
  if (distancia < 2 || distancia > 400) {
    return -1;
  }
  
  return distancia;
}

void inicializarRadar() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  radarServo.attach(SERVO_PIN);
  radarServo.write(angulo);
  digitalWrite(TRIG_PIN, LOW);
}

void actualizarRadar() {
  if (millis() - lastRadarMillis >= radarInterval) {
    lastRadarMillis = millis();

    // Mover servo al ángulo actual
    radarServo.write(angulo);
    
    // Pequeña pausa para estabilizar el servo antes de medir
    delay(15);
    
    // Medir distancia
    distanciaActual = medirDistancia();

    // Actualizar ángulo para siguiente movimiento
    if (direccionDerecha) {
      angulo += pasoBarrido;
      if (angulo >= 180) {
        angulo = 180;
        direccionDerecha = false;
      }
    } else {
      angulo -= pasoBarrido;
      if (angulo <= 0) {
        angulo = 0;
        direccionDerecha = true;
      }
    }
  }
}

// FUNCIONES DE TEMPERATURA
float calcularMediaTemperatura() {
  float suma = 0.0;
  int elementos = bufferLleno ? 10 : bufferIndex;
  
  if (elementos == 0) return 0.0;
  
  for (int i = 0; i < elementos; i++) {
    suma += tempBuffer[i];
  }
  return suma / elementos;
}

void agregarTemperaturaAlBuffer(float temperatura) {
  tempBuffer[bufferIndex] = temperatura;
  bufferIndex = (bufferIndex + 1) % 10;
  if (!bufferLleno && bufferIndex == 0) bufferLleno = true;
}

// FUNCIONES DE ALARMA
bool verificarAlarma() {
  for (int i = 0; i < 3; i++) {
    if (ultimasMedias[i] <= valorLimite) return false;
  }
  return true;
}

void actualizarSistemaAlarma(float mediaTemp) {
  if (!bufferLleno && bufferIndex < 3) return;
  
  ultimasMedias[mediaIndex] = mediaTemp;
  mediaIndex = (mediaIndex + 1) % 3;
  
  bool alarmaDetectada = verificarAlarma();
  
  if (alarmaDetectada && !alarmaActivada) {
    alarmaActivada = true;
    Serial.println("ALARMA ACTIVADA!");
    mySerial.println("ALARMA:ACTIVADA");
  }
  else if (!alarmaDetectada && alarmaActivada) {
    alarmaActivada = false;
    Serial.println("ALARMA DESACTIVADA");
    mySerial.println("ALARMA:DESACTIVADA");
  }
}

// FUNCIONES DE COMUNICACIÓN
void procesarComandos(String cmd) {
  cmd.trim();
  
  if (cmd.equalsIgnoreCase("Parar")) {
    enviarDatos = false;
    Serial.println("Parando envío de datos");
  }
  else if (cmd.equalsIgnoreCase("Reanudar")) {
    enviarDatos = true;
    Serial.println("Reanudando envío de datos");
  }
  else if (cmd.startsWith("Limite:")) {
    valorLimite = cmd.substring(7).toFloat();
    Serial.print("Nuevo límite de alarma: ");
    Serial.println(valorLimite);
    mySerial.print("LimiteActualizado:");
    mySerial.println(valorLimite);
  }
  else if (cmd.equalsIgnoreCase("Modo:Satélite")) {
    calculoEnSatelite = true;
    Serial.println("Procesamiento en satélite");
    mySerial.println("Modo:Satelite");
    // Reiniciar sistema de alarma
    for (int i = 0; i < 3; i++) ultimasMedias[i] = 0.0;
    mediaIndex = 0;
    alarmaActivada = false;
  }
  else if (cmd.equalsIgnoreCase("Modo:Tierra")) {
    calculoEnSatelite = false;
    Serial.println("Procesamiento en tierra");
    mySerial.println("Modo:Tierra");
  }
}

void procesarEntradaSerial() {
  while (mySerial.available()) {
    String cmd = mySerial.readStringUntil('\n');
    procesarComandos(cmd);
  }
  
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    procesarComandos(cmd);
  }
}

// FUNCIONES PRINCIPALES DEL SISTEMA
void manejarTimeoutDHT() {
  if (esperandoTimeout && (millis() >= nextTimeoutHT)) {
    Serial.println("Error: Timeout DHT");
    mySerial.println("Fallo");
    esperandoTimeout = false;
  }
}

void enviarDatosCompletos() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (isnan(t) || isnan(h)) {
    Serial.println("Error leyendo sensor DHT");
    return;
  }
  
  // Configurar timeout
  esperandoTimeout = true;
  nextTimeoutHT = millis() + 5000;
  
  // Agregar temperatura al buffer
  agregarTemperaturaAlBuffer(t);
  
  String mensajeCompleto = "";
  
  if (calculoEnSatelite) {
    // MODO SATÉLITE - Calcular y enviar media
    float mediaTemp = calcularMediaTemperatura();
    actualizarSistemaAlarma(mediaTemp);
    
    // Construir mensaje completo con todos los datos
    mensajeCompleto = "T:" + String(t, 1) + ":H:" + String(h, 1) + ":M:" + String(mediaTemp, 1) + 
                     ":A:" + String(angulo) + ":D:" + String(distanciaActual);
    
  } else {
    // MODO TIERRA - Enviar solo temperatura, humedad y datos del radar
    mensajeCompleto = "T:" + String(t, 1) + ":H:" + String(h, 1) + 
                     ":A:" + String(angulo) + ":D:" + String(distanciaActual);
  }
  
  // Enviar mensaje completo por ambos puertos seriales
  Serial.println(mensajeCompleto);
  mySerial.println(mensajeCompleto);
  
  
  Serial.print("Ángulo: ");
  Serial.print(angulo);
  Serial.print("° | Distancia: ");
  if (distanciaActual == -1) {
    Serial.println("Fuera de rango o error");
  } else {
    Serial.print(distanciaActual);
    Serial.println(" cm\n");
  }
  
}

void leerYProcesarSensores() {
  unsigned long now = millis();
  if (!enviarDatos || (now - lastSendMillis < sendInterval)) return;
  
  lastSendMillis = now;
  enviarDatosCompletos();
}

// CONFIGURACIÓN INICIAL
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  dht.begin();
  
  // Inicializar buffers
  for (int i = 0; i < 10; i++) tempBuffer[i] = 0.0;
  for (int i = 0; i < 3; i++) ultimasMedias[i] = 0.0;
  
  inicializarRadar();
  
  Serial.println("Sistema de satélite listo");
  Serial.println("Radar iniciado - barrido 0° a 180°");
  delay(1000);
}

// LOOP PRINCIPAL
void loop() {
  procesarEntradaSerial();
  manejarTimeoutDHT();
  leerYProcesarSensores();
  actualizarRadar();
}
