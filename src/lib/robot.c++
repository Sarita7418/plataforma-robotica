#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --------------------------------------------------------
// 1. CONFIGURACIÓN DE PINES (Mauricio debe ajustar estos)
// --------------------------------------------------------
// Motor Izquierdo (A)
const int pinMotorA_IN1 = 14; 
const int pinMotorA_IN2 = 27;
const int pinMotorA_PWM = 26; // Pin de velocidad

// Motor Derecho (B)
const int pinMotorB_IN3 = 25;
const int pinMotorB_IN4 = 33;
const int pinMotorB_PWM = 32; // Pin de velocidad

// --------------------------------------------------------
// 2. CONSTANTES PID Y CONTROL
// --------------------------------------------------------
// El objetivo de este PID es mantener el robot yendo en línea recta
// compensando si un motor gira más rápido que el otro por fricción.
float Kp = 2.0; 
float Ki = 0.05;
float Kd = 0.5;

float error = 0, errorAnterior = 0, integral = 0, salidaPID = 0;
unsigned long tiempoAnterior = 0;

int velocidadObjetivo = 0; // Se actualiza por Bluetooth
String estadoActual = "STOP"; // STOP, AVANZANDO, ERROR_DESCONTROL

// --------------------------------------------------------
// 3. CONFIGURACIÓN BLUETOOTH (BLE)
// --------------------------------------------------------
// UUIDs generados para este proyecto
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool dispositivoConectado = false;

// Clase para manejar cuando el estudiante se conecta o desconecta desde la web
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      dispositivoConectado = true;
      Serial.println("💻 Estudiante conectado vía Web Bluetooth.");
    }
    void onDisconnect(BLEServer* pServer) {
      dispositivoConectado = false;
      Serial.println("❌ Estudiante desconectado.");
      estadoActual = "STOP"; // Parada de emergencia
    }
};

// Clase para recibir comandos desde Astro (Ej: "AVANZAR_80")
class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String valorRecibido = pCharacteristic->getValue().c_str();
      
      if (valorRecibido.length() > 0) {
        Serial.print("Comando recibido desde la web: ");
        Serial.println(valorRecibido);

        // Parseo del comando
        if (valorRecibido == "ERROR_GIRO") {
          estadoActual = "ERROR_DESCONTROL";
        } 
        else if (valorRecibido.startsWith("AVANZAR_")) {
          // Extraemos el número del string "AVANZAR_80" -> 80
          String velString = valorRecibido.substring(8);
          velocidadObjetivo = velString.toInt();
          estadoActual = "AVANZANDO";
        }
        else if (valorRecibido == "STOP") {
          estadoActual = "STOP";
        }
      }
    }
};

// --------------------------------------------------------
// 4. FUNCIONES DE HARDWARE (MOTORES)
// --------------------------------------------------------
void detenerMotores() {
  ledcWrite(0, 0); // Canal 0, Velocidad 0
  ledcWrite(1, 0); // Canal 1, Velocidad 0
  digitalWrite(pinMotorA_IN1, LOW);
  digitalWrite(pinMotorA_IN2, LOW);
  digitalWrite(pinMotorB_IN3, LOW);
  digitalWrite(pinMotorB_IN4, LOW);
}

void comportamientoError() {
  // El estudiante mandó un valor fuera de rango. El robot gira a lo loco.
  digitalWrite(pinMotorA_IN1, HIGH);
  digitalWrite(pinMotorA_IN2, LOW);
  digitalWrite(pinMotorB_IN3, LOW);
  digitalWrite(pinMotorB_IN4, HIGH);
  
  ledcWrite(0, 255); // Motor izquierdo a tope hacia adelante
  ledcWrite(1, 255); // Motor derecho a tope hacia atrás
}

// --------------------------------------------------------
// 5. INICIALIZACIÓN
// --------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Configurar pines de motores como salida
  pinMode(pinMotorA_IN1, OUTPUT); pinMode(pinMotorA_IN2, OUTPUT);
  pinMode(pinMotorB_IN3, OUTPUT); pinMode(pinMotorB_IN4, OUTPUT);

  // Configurar PWM para el ESP32 (Canal 0 y 1, 5kHz, 8 bits de resolución)
  ledcSetup(0, 5000, 8); ledcAttachPin(pinMotorA_PWM, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(pinMotorB_PWM, 1);

  // Inicializar Bluetooth
  BLEDevice::init("RoboLogic_Rover"); // Este nombre aparecerá en la página web
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->setCallbacks(new CommandCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("🤖 Sistema Rover iniciado. Esperando conexión BLE...");
}

// --------------------------------------------------------
// 6. BUCLE PRINCIPAL Y PID
// --------------------------------------------------------
void loop() {
  if (!dispositivoConectado) {
    detenerMotores();
    delay(500);
    return;
  }

  // Máquina de estados dictada por la plataforma web
  if (estadoActual == "STOP") {
    detenerMotores();
  } 
  else if (estadoActual == "ERROR_DESCONTROL") {
    comportamientoError();
  } 
  else if (estadoActual == "AVANZANDO") {
    
    // Aquí entra el PID. Para que la línea recta sea perfecta, 
    // calculamos la compensación.
    unsigned long tiempoActual = millis();
    float dt = (tiempoActual - tiempoAnterior) / 1000.0;
    
    // (En una versión final con encoders, aquí leeríamos la diferencia entre ruedas)
    // Para esta Beta, simulamos el error. 
    float errorSimulado = 0; // Si el robot se desvía, esto sería distinto de 0
    
    // Cálculos PID
    float P = Kp * errorSimulado;
    integral += errorSimulado * dt;
    float I = Ki * integral;
    float D = Kd * ((errorSimulado - errorAnterior) / dt);
    salidaPID = P + I + D;
    
    errorAnterior = errorSimulado;
    tiempoAnterior = tiempoActual;

    // Aplicar la velocidad base dictada por el código del estudiante en la web,
    // pero sumando/restando el PID para mantener el robot derecho.
    int pwmIzquierdo = velocidadObjetivo - salidaPID;
    int pwmDerecho = velocidadObjetivo + salidaPID;

    // Constreñir valores para no quemar motores
    pwmIzquierdo = constrain(pwmIzquierdo, 0, 255);
    pwmDerecho = constrain(pwmDerecho, 0, 255);

    // Activar motores hacia adelante con la potencia corregida
    digitalWrite(pinMotorA_IN1, HIGH); digitalWrite(pinMotorA_IN2, LOW);
    digitalWrite(pinMotorB_IN3, HIGH); digitalWrite(pinMotorB_IN4, LOW);
    
    ledcWrite(0, pwmIzquierdo);
    ledcWrite(1, pwmDerecho);
  }

  // Pequeño delay para no saturar el procesador
  delay(10); 
}