/*
 * Ejemplo de uso de la biblioteca PIDController para control de péndulo invertido
 * 
 * Este ejemplo demuestra cómo usar los tres tipos de controladores PID
 * implementados en la biblioteca para controlar un péndulo invertido.
 */

#include "PIDController.h"

// Pines del ESP32
const int ENCODER_PIN_A = 2;
const int ENCODER_PIN_B = 3;
const int MOTOR_PWM_PIN = 9;
const int MOTOR_DIR_PIN = 8;
const int ACCEL_SDA = 21;
const int ACCEL_SCL = 22;

// Variables del sistema
volatile long encoder_position = 0;
double angle = 0.0;  // Ángulo del péndulo (grados)
double position = 0.0;  // Posición del carro (mm)
double setpoint_angle = 0.0;  // Referencia de ángulo (péndulo vertical)
double setpoint_position = 0.0;  // Referencia de posición del carro

// Controladores PID
PIDController pidAngle(50.0, 0.1, 5.0, 0.01, PID_FILTERED, 5);  // Control de ángulo
PIDController pidPosition(2.0, 0.05, 0.1, 0.01, PID_BASIC);      // Control de posición

// Variables de tiempo
unsigned long lastTime = 0;
const unsigned long sampleTime = 10;  // 10ms = 100Hz

// Modo de operación
enum ControlMode {
    BALANCE_ONLY,     // Solo balancear el péndulo
    BALANCE_POSITION  // Balancear y controlar posición
};
ControlMode currentMode = BALANCE_ONLY;

void setup() {
    Serial.begin(115200);
    Serial.println("=== Control PID Péndulo Invertido ===");
    
    // Configurar pines
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    
    // Configurar interrupciones del encoder
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR, CHANGE);
    
    // Inicializar controladores PID
    pidAngle.begin(0.0);
    pidPosition.begin(0.0);
    
    // Configurar I2C para acelerómetro (simulado en este ejemplo)
    // Wire.begin(ACCEL_SDA, ACCEL_SCL);
    
    Serial.println("Sistema inicializado. Comandos disponibles:");
    Serial.println("1 - Modo solo balance");
    Serial.println("2 - Modo balance + posición");
    Serial.println("r - Reset controladores");
    Serial.println("p [kp] [ki] [kd] - Ajustar PID ángulo");
    Serial.println("pos [kp] [ki] [kd] - Ajustar PID posición");
    Serial.println("set [pos] - Cambiar setpoint posición");
    
    lastTime = millis();
}

void loop() {
    unsigned long now = millis();
    
    // Control a frecuencia fija
    if(now - lastTime >= sampleTime) {
        // Leer sensores
        readSensors();
        
        // Ejecutar control PID
        controlLoop();
        
        // Enviar comando al motor
        updateMotor();
        
        // Mostrar datos cada 100ms
        static unsigned long printTime = 0;
        if(now - printTime >= 100) {
            printStatus();
            printTime = now;
        }
        
        lastTime = now;
    }
    
    // Procesar comandos serie
    processSerialCommands();
}

void readSensors() {
    // Leer posición del encoder (simulado)
    // En implementación real, convertir pulsos a mm
    position = encoder_position * 0.1;  // Ejemplo: 0.1mm por pulso
    
    // Leer ángulo del acelerómetro/giroscopio (simulado)
    // En implementación real, usar IMU para obtener ángulo
    angle = readIMUAngle();  // Función que implementarías según tu IMU
}

double readIMUAngle() {
    // SIMULACIÓN - En implementación real, leer IMU
    // Retorna ángulo en grados (-180 a +180)
    // 0 grados = péndulo vertical hacia arriba
    
    // Para pruebas, generar un ángulo con algo de ruido
    static double simAngle = 0.0;
    simAngle += (random(-100, 100) / 1000.0);  // Pequeña deriva
    simAngle = constrain(simAngle, -30, 30);   // Limitar rango
    
    return simAngle;
}

void controlLoop() {
    double motorOutput = 0.0;
    
    switch(currentMode) {
        case BALANCE_ONLY:
            // Solo control de ángulo
            motorOutput = pidAngle.compute(setpoint_angle, angle);
            break;
            
        case BALANCE_POSITION:
            // Control cascada: posición -> ángulo -> motor
            double angleReference = pidPosition.compute(setpoint_position, position);
            angleReference = constrain(angleReference, -15.0, 15.0);  // Limitar referencia
            motorOutput = pidAngle.compute(angleReference, angle);
            break;
    }
    
    // Limitar salida del motor
    motorOutput = constrain(motorOutput, -255, 255);
    
    // Aplicar zona muerta para evitar ruido
    if(abs(motorOutput) < 5) {
        motorOutput = 0;
    }
    
    // Guardar para envío al motor
    setMotorOutput(motorOutput);
}

void setMotorOutput(double output) {
    // Determinar dirección
    if(output >= 0) {
        digitalWrite(MOTOR_DIR_PIN, HIGH);
    } else {
        digitalWrite(MOTOR_DIR_PIN, LOW);
        output = -output;
    }
    
    // Aplicar PWM
    analogWrite(MOTOR_PWM_PIN, (int)output);
}

void updateMotor() {
    // Esta función se llama desde controlLoop()
    // La salida ya fue aplicada en setMotorOutput()
}

void encoderISR() {
    // Lectura del encoder en cuadratura
    static int lastA = HIGH;
    static int lastB = HIGH;
    
    int currentA = digitalRead(ENCODER_PIN_A);
    int currentB = digitalRead(ENCODER_PIN_B);
    
    if(lastA == LOW && currentA == HIGH) {
        if(currentB == LOW) {
            encoder_position++;
        } else {
            encoder_position--;
        }
    }
    
    lastA = currentA;
    lastB = currentB;
}

void printStatus() {
    Serial.print("Modo: ");
    Serial.print(currentMode == BALANCE_ONLY ? "BALANCE" : "BAL+POS");
    Serial.print(" | Ang: ");
    Serial.print(angle, 2);
    Serial.print("° | Pos: ");
    Serial.print(position, 1);
    Serial.print("mm | PID_Ang: ");
    Serial.print(pidAngle.getOutput(), 1);
    Serial.print(" | PID_Pos: ");
    Serial.println(pidPosition.getOutput(), 1);
}

void processSerialCommands() {
    if(!Serial.available()) return;
    
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command == "1") {
        currentMode = BALANCE_ONLY;
        Serial.println("Modo: Solo balance");
    }
    else if(command == "2") {
        currentMode = BALANCE_POSITION;
        Serial.println("Modo: Balance + posición");
    }
    else if(command == "r") {
        pidAngle.reset();
        pidPosition.reset();
        Serial.println("Controladores reiniciados");
    }
    else if(command.startsWith("p ")) {
        // Ajustar PID de ángulo: p kp ki kd
        float kp, ki, kd;
        if(sscanf(command.c_str(), "p %f %f %f", &kp, &ki, &kd) == 3) {
            pidAngle.setTunings(kp, ki, kd);
            Serial.printf("PID Ángulo: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
        }
    }
    else if(command.startsWith("pos ")) {
        // Ajustar PID de posición: pos kp ki kd
        float kp, ki, kd;
        if(sscanf(command.c_str(), "pos %f %f %f", &kp, &ki, &kd) == 3) {
            pidPosition.setTunings(kp, ki, kd);
            Serial.printf("PID Posición: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
        }
    }
    else if(command.startsWith("set ")) {
        // Cambiar setpoint de posición: set posicion
        float pos;
        if(sscanf(command.c_str(), "set %f", &pos) == 1) {
            setpoint_position = pos;
            Serial.printf("Setpoint posición: %.1f mm\n", pos);
        }
    }
}