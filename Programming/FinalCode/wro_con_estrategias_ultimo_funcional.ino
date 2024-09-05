#include <Servo.h> // Biblioteca para el control del servomotor

// Definición de pines
#define DIP1 5
#define DIP2 6
#define DIP3 7
#define PULSO1 10

// Pines de sensores
int RightSensor = A2;   // Pin del sensor de obstáculo derecho
int MiddleSensor = 4;  // Pin del sensor de obstáculo medio
int LeftSensor = A5;    // Pin del sensor de obstáculo izquierdo

// Pines del sensor ultrasónico
int trigPin = 1;  // Pin TRIG del sensor ultrasónico
int echoPin = 0;  // Pin ECHO del sensor ultrasónico

// Pines de control
int Led1 = 8;           // Pin del LED 1
int motorPWMPin = 11;   // Pin PWM para control de velocidad del motor
int motorDirPin = 13;   // Pin de dirección del motor

Servo servoMotor;       // Objeto servo para controlar la dirección del robot
int servoPin = 2;       // Pin del servomotor
int currentAngle = 90;  // Ángulo actual del servomotor (inicialmente en 90 grados - hacia adelante)

int baseSpeed = 40;    // Velocidad base del motor (0 a 255 para control PWM)

// Variables de estado
bool started = false; // Variable para verificar si el robot ha sido iniciado

// Prototipos de funciones
int measureDistance();
void smoothServoMove(int targetAngle);
void adjustMotorSpeed(int speed);
int readDiswitch();
void estrategiaIzquierda(int frontDistance, int rightSensorValue, int leftSensorValue);
void estrategiaDerecha(int frontDistance, int rightSensorValue, int leftSensorValue);

void setup() {
  pinMode(Led1, OUTPUT);
  pinMode(PULSO1, INPUT_PULLUP);

  // Configurar pines del diswitch como entradas
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);
  pinMode(DIP3, INPUT_PULLUP);

  pinMode(motorPWMPin, OUTPUT); // Configurar el pin PWM del motor como salida
  pinMode(motorDirPin, OUTPUT); // Configurar el pin de dirección del motor como salida

  servoMotor.attach(servoPin);  // Adjuntar el servo al pin especificado
  servoMotor.write(currentAngle); // Posición inicial del servo

  // Configuración del sensor ultrasónico
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600); // Inicializar comunicación serie
}

void loop() {
  // Detectar si el botón de inicio ha sido presionado
  if (digitalRead(PULSO1) == LOW) { // Botón presionado
    if (!started) {
      // Si el robot no ha sido iniciado aún
      started = true;
      Serial.println("Robot iniciado");
    }
  }

  if (started) {
    int diswitchValue = readDiswitch();
    int frontDistance = measureDistance();
    int rightSensorValue = analogRead(RightSensor);
    int leftSensorValue = analogRead(LeftSensor);

    // Imprimir los valores
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
    Serial.print("Right Sensor Value: ");
    Serial.println(rightSensorValue);
    Serial.print("Left Sensor Value: ");
    Serial.println(leftSensorValue);
    Serial.print("Diswitch Value: ");
    Serial.println(diswitchValue, BIN); // Imprimir el valor del diswitch en binario

    // Control de dirección basado en el diswitch
    switch (diswitchValue) {
      case 0b001: // Estrategia izquierda
        estrategiaIzquierda(frontDistance, rightSensorValue, leftSensorValue);
        break;

      case 0b011: // Estrategia derecha
        estrategiaDerecha(frontDistance, rightSensorValue, leftSensorValue);
        break;

      default:
        // Comportamiento por defecto si no se activa ninguna estrategia
        if (frontDistance < 65) {  // Objeto cercano en el centro o frente
          analogWrite(motorPWMPin, 0);  // STOP
          delay(100);
          digitalWrite(motorDirPin, LOW);  // Retroceder
          analogWrite(motorPWMPin, 60);  // Retroceder a la velocidad base
          delay(2000);
          digitalWrite(motorDirPin, HIGH);  // Restablecer la dirección del motor para avanzar
          smoothServoMove(60); 
          delay(30);
          adjustMotorSpeed(40);  // Avanzar a velocidad base
          delay(1000);
        } else if (rightSensorValue > 600) {  // Objeto cercano a la derecha
          smoothServoMove(85);  // Girar suavemente a la izquierda después de retroceder
          adjustMotorSpeed(30);  // Avanzar a velocidad base
        } else if (leftSensorValue > 600) {  // Objeto cercano a la izquierda
          smoothServoMove(95); // Girar suavemente a la derecha después de retroceder
          adjustMotorSpeed(30);  // Avanzar a velocidad base
        } else {  // No hay objetos cercanos
          smoothServoMove(88);  // Mantener servo recto
          adjustMotorSpeed(40);  // Avanzar a velocidad base
        }
        break;
    }

    delay(60); // Pequeño retraso para evitar lecturas excesivas
  } else {
    // Detener el robot si no ha sido iniciado
    smoothServoMove(90);  // Mantener servo recto
    adjustMotorSpeed(0);  // Detener el motor
  }
}

// Función para mover el servomotor suavemente
void smoothServoMove(int targetAngle) {
  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      servoMotor.write(angle);
      delay(3); // Ajuste del delay para suavidad
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      servoMotor.write(angle);
      delay(3); // Ajuste del delay para suavidad
    }
  }
  currentAngle = targetAngle; // Actualizar el ángulo actual
}

// Función para ajustar la velocidad del motor
void adjustMotorSpeed(int speed) {
  analogWrite(motorPWMPin, speed); // Configurar la velocidad del motor con control PWM
}

// Función para medir la distancia utilizando el sensor ultrasónico
int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.054 / 2; // Convertir el tiempo en distancia
  return distance;
}

// Función para leer el estado del diswitch
int readDiswitch() {
  int value = 0;
  value |= (digitalRead(DIP1) == LOW) ? (1 << 0) : 0;
  value |= (digitalRead(DIP2) == LOW) ? (1 << 1) : 0;
  value |= (digitalRead(DIP3) == LOW) ? (1 << 2) : 0;
  return value;
}

// Estrategia para girar a la izquierda
void estrategiaIzquierda(int frontDistance, int rightSensorValue, int leftSensorValue) {
  if (frontDistance < 65) {  // Objeto cercano en el centro o frente
    analogWrite(motorPWMPin, 0);  // STOP
    delay(100);
    digitalWrite(motorDirPin, LOW);  // Retroceder
    analogWrite(motorPWMPin, 60);  // Retroceder a la velocidad base
    delay(2000);
    digitalWrite(motorDirPin, HIGH);  // Restablecer la dirección del motor para avanzar
    smoothServoMove(60); 
    delay(30);
    adjustMotorSpeed(40);  // Avanzar a velocidad base
    delay(1000);
  } else if (rightSensorValue > 600) {  // Objeto cercano a la derecha
    smoothServoMove(85);  // Girar suavemente a la izquierda después de retroceder
    adjustMotorSpeed(30);  // Avanzar a velocidad base
  } else if (leftSensorValue > 600) {  // Objeto cercano a la izquierda
    smoothServoMove(95); // Girar suavemente a la derecha después de retroceder
    adjustMotorSpeed(30);  // Avanzar a velocidad base
  } else {  // No hay objetos cercanos
    smoothServoMove(88);  // Mantener servo recto
    adjustMotorSpeed(40);  // Avanzar a velocidad base
  }
}

// Estrategia para girar a la derecha
void estrategiaDerecha(int frontDistance, int rightSensorValue, int leftSensorValue) {
  if (frontDistance < 65) {  // Objeto cercano en el centro o frente
    analogWrite(motorPWMPin, 0);  // STOP
    delay(100);
    digitalWrite(motorDirPin, LOW);  // Retroceder
    analogWrite(motorPWMPin, 60);  // Retroceder a la velocidad base
    delay(2000);
    digitalWrite(motorDirPin, HIGH);  // Restablecer la dirección del motor para avanzar
    smoothServoMove(120); 
    delay(30);
    adjustMotorSpeed(40);  // Avanzar a velocidad base
    delay(1000);
  } else if (rightSensorValue > 600) {  // Objeto cercano a la derecha
    smoothServoMove(85);  // Girar suavemente a la izquierda después de retroceder
    adjustMotorSpeed(30);  // Avanzar a velocidad base
  } else if (leftSensorValue > 600) {  // Objeto cercano a la izquierda
    smoothServoMove(95); // Girar suavemente a la derecha después de retroceder
    adjustMotorSpeed(30);  // Avanzar a velocidad base
  } else {  // No hay objetos cercanos
    smoothServoMove(88);  // Mantener servo recto
    adjustMotorSpeed(40);  // Avanzar a velocidad base
  }
}
