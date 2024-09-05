#include <Servo.h> // Biblioteca para el control del servomotor

// Pines de sensores
int RightSensor = A2;   // Pin del sensor de obstáculo derecho
int MiddleSensor = A4;  // Pin del sensor de obstáculo medio
int LeftSensor = A5;    // Pin del sensor de obstáculo izquierdo

// Pines de control
int Start = 10;         // Pin del botón de inicio
int Led1 = 8;           // Pin del LED 1
int motorPWMPin = 11;   // Pin PWM para control de velocidad del motor
int motorDirPin = 13;   // Pin de dirección del motor

Servo servoMotor;       // Objeto servo para controlar la dirección del robot
int servoPin = 2;       // Pin del servomotor
int currentAngle = 90;  // Ángulo actual del servomotor (inicialmente en 90 grados - hacia adelante)

int baseSpeed = 30;    // Velocidad base del motor (0 a 255 para control PWM)

void setup() {
  pinMode(Led1, OUTPUT);
  pinMode(Start, INPUT);
  
  pinMode(motorPWMPin, OUTPUT); // Configurar el pin PWM del motor como salida
  pinMode(motorDirPin, OUTPUT); // Configurar el pin de dirección del motor como salida
  
  servoMotor.attach(servoPin);  // Adjuntar el servo al pin especificado
  servoMotor.write(currentAngle); // Posición inicial del servo

  Serial.begin(9600); // Inicializar comunicación serie
}

void loop() {
  // Esperar a que se presione el botón de inicio
  while (digitalRead(Start) == 0) {
    if (analogRead(LeftSensor) < 500 || analogRead(MiddleSensor) < 500 || analogRead(RightSensor) < 500) {
      digitalWrite(Led1, HIGH);
    } else {
      digitalWrite(Led1, LOW);
    }
    delay(100); // Parpadeo del LED 1 por 100ms
  }
  
  while (1) {
    // Leer los valores analógicos de los sensores
    int rightSensorValue = analogRead(RightSensor);
    int middleSensorValue = analogRead(MiddleSensor);
    int leftSensorValue = analogRead(LeftSensor);

    // Imprimir los valores de los sensores
    Serial.print("Right Sensor: ");
    Serial.print(rightSensorValue);
    Serial.print(", Middle Sensor: ");
    Serial.print(middleSensorValue);
    Serial.print(", Left Sensor: ");
    Serial.println(leftSensorValue);
    delay(1000);

    // Control de dirección basado en los valores analógicos de los sensores
    if (middleSensorValue < 300) {  // Objeto cercano en el centro
      smoothServoMove(90);  // Mantener servo recto
      adjustMotorSpeed(baseSpeed);  // Avanzar a velocidad base
    } else if (rightSensorValue < 300) {  // Objeto cercano a la derecha
      smoothServoMove(70);  // Girar suavemente a la izquierda
      adjustMotorSpeed(baseSpeed - map(rightSensorValue, 0, 1023, 0, baseSpeed));  // Reducir velocidad al acercarse al objeto
    } else if (leftSensorValue < 300) {  // Objeto cercano a la izquierda
      smoothServoMove(110); // Girar suavemente a la derecha
      adjustMotorSpeed(baseSpeed - map(leftSensorValue, 0, 1023, 0, baseSpeed));  // Reducir velocidad al acercarse al objeto
    } else {  // No hay objetos cercanos
      smoothServoMove(90);  // Mantener servo recto
      adjustMotorSpeed(baseSpeed);  // Avanzar a velocidad base
    }
    
    delay(10); // Pequeño retraso para evitar lecturas excesivas
  }
}

// Función para mover el servomotor suavemente
void smoothServoMove(int targetAngle) {
  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      servoMotor.write(angle);
      delay(15); // Ajuste del delay para suavidad
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      servoMotor.write(angle);
      delay(15); // Ajuste del delay para suavidad
    }
  }
  currentAngle = targetAngle; // Actualizar el ángulo actual
}

// Función para ajustar la velocidad del motor
void adjustMotorSpeed(int speed) {
  digitalWrite(motorDirPin, HIGH); // Configurar la dirección del motor
  analogWrite(motorPWMPin, speed); // Configurar la velocidad del motor con control PWM
}
