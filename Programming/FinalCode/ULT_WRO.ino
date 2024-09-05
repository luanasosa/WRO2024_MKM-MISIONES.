#include <Servo.h> // Biblioteca para el control del servomotor

// Pines de sensores
int RightSensor = A2;   // Pin del sensor de obstáculo derecho
int MiddleSensor = 4;  // Pin del sensor de obstáculo medio
int LeftSensor = A5;    // Pin del sensor de obstáculo izquierdo

// Pines del sensor ultrasónico
int trigPin = 1;  // Pin TRIG del sensor ultrasónico
int echoPin = 0;  // Pin ECHO del sensor ultrasónico

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

  // Configuración del sensor ultrasónico
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600); // Inicializar comunicación serie
}

void loop() {
  // Esperar a que se presione el botón de inicio
  while (digitalRead(Start) == 1) {
    if (analogRead(LeftSensor) < 600 || analogRead(MiddleSensor) < 600 || analogRead(RightSensor) < 600 || measureDistance() < 20) {
      digitalWrite(Led1, HIGH);
    } else {
      digitalWrite(Led1, LOW);
    }
    delay(10); // Parpadeo del LED 1 por 100ms
  }
  
  while (1) {
    // Leer los valores analógicos de los sensores
    int rightSensorValue = analogRead(RightSensor);
    int middleSensorValue = analogRead(MiddleSensor);
    int leftSensorValue = analogRead(LeftSensor);
    int frontDistance = measureDistance();

    // Imprimir los valores de los sensores
    Serial.print("Right Sensor: ");
    Serial.print(rightSensorValue);
    Serial.print(", Middle Sensor: ");
    Serial.print(middleSensorValue);
    Serial.print(", Left Sensor: ");
    Serial.print(leftSensorValue);
    Serial.print(", Front Distance: ");
    Serial.println(frontDistance);
    delay(1000);

    // Control de dirección basado en los valores analógicos de los sensores y el sensor ultrasónico
    if (middleSensorValue > 800 || frontDistance < 60) {  // Objeto cercano en el centro o frente
      digitalWrite(motorDirPin, LOW);  // Cambiar la dirección del motor para retroceder
      analogWrite(motorPWMPin, 110);  // Retroceder a la velocidad base
      delay(800);
      digitalWrite(motorDirPin, HIGH);  // Restablecer la dirección del motor para avanzar
      smoothServoMove(70); 
      
    } else if (rightSensorValue > 800) {  // Objeto cercano a la derecha
      smoothServoMove(70);  // Girar suavemente a la izquierda después de retroceder
      adjustMotorSpeed(baseSpeed);  // Avanzar a velocidad base
     
      
    } else if (leftSensorValue > 800) {  // Objeto cercano a la izquierda
      smoothServoMove(120); // Girar suavemente a la derecha después de retroceder
      adjustMotorSpeed(baseSpeed);  // Avanzar a velocidad base
      
      
    } else {  // No hay objetos cercanos
      smoothServoMove(90);  // Mantener servo recto
      adjustMotorSpeed(30);  // Avanzar a velocidad base
      
    }
    
    delay(1); // Pequeño retraso para evitar lecturas excesivas
  }
}

// Función para retroceder el robot
void reverseRobot() {
  digitalWrite(motorDirPin, LOW);  // Cambiar la dirección del motor para retroceder
  analogWrite(motorPWMPin, baseSpeed);  // Retroceder a la velocidad base
  digitalWrite(motorDirPin, HIGH);  // Restablecer la dirección del motor para avanzar
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
  digitalWrite(motorDirPin, HIGH); // Configurar la dirección del motor para avanzar
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
  int distance = duration * 0.034 / 2; // Convertir el tiempo en distancia
  return distance;
}
