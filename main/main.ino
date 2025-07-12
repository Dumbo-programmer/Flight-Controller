// Motor Pins
#define ENA 14  // D5 - Enable pin for Motor A (Right Side)
#define IN1 4   // D2 - Motor A Input 1
#define IN2 0   // D3 - Motor A Input 2
#define ENB 5   // D1 - Enable pin for Motor B (Left Side)
#define IN3 2   // D4 - Motor B Input 1
#define IN4 12  // D6 - Motor B Input 2

void setup() {
  Serial.begin(9600);  // Use Hardware Serial for HC-05

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  stopMotors();  
  Serial.println("Bluetooth Ready!");
}

void loop() {
  if (Serial.available()) {  // Check for Bluetooth data
    char command = Serial.read();
    Serial.print("Received: "); Serial.println(command);

    if (command == 'F') moveForward();
    else if (command == 'B') moveBackward();
    else if (command == 'L') turnLeft();
    else if (command == 'R') turnRight();
    else if (command == 'S') stopMotors();
    else if (command == 'G') moveForwardLeft();
    else if (command == 'I') moveForwardRight();
    else if (command == 'H') moveBackwardLeft();
    else if (command == 'J') moveBackwardRight();
  }
}

// 🚀 Basic Movements
void moveForward() {
  Serial.println("Moving Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  Serial.println("Moving Backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  Serial.println("Turning Left");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  Serial.println("Turning Right");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  Serial.println("Stopping Motors");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// 🚀 Diagonal Movements
void moveForwardLeft() {
  Serial.println("Moving Forward Left");
  digitalWrite(IN1, LOW);  // Right motor stops
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Left motor moves forward
  digitalWrite(IN4, LOW);
}

void moveForwardRight() {
  Serial.println("Moving Forward Right");
  digitalWrite(IN1, HIGH); // Right motor moves forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Left motor stops
  digitalWrite(IN4, LOW);
}

void moveBackwardLeft() {
  Serial.println("Moving Backward Left");
  digitalWrite(IN1, LOW);  // Right motor stops
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // Left motor moves backward
}

void moveBackwardRight() {
  Serial.println("Moving Backward Right");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // Right motor moves backward
  digitalWrite(IN3, LOW);  // Left motor stops
  digitalWrite(IN4, LOW);
}
