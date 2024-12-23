// Motor 1 pin definitions
int ENA1 = 4;   // PWM pin for Motor 1
int IN1_1 = 2;  // Direction pin 1 for Motor 1
int IN2_1 = 14; // Direction pin 2 for Motor 1

// Motor 2 pin definitions
int ENA2 = 15;  // PWM pin for Motor 2
int IN1_2 = 13; // Direction pin 1 for Motor 2
int IN2_2 = 12;  // Direction pin 2 for Motor 2

// PWM configuration
const int frequency = 500;    // PWM frequency in Hz
const int resolution = 8;     // PWM resolution in bits

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Set motor 1 direction pins as outputs
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);

  // Set motor 2 direction pins as outputs
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);

  // Attach PWM to Motor 1 enable pin
  if (!ledcAttach(ENA1, frequency, resolution)) {
    Serial.println("Error attaching LEDC to Motor 1 ENA pin");
    while (true); // Halt execution
  }

  // Attach PWM to Motor 2 enable pin
  if (!ledcAttach(ENA2, frequency, resolution)) {
    Serial.println("Error attaching LEDC to Motor 2 ENA pin");
    while (true); // Halt execution
  }
}

void loop() {
  // Example: Set Motor 1 to forward at full speed
  setMotorDirection(1, true);
  setMotorSpeed(1, 255);
  delay(5000);

  // Example: Set Motor 2 to backward at half speed
  setMotorDirection(2, false);
  setMotorSpeed(2, 128);
  delay(5000);

  // Stop both motors
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  delay(2000);
}

// Function to set motor direction
void setMotorDirection(int motor, bool forward) {
  if (motor == 1) {
    digitalWrite(IN1_1, forward ? HIGH : LOW);
    digitalWrite(IN2_1, forward ? LOW : HIGH);
  } else if (motor == 2) {
    digitalWrite(IN1_2, forward ? HIGH : LOW);
    digitalWrite(IN2_2, forward ? LOW : HIGH);
  }
}

// Function to set motor speed
void setMotorSpeed(int motor, int speed) {
  int dutyCycle = constrain(speed, 0, 255); // Ensure speed is within 0-255 range
  if (motor == 1) {
    ledcWrite(ENA1, dutyCycle);
  } else if (motor == 2) {
    ledcWrite(ENA2, dutyCycle);
  }
}