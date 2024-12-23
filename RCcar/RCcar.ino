#include <WiFi.h>
#include <MQTT.h>
#include "esp_camera.h"

// Camera pin definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define MQTT_BROKER_IP "192.168.137.24" // Your PC's IP address
#define MQTT_BROKER_PORT 1883
#define ESP32CAM_PUBLISH_TOPIC   "esp32/cam_0"
#define ESP32CAM_COMMAND_TOPIC   "esp32/cam_commands"

const char *ssid = "*";       // Your WiFi SSID
const char *password = "*";  // Your WiFi Password

// Motor 1 pin definitions
const int ENA1 = 4;   // PWM pin for Motor 1
const int IN1_1 = 2;  // Direction pin 1 for Motor 1
const int IN2_1 = 14; // Direction pin 2 for Motor 1

// Motor 2 pin definitions
const int ENA2 = 15;  // PWM pin for Motor 2
const int IN1_2 = 13; // Direction pin 1 for Motor 2
const int IN2_2 = 12; // Direction pin 2 for Motor 2

// PWM configuration
const int frequency = 500;           // PWM frequency in Hz
const int resolutionPWM = 8;         // PWM resolution in bits

// MQTT and WiFi clients
WiFiClient net;
MQTTClient client;

// Define motor commands
enum Command { STOP, UP, DOWN, LEFT, RIGHT };

// FreeRTOS queue handle for motor commands
QueueHandle_t motorCommandQueue;

// Forward declarations of functions
void connectMQTT();
void onMQTTMessage(String &topic, String &payload);
void cameraInit();
void grabImage();
void motorTask(void *parameter);
void mqttCameraTask(void *parameter);

// Function to connect to MQTT broker
void connectMQTT()
{
  Serial.print("Connecting to MQTT broker at ");
  Serial.print(MQTT_BROKER_IP);
  Serial.print(":");
  Serial.println(MQTT_BROKER_PORT);

  client.begin(MQTT_BROKER_IP, MQTT_BROKER_PORT, net);

  while (!client.connect("ESP32CAM_Client")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConnected to MQTT broker");
  client.subscribe(ESP32CAM_COMMAND_TOPIC);
}

// Callback function for MQTT messages
void onMQTTMessage(String &topic, String &payload) {
  if (topic == ESP32CAM_COMMAND_TOPIC) {
    Serial.print("Received command: ");
    Serial.println(payload);

    Command cmd = STOP; // Default to STOP

    if (payload.equalsIgnoreCase("UP")) {
      cmd = UP;
    }
    else if (payload.equalsIgnoreCase("DOWN")) {
      cmd = DOWN;
    }
    else if (payload.equalsIgnoreCase("LEFT")) {
      cmd = LEFT;
    }
    else if (payload.equalsIgnoreCase("RIGHT")) {
      cmd = RIGHT;
    }
    else if (payload.equalsIgnoreCase("STOP")) {
      cmd = STOP;
    }
    else {
      Serial.println("Unknown command received.");
      return;
    }

    // Send the command to the motor command queue
    if (xQueueSend(motorCommandQueue, &cmd, (TickType_t)10) != pdPASS) {
      Serial.println("Failed to send command to queue.");
    }
  }
}

// Function to initialize the camera
void cameraInit(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 20;
  config.fb_count = 2;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);       // -2 to 2
    s->set_saturation(s, 2);       // -2 to 2
    s->set_exposure_ctrl(s, 1);    // enable auto-exposure
    s->set_ae_level(s, 2);         // -2 to +2
  }
  else {
    Serial.println("Failed to get sensor_t object.");
  }
}

// Function to grab and publish an image
void grabImage(){
  camera_fb_t * fb = esp_camera_fb_get();
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < 1000000){
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publish Image: ");
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char*)fb->buf, fb->len);
    Serial.println(result ? "Success" : "Fail");

    if(!result){
      Serial.println("Publish failed, reconnecting...");
      client.disconnect();
      connectMQTT();
    }
  }
  esp_camera_fb_return(fb);
  // No delay here as this is handled in the task
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
  int dutyCycle = constrain(speed, 0, 255); 
  if (motor == 1) {
    ledcWrite(ENA1, dutyCycle);
  } else if (motor == 2) {
    ledcWrite(ENA2, dutyCycle);
  }
}

// Motor control task
void motorTask(void *parameter){
  Command currentCmd = STOP;

  while (true) {
    // Check if a new command is available
    if (xQueueReceive(motorCommandQueue, &currentCmd, (TickType_t)0) == pdPASS) {
      switch(currentCmd){
        case UP:
          // Move forward: Both motors forward at full speed
          setMotorDirection(1, true);
          setMotorDirection(2, true);
          setMotorSpeed(1, 255);
          setMotorSpeed(2, 255);
          Serial.println("Action: MOVE FORWARD");
          break;
        case DOWN:
          // Move backward: Both motors backward at full speed
          setMotorDirection(1, false);
          setMotorDirection(2, false);
          setMotorSpeed(1, 255);
          setMotorSpeed(2, 255);
          Serial.println("Action: MOVE BACKWARD");
          break;
        case LEFT:
          // Turn left: Motor 1 backward, Motor 2 forward
          setMotorDirection(1, false);
          setMotorDirection(2, true);
          setMotorSpeed(1, 255);
          setMotorSpeed(2, 255);
          Serial.println("Action: TURN LEFT");
          break;
        case RIGHT:
          // Turn right: Motor 1 forward, Motor 2 backward
          setMotorDirection(1, true);
          setMotorDirection(2, false);
          setMotorSpeed(1, 255);
          setMotorSpeed(2, 255);
          Serial.println("Action: TURN RIGHT");
          break;
        case STOP:
          // Stop both motors
          setMotorSpeed(1, 0);
          setMotorSpeed(2, 0);
          Serial.println("Action: STOP");
          break;
        default:
          // Stop both motors
          setMotorSpeed(1, 0);
          setMotorSpeed(2, 0);
          Serial.println("Action: STOP");
          break;
      }
    }

    // Optional: Add a short delay to prevent task hogging
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// MQTT and Camera handling task
void mqttCameraTask(void *parameter){
  while (true) {
    if (!client.connected()) {
      connectMQTT();
    }
    client.loop();
    grabImage();
    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust as needed
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize motor direction pins as outputs
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);

  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);

  // Attach PWM to Motor 1 enable pin
  if (!ledcAttach(ENA1, frequency, resolutionPWM)) { // Corrected to ledcAttachPin
    Serial.println("Error attaching LEDC to Motor 1 ENA pin");
    while (true); // Halt execution
  }

  // Attach PWM to Motor 2 enable pin
  if (!ledcAttach(ENA2, frequency, resolutionPWM)) { // Corrected to ledcAttachPin
    Serial.println("Error attaching LEDC to Motor 2 ENA pin");
    while (true); // Halt execution
  }

  cameraInit();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  // Display the ESP32's IP address
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize MQTT
  client.onMessage(onMQTTMessage);
  connectMQTT();

  // Create the motor command queue
  motorCommandQueue = xQueueCreate(10, sizeof(Command));
  if (motorCommandQueue == NULL) {
    Serial.println("Failed to create motor command queue.");
    while (true); // Halt execution
  }

  // Create FreeRTOS tasks
  xTaskCreate(
    motorTask,          // Task function
    "Motor Control",    // Task name
    2048,               // Stack size
    NULL,               // Task input parameter
    1,                  // Priority
    NULL                // Task handle
  );

  xTaskCreate(
    mqttCameraTask,     // Task function
    "MQTT and Camera",  // Task name
    8192,               // Stack size (increased for camera handling)
    NULL,               // Task input parameter
    1,                  // Priority
    NULL                // Task handle
  );
}

void loop() {
  // Empty loop as tasks handle everything
}
