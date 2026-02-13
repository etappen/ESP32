// Simple ESP32 sketch with HTTP control interface + SN754410 Motor Driver
#define FRONTLAMPS 2
#define REARLAMPS 0
#define BUZZER_PIN 21

// SN754410 Motor Driver pins
#define LEFT_EN 25     // PWM enable (speed control)
#define LEFT_IN1 16    // Left motor input 1
#define LEFT_IN2 17    // Left motor input 2

#define RIGHT_EN 26    // PWM enable (speed control)
#define RIGHT_IN1 18   // Right motor input 1
#define RIGHT_IN2 19   // Right motor input 2

// Encoder pins
#define LEFT_ENC_A 34  // Left motor encoder A
#define LEFT_ENC_B 35  // Left motor encoder B
#define RIGHT_ENC_A 36 // Right motor encoder A
#define RIGHT_ENC_B 39 // Right motor encoder B

// Encoder counters
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

#include <WiFi.h>

// Buzzer PWM (LEDC) settings
const int BUZZER_CH = 0;
const int BUZZER_FREQ = 2000; // 2 kHz default tone
const int BUZZER_RES = 8;     // 8-bit resolution (0-255)

// Line-following sensors (analog)
#define SENSOR_LEFT_PIN 32
#define SENSOR_RIGHT_PIN 33

// Ultrasonic sensor pins
#define TRIG_PIN 4
#define ECHO_PIN 27
const int OBSTACLE_DISTANCE_CM = 5;  // Stop if object closer than this

// ADC and line-follow parameters
const int READ_INTERVAL_MS = 100;
const int AVG_SAMPLES = 6;
// Ultrasonic sampling
const int ULTRASONIC_SAMPLES = 3;
const bool ULTRASONIC_DEBUG = true;
int thresholdLeft = 2000;
int thresholdRight = 2000;
long sensorLeft = 0;
long sensorRight = 0;
bool lineFollowing = false;
unsigned long lastLineRead = 0;
long distanceCm = 0;

// Virtual Joystick variables (controlled via web interface)
int joystickX = 0;  // -100 to 100
int joystickY = 0;  // -100 to 100
bool joystickMode = false;  // Enable/disable joystick control

// Access Point settings
const char* ap_ssid = "Mater";
const char* ap_pass = "cars808";

WiFiServer server(80);

// Encoder interrupt handlers
void IRAM_ATTR handleLeftEncoderA() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void IRAM_ATTR handleRightEncoderA() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ESP32 LED & Horn Control with Motors ===");
  
  // Start Access Point (no external network connection needed)
  WiFi.mode(WIFI_AP); // AP only mode
  Serial.println("\nStarting Access Point...");
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("AP SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  Serial.println("\nWeb server started on port 80");
  // ADC setup for line sensors
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_RIGHT_PIN, ADC_11db);
  Serial.println("Line sensors initialized");
  
  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic sensor initialized");
  // Initialize buzzer PWM (LEDC)
  // Use simple digital pin for buzzer (manual tone generator used)
  // Motor control pins as outputs
  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Encoder pins as inputs
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoderA, CHANGE);
  Serial.println("Encoders initialized");

  pinMode(FRONTLAMPS, OUTPUT);
  pinMode(REARLAMPS, OUTPUT);
}

void sendHttpResponse(WiFiClient &client, const String &body) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println(body);
}

long readAverage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / samples;
}

// Read ultrasonic distance in cm
long readUltrasonicDistance() {
  long totalDistance = 0;
  int valid = 0;
  for (int s = 0; s < ULTRASONIC_SAMPLES; ++s) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
    if (ULTRASONIC_DEBUG) {
      Serial.print("Ultrasonic raw duration: "); Serial.println(duration);
    }
    if (duration > 0) {
      // distance = (duration in microseconds * speed of sound) / 2
      // speed of sound = 343 m/s = 0.0343 cm/microsecond
      long distance = (duration * 343) / 20000;  // ~ duration * 0.01715
      totalDistance += distance;
      valid++;
    }
    delay(10);
  }

  if (valid == 0) {
    if (ULTRASONIC_DEBUG) Serial.println("Ultrasonic: timeout / no echo");
    return 0;
  }

  long avg = totalDistance / valid;
  if (ULTRASONIC_DEBUG) {
    Serial.print("Ultrasonic distance (cm) avg: "); Serial.println(avg);
  }
  return avg;
}

// Simple line-follow decision: if left sensor sees line -> turn left;
// if right sees line -> turn right; if neither -> forward.
void runLineFollow() {
  // read sensors
  sensorLeft = readAverage(SENSOR_LEFT_PIN, AVG_SAMPLES);
  sensorRight = readAverage(SENSOR_RIGHT_PIN, AVG_SAMPLES);

  bool leftOnLine = sensorLeft < thresholdLeft;
  bool rightOnLine = sensorRight < thresholdRight;

  if (leftOnLine && !rightOnLine) {
    motorLeft(200);
  } else if (rightOnLine && !leftOnLine) {
    motorRight(200);
  } else if (leftOnLine && rightOnLine) {
    // Both sensors detect line — stop or move forward slowly
    motorForward(150);
  } else {
    // No sensor sees line — proceed forward
    motorForward(200);
  }
}

void BuzzerTest() {
  // Play two short tones for an audible beep using manual toggling
  Serial.println("Buzzer Test - ON");
  playTone(BUZZER_PIN, BUZZER_FREQ, 300);
  delay(150);
  playTone(BUZZER_PIN, BUZZER_FREQ, 300);
  Serial.println("Buzzer Test Complete");
}

// Simple blocking tone generator using digital toggling (works when LEDC not available)
void playTone(int pin, int freq, int duration_ms) {
  if (freq <= 0 || duration_ms <= 0) return;
  unsigned long period_us = 1000000UL / (unsigned long)freq;
  unsigned long half_us = period_us / 2;
  unsigned long end = millis() + (unsigned long)duration_ms;
  while (millis() < end) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(half_us);
    digitalWrite(pin, LOW);
    delayMicroseconds(half_us);
  }
}

void FrontLampTest(){
  Serial.println("Front Lamp Test");
  digitalWrite(FRONTLAMPS, HIGH);
  delay(1000);
  digitalWrite(FRONTLAMPS, LOW);
}

void RearLampTest(){
  Serial.println("Rear Lamp Test");
  digitalWrite(REARLAMPS, HIGH);
  delay(1000);
  digitalWrite(REARLAMPS, LOW);
}

// Motor control functions
void motorForward(uint8_t speed) {
  Serial.println("Motors Forward");

  // Left motor forward
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, HIGH);

  // Right motor forward
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, HIGH);
}

void motorBackward(uint8_t speed) {
  Serial.println("Motors Backward");

  // Left motor backward
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_EN, HIGH);

  // Right motor backward
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_EN, HIGH);
}

void motorStop() {
  Serial.println("Motors Stopped");
  
  // Left motor stop
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, LOW);
  
  // Right motor stop
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, LOW);
}

void motorLeft(uint8_t speed) {
  Serial.println("Turn Left");

  // Left motor stop, right motor forward
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, LOW);

  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, HIGH);
}

void motorRight(uint8_t speed) {
  Serial.println("Turn Right");

  // Left motor forward, right motor stop
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, HIGH);

  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, LOW);
}

// Joystick control function - maps joystick values to motor speeds
void handleJoystickControl(int joyX, int joyY) {
  // joyX and joyY are in range -100 to 100 from web interface
  // Calculate motor speeds based on differential drive
  // Forward/Backward: Y axis (positive = forward, negative = backward)
  // Left/Right: X axis (positive = turn right, negative = turn left)
  int leftMotorSpeed = joyY - joyX;
  int rightMotorSpeed = joyY + joyX;

  // Limit motor speeds to [-255, 255]
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Control motors based on speeds
  if (leftMotorSpeed == 0 && rightMotorSpeed == 0) {
    motorStop();
  } else {
    // Left motor
    if (leftMotorSpeed > 0) {
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
    } else if (leftMotorSpeed < 0) {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
    } else {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, LOW);
    }
    analogWrite(LEFT_EN, abs(leftMotorSpeed));

    // Right motor
    if (rightMotorSpeed > 0) {
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
    } else if (rightMotorSpeed < 0) {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
    } else {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, LOW);
    }
    analogWrite(RIGHT_EN, abs(rightMotorSpeed));
  }
}

void handleClient(WiFiClient client) {
  if (!client) return;

  String currentLine = "";
  String requestLine = "";

  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') {
        if (currentLine.length() == 0) {
          break; // end of headers
        }
        if (requestLine.length() == 0) requestLine = currentLine;
        currentLine = "";
      } else if (c != '\r') {
        currentLine += c;
      }
    }
  }

  String responseBody = "";

  if (requestLine.indexOf("GET /buzzer") == 0) {
    BuzzerTest();
    responseBody = "<html><body><h1>Buzzer triggered</h1></body></html>";
  } else if (requestLine.indexOf("GET /front") == 0) {
    FrontLampTest();
    responseBody = "<html><body><h1>Front lamp toggled</h1></body></html>";
  } else if (requestLine.indexOf("GET /rear") == 0) {
    RearLampTest();
    responseBody = "<html><body><h1>Rear lamp toggled</h1></body></html>";
  } else if (requestLine.indexOf("GET /forward") == 0) {
    motorForward(200);
    responseBody = "<html><body><h1>Motors Forward</h1></body></html>";
  } else if (requestLine.indexOf("GET /backward") == 0) {
    motorBackward(200);
    responseBody = "<html><body><h1>Motors Backward</h1></body></html>";
  } else if (requestLine.indexOf("GET /left") == 0) {
    motorLeft(200);
    responseBody = "<html><body><h1>Turn Left</h1></body></html>";
  } else if (requestLine.indexOf("GET /right") == 0) {
    motorRight(200);
    responseBody = "<html><body><h1>Turn Right</h1></body></html>";
  } else if (requestLine.indexOf("GET /stop") == 0) {
    motorStop();
    responseBody = "<html><body><h1>Motors Stopped</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/start") == 0) {
    lineFollowing = true;
    responseBody = "<html><body><h1>Line follow started</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/stop") == 0) {
    lineFollowing = false;
    motorStop();
    responseBody = "<html><body><h1>Line follow stopped</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/toggle") == 0) {
    lineFollowing = !lineFollowing;
    if (!lineFollowing) motorStop();
    responseBody = String("<html><body><h1>Line follow ") + (lineFollowing ? "started" : "stopped") + "</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/status") == 0) {
    responseBody = String("<html><body><h1>Line Status</h1><p>Left=") + String(sensorLeft) + " Right=" + String(sensorRight) + "</p><p>State=" + (lineFollowing ? "ON" : "OFF") + "</p></body></html>";
  } else if (requestLine.indexOf("GET /joystick/toggle") == 0) {
    joystickMode = !joystickMode;
    if (!joystickMode) motorStop();
    responseBody = String("<html><body><h1>Joystick Control ") + (joystickMode ? "Enabled" : "Disabled") + "</h1></body></html>";
  } else if (requestLine.indexOf("GET /joystick/move?x=") == 0) {
    // Parse joystick X value
    int xStart = requestLine.indexOf("x=") + 2;
    int xEnd = requestLine.indexOf("&", xStart);
    if (xEnd == -1) xEnd = requestLine.indexOf(" ", xStart);
    String xStr = requestLine.substring(xStart, xEnd);
    joystickX = xStr.toInt();
    
    // Parse joystick Y value
    int yStart = requestLine.indexOf("y=") + 2;
    int yEnd = requestLine.indexOf(" ", yStart);
    String yStr = requestLine.substring(yStart, yEnd);
    joystickY = yStr.toInt();
    
    responseBody = "{\"status\":\"ok\",\"x\":" + String(joystickX) + ",\"y\":" + String(joystickY) + "}";
  } else {
    // Interactive status page with buttons and AJAX
    responseBody = "<!DOCTYPE html>";
    responseBody += "<html><head><title>ESP32 Control</title>";
    responseBody += "<style>";
    responseBody += "body { font-family: Arial; margin: 20px; background: #f0f0f0; }";
    responseBody += "h1 { color: #333; }";
    responseBody += ".info { background: white; padding: 15px; border-radius: 5px; margin: 10px 0; }";
    responseBody += ".section { background: white; padding: 15px; border-radius: 5px; margin: 15px 0; }";
    responseBody += "button { padding: 10px 20px; margin: 5px; font-size: 14px; border: none; border-radius: 5px; cursor: pointer; background: #007bff; color: white; }";
    responseBody += "button:hover { background: #0056b3; }";
    responseBody += "button:active { transform: scale(0.98); }";
    responseBody += ".status { margin-top: 10px; padding: 10px; background: #e7f3ff; border-left: 4px solid #007bff; }";
    responseBody += "</style></head><body>";
    responseBody += "<h1>ESP32 Control Panel</h1>";
    responseBody += "<div class='info'>";

    responseBody += "<p><strong>AP IP:</strong> ";
    responseBody += WiFi.softAPIP().toString();
    responseBody += "</p>";
    responseBody += "<p><strong>Left Encoder:</strong> ";
    responseBody += leftEncoderCount;
    responseBody += "</p>";
    responseBody += "<p><strong>Right Encoder:</strong> ";
    responseBody += rightEncoderCount;
    responseBody += "</p></div>";
    
    responseBody += "<div class='section'><h2>Buzzer & Lights</h2>";
    responseBody += "<button onclick='cmd(\"/buzzer\")'>Buzzer</button>";
    responseBody += "<button onclick='cmd(\"/front\")'>Front Lamp</button>";
    responseBody += "<button onclick='cmd(\"/rear\")'>Rear Lamp</button>";
    responseBody += "</div>";
    
    // Line follow section
    responseBody += "<div class='section'><h2>Line Follow</h2>";
    responseBody += "<p>Left: ";
    responseBody += String(sensorLeft);
    responseBody += " Right: ";
    responseBody += String(sensorRight);
    responseBody += "</p>";
    responseBody += "<p>Status: ";
    responseBody += (lineFollowing ? "ON" : "OFF");
    responseBody += "</p>";
    responseBody += "<button onclick='cmd(\"/linefollow/toggle\")'> Toggle Line Follow</button>";
    responseBody += "</div>";
    
    // Joystick section
    responseBody += "<div class='section'><h2>Virtual Joystick</h2>";
    responseBody += "<button onclick='cmd(\"/joystick/toggle\")'>";
    responseBody += (joystickMode ? "Disable Joystick" : "Enable Joystick");
    responseBody += "</button>";
    responseBody += "<p>Status: ";
    responseBody += (joystickMode ? "<span style='color:green;'><strong>ENABLED</strong></span>" : "<span style='color:red;'>DISABLED</span>");
    responseBody += "</p>";
    responseBody += "<canvas id='joystick' width='200' height='200' style='border:2px solid #ccc; border-radius:5px; background:#f9f9f9; cursor:crosshair; margin-top:10px;'></canvas>";
    responseBody += "<p>X: <span id='joyX'>0</span> | Y: <span id='joyY'>0</span></p>";
    responseBody += "</div>";
    
    responseBody += "<div id='status' class='status' style='display:none;'></div>";
    
    responseBody += "<script>";
    responseBody += "let canvas = document.getElementById('joystick');";
    responseBody += "let ctx = canvas.getContext('2d');";
    responseBody += "let joyX = 0, joyY = 0;";
    responseBody += "let isMouseDown = false;";
    responseBody += "let touchId = null;";
    responseBody += "";
    responseBody += "function drawJoystick() {";
    responseBody += "  ctx.fillStyle = '#f9f9f9';";
    responseBody += "  ctx.fillRect(0, 0, 200, 200);";
    responseBody += "  ctx.strokeStyle = '#ccc';";
    responseBody += "  ctx.lineWidth = 2;";
    responseBody += "  ctx.beginPath();";
    responseBody += "  ctx.arc(100, 100, 80, 0, Math.PI * 2);";
    responseBody += "  ctx.stroke();";
    responseBody += "  ctx.strokeStyle = '#999';";
    responseBody += "  ctx.beginPath();";
    responseBody += "  ctx.moveTo(100, 20);";
    responseBody += "  ctx.lineTo(100, 180);";
    responseBody += "  ctx.moveTo(20, 100);";
    responseBody += "  ctx.lineTo(180, 100);";
    responseBody += "  ctx.stroke();";
    responseBody += "  let posX = 100 + (joyX / 100) * 70;";
    responseBody += "  let posY = 100 - (joyY / 100) * 70;";
    responseBody += "  ctx.fillStyle = joyX == 0 && joyY == 0 ? '#ccc' : '#007bff';";
    responseBody += "  ctx.beginPath();";
    responseBody += "  ctx.arc(posX, posY, 15, 0, Math.PI * 2);";
    responseBody += "  ctx.fill();";
    responseBody += "  document.getElementById('joyX').textContent = joyX;";
    responseBody += "  document.getElementById('joyY').textContent = joyY;";
    responseBody += "}";
    responseBody += "";
    responseBody += "function handleJoystickMove(x, y) {";
    responseBody += "  let rect = canvas.getBoundingClientRect();";
    responseBody += "  let centerX = rect.width / 2;";
    responseBody += "  let centerY = rect.height / 2;";
    responseBody += "  let dx = x - rect.left - centerX;";
    responseBody += "  let dy = y - rect.top - centerY;";
    responseBody += "  let distance = Math.sqrt(dx * dx + dy * dy);";
    responseBody += "  let maxDistance = 70;";
    responseBody += "  if (distance > maxDistance) {";
    responseBody += "    dx = (dx / distance) * maxDistance;";
    responseBody += "    dy = (dy / distance) * maxDistance;";
    responseBody += "  }";
    responseBody += "  joyX = Math.round((dx / 70) * 100);";
    responseBody += "  joyY = Math.round((dy / 70) * -100);";
    responseBody += "  joyX = Math.max(-100, Math.min(100, joyX));";
    responseBody += "  joyY = Math.max(-100, Math.min(100, joyY));";
    responseBody += "  sendJoystickUpdate();";
    responseBody += "  drawJoystick();";
    responseBody += "}";
    responseBody += "";
    responseBody += "function sendJoystickUpdate() {";
    responseBody += "  fetch('/joystick/move?x=' + joyX + '&y=' + joyY).catch(e => console.log('Error:', e));";
    responseBody += "}";
    responseBody += "";
    responseBody += "canvas.addEventListener('mousedown', () => isMouseDown = true);";
    responseBody += "canvas.addEventListener('mousemove', (e) => {";
    responseBody += "  if (isMouseDown) handleJoystickMove(e.clientX, e.clientY);";
    responseBody += "});";
    responseBody += "canvas.addEventListener('mouseup', () => {";
    responseBody += "  isMouseDown = false;";
    responseBody += "  joyX = 0; joyY = 0;";
    responseBody += "  sendJoystickUpdate();";
    responseBody += "  drawJoystick();";
    responseBody += "});";
    responseBody += "canvas.addEventListener('mouseleave', () => {";
    responseBody += "  isMouseDown = false;";
    responseBody += "  joyX = 0; joyY = 0;";
    responseBody += "  sendJoystickUpdate();";
    responseBody += "  drawJoystick();";
    responseBody += "});";
    responseBody += "";
    responseBody += "canvas.addEventListener('touchstart', (e) => { touchId = e.touches[0].identifier; });";
    responseBody += "canvas.addEventListener('touchmove', (e) => {";
    responseBody += "  for (let t of e.touches) {";
    responseBody += "    if (t.identifier === touchId) {";
    responseBody += "      handleJoystickMove(t.clientX, t.clientY);";
    responseBody += "      break;";
    responseBody += "    }";
    responseBody += "  }";
    responseBody += "});";
    responseBody += "canvas.addEventListener('touchend', (e) => {";
    responseBody += "  joyX = 0; joyY = 0;";
    responseBody += "  sendJoystickUpdate();";
    responseBody += "  drawJoystick();";
    responseBody += "  touchId = null;";
    responseBody += "});";
    responseBody += "";
    responseBody += "drawJoystick();";
    responseBody += "";
    responseBody += "function cmd(path) {";
    responseBody += "  fetch(path).then(r => r.text()).then(d => {";
    responseBody += "    let st = document.getElementById('status');";
    responseBody += "    st.style.display = 'block';";
    responseBody += "    st.innerHTML = '<strong>OK:</strong> ' + path;";
    responseBody += "    setTimeout(() => st.style.display = 'none', 2000);";
    responseBody += "  }).catch(e => {";
    responseBody += "    document.getElementById('status').innerHTML = '<strong>Error:</strong> ' + e;";
    responseBody += "</div>";    
    // Ultrasonic sensor section
    responseBody += "<div class='section'><h2>Ultrasonic Sensor</h2>";
    responseBody += "<p>Distance: ";
    if (distanceCm > 0) {
      responseBody += String(distanceCm);
      responseBody += " cm";
      if (distanceCm < OBSTACLE_DISTANCE_CM) {
        responseBody += " <strong style='color: red;'>(OBSTACLE!)</strong>";
      }
    } else {
      responseBody += "No object detected";
    }
    responseBody += "</p>";
    responseBody += "</div>";    
    responseBody += "<div id='status' class='status' style='display:none;'></div>";
    
    responseBody += "<script>";
    responseBody += "function cmd(path) {";
    responseBody += "  fetch(path).then(r => r.text()).then(d => {";
    responseBody += "    let st = document.getElementById('status');";
    responseBody += "    st.style.display = 'block';";
    responseBody += "    st.innerHTML = '<strong>OK:</strong> ' + path;";
    responseBody += "    setTimeout(() => st.style.display = 'none', 2000);";
    responseBody += "  }).catch(e => {";
    responseBody += "    document.getElementById('status').innerHTML = '<strong>Error:</strong> ' + e;";
    responseBody += "    document.getElementById('status').style.display = 'block';";
    responseBody += "  });";
    responseBody += "}";
    responseBody += "</script>";
    responseBody += "</body></html>";
  }

  sendHttpResponse(client, responseBody);
  delay(1);
  client.stop();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLineRead >= READ_INTERVAL_MS) {
    lastLineRead = now;
    
    // Check ultrasonic distance
    distanceCm = readUltrasonicDistance();
    
    // Emergency stop if obstacle too close
    if (distanceCm > 0 && distanceCm < OBSTACLE_DISTANCE_CM) {
      motorStop();
      lineFollowing = false;
      joystickMode = false;
      Serial.print("Obstacle detected at ");
      Serial.print(distanceCm);
      Serial.println(" cm - STOPPED");
    } else if (joystickMode) {
      // Joystick control enabled
      handleJoystickControl(joystickX, joystickY);
    } else if (lineFollowing) {
      runLineFollow();
    } else {
      // keep sensors updated for the web UI
      sensorLeft = readAverage(SENSOR_LEFT_PIN, 2);
      sensorRight = readAverage(SENSOR_RIGHT_PIN, 2);
    }
  }

  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }
  delay(1);
}
