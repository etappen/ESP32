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
const int BLACK_THRESHOLD = 3300;
const int UPDATE_INTERVAL = 20;
const int LINE_SWEEP_SPEED = 180;
// Ultrasonic sampling
const int ULTRASONIC_SAMPLES = 3;
const bool ULTRASONIC_DEBUG = true;
int thresholdLeft = 2000;
int thresholdRight = 2000;
long sensorLeft = 0;
long sensorRight = 0;
bool lineFollowing = false;
unsigned long lastLineRead = 0;
unsigned long lastLineFollowUpdate = 0;
long distanceCm = 0;

// Virtual Joystick variables (controlled via web interface)
int joystickX = 0;  // -100 to 100
int joystickY = 0;  // -100 to 100
bool joystickMode = true;  // Joystick control ON by default

// Access Point settings (updated for UDP control)
const char* ap_ssid = "JohnDeere";
const char* ap_pass = "tractor80085";

// UDP control settings
#include <WiFiUdp.h>
const char* sta_ssid = "SU-ECE-LAB";
const char* sta_password = "FaraDay8086!";
WiFiUDP udp;
#define UDP_PORT 4210
char incomingPacket[255];
int robotID = 19;
#define UDP_LED_PIN 2

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
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_pass);
  WiFi.begin(sta_ssid, sta_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);
  server.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_RIGHT_PIN, ADC_11db);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
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
  pinMode(UDP_LED_PIN, OUTPUT); // For UDP dance routine
}
void sendHttpResponse(WiFiClient &client, const String &body) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
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
      // ...removed serial print...
    }
    if (duration > 0) {
      long distance = (duration * 343) / 20000;
      totalDistance += distance;
      valid++;
    }
    delay(10);
  }

  if (valid == 0) {
    if (ULTRASONIC_DEBUG) {
      // ...removed serial print...
    }
    return 0;
  }

  long avg = totalDistance / valid;
  if (ULTRASONIC_DEBUG) {
    // ...removed serial print...
  }
  return avg;
}

bool seesBlack() {
  return analogRead(SENSOR_LEFT_PIN) > BLACK_THRESHOLD;
}

// Line-follow logic from provided module:
// - If on line: drive forward
// - If line lost: sweep left/right with increasing sweep duration until line is found
void runLineFollow() {
  unsigned long now = millis();
  if (now - lastLineFollowUpdate < UPDATE_INTERVAL) return;
  lastLineFollowUpdate = now;

  sensorLeft = readAverage(SENSOR_LEFT_PIN, AVG_SAMPLES);
  sensorRight = readAverage(SENSOR_RIGHT_PIN, AVG_SAMPLES);

  if (seesBlack()) {
    motorForward(150);
    return;
  }

  int sweep = 7;
  bool goingLeft = true;
  while (!seesBlack()) {
    if (goingLeft) {
      pivotLeftInPlace(LINE_SWEEP_SPEED);
      delay(sweep);
      goingLeft = false;
      if (seesBlack()) {
        motorForward(150);
        return;
      }
    } else {
      pivotRightInPlace(LINE_SWEEP_SPEED);
      delay(sweep);
      goingLeft = true;
      if (seesBlack()) {
        motorForward(150);
        return;
      }
    }
    if (!seesBlack()) {
      if (goingLeft) {
        pivotLeftInPlace(LINE_SWEEP_SPEED);
        delay(sweep);
      } else {
        pivotRightInPlace(LINE_SWEEP_SPEED);
        delay(sweep);
      }
    }
    sweep += 7;
  }
  if (seesBlack()) {
    motorForward(150);
    return;
  }
  motorStop();
}

void BuzzerTest() {
  playTone(BUZZER_PIN, BUZZER_FREQ, 300);
  delay(150);
  playTone(BUZZER_PIN, BUZZER_FREQ, 300);
}

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

void FrontLampTest() {
  digitalWrite(FRONTLAMPS, HIGH);
  delay(1000);
  digitalWrite(FRONTLAMPS, LOW);
}

void RearLampTest() {
  digitalWrite(REARLAMPS, HIGH);
  delay(1000);
  digitalWrite(REARLAMPS, LOW);
}

void motorForward(uint8_t speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, speed);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, speed);
}

void motorBackward(uint8_t speed) {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  analogWrite(LEFT_EN, speed);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(RIGHT_EN, speed);
}

void motorStop() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, 0);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, 0);
}

void motorLeft(uint8_t speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, 0);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, speed);
}

void motorRight(uint8_t speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, speed);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, 0);
}

void pivotLeftInPlace(uint8_t speed) {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  analogWrite(LEFT_EN, speed);

  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, speed);
}

void pivotRightInPlace(uint8_t speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, speed);

  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(RIGHT_EN, speed);
}

// Joystick control function - maps joystick values to motor speeds
// Responsive joystick control: direct mapping with smoothing and small deadzone
float joyXFiltered = 0;
float joyYFiltered = 0;
const float FILTER_ALPHA = 0.25; // Smoothing factor (0.0-1.0)
const int PRECISE_DEADZONE = 8;  // Small deadzone for precision
const float PRECISE_SCALE = 2.55f; // Full PWM range

void handleJoystickControl(int joyX, int joyY) {
  // Smoothing/filtering
  joyXFiltered = FILTER_ALPHA * joyX + (1.0 - FILTER_ALPHA) * joyXFiltered;
  joyYFiltered = FILTER_ALPHA * joyY + (1.0 - FILTER_ALPHA) * joyYFiltered;

  // Deadzone for small joystick movements
  if (abs(joyXFiltered) < PRECISE_DEADZONE && abs(joyYFiltered) < PRECISE_DEADZONE) {
    motorStop();
    return;
  }

  // Map to full PWM range [-255, 255]
  int pwmX = (int)(joyXFiltered * PRECISE_SCALE);
  int pwmY = (int)(joyYFiltered * PRECISE_SCALE);

  // Scale turn aggressiveness by forward speed (gentler turns at low speed)
  float turnScale = abs(pwmY) / 150.0; // 0 (no forward) to 1 (full speed)
  if (turnScale > 1.0) turnScale = 1.0;
  int scaledPwmX = (int)(pwmX * turnScale);

  // Differential drive: Y positive = forward, X positive = right
  int leftMotorSpeed = pwmY + scaledPwmX;
  int rightMotorSpeed = pwmY - scaledPwmX;

  // Limit motor speeds to [-250, 250] for safer, slower control
  leftMotorSpeed = constrain(leftMotorSpeed, -250, 250);
  rightMotorSpeed = constrain(rightMotorSpeed, -250, 250);

  // Minimum PWM threshold for non-zero speeds
  const int MIN_PWM = 30;
  int leftPwm = (leftMotorSpeed != 0) ? max(abs(leftMotorSpeed), MIN_PWM) : 0;
  int rightPwm = (rightMotorSpeed != 0) ? max(abs(rightMotorSpeed), MIN_PWM) : 0;

  // Left motor direction
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
  analogWrite(LEFT_EN, leftPwm);

  // Right motor direction
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
  analogWrite(RIGHT_EN, rightPwm);
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
    motorRight(200);
    responseBody = "<html><body><h1>Turn Left</h1></body></html>";
  } else if (requestLine.indexOf("GET /right") == 0) {
    motorLeft(200);
    responseBody = "<html><body><h1>Turn Right</h1></body></html>";
  } else if (requestLine.indexOf("GET /stop") == 0) {
    motorStop();
    responseBody = "<html><body><h1>Motors Stopped</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/start") == 0) {
    lineFollowing = true;
    joystickMode = false;
    responseBody = "<html><body><h1>Line follow started</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/stop") == 0) {
    lineFollowing = false;
    joystickMode = true;
    motorStop();
    responseBody = "<html><body><h1>Line follow stopped</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/toggle") == 0) {
    lineFollowing = !lineFollowing;
    joystickMode = !lineFollowing;
    if (!lineFollowing) motorStop();
    responseBody = String("<html><body><h1>Line follow ") + (lineFollowing ? "started" : "stopped") + "</h1></body></html>";
  } else if (requestLine.indexOf("GET /linefollow/status") == 0) {
    responseBody = String("<html><body><h1>Line Status</h1><p>Left=") + String(sensorLeft) + " Right=" + String(sensorRight) + "</p><p>State=" + (lineFollowing ? "ON" : "OFF") + "</p></body></html>";
  } else if (requestLine.indexOf("GET /joystick/toggle") == 0) {
    joystickMode = !joystickMode;
    if (joystickMode) {
      lineFollowing = false;
    }
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
    responseBody = "<!DOCTYPE html><html><head><title>ESP32 Control</title>";
    responseBody += "<style>";
    responseBody += "body{font-family:Arial;margin:20px;background:#f0f0f0;}";
    responseBody += "h1{color:#333;}";
    responseBody += ".info,.section{background:white;padding:15px;border-radius:6px;margin:12px 0;}";
    responseBody += "button{padding:10px 18px;margin:5px;font-size:14px;border:none;border-radius:5px;cursor:pointer;background:#007bff;color:white;}";
    responseBody += "button:hover{background:#0056b3;}";
    responseBody += ".status{margin-top:10px;padding:10px;background:#e7f3ff;border-left:4px solid #007bff;display:none;}";
    responseBody += ".joystick-panel{text-align:center;}";
    responseBody += "#joystick{display:block;margin:10px auto 0 auto;border:none;border-radius:22px;background:transparent;cursor:crosshair;touch-action:none;}";
    responseBody += "</style></head><body>";

    responseBody += "<h1>ESP32 Control Panel</h1>";
    responseBody += "<div class='info'>";
    responseBody += "<p><strong>AP IP:</strong> " + WiFi.softAPIP().toString() + "</p>";
    responseBody += "<p><strong>Left Encoder:</strong> " + String(leftEncoderCount) + "</p>";
    responseBody += "<p><strong>Right Encoder:</strong> " + String(rightEncoderCount) + "</p>";
    responseBody += "</div>";

    responseBody += "<div class='section'><h2>Buzzer & Lights</h2>";
    responseBody += "<button onclick='cmd(\"/buzzer\")'>Buzzer</button>";
    responseBody += "<button onclick='cmd(\"/front\")'>Front Lamp</button>";
    responseBody += "<button onclick='cmd(\"/rear\")'>Rear Lamp</button>";
    responseBody += "</div>";

    responseBody += "<div class='section'><h2>Line Follow</h2>";
    responseBody += "<p>Left: " + String(sensorLeft) + " Right: " + String(sensorRight) + "</p>";
    responseBody += "<p>Status: " + String(lineFollowing ? "ON" : "OFF") + "</p>";
    responseBody += "<button onclick='cmd(\"/linefollow/toggle\")'>Toggle Line Follow</button>";
    responseBody += "</div>";

    responseBody += "<div class='section'><h2>Ultrasonic Sensor</h2><p>Distance: ";
    if (distanceCm > 0) {
      responseBody += String(distanceCm) + " cm";
      if (distanceCm < OBSTACLE_DISTANCE_CM) {
        responseBody += " <strong style='color:red;'>(OBSTACLE!)</strong>";
      }
    } else {
      responseBody += "No object detected";
    }
    responseBody += "</p></div>";

    responseBody += "<div class='section joystick-panel'><h2>Joystick Control</h2>";
    responseBody += "<button onclick='cmd(\"/joystick/toggle\")'>";
    responseBody += (joystickMode ? "Disable Joystick" : "Enable Joystick");
    responseBody += "</button>";
    responseBody += "<p>Status: ";
    responseBody += (joystickMode ? "<span style='color:green;'><strong>ENABLED</strong></span>" : "<span style='color:red;'>DISABLED</span>");
    responseBody += "</p>";
    responseBody += "<canvas id='joystick' width='320' height='320'></canvas>";
    responseBody += "<p>X: <span id='joyX'>0</span> | Y: <span id='joyY'>0</span></p>";
    responseBody += "</div>";

    responseBody += "<div id='status' class='status'></div>";
    responseBody += "<script>";
    responseBody += "const canvas=document.getElementById('joystick');";
    responseBody += "const ctx=canvas.getContext('2d');";
    responseBody += "let joyX=0,joyY=0,isMouseDown=false,touchId=null;";
    responseBody += "let inFlight=false,pending=false,pendingX=0,pendingY=0;";
    responseBody += "const JOY_SIZE=canvas.width,JOY_CENTER=JOY_SIZE/2,JOY_RADIUS=JOY_SIZE*0.4,KNOB_TRAVEL=JOY_SIZE*0.35,KNOB_RADIUS=JOY_SIZE*0.075;";

    responseBody += "function drawJoystick(){";
    responseBody += "ctx.clearRect(0,0,JOY_SIZE,JOY_SIZE);";
    responseBody += "const bg=ctx.createRadialGradient(JOY_CENTER,JOY_CENTER,JOY_RADIUS*0.2,JOY_CENTER,JOY_CENTER,JOY_RADIUS*1.2);";
    responseBody += "bg.addColorStop(0,'#ffffff');bg.addColorStop(1,'#f9f9f9');";
    responseBody += "ctx.fillStyle=bg;ctx.beginPath();ctx.arc(JOY_CENTER,JOY_CENTER,JOY_RADIUS+8,0,Math.PI*2);ctx.fill();";
    responseBody += "ctx.shadowColor='#999';ctx.shadowBlur=16;ctx.strokeStyle='#ccc';ctx.lineWidth=8;ctx.beginPath();ctx.arc(JOY_CENTER,JOY_CENTER,JOY_RADIUS,0,Math.PI*2);ctx.stroke();ctx.shadowBlur=0;";
    responseBody += "ctx.strokeStyle='#ccc';ctx.lineWidth=1;ctx.beginPath();ctx.arc(JOY_CENTER,JOY_CENTER,JOY_RADIUS*0.55,0,Math.PI*2);ctx.stroke();";
    responseBody += "const posX=JOY_CENTER+(joyX/100)*KNOB_TRAVEL;const posY=JOY_CENTER-(joyY/100)*KNOB_TRAVEL;";
    responseBody += "ctx.strokeStyle='#999';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(JOY_CENTER,JOY_CENTER);ctx.lineTo(posX,posY);ctx.stroke();";
    responseBody += "ctx.fillStyle='#ccc';ctx.beginPath();ctx.arc(posX,posY,KNOB_RADIUS+10,0,Math.PI*2);ctx.fill();";
    responseBody += "const knob=ctx.createRadialGradient(posX-KNOB_RADIUS*0.35,posY-KNOB_RADIUS*0.35,KNOB_RADIUS*0.2,posX,posY,KNOB_RADIUS);";
    responseBody += "knob.addColorStop(0,'#ffffff');knob.addColorStop(1,'#007bff');";
    responseBody += "ctx.fillStyle=knob;ctx.beginPath();ctx.arc(posX,posY,KNOB_RADIUS,0,Math.PI*2);ctx.fill();";
    responseBody += "ctx.fillStyle='#ffffff';ctx.beginPath();ctx.arc(posX-KNOB_RADIUS*0.25,posY-KNOB_RADIUS*0.25,KNOB_RADIUS*0.22,0,Math.PI*2);ctx.fill();";
    responseBody += "document.getElementById('joyX').textContent=joyX;document.getElementById('joyY').textContent=joyY;";
    responseBody += "}";

    responseBody += "function queueSend(){pendingX=joyX;pendingY=joyY;pending=true;if(!inFlight)sendJoystick();}";
    responseBody += "function sendJoystick(){if(!pending)return;inFlight=true;pending=false;const x=pendingX,y=pendingY;fetch('/joystick/move?x='+x+'&y='+y).catch(()=>{}).finally(()=>{inFlight=false;if(pending)sendJoystick();});}";
    responseBody += "function handleJoystickMove(x,y){const r=canvas.getBoundingClientRect();let dx=x-r.left-r.width/2;let dy=y-r.top-r.height/2;const d=Math.sqrt(dx*dx+dy*dy);if(d>KNOB_TRAVEL){dx=(dx/d)*KNOB_TRAVEL;dy=(dy/d)*KNOB_TRAVEL;}joyX=Math.max(-100,Math.min(100,Math.round((dx/KNOB_TRAVEL)*100)));joyY=Math.max(-100,Math.min(100,Math.round((dy/KNOB_TRAVEL)*-100)));queueSend();drawJoystick();}";

    responseBody += "canvas.addEventListener('mousedown',()=>isMouseDown=true);";
    responseBody += "canvas.addEventListener('mousemove',e=>{if(isMouseDown)handleJoystickMove(e.clientX,e.clientY);});";
    responseBody += "canvas.addEventListener('mouseup',()=>{isMouseDown=false;joyX=0;joyY=0;queueSend();drawJoystick();});";
    responseBody += "canvas.addEventListener('mouseleave',()=>{isMouseDown=false;joyX=0;joyY=0;queueSend();drawJoystick();});";
    responseBody += "canvas.addEventListener('touchstart',e=>{e.preventDefault();touchId=e.touches[0].identifier;},{passive:false});";
    responseBody += "canvas.addEventListener('touchmove',e=>{e.preventDefault();for(const t of e.touches){if(t.identifier===touchId){handleJoystickMove(t.clientX,t.clientY);break;}}},{passive:false});";
    responseBody += "canvas.addEventListener('touchend',e=>{e.preventDefault();joyX=0;joyY=0;queueSend();drawJoystick();touchId=null;},{passive:false});";

    responseBody += "function cmd(path){fetch(path).then(r=>r.text()).then(()=>{const st=document.getElementById('status');st.style.display='block';st.innerHTML='<strong>OK:</strong> '+path;if(path==='/joystick/toggle'){setTimeout(()=>location.reload(),200);}setTimeout(()=>st.style.display='none',2000);}).catch(e=>{const st=document.getElementById('status');st.innerHTML='<strong>Error:</strong> '+e;st.style.display='block';});}";
    responseBody += "drawJoystick();";
    responseBody += "</script></body></html>";
  }

  sendHttpResponse(client, responseBody);
  delay(1);
  client.stop();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLineRead >= READ_INTERVAL_MS) {
    lastLineRead = now;

    // UDP packet handling
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(incomingPacket, 255);
      if (len > 0) incomingPacket[len] = 0;
      String command = String(incomingPacket);
      handleUDPCommand(command);
    }

    // Check ultrasonic distance
    distanceCm = readUltrasonicDistance();

    // Emergency stop if obstacle too close
    if (distanceCm > 0 && distanceCm < OBSTACLE_DISTANCE_CM) {
      motorStop();
      lineFollowing = false;
      joystickMode = true;
      joystickX = 0;
      joystickY = 0;
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

// UDP command handler
void handleUDPCommand(String cmd) {
  if (cmd == "DANCE_ALL") {
    udpDance();
  }
  if (cmd.startsWith("DANCE_")) {
    int target = cmd.substring(6).toInt();
    if (target == robotID) {
      udpDance();
    }
  }
}

// UDP dance routine (LED blink)
void udpDance() {
  Serial.println("Robot Dancing!");
  // Siren buzzer effect while dancing
  for(int i=0;i<6;i++){
    // Alternate front/rear lamps quickly
    for (int j = 0; j < 8; j++) {
      if (j % 2 == 0) {
        digitalWrite(FRONTLAMPS, HIGH);
        digitalWrite(REARLAMPS, LOW);
      } else {
        digitalWrite(FRONTLAMPS, LOW);
        digitalWrite(REARLAMPS, HIGH);
      }
      delay(40);
    }
    digitalWrite(FRONTLAMPS, LOW);
    digitalWrite(REARLAMPS, LOW);

    // Siren buzzer: sweep frequency up and down
    for (int freq = 1000; freq <= 2500; freq += 100) {
      playTone(BUZZER_PIN, freq, 10);
    }
    for (int freq = 2500; freq >= 1000; freq -= 100) {
      playTone(BUZZER_PIN, freq, 10);
    }
    delay(100);
  }
}
