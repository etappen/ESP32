#include <WiFi.h>

// ============================================================================
// MOTOR DRIVER MODULE (SN754410)
// ============================================================================
#define LEFT_EN 25     // PWM enable (speed control)
#define LEFT_IN1 16    // Left motor input 1
#define LEFT_IN2 17    // Left motor input 2

#define RIGHT_EN 26    // PWM enable (speed control)
#define RIGHT_IN1 18   // Right motor input 1
#define RIGHT_IN2 19   // Right motor input 2

// Virtual Joystick variables (controlled via web interface)
int joystickX = 0;  // -100 to 100
int joystickY = 0;  // -100 to 100
bool joystickMode = true;  // Enable joystick control by default

// Motor speed smoothing variables for gradual acceleration/deceleration
int currentLeftSpeed = 0;   // Current left motor speed
int currentRightSpeed = 0;  // Current right motor speed
int targetLeftSpeed = 0;    // Target left motor speed
int targetRightSpeed = 0;   // Target right motor speed
const int ACCEL_RATE = 15;  // Speed change per update cycle (higher = faster response)

// ============================================================================
// ENCODER MODULE
// ============================================================================
#define LEFT_ENC_A 34  // Left motor encoder A
#define LEFT_ENC_B 35  // Left motor encoder B
#define RIGHT_ENC_A 36 // Right motor encoder A
#define RIGHT_ENC_B 39 // Right motor encoder B

volatile int leftEncoderCount = 0;  // Tracks left wheel rotation, displayed on web interface
volatile int rightEncoderCount = 0; // Tracks right wheel rotation, displayed on web interface

// Encoder interrupt handlers - CALLED BY: Hardware interrupts (attached in setup())
// These functions are triggered automatically when encoder signals change
// They update the encoder counts which are displayed on the web interface
void IRAM_ATTR handleLeftEncoderA() {
  // Determine rotation direction by comparing encoder phases
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) {
    leftEncoderCount++;  // Forward rotation
  } else {
    leftEncoderCount--;  // Backward rotation
  }
}

void IRAM_ATTR handleRightEncoderA() {
  // Determine rotation direction by comparing encoder phases
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) {
    rightEncoderCount++;  // Forward rotation
  } else {
    rightEncoderCount--;  // Backward rotation
  }
}

// ============================================================================
// LINE FOLLOWING MODULE (PHOTOTRANSISTOR)
// ============================================================================
#define PHOTOTRANSISTOR_PIN 32
const int BLACK_THRESHOLD = 3800;  // Threshold for detecting black line
const int UPDATE_INTERVAL = 20;
long sensorReading = 0;
bool lineFollowing = false;
unsigned long lastUpdate = 0;

// Line following state machine variables (for non-blocking sweep)
int sweepDuration = 7;          // Current sweep duration in ms
bool sweepingLeft = true;       // Current sweep direction
unsigned long sweepStartTime = 0;  // When current sweep began
bool inSweep = false;           // Whether we're currently sweeping

// ============================================================================
// ULTRASONIC SENSOR MODULE
// ============================================================================
#define TRIG_PIN 4
#define ECHO_PIN 27
const int OBSTACLE_DISTANCE_CM = 5;  // Stop if object closer than this
const int ULTRASONIC_SAMPLES = 3;
const bool ULTRASONIC_DEBUG = true;
long distanceCm = 0;

// ============================================================================
// BUZZER MODULE
// ============================================================================
#define BUZZER_PIN 21
const int BUZZER_FREQ = 2000; // 2 kHz default tone
const int BUZZER_RES = 8;     // 8-bit resolution (0-255)

// Non-blocking buzzer state tracking
bool buzzerActive = false;
unsigned long buzzerEndTime = 0;
const unsigned long BUZZER_DURATION_MS = 300;

// ============================================================================
// LAMPS MODULE
// ============================================================================
#define FRONTLAMPS 2
#define REARLAMPS 0

// Non-blocking lamp state tracking
bool frontLampActive = false;
unsigned long frontLampEndTime = 0;
bool rearLampActive = false;
unsigned long rearLampEndTime = 0;
const unsigned long LAMP_DURATION_MS = 1000;

// ============================================================================
// SIREN MODULE
// ============================================================================
const int SIREN_MIN_FREQ = 300;
const int SIREN_MAX_FREQ = 600;
const int SIREN_FREQ_STEP = 30;
const unsigned long SIREN_DURATION_MS = 2000;
const unsigned long SIREN_LAMP_INTERVAL_MS = 80;
const unsigned long SIREN_FREQ_INTERVAL_MS = 80;
bool sirenActive = false;
bool sirenLampFront = true;
bool sirenIncreasing = true;
int sirenFreq = SIREN_MIN_FREQ;
unsigned long sirenEndMs = 0;
unsigned long sirenLastToggleMs = 0;
unsigned long sirenLastFreqMs = 0;

// ============================================================================
// WIFI & WEB SERVER MODULE
// ============================================================================
const char* ap_ssid = "JohnDeere";
const char* ap_pass = "tractor808";
WiFiServer server(80);

// ============================================================================
// GENERAL TIMING
// ============================================================================
const int READ_INTERVAL_MS = 20;  // Read sensors every 20ms (faster updates)
const int AVG_SAMPLES = 1;  // Single sample for instant response
unsigned long lastLineRead = 0;

// ============================================================================
// SETUP FUNCTION - CALLED ONCE: At ESP32 startup/reset
// ============================================================================
void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging
  delay(1000);           // Wait for serial to stabilize
  
  // === Motor Driver Setup ===
  // Configure pins for SN754410 motor driver
  // These pins control motor speed and direction
  // USED BY: motorForward(), motorBackward(), motorStop(), motorLeft(), motorRight(),
  //          handleJoystickControl(), forward(), left(), right(), stop()
  pinMode(LEFT_EN, OUTPUT);   // Left motor speed control (PWM)
  pinMode(LEFT_IN1, OUTPUT);  // Left motor direction bit 1
  pinMode(LEFT_IN2, OUTPUT);  // Left motor direction bit 2
  pinMode(RIGHT_EN, OUTPUT);  // Right motor speed control (PWM)
  pinMode(RIGHT_IN1, OUTPUT); // Right motor direction bit 1
  pinMode(RIGHT_IN2, OUTPUT); // Right motor direction bit 2

  // === Encoder Setup ===
  // Configure encoder pins as inputs to track wheel rotation
  // Attach interrupt handlers that automatically trigger on encoder signal changes
  // INTERRUPTS CALL: handleLeftEncoderA(), handleRightEncoderA()
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoderA, CHANGE);

  // === Line Following Sensor Setup ===
  // Configure ADC for phototransistor analog reading
  // USED BY: seesBlack() which reads PHOTOTRANSISTOR_PIN
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetPinAttenuation(PHOTOTRANSISTOR_PIN, ADC_11db);  // Full voltage range
  
  // === Ultrasonic Sensor Setup ===
  // Configure pins for HC-SR04 distance sensor
  // USED BY: readUltrasonicDistance() called in loop()
  pinMode(TRIG_PIN, OUTPUT);  // Trigger pin sends ultrasonic pulse
  pinMode(ECHO_PIN, INPUT);   // Echo pin receives reflected pulse

  // === Buzzer Setup ===
  // Configure PWM (LEDC) for buzzer tone generation
  // USED BY: BuzzerTest(), playTone(), startSiren(), stopSiren(), updateSiren()
  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES);  // Setup PWM channel and attach pin
  ledcWriteTone(BUZZER_PIN, 0);                     // Start with buzzer off

  // === Lamps Setup ===
  // Configure LED lamp outputs
  // USED BY: FrontLampTest(), RearLampTest(), updateSiren(), stopSiren()
  pinMode(FRONTLAMPS, OUTPUT);
  pinMode(REARLAMPS, OUTPUT);

  // === WiFi & Web Server Setup ===
  // Start WiFi Access Point and web server for remote control
  // USED BY: handleClient() processes incoming web requests in loop()
  WiFi.mode(WIFI_AP);              // Set to Access Point mode (no router needed)
  WiFi.softAP(ap_ssid, ap_pass);   // Create AP with SSID "JohnDeere"
  server.begin();                  // Start web server on port 80
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// CALLED BY: handleClient() - Sends HTTP response to web browser
// Formats and sends HTML content as HTTP 200 OK response
void sendHttpResponse(WiFiClient &client, const String &body) {
  client.println("HTTP/1.1 200 OK");        // HTTP status code
  client.println("Content-Type: text/html"); // Tell browser it's HTML
  client.println("Connection: close");       // Close connection after response
  client.println();                          // Empty line separates headers from body
  client.println(body);                      // Send HTML content
}

// CALLED BY: readUltrasonicDistance(), loop() (legacy) - Averages multiple analog readings
// Takes multiple sensor samples and returns the average to reduce noise (non-blocking version)
long readAverage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(pin);  // Read analog value
    // Removed delay - readings happen immediately
  }
  return sum / samples;      // Return average
}

// ============================================================================
// ULTRASONIC SENSOR FUNCTIONS
// ============================================================================

// CALLED BY: loop() - Measures distance to obstacles using HC-SR04 sensor
// Returns: Distance in centimeters, or 0 if no valid reading
// Uses multiple samples (ULTRASONIC_SAMPLES) and averages them for accuracy (non-blocking)
long readUltrasonicDistance() {
  long totalDistance = 0;  // Accumulator for valid distance readings
  int valid = 0;           // Count of successful readings
  
  // Take multiple samples and average them
  for (int s = 0; s < ULTRASONIC_SAMPLES; ++s) {
    // Send ultrasonic pulse (10μs HIGH pulse)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
    if (duration > 0) {
      // distance = (duration in microseconds * speed of sound) / 2
      // speed of sound = 343 m/s = 0.0343 cm/microsecond
      long distance = (duration * 343) / 20000;  // ~ duration * 0.01715
      totalDistance += distance;
      valid++;
    }
    // Removed delay between samples - measurements happen immediately
  }

  if (valid == 0) {
    return 0;
  }

  long avg = totalDistance / valid;
  return avg;
}

// ============================================================================
// LINE FOLLOWING FUNCTIONS
// ============================================================================

// CALLED BY: runLineFollow() - Checks if phototransistor detects black line
// Returns: true if sensor reading is above BLACK_THRESHOLD (on black line)
//          false if sensor reading is below BLACK_THRESHOLD (off line)
bool seesBlack() {
  return analogRead(PHOTOTRANSISTOR_PIN) > BLACK_THRESHOLD;
}

// Motor control helper functions for line following
// CALLED BY: runLineFollow() during line following behavior

// Move both motors forward at speed 200
void forward() {
  digitalWrite(LEFT_IN1, HIGH);   // Left motor forward
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, 200);      // Left motor speed
  
  digitalWrite(RIGHT_IN1, HIGH);  // Right motor forward
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, 200);     // Right motor speed
}

// Turn left by stopping left motor, running right motor forward
void left() {
  digitalWrite(LEFT_IN1, LOW);    // Left motor stopped
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, 0);        // Left motor speed = 0
  
  digitalWrite(RIGHT_IN1, HIGH);  // Right motor forward
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, 200);     // Right motor speed = 200
}

// Turn right by running left motor forward, stopping right motor
void right() {
  digitalWrite(LEFT_IN1, HIGH);   // Left motor forward
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, 200);      // Left motor speed = 200
  
  digitalWrite(RIGHT_IN1, LOW);   // Right motor stopped
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, 0);       // Right motor speed = 0
}

// Stop both motors completely
void stop() {
  digitalWrite(LEFT_IN1, LOW);    // Left motor stopped
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, 0);        // Left motor speed = 0
  
  digitalWrite(RIGHT_IN1, LOW);   // Right motor stopped
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, 0);       // Right motor speed = 0
}

// CALLED BY: loop() when lineFollowing == true
// Main line following algorithm with expanding sweep pattern (NON-BLOCKING)
// Uses state machine to avoid blocking delays
// CALLS: seesBlack(), forward(), left(), right(), stop()
void runLineFollow() {
  unsigned long now = millis();
  // Rate limiting: Only run line follow logic every UPDATE_INTERVAL ms
  if (now - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = now;

  // CALLS: seesBlack() to check sensor
  // If on the line, just go forward
  if (seesBlack()) {
    forward();  // CALLS: forward() to move straight
    inSweep = false;
    return;
  }

  // Line lost - use state machine for non-blocking sweep
  if (!inSweep) {
    // Start new sweep
    inSweep = true;
    sweepStartTime = now;
  }

  unsigned long sweepElapsed = now - sweepStartTime;

  // Check if current sweep is complete
  if (sweepElapsed >= sweepDuration) {
    // Sweep finished, check if we found the line
    if (seesBlack()) {
      forward();  // CALLS: forward() if line found
      inSweep = false;
      return;
    }
    
    // Line not found during sweep - alternate direction and expand sweep duration
    sweepingLeft = !sweepingLeft;
    sweepDuration += 7;  // Expand sweep by 7ms
    
    // Safety limit: if sweep gets too large, stop searching
    if (sweepDuration > 100) {
      stop();  // CALLS: stop() if sweep exceeds 100ms
      inSweep = false;
      return;
    }
    
    // Start next sweep
    sweepStartTime = now;
  }

  // Apply current sweep direction
  if (sweepingLeft) {
    left();   // CALLS: left() - turn left
  } else {
    right();  // CALLS: right() - turn right
  }
}

// ============================================================================
// BUZZER FUNCTIONS
// ============================================================================

// CALLED BY: handleClient() when /buzzer endpoint is requested
// Starts a non-blocking buzzer beep (300ms duration)
void BuzzerTest() {
  buzzerActive = true;
  buzzerEndTime = millis() + BUZZER_DURATION_MS;  // Set end time
  ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);  // Turn on buzzer
}

// CALLED BY: loop() every iteration - Updates buzzer (non-blocking)
void updateBuzzer() {
  if (!buzzerActive) return;  // Exit if buzzer is off
  
  if (millis() >= buzzerEndTime) {
    buzzerActive = false;
    ledcWriteTone(BUZZER_PIN, 0);  // Turn off buzzer
  }
}

// CALLED BY: (Legacy function, not currently used in code)
// Simple blocking tone generator using digital toggling
// Alternative to PWM when LEDC is not available
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

// ============================================================================
// LAMPS FUNCTIONS
// ============================================================================

// CALLED BY: handleClient() when /front endpoint is requested
// Starts non-blocking front lamp effect (1 second duration)
void FrontLampTest(){
  frontLampActive = true;
  frontLampEndTime = millis() + LAMP_DURATION_MS;  // Set end time
  digitalWrite(FRONTLAMPS, HIGH);  // Turn on front lamps
}

// CALLED BY: handleClient() when /rear endpoint is requested
// Starts non-blocking rear lamp effect (1 second duration)
void RearLampTest(){
  rearLampActive = true;
  rearLampEndTime = millis() + LAMP_DURATION_MS;  // Set end time
  digitalWrite(REARLAMPS, HIGH);   // Turn on rear lamps
}

// CALLED BY: loop() every iteration - Updates lamps (non-blocking)
void updateLamps() {
  if (frontLampActive && millis() >= frontLampEndTime) {
    frontLampActive = false;
    digitalWrite(FRONTLAMPS, LOW);  // Turn off front lamps
  }
  
  if (rearLampActive && millis() >= rearLampEndTime) {
    rearLampActive = false;
    digitalWrite(REARLAMPS, LOW);   // Turn off rear lamps
  }
}

// ============================================================================
// SIREN FUNCTIONS
// ============================================================================

// CALLED BY: SirenAlarm() when siren is turned on
// Initializes siren state and starts the effect
void startSiren() {
  sirenActive = true;                        // Enable siren state
  sirenLampFront = true;                     // Start with front lamps
  sirenIncreasing = true;                    // Start frequency sweep going up
  sirenFreq = SIREN_MIN_FREQ;                // Start at minimum frequency (300 Hz)
  sirenEndMs = millis() + SIREN_DURATION_MS; // Set end time (2 seconds from now)
  sirenLastToggleMs = 0;                     // Reset lamp toggle timer
  sirenLastFreqMs = 0;                       // Reset frequency change timer
  ledcWriteTone(BUZZER_PIN, sirenFreq);      // Start buzzer tone
}

// CALLED BY: updateSiren() when siren duration expires, or SirenAlarm() to turn off
// Stops siren sound and turns off all lamps
void stopSiren() {
  sirenActive = false;              // Disable siren state
  ledcWriteTone(BUZZER_PIN, 0);     // Turn off buzzer
  digitalWrite(FRONTLAMPS, LOW);    // Turn off front lamps
  digitalWrite(REARLAMPS, LOW);     // Turn off rear lamps
}

// CALLED BY: loop() every iteration to maintain non-blocking siren effect
// Manages alternating lamps and sweeping frequency for siren effect
// CALLS: stopSiren() when duration expires
void updateSiren() {
  if (!sirenActive) return;  // Exit if siren is not active

  unsigned long now = millis();
  
  // Check if siren duration has expired
  if (now >= sirenEndMs) {
    stopSiren();  // CALLS: stopSiren() to turn off
    return;
  }

  // Alternate front/rear lamps every SIREN_LAMP_INTERVAL_MS (80ms)
  if (sirenLastToggleMs == 0 || now - sirenLastToggleMs >= SIREN_LAMP_INTERVAL_MS) {
    sirenLastToggleMs = now;
    if (sirenLampFront) {
      digitalWrite(FRONTLAMPS, HIGH);  // Front on
      digitalWrite(REARLAMPS, LOW);    // Rear off
    } else {
      digitalWrite(FRONTLAMPS, LOW);   // Front off
      digitalWrite(REARLAMPS, HIGH);   // Rear on
    }
    sirenLampFront = !sirenLampFront;  // Toggle for next time
  }

  // Sweep buzzer frequency up and down every SIREN_FREQ_INTERVAL_MS (80ms)
  if (sirenLastFreqMs == 0 || now - sirenLastFreqMs >= SIREN_FREQ_INTERVAL_MS) {
    sirenLastFreqMs = now;
    if (sirenIncreasing) {
      // Frequency going up (300 Hz -> 600 Hz)
      sirenFreq += SIREN_FREQ_STEP;  // Increase by 30 Hz
      if (sirenFreq >= SIREN_MAX_FREQ) {
        sirenFreq = SIREN_MAX_FREQ;
        sirenIncreasing = false;     // Start going down
      }
    } else {
      // Frequency going down (600 Hz -> 300 Hz)
      sirenFreq -= SIREN_FREQ_STEP;  // Decrease by 30 Hz
      if (sirenFreq <= SIREN_MIN_FREQ) {
        sirenFreq = SIREN_MIN_FREQ;
        sirenIncreasing = true;      // Start going up
      }
    }
    ledcWriteTone(BUZZER_PIN, sirenFreq);  // Update buzzer frequency
  }
}

// CALLED BY: handleClient() when /siren endpoint is requested
// Toggles siren on/off
// CALLS: startSiren() to turn on, stopSiren() to turn off
void SirenAlarm() {
  if (sirenActive) {
    stopSiren();   // CALLS: stopSiren() if already active
  } else {
    startSiren();  // CALLS: startSiren() if currently off
  }
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================
// These are high-level motor control functions
// CALLED BY: handleClient() when web interface buttons are pressed

// CALLED BY: handleClient() when /forward endpoint is requested
// Moves robot forward at fixed speed
void motorForward(uint8_t speed) {
  // Left motor forward (IN1=HIGH, IN2=LOW)
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, HIGH);  // Enable motor at full speed

  // Right motor forward (IN1=HIGH, IN2=LOW)
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, HIGH);  // Enable motor at full speed
}

// CALLED BY: handleClient() when /backward endpoint is requested
// Moves robot backward at fixed speed
void motorBackward(uint8_t speed) {
  // Left motor backward (IN1=LOW, IN2=HIGH)
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_EN, HIGH);  // Enable motor at full speed

  // Right motor backward (IN1=LOW, IN2=HIGH)
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_EN, HIGH);  // Enable motor at full speed
}

// CALLED BY: handleClient() when /stop endpoint is requested, or loop() when obstacle detected
// Stops both motors completely
void motorStop() {
  // Left motor stop (both inputs LOW, enable LOW)
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, LOW);
  
  // Right motor stop (both inputs LOW, enable LOW)
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, LOW);
}

// CALLED BY: handleClient() when /left endpoint is requested
// Turns robot left (right motor only)
void motorLeft(uint8_t speed) {
  // Left motor stop
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, LOW);    // Left motor disabled

  // Right motor forward - creates left turn
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, HIGH);  // Right motor at full speed
}

// CALLED BY: handleClient() when /right endpoint is requested
// Turns robot right (left motor only)
void motorRight(uint8_t speed) {
  // Left motor forward - creates right turn
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_EN, HIGH);   // Left motor at full speed

  // Right motor stop
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_EN, LOW);   // Right motor disabled
}

// CALLED BY: loop() when joystickMode is enabled
// Maps virtual joystick X/Y values to differential motor speeds
// Joystick control function - implements tank/differential drive with gradual turning
void handleJoystickControl(int joyX, int joyY) {
  // INPUT: joyX and joyY are in range -100 to 100 from web interface
  //   joyY: Forward/Backward (positive = forward, negative = backward)
  //   joyX: Left/Right turning (positive = turn right, negative = turn left)
  
  // Deadzone to prevent drift from small joystick movements (5% threshold)
  const int DEADZONE = 5;
  if (abs(joyX) < DEADZONE) joyX = 0;
  if (abs(joyY) < DEADZONE) joyY = 0;
  
  // Scale joystick inputs to motor PWM range (0-255)
  // Using 2.55 multiplier to convert -100..100 range to -255..255
  float scaledY = joyY * 2.55;
  
  // Reduce turning sensitivity for more gradual turns (50% of forward speed)
  // This makes slight joystick movements produce gentle curves
  float scaledX = joyX * 1.275;  // 50% turning rate (half of 2.55)
  
  // Calculate differential drive motor speeds
  // Left motor: forward speed minus turning (turning right reduces left speed)
  // Right motor: forward speed plus turning (turning right increases right speed)
  targetLeftSpeed = (int)(scaledY - scaledX);
  targetRightSpeed = (int)(scaledY + scaledX);

  // Limit motor speeds to valid PWM range [-255, 255]
  targetLeftSpeed = constrain(targetLeftSpeed, -255, 255);
  targetRightSpeed = constrain(targetRightSpeed, -255, 255);

  // Apply minimum speed threshold to overcome motor friction (25 PWM)
  const int MIN_SPEED = 25;
  if (abs(targetLeftSpeed) > 0 && abs(targetLeftSpeed) < MIN_SPEED) {
    targetLeftSpeed = (targetLeftSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
  }
  if (abs(targetRightSpeed) > 0 && abs(targetRightSpeed) < MIN_SPEED) {
    targetRightSpeed = (targetRightSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
  }

  // Smooth acceleration/deceleration - gradually move current speed toward target
  // This prevents jerky movements by limiting speed changes per update cycle
  if (currentLeftSpeed < targetLeftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + ACCEL_RATE, targetLeftSpeed);
  } else if (currentLeftSpeed > targetLeftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - ACCEL_RATE, targetLeftSpeed);
  }
  
  if (currentRightSpeed < targetRightSpeed) {
    currentRightSpeed = min(currentRightSpeed + ACCEL_RATE, targetRightSpeed);
  } else if (currentRightSpeed > targetRightSpeed) {
    currentRightSpeed = max(currentRightSpeed - ACCEL_RATE, targetRightSpeed);
  }

  // Apply smoothed speeds to motors
  if (currentLeftSpeed == 0 && currentRightSpeed == 0) {
    motorStop();  // CALLS: motorStop() when both motors at zero
  } else {
    // Left motor
    if (currentLeftSpeed > 0) {
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
    } else if (currentLeftSpeed < 0) {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
    } else {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, LOW);
    }
    analogWrite(LEFT_EN, abs(currentLeftSpeed));

    // Right motor
    if (currentRightSpeed > 0) {
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
    } else if (currentRightSpeed < 0) {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
    } else {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, LOW);
    }
    analogWrite(RIGHT_EN, abs(currentRightSpeed));
  }
}

// ============================================================================
// WEB SERVER & CLIENT HANDLING
// ============================================================================

// CALLED BY: loop() when client connects to web server
// Processes HTTP requests from web browser and calls appropriate functions
// CALLS: Many functions based on URL endpoint requested
void handleClient(WiFiClient client) {
  if (!client) return;  // Exit if no valid client

  String requestLine = "";   // First line of request (contains URL/endpoint)
  
  // Read only the first line of the HTTP request (non-blocking timeout)
  unsigned long startTime = millis();
  const int TIMEOUT_MS = 100;  // 100ms timeout for reading request
  
  while (client.available() && requestLine.indexOf('\n') == -1) {
    if (millis() - startTime > TIMEOUT_MS) break;  // Timeout to prevent hanging
    
    char c = client.read();
    if (c == '\n') {
      break;  // Got complete first line
    } else if (c != '\r') {
      requestLine += c;  // Build first line character by character
    }
  }
  
  // Drain remaining data from client to clear the buffer
  while (client.available()) {
    client.read();
  }

  String responseBody = "";  // HTML response to send back to browser

  // ===== PARSE URL ENDPOINT AND CALL APPROPRIATE FUNCTIONS =====
  
  // ENDPOINT: /buzzer - Test buzzer sound
  if (requestLine.indexOf("GET /buzzer") == 0) {
    BuzzerTest();  // CALLS: BuzzerTest() to play beep
    responseBody = "<html><body><h1>Buzzer triggered</h1></body></html>";
    
  // ENDPOINT: /siren - Toggle siren effect
  } else if (requestLine.indexOf("GET /siren") == 0) {
    SirenAlarm();  // CALLS: SirenAlarm() which calls startSiren() or stopSiren()
    responseBody = "<html><body><h1>Siren activated</h1></body></html>";
    
  // ENDPOINT: /front - Test front lamps
  } else if (requestLine.indexOf("GET /front") == 0) {
    FrontLampTest();  // CALLS: FrontLampTest() to blink front lamps
    responseBody = "<html><body><h1>Front lamp toggled</h1></body></html>";
    
  // ENDPOINT: /rear - Test rear lamps
  } else if (requestLine.indexOf("GET /rear") == 0) {
    RearLampTest();  // CALLS: RearLampTest() to blink rear lamps
    responseBody = "<html><body><h1>Rear lamp toggled</h1></body></html>";
    
  // ENDPOINT: /forward - Move robot forward
  } else if (requestLine.indexOf("GET /forward") == 0) {
    motorForward(200);  // CALLS: motorForward() to move forward
    responseBody = "<html><body><h1>Motors Forward</h1></body></html>";
    
  // ENDPOINT: /backward - Move robot backward
  } else if (requestLine.indexOf("GET /backward") == 0) {
    motorBackward(200);  // CALLS: motorBackward() to move backward
    responseBody = "<html><body><h1>Motors Backward</h1></body></html>";
    
  // ENDPOINT: /left - Turn robot left
  } else if (requestLine.indexOf("GET /left") == 0) {
    motorLeft(200);  // CALLS: motorLeft() to turn left
    responseBody = "<html><body><h1>Turn Left</h1></body></html>";
    
  // ENDPOINT: /right - Turn robot right
  } else if (requestLine.indexOf("GET /right") == 0) {
    motorRight(200);  // CALLS: motorRight() to turn right
    responseBody = "<html><body><h1>Turn Right</h1></body></html>";
    
  // ENDPOINT: /stop - Stop all motors
  } else if (requestLine.indexOf("GET /stop") == 0) {
    motorStop();  // CALLS: motorStop() to halt motors
    responseBody = "<html><body><h1>Motors Stopped</h1></body></html>";
    
  // ENDPOINT: /linefollow/start - Enable line following mode
  } else if (requestLine.indexOf("GET /linefollow/start") == 0) {
    lineFollowing = true;   // Set flag, loop() will call runLineFollow()
    joystickMode = false;   // Disable joystick when line following starts
    // Reset smoothing variables
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    // Reset sweep state variables
    sweepDuration = 7;
    sweepingLeft = true;
    inSweep = false;
    responseBody = "<html><body><h1>Line follow started</h1></body></html>";
    
  // ENDPOINT: /linefollow/stop - Disable line following mode
  } else if (requestLine.indexOf("GET /linefollow/stop") == 0) {
    lineFollowing = false;  // Clear flag, loop() stops calling runLineFollow()
    joystickMode = true;    // Re-enable joystick mode
    // Ensure both motors are fully reset
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(LEFT_EN, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    digitalWrite(RIGHT_EN, LOW);
    // Reset smoothing variables
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    // Reset sweep state variables
    sweepDuration = 7;
    sweepingLeft = true;
    inSweep = false;
    responseBody = "<html><body><h1>Line follow stopped</h1></body></html>";
    
  // ENDPOINT: /linefollow/toggle - Toggle line following on/off
  } else if (requestLine.indexOf("GET /linefollow/toggle") == 0) {
    lineFollowing = !lineFollowing;  // Toggle the flag
    joystickMode = !lineFollowing;   // Joystick mode opposite of line following
    if (!lineFollowing) {
      // Ensure both motors are fully reset when stopping
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, LOW);
      digitalWrite(LEFT_EN, LOW);
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, LOW);
      digitalWrite(RIGHT_EN, LOW);
    }
    // Reset smoothing variables
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    // Reset sweep state variables
    sweepDuration = 7;
    sweepingLeft = true;
    inSweep = false;
    responseBody = String("<html><body><h1>Line follow ") + (lineFollowing ? "started" : "stopped") + "</h1></body></html>";
    
  // ENDPOINT: /linefollow/status - Get current sensor reading and state
  } else if (requestLine.indexOf("GET /linefollow/status") == 0) {
    responseBody = String("<html><body><h1>Line Status</h1><p>Sensor=") + String(sensorReading) + "</p><p>State=" + (lineFollowing ? "ON" : "OFF") + "</p></body></html>";
    
  // ENDPOINT: /status - Get all status values as JSON for dynamic updates
  } else if (requestLine.indexOf("GET /status") == 0) {
    // Send JSON response directly with minimal overhead
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    
    // Build and send JSON in one go
    client.print("{\"sensor\":");
    client.print(sensorReading);
    client.print(",\"lineFollowing\":");
    client.print(lineFollowing ? "true" : "false");
    client.print(",\"joystickMode\":");
    client.print(joystickMode ? "true" : "false");
    client.print(",\"distanceCm\":");
    client.print(distanceCm);
    client.print(",\"leftEncoder\":");
    client.print(leftEncoderCount);
    client.print(",\"rightEncoder\":");
    client.println(rightEncoderCount);
    client.println("}");
    
    client.stop();
    return;
    
  // ENDPOINT: /joystick/toggle - Toggle joystick control mode
  } else if (requestLine.indexOf("GET /joystick/toggle") == 0) {
    joystickMode = !joystickMode;    // Toggle joystick mode flag
    if (!joystickMode) motorStop();  // CALLS: motorStop() when disabling
    responseBody = String("<html><body><h1>Joystick Control ") + (joystickMode ? "Enabled" : "Disabled") + "</h1></body></html>";
    
  // ENDPOINT: /joystick/move?x=___&y=___ - Update joystick position
  // Called continuously by web interface JavaScript to send joystick values
  // These values are used by handleJoystickControl() in loop()
  } else if (requestLine.indexOf("GET /joystick/move?x=") == 0) {
    // Parse joystick X value from URL query string
    int xStart = requestLine.indexOf("x=") + 2;
    int xEnd = requestLine.indexOf("&", xStart);
    if (xEnd == -1) xEnd = requestLine.indexOf(" ", xStart);
    String xStr = requestLine.substring(xStart, xEnd);
    joystickX = xStr.toInt();  // Update global joystickX variable
    
    // Parse joystick Y value from URL query string
    int yStart = requestLine.indexOf("y=") + 2;
    int yEnd = requestLine.indexOf(" ", yStart);
    String yStr = requestLine.substring(yStart, yEnd);
    joystickY = yStr.toInt();  // Update global joystickY variable
    
    // Send minimal JSON response for fast acknowledgement
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.print("{\"x\":");
    client.print(joystickX);
    client.print(",\"y\":");
    client.print(joystickY);
    client.println("}");
    
    client.stop();
    return;
    
  // DEFAULT: No specific endpoint - send main web interface HTML page
  // This massive HTML page contains the interactive control panel with:
  //   - System info display (encoder counts, sensor readings)
  //   - Buzzer & Lights control buttons
  //   - Line following toggle
  //   - Virtual joystick canvas with JavaScript
  //   - Ultrasonic sensor display
  } else {
    // Build complete HTML page for web interface
    responseBody = "<!DOCTYPE html>";
    responseBody += "<html><head><title>ESP32 Control</title>";
    
    // CSS Styles for web interface
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
    
    // System Information Display
    responseBody += "<div class='info'>";
    responseBody += "<p><strong>AP IP:</strong> ";
    responseBody += WiFi.softAPIP().toString();  // Show ESP32's IP address
    responseBody += "</p>";
    responseBody += "<p><strong>Left Encoder:</strong> <span id='leftEnc'>";
    responseBody += leftEncoderCount;  // Display left wheel encoder count
    responseBody += "</span></p>";
    responseBody += "<p><strong>Right Encoder:</strong> <span id='rightEnc'>";
    responseBody += rightEncoderCount;  // Display right wheel encoder count
    responseBody += "</span></p>";
    responseBody += "<p><strong>Joystick Mode:</strong> <span id='joyMode'>";
    responseBody += (joystickMode ? "ON" : "OFF");
    responseBody += "</span></p>";
    responseBody += "</div>";
    
    // Buzzer & Lights Control Section
    // Buttons call cmd() JavaScript function which makes GET requests
    responseBody += "<div class='section'><h2>Buzzer & Lights</h2>";
    // Buttons trigger cmd() JavaScript function to make HTTP requests
    responseBody += "<button onclick='cmd(\"/buzzer\")'>Buzzer</button>";
    responseBody += "<button onclick='cmd(\"/siren\")' style='background: #ff4444;'>Siren</button>";
    responseBody += "<button onclick='cmd(\"/front\")'>Front Lamp</button>";
    responseBody += "<button onclick='cmd(\"/rear\")'>Rear Lamp</button>";
    responseBody += "</div>";
    
    // Line Follow Control Section
    responseBody += "<div class='section'><h2>Line Follow</h2>";
    responseBody += "<p>Phototransistor: <span id='sensorVal'>";
    responseBody += String(sensorReading);  // Display current sensor reading
    responseBody += "</span></p>";
    responseBody += "<p>Status: <span id='lineStatus'>";
    responseBody += (lineFollowing ? "ON" : "OFF");  // Show if line following is active
    responseBody += "</span></p>";
    responseBody += "<button onclick='cmd(\"/linefollow/toggle\")'> Toggle Line Follow</button>";
    responseBody += "</div>";
    
    // Virtual Joystick Control Section
    // HTML5 Canvas element for drawing interactive joystick
    responseBody += "<div class='section'><h2>Virtual Joystick</h2>";
    responseBody += "<canvas id='joystick' width='200' height='200' style='border:2px solid #ccc; border-radius:5px; background:#f9f9f9; cursor:crosshair; margin-top:10px;'></canvas>";
    responseBody += "<p>X: <span id='joyX'>0</span> | Y: <span id='joyY'>0</span></p>";
    responseBody += "</div>";
    
    // Status message display area (hidden by default)
    responseBody += "<div id='status' class='status' style='display:none;'></div>";
    
    // ===== JAVASCRIPT CODE FOR WEB INTERFACE =====
    responseBody += "<script>";
    
    // Joystick canvas and state variables
    responseBody += "let canvas = document.getElementById('joystick');";
    responseBody += "let ctx = canvas.getContext('2d');";
    responseBody += "let joyX = 0, joyY = 0;";  // Current joystick position
    responseBody += "let isMouseDown = false;";   // Track mouse button state
    responseBody += "let touchId = null;";        // Track touch ID for mobile
    responseBody += "";
    
    // Function: drawJoystick() - Renders joystick visual on canvas
    // CALLED BY: Mouse/touch event handlers, sendJoystickUpdate()
    responseBody += "function drawJoystick() {";
    responseBody += "  ctx.fillStyle = '#f9f9f9';";
    responseBody += "  ctx.fillRect(0, 0, 200, 200);";  // Clear canvas
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
    // Draw crosshair guides and joystick knob
    responseBody += "  let posX = 100 + (joyX / 100) * 70;";   // Convert X to pixel position
    responseBody += "  let posY = 100 - (joyY / 100) * 70;";   // Convert Y to pixel position (inverted)
    responseBody += "  ctx.fillStyle = joyX == 0 && joyY == 0 ? '#ccc' : '#007bff';";  // Gray when centered, blue when moved
    responseBody += "  ctx.beginPath();";
    responseBody += "  ctx.arc(posX, posY, 15, 0, Math.PI * 2);";  // Draw joystick knob circle
    responseBody += "  ctx.fill();";
    responseBody += "  document.getElementById('joyX').textContent = joyX;";  // Update X display
    responseBody += "  document.getElementById('joyY').textContent = joyY;";  // Update Y display
    responseBody += "}";
    responseBody += "";
    
    // Function: handleJoystickMove(x, y) - Processes mouse/touch movement
    // CALLED BY: Mouse/touch event listeners
    // CALLS: sendJoystickUpdate(), drawJoystick()
    responseBody += "function handleJoystickMove(x, y) {";
    responseBody += "  let rect = canvas.getBoundingClientRect();";  // Get canvas position on screen
    responseBody += "  let centerX = rect.width / 2;";               // Find center point
    responseBody += "  let centerY = rect.height / 2;";
    responseBody += "  let dx = x - rect.left - centerX;";           // Calculate distance from center
    responseBody += "  let dy = y - rect.top - centerY;";
    responseBody += "  let distance = Math.sqrt(dx * dx + dy * dy);";  // Distance from center
    responseBody += "  let maxDistance = 70;";                       // Max joystick radius in pixels
    responseBody += "  if (distance > maxDistance) {";               // Limit to circle boundary
    responseBody += "    dx = (dx / distance) * maxDistance;";       // Constrain to max radius
    responseBody += "    dy = (dy / distance) * maxDistance;";
    responseBody += "  }";
    responseBody += "  joyX = Math.round((dx / 70) * 100);";        // Convert to -100 to 100 range
    responseBody += "  joyY = Math.round((dy / 70) * -100);";       // Invert Y (canvas Y is downward)
    responseBody += "  joyX = Math.max(-100, Math.min(100, joyX));";  // Clamp to range
    responseBody += "  joyY = Math.max(-100, Math.min(100, joyY));";
    responseBody += "  sendJoystickUpdate();";  // CALLS: sendJoystickUpdate() to send to ESP32
    responseBody += "  drawJoystick();";        // CALLS: drawJoystick() to redraw visual
    responseBody += "}";
    responseBody += "";
    
    // Function: sendJoystickUpdate() - Sends joystick values to ESP32
    // CALLED BY: handleJoystickMove(), mouse/touch event handlers
    // Makes HTTP GET request to /joystick/move endpoint
    responseBody += "function sendJoystickUpdate() {";
    responseBody += "  fetch('/joystick/move?x=' + joyX + '&y=' + joyY).catch(e => console.log('Error:', e));";
    responseBody += "}";
    responseBody += "";
    
    // Mouse event handlers for desktop interaction
    responseBody += "canvas.addEventListener('mousedown', () => isMouseDown = true);";
    responseBody += "canvas.addEventListener('mousemove', (e) => {";
    responseBody += "  if (isMouseDown) handleJoystickMove(e.clientX, e.clientY);";  // CALLS: handleJoystickMove()
    responseBody += "});";
    responseBody += "canvas.addEventListener('mouseup', () => {";
    responseBody += "  isMouseDown = false;";
    responseBody += "  joyX = 0; joyY = 0;";  // Reset to center when released
    responseBody += "  sendJoystickUpdate();";  // CALLS: sendJoystickUpdate() to send center position
    responseBody += "  drawJoystick();";        // CALLS: drawJoystick() to redraw
    responseBody += "});";
    responseBody += "canvas.addEventListener('mouseleave', () => {";
    responseBody += "  isMouseDown = false;";
    responseBody += "  joyX = 0; joyY = 0;";  // Reset when mouse leaves canvas
    responseBody += "  sendJoystickUpdate();";  // CALLS: sendJoystickUpdate()
    responseBody += "  drawJoystick();";        // CALLS: drawJoystick()
    responseBody += "});";
    responseBody += "";
    
    // Touch event handlers for mobile/tablet interaction
    responseBody += "canvas.addEventListener('touchstart', (e) => { touchId = e.touches[0].identifier; });";
    responseBody += "canvas.addEventListener('touchmove', (e) => {";
    responseBody += "  for (let t of e.touches) {";
    responseBody += "    if (t.identifier === touchId) {";
    responseBody += "      handleJoystickMove(t.clientX, t.clientY);";  // CALLS: handleJoystickMove()
    responseBody += "      break;";
    responseBody += "    }";
    responseBody += "  }";
    responseBody += "});";
    responseBody += "canvas.addEventListener('touchend', (e) => {";
    responseBody += "  joyX = 0; joyY = 0;";  // Reset to center when touch released
    responseBody += "  sendJoystickUpdate();";  // CALLS: sendJoystickUpdate()
    responseBody += "  drawJoystick();";        // CALLS: drawJoystick()
    responseBody += "  touchId = null;";
    responseBody += "});";
    responseBody += "";
    
    // Initialize joystick display
    responseBody += "drawJoystick();";  // CALLS: drawJoystick() on page load
    responseBody += "";
    
    // Auto-refresh status values every 2 seconds
    responseBody += "setInterval(() => {";
    responseBody += "  fetch('/status')";
    responseBody += "    .then(r => r.json())";
    responseBody += "    .then(data => {";
    responseBody += "      document.getElementById('sensorVal').textContent = data.sensor;";  
    responseBody += "      document.getElementById('lineStatus').textContent = data.lineFollowing ? 'ON' : 'OFF';";  
    responseBody += "      document.getElementById('joyMode').textContent = data.joystickMode ? 'ON' : 'OFF';";  
    responseBody += "      document.getElementById('leftEnc').textContent = data.leftEncoder;";  
    responseBody += "      document.getElementById('rightEnc').textContent = data.rightEncoder;";  
    responseBody += "      let distText = data.distanceCm > 0 ? data.distanceCm + ' cm' : 'No object detected';";  
    responseBody += "      if (data.distanceCm > 0 && data.distanceCm < " + String(OBSTACLE_DISTANCE_CM) + ") distText += ' <strong style=\\\"color: red;\\\">(OBSTACLE!)</strong>';";  
    responseBody += "      document.getElementById('distanceDisplay').innerHTML = 'Distance: ' + distText;";  
    responseBody += "    })";
    responseBody += "    .catch(e => console.log('Status update error:', e));";  
    responseBody += "}, 500);";  // Update every 500ms (half second)
    responseBody += "";
    
    // Function: cmd(path) - Makes HTTP GET request to ESP32 endpoint
    // CALLED BY: Button onclick handlers in HTML
    // This function sends commands from buttons to ESP32
    responseBody += "function cmd(path) {";
    responseBody += "  fetch(path).then(r => r.text()).then(d => {";  // Make HTTP request
    responseBody += "    let st = document.getElementById('status');";
    responseBody += "    st.style.display = 'block';";
    responseBody += "    st.innerHTML = '<strong>OK:</strong> ' + path;";  // Show success message
    responseBody += "    setTimeout(() => st.style.display = 'none', 2000);";  // Hide after 2 seconds
    responseBody += "  }).catch(e => {";
    responseBody += "    document.getElementById('status').innerHTML = '<strong>Error:</strong> ' + e;";
    responseBody += "    document.getElementById('status').style.display = 'block';";
    responseBody += "  });";
    responseBody += "}";
    responseBody += "</script>";
    
    // Ultrasonic Sensor Display Section
    responseBody += "<div class='section'><h2>Ultrasonic Sensor</h2>";
    responseBody += "<p id='distanceDisplay'>Distance: ";
    if (distanceCm > 0) {
      responseBody += String(distanceCm);  // Show measured distance
      responseBody += " cm";
      if (distanceCm < OBSTACLE_DISTANCE_CM) {
        responseBody += " <strong style='color: red;'>(OBSTACLE!)</strong>";  // Warning if too close
      }
    } else {
      responseBody += "No object detected";  // No valid reading
    }
    responseBody += "</p>";
    responseBody += "</div>";
    
    responseBody += "</body></html>";  // Close HTML page
  }

  // Send HTTP response back to browser
  // CALLS: sendHttpResponse() to send the HTML/JSON response
  sendHttpResponse(client, responseBody);
  delay(1);         // Brief delay for stability
  client.stop();    // Close client connection
}

// ============================================================================
// MAIN LOOP - CALLED REPEATEDLY: Runs continuously after setup() completes
// ============================================================================
void loop() {
  unsigned long now = millis();
  
  // CALLS: updateSiren() - Update siren effect if active (non-blocking)
  updateSiren();
  
  // CALLS: updateBuzzer() - Update buzzer state if active (non-blocking)
  updateBuzzer();
  
  // CALLS: updateLamps() - Update lamp states if active (non-blocking)
  updateLamps();
  
  // Rate-limited sensor reading and control logic (every READ_INTERVAL_MS = 20ms)
  if (now - lastLineRead >= READ_INTERVAL_MS) {
    lastLineRead = now;
    
    // CALLS: readUltrasonicDistance() - Check for obstacles
    distanceCm = readUltrasonicDistance();
    
    // PRIORITY 1: Emergency stop if obstacle detected
    // CALLS: motorStop() to halt all movement
    if (distanceCm > 0 && distanceCm < OBSTACLE_DISTANCE_CM) {
      motorStop();           // CALLS: motorStop() for safety
      lineFollowing = false; // Disable line following mode
      
    // PRIORITY 2: Joystick control mode (default mode)
    } else if (joystickMode) {
      // CALLS: handleJoystickControl() to process virtual joystick input
      // Uses joystickX and joystickY values set by web interface
      handleJoystickControl(joystickX, joystickY);
      
    // PRIORITY 3: Line following mode (when enabled via web interface)
    } else if (lineFollowing) {
      // CALLS: runLineFollow() to execute line following algorithm
      runLineFollow();
      
    // PRIORITY 4: Idle mode - just update sensor reading for web UI
    } else {
      // CALLS: readAverage() - Keep sensor reading fresh for web interface display
      sensorReading = readAverage(PHOTOTRANSISTOR_PIN, 2);
    }
  }

  // CALLS: handleClient() - Check for and process incoming web requests
  WiFiClient client = server.available();  // Check if client connected
  if (client) {
    handleClient(client);  // CALLS: handleClient() to process HTTP request
  }
  
  delay(1);  // Small delay to prevent watchdog timer reset
}
