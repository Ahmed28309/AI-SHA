/*
 * Mecanum Robot Motor Control - Arduino Uno
 * 
 * Receives wheel speed commands from Pi4 over serial.
 * Drives 4 motors via 2x L293D motor drivers.
 * 
 * Serial Protocol:
 *   Command IN:  "M <fl> <fr> <rl> <rr>\n"  (-255 to 255)
 *   Response OUT: "OK <fl> <fr> <rl> <rr>\n"
 *   Stop:         "S\n" = emergency stop
 *   Ping:         "P\n" = responds "PONG\n"
 * 
 * Pin Mapping (2x L293D) - ADJUST TO YOUR WIRING:
 * ─────────────────────────────────────────────
 *  Motor          L293D    EN(PWM)  IN1   IN2
 * ─────────────────────────────────────────────
 *  Front-Left     -        6        3     2
 *  Front-Right    -        10       9     8
 *  Rear-Left      -        11       12    13
 *  Rear-Right     -        5        4     7
 * ─────────────────────────────────────────────
 */

// Front-Left Motor (Top-Left)
#define FL_EN   6
#define FL_IN1  3
#define FL_IN2  2

// Front-Right Motor (L293D #1, Channel B)
#define FR_EN   10
#define FR_IN1  9
#define FR_IN2  8

// Rear-Left Motor (Bottom-Left)
#define RL_EN   11
#define RL_IN1  12
#define RL_IN2  13

// Rear-Right Motor (Bottom-Right)
#define RR_EN   5
#define RR_IN1  4
#define RR_IN2  7

#define CMD_TIMEOUT_MS  1000
#define SERIAL_BAUD     115200

unsigned long lastCmdTime = 0;
String inputBuffer = "";

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  int pins[] = {FL_EN, FL_IN1, FL_IN2,
                FR_EN, FR_IN1, FR_IN2,
                RL_EN, RL_IN1, RL_IN2,
                RR_EN, RR_IN1, RR_IN2};
  
  for (int i = 0; i < 12; i++) {
    pinMode(pins[i], OUTPUT);
  }
  
  stopAllMotors();
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 64) {
        inputBuffer = "";
      }
    }
  }
  
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopAllMotors();
  }
}

void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.charAt(0) == 'S') {
    stopAllMotors();
    Serial.println("STOPPED");
    lastCmdTime = millis();
    return;
  }
  
  if (cmd.charAt(0) == 'M') {
    int fl, fr, rl, rr;
    int parsed = sscanf(cmd.c_str(), "M %d %d %d %d", &fl, &fr, &rl, &rr);
    
    if (parsed == 4) {
      fl = constrain(fl, -255, 255);
      fr = constrain(fr, -255, 255);
      rl = constrain(rl, -255, 255);
      rr = constrain(rr, -255, 255);
      
      setMotor(FL_EN, FL_IN1, FL_IN2, fl);
      setMotor(FR_EN, FR_IN1, FR_IN2, fr);
      setMotor(RL_EN, RL_IN1, RL_IN2, rl);
      setMotor(RR_EN, RR_IN1, RR_IN2, rr);
      
      lastCmdTime = millis();
      
      Serial.print("OK ");
      Serial.print(fl); Serial.print(" ");
      Serial.print(fr); Serial.print(" ");
      Serial.print(rl); Serial.print(" ");
      Serial.println(rr);
    } else {
      Serial.println("ERR PARSE");
    }
    return;
  }
  
  if (cmd.charAt(0) == 'P') {
    Serial.println("PONG");
    return;
  }
  
  Serial.println("ERR UNKNOWN");
}

void setMotor(int enPin, int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

void stopAllMotors() {
  setMotor(FL_EN, FL_IN1, FL_IN2, 0);
  setMotor(FR_EN, FR_IN1, FR_IN2, 0);
  setMotor(RL_EN, RL_IN1, RL_IN2, 0);
  setMotor(RR_EN, RR_IN1, RR_IN2, 0);
}
