/*
 * Mecanum Robot Motor Control - Arduino Mega 2560
 *
 * Receives wheel speed commands from RPi 4b over serial.
 * Drives 4 motors via 4x BTS7960 H-bridge drivers.
 *
 * Serial Protocol:
 *   Command IN:  "M <fl> <fr> <rl> <rr>\n"  (-255 to 255)
 *   Response OUT: "OK <fl> <fr> <rl> <rr>\n"
 *   Stop:         "S\n" = emergency stop
 *   Ping:         "P\n" = responds "PONG\n"
 *   Encoder OUT:  "E <fl> <fr> <rl> <rr>\n"  (cumulative ticks)
 *
 * BTS7960 Driver Wiring - ADJUST TO YOUR WIRING:
 * ─────────────────────────────────────────────────────────
 *  Motor          RPWM(fwd)  LPWM(rev)  R_EN  L_EN
 * ─────────────────────────────────────────────────────────
 *  Front-Left      2          3          4     4  (tied)
 *  Front-Right     5          6          7     7  (tied)
 *  Rear-Left       8          9         10    10  (tied)
 *  Rear-Right     11         12         13    13  (tied)
 * ─────────────────────────────────────────────────────────
 *
 * BTS7960 logic:
 *   Forward:  RPWM = speed (0-255), LPWM = 0
 *   Reverse:  RPWM = 0,            LPWM = speed (0-255)
 *   Brake:    RPWM = 0,            LPWM = 0
 *   R_EN and L_EN must be HIGH to enable the driver.
 *   They can be tied together to a single enable pin per driver.
 *
 * ENCODER SUPPORT:
 *   The Arduino Mega has 6 external interrupt pins (2, 3, 18, 19, 20, 21)
 *   and plenty of digital pins.  With BTS7960 using 3 pins per motor
 *   (12 total for 4 motors), the Mega has ample free pins for 4 quadrature
 *   encoders (8 pins: 4 channel-A on interrupt pins, 4 channel-B on any
 *   digital pin).
 *
 *   Encoder pins (adjust to your wiring):
 *     FL: ChA = 18 (INT5), ChB = 22
 *     FR: ChA = 19 (INT4), ChB = 23
 *     RL: ChA = 20 (INT3), ChB = 24
 *     RR: ChA = 21 (INT2), ChB = 25
 *
 *   When USE_ENCODERS is defined, the sketch reads quadrature encoders
 *   and sends cumulative tick counts at ENCODER_REPORT_MS intervals
 *   using the "E <fl> <fr> <rl> <rr>\n" protocol.
 */

// ── Uncomment to enable encoder reading on the Mega ──────────────────
// #define USE_ENCODERS

// ── BTS7960 Pin Definitions ──────────────────────────────────────────
// Front-Left Motor
#define FL_RPWM  2
#define FL_LPWM  3
#define FL_EN    4

// Front-Right Motor
#define FR_RPWM  5
#define FR_LPWM  6
#define FR_EN    7

// Rear-Left Motor
#define RL_RPWM  8
#define RL_LPWM  9
#define RL_EN   10

// Rear-Right Motor
#define RR_RPWM 11
#define RR_LPWM 12
#define RR_EN   13

// ── Encoder Pin Definitions (Mega interrupt-capable pins) ────────────
#ifdef USE_ENCODERS
#define FL_ENC_A  18   // INT5
#define FL_ENC_B  22
#define FR_ENC_A  19   // INT4
#define FR_ENC_B  23
#define RL_ENC_A  20   // INT3
#define RL_ENC_B  24
#define RR_ENC_A  21   // INT2
#define RR_ENC_B  25

volatile long encFL = 0, encFR = 0, encRL = 0, encRR = 0;

#define ENCODER_REPORT_MS  50   // Report ticks every 50ms (20 Hz)
unsigned long lastEncoderReport = 0;
#endif

#define CMD_TIMEOUT_MS  1000
#define SERIAL_BAUD     115200

unsigned long lastCmdTime = 0;
String inputBuffer = "";

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Motor driver pins
  int motorPins[] = {FL_RPWM, FL_LPWM, FL_EN,
                     FR_RPWM, FR_LPWM, FR_EN,
                     RL_RPWM, RL_LPWM, RL_EN,
                     RR_RPWM, RR_LPWM, RR_EN};

  for (int i = 0; i < 12; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  // Enable all BTS7960 drivers (R_EN and L_EN tied together)
  digitalWrite(FL_EN, HIGH);
  digitalWrite(FR_EN, HIGH);
  digitalWrite(RL_EN, HIGH);
  digitalWrite(RR_EN, HIGH);

#ifdef USE_ENCODERS
  // Encoder channel A pins (interrupt-driven)
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP);
  pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP);
  pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP);
  pinMode(RR_ENC_B, INPUT_PULLUP);

  // CHANGE on Channel A for 2x quadrature decoding.
  // A 600 PPR encoder yields 1200 counts/rev → set encoder_cpr=1200
  // in mecanum_params.yaml.
  //
  // NOTE: True 4x decoding requires CHANGE interrupts on BOTH channels.
  // The Mega has 6 interrupt pins (2, 3, 18, 19, 20, 21).  Pins 18-21
  // are used for Channel A; pins 2-3 are used by the FL BTS7960 driver.
  // If you remap FL_RPWM/FL_LPWM off pins 2-3 (e.g. to pins 44-45),
  // you can attach Channel B interrupts on pins 2-3 for FL/FR encoders,
  // achieving 4x on two motors.  For full 4x on all four, use the RPi
  // pigpio encoder_node instead (it has unlimited GPIO interrupts).
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR_A, CHANGE);
#endif

  stopAllMotors();
  Serial.println("READY");
}

void loop() {
  // ── Parse serial commands ──────────────────────────────────────────
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

  // ── Watchdog: stop motors if no command received ───────────────────
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopAllMotors();
  }

#ifdef USE_ENCODERS
  // ── Report encoder ticks at fixed interval ─────────────────────────
  if (millis() - lastEncoderReport >= ENCODER_REPORT_MS) {
    lastEncoderReport = millis();
    noInterrupts();
    long fl = encFL, fr = encFR, rl = encRL, rr = encRR;
    interrupts();
    Serial.print("E ");
    Serial.print(fl); Serial.print(" ");
    Serial.print(fr); Serial.print(" ");
    Serial.print(rl); Serial.print(" ");
    Serial.println(rr);
  }
#endif
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

      setMotor(FL_RPWM, FL_LPWM, fl);
      setMotor(FR_RPWM, FR_LPWM, fr);
      setMotor(RL_RPWM, RL_LPWM, rl);
      setMotor(RR_RPWM, RR_LPWM, rr);

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

/*
 * BTS7960 motor control:
 *   speed > 0  → forward: RPWM = speed, LPWM = 0
 *   speed < 0  → reverse: RPWM = 0,     LPWM = |speed|
 *   speed == 0 → brake:   RPWM = 0,     LPWM = 0
 */
void setMotor(int rpwmPin, int lpwmPin, int speed) {
  if (speed > 0) {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  } else if (speed < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void stopAllMotors() {
  setMotor(FL_RPWM, FL_LPWM, 0);
  setMotor(FR_RPWM, FR_LPWM, 0);
  setMotor(RL_RPWM, RL_LPWM, 0);
  setMotor(RR_RPWM, RR_LPWM, 0);
}

// ── Encoder ISRs — 2x quadrature decoding (CHANGE on Channel A) ─────
// On each Channel-A edge, read Channel B to determine direction.
// CHANGE doubles the count vs RISING: 600 PPR → 1200 counts/rev.
#ifdef USE_ENCODERS
void isrFL_A() { encFL += digitalRead(FL_ENC_B) ? -1 : 1; }
void isrFR_A() { encFR += digitalRead(FR_ENC_B) ? -1 : 1; }
void isrRL_A() { encRL += digitalRead(RL_ENC_B) ? -1 : 1; }
void isrRR_A() { encRR += digitalRead(RR_ENC_B) ? -1 : 1; }
#endif
