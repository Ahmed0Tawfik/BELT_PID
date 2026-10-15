#include <Servo.h>
#include <avr/pgmspace.h>
// =====================================================
//  CONVEYOR: PID speed control + height sorting  (simple console)
//  Wiring:  D2 <- encoder | D3 -> L293D IN2 | D4 -> IN1 | D5 -> EN1
//           D8 <- HC-SR04 ECHO | D9 -> TRIG | D10 -> Servo | Terminal 9600 baud
//  Load: switch D6 -> GND (closed = load ON) | pot wiper -> A0 | m<n> picks source
//  Terminal commands (type then Enter):  h = help
// =====================================================

// ---------- Forward declarations ----------
void countPulse();
void setGate(byte pos);
void startStep();
void resetPID();
void setLoad(byte pct);
void updateLoad(unsigned long now);

// ---------- Pins ----------
const byte IR_PULSE_PIN = 2;
const byte IN2_PIN = 3;
const byte IN1_PIN = 4;
const byte ENA_PIN = 5;
const byte ECHO_PIN = 8;
const byte TRIG_PIN = 9;
const byte SERVO_PIN = 10;

// ---------- PID ----------
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 150.00;
const double MAX_RPM = 340.0;
const unsigned long SAMPLE_MS = 20;
double pTerm = 0, iTerm = 0, dTerm = 0;

// ---------- Software PWM ----------
const unsigned long SW_PERIOD_US = 10000UL;
byte duty = 0;

// ---------- Simulated belt load ----------
// loadPct = how much of the drive power is "eaten" by the load (0 = empty belt)
byte loadPct = 0;
byte loadSet = 40;                           // load size used by switch / pulse / terminal
byte loadMode = 2;                           // 0 = terminal, 1 = pot on A0, 2 = switch on D6, 3 = auto pulse
const byte SW_PIN = 6;                       // switch to GND (INPUT_PULLUP)
unsigned long lastLoadRead = 0;
unsigned long loadToggleMs = 0;
const unsigned long LOAD_PULSE_MS = 8000;    // pulse mode toggles every 8 s

// ---------- Speed measurement ----------
const float PULSES_PER_REV = 24.0;
const float RPM_FILTER = 0.5;
volatile unsigned long lastPulseUs = 0;
volatile unsigned long periodUs = 0;
double rawRPM = 0.0, currentRPM = 0.0, error = 0.0;
int pwmOutput = 0;

// ---------- Sorting ----------
Servo sortingGate;
const float SENSOR_TO_BELT_DIST = 100.0;
const float HEIGHT_THRESHOLD = 30.0;
const unsigned long ECHO_TIMEOUT_US = 30000;
const unsigned long ULTRA_MS = 100;
const unsigned long DIVERT_HOLD_MS = 1500;
const byte GATE_PASS = 0, GATE_REJECT = 90;
unsigned long lastUltraTime = 0, divertUntil = 0;
byte tallCount = 0, gatePos = 255;
float objectHeight = 0.0, lastDistance = -1.0;
unsigned int rejectCount = 0;

// ---------- Logging settings ----------
const unsigned long PRINT_MS = 500;
const double SETTLE_BAND = 1.0;
const unsigned long SETTLE_HOLD_MS = 2000;
const unsigned long SS_WINDOW_MS = 5000;
bool csvMode = false;
unsigned long lastPrint = 0;

// ---------- Step-response metrics ----------
bool stepActive = false;
unsigned long stepStartMs = 0, inBandSince = 0;
double stepFrom = 0, stepSpan = 0, stepExtreme = 0;
int stepDir = 1;
long t10 = -1, t90 = -1;
double lastRise = -1, lastSettle = -1, lastOver = -1, lastOverPct = -1;

// ---------- Steady-state statistics ----------
unsigned long ssStart = 0;
unsigned int ssN = 0, ssIn = 0;
double ssSumErr = 0, ssMaxAbs = 0, ssSumPwm = 0;

// =====================================================
//  Non-blocking log queue
// =====================================================
const int QSIZE = 320;
char q[QSIZE];
int qHead = 0, qTail = 0, qCount = 0;

void softPWM()
{
  unsigned long t = micros() % SW_PERIOD_US;
  unsigned int eff = ((unsigned int)duty * (100 - loadPct)) / 100;   // power the motor really gets
  unsigned long onTime = (SW_PERIOD_US * (unsigned long)eff) / 255UL;
  digitalWrite(ENA_PIN, t < onTime ? HIGH : LOW);
}

void pumpSerial()
{
  while (qCount > 0 && Serial.availableForWrite() > 0)
  {
    Serial.write(q[qHead]);
    qHead = (qHead + 1) % QSIZE;
    qCount--;
  }
}

void qRaw(char c)
{
  while (qCount >= QSIZE) { softPWM(); pumpSerial(); }
  q[qTail] = c;
  qTail = (qTail + 1) % QSIZE;
  qCount++;
}

void qPut(char c)
{
  if (c == '\n') qRaw('\r');   // terminal needs CR + LF
  qRaw(c);
}

void qStr(const char *s) { while (*s) qPut(*s++); }

void qP(const __FlashStringHelper *f)
{
  const char *p = (const char *)f;
  char c;
  while ((c = pgm_read_byte(p++))) qPut(c);
}

void qNum(double v, int w, int p)
{
  char t[16];
  dtostrf(v, w, p, t);
  qStr(t);
}

void qTime()
{
  qP(F("[")); qNum(millis() / 1000.0, 0, 1); qP(F("s] "));
}

int qFree() { return QSIZE - qCount; }

// =====================================================
//  Banner / help / header
// =====================================================
void printHelp()
{
  qP(F("\nCommands (type then Enter):\n"));
  qP(F("  s<rpm>  set speed   e.g. s120\n"));
  qP(F("  p<val>  set kp      e.g. p0.5\n"));
  qP(F("  i<val>  set ki      e.g. i2\n"));
  qP(F("  d<val>  set kd      e.g. d0.01\n"));
  qP(F("  l<pct>  load size 0-90   e.g. l40\n"));
  qP(F("  m<n>    load source: 0 term, 1 pot, 2 switch, 3 pulse\n"));
  qP(F("  r       report\n"));
  qP(F("  c       toggle CSV / table\n"));
  qP(F("  h       help\n\n"));
}

void printHeader()
{
  qP(F("  t(s)     SP     PV    ERR      P      I      D PWM LD% H(cm) GATE\n"));
}

void printBanner()
{
  qP(F("\n=== CONVEYOR PID SPEED CONTROL + HEIGHT SORTING ===\n"));
  qP(F("Setpoint: ")); qNum(setpoint, 0, 1); qP(F(" RPM (motor max ~")); qNum(MAX_RPM, 0, 0); qP(F(")\n"));
  qP(F("Gains: kp=")); qNum(kp, 0, 3); qP(F(" ki=")); qNum(ki, 0, 3); qP(F(" kd=")); qNum(kd, 0, 3); qPut('\n');
  qP(F("Reject height >= ")); qNum(HEIGHT_THRESHOLD, 0, 1); qP(F(" cm\n"));
  qP(F("Type h for commands.\n"));
}

// =====================================================
//  Setup
// =====================================================
void setup()
{
  //PESSEN'S INTEGRAL RULE
	kp = 23.599;
	ki = 118.945;
	kd = 1.756;

  Serial.begin(9600);

  pinMode(IR_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PULSE_PIN), countPulse, RISING);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  sortingGate.attach(SERVO_PIN, 1000, 2000);
  setGate(GATE_PASS);

  printBanner();
  printHelp();
  printHeader();

  last_time = millis();
  startStep();
}

// =====================================================
//  Main loop
// =====================================================
void loop()
{
  softPWM();
  pumpSerial();
  unsigned long now = millis();

  if (now - last_time >= SAMPLE_MS)
  {
    dt = (now - last_time) / 1000.00;
    last_time = now;

    handleSerial();
    updateRPM();
    double actual = currentRPM;
    error = setpoint - actual;
    output = pid(error);
    setMotor((int)output);

    trackStep(now);
    trackSteady(now);

    if (now - lastPrint >= PRINT_MS && qFree() >= 100)
    {
      lastPrint = now;
      logRow(actual);
    }
  }

  updateLoad(now);
  updateSorting(now);
}

// =====================================================
//  PID
// =====================================================
double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  integral = constrain(integral, 0.0, 255.0 / (ki > 0 ? ki : 1.0));
  double derivative = (dt > 0) ? (error - previous) / dt : 0.0;
  previous = error;

  pTerm = kp * proportional;
  iTerm = ki * integral;
  dTerm = kd * derivative;
  return pTerm + iTerm + dTerm;
}

void resetPID()
{
  integral = 0;
  previous = 0;
}

// =====================================================
//  Motor / speed measurement
// =====================================================
void setMotor(int pwm)
{
  pwmOutput = constrain(pwm, 0, 255);
  duty = (byte)pwmOutput;
}

void setLoad(byte pct)
{
  if (pct != loadPct)
  {
    loadPct = pct;
    qTime(); qP(F("LOAD -> ")); qNum(loadPct, 0, 0); qP(F(" %\n"));
  }
}

void updateLoad(unsigned long now)
{
  if (loadMode == 1 || loadMode == 2)
  {
    if (now - lastLoadRead < 100) return;          // read inputs 10x per second
    lastLoadRead = now;
    if (loadMode == 1) setLoad((byte)(((long)analogRead(A0) * 18 / 1023) * 5));   // pot: 0..90 % in 5 % steps
    else setLoad(digitalRead(SW_PIN) == LOW ? loadSet : 0);                        // switch closed = load on
  }
  else if (loadMode == 3 && now - loadToggleMs >= LOAD_PULSE_MS)
  {
    loadToggleMs = now;
    setLoad(loadPct == 0 ? loadSet : 0);
  }
}

void countPulse()
{
  unsigned long t = micros();
  periodUs = t - lastPulseUs;
  lastPulseUs = t;
}

void updateRPM()
{
  noInterrupts();
  unsigned long p = periodUs;
  unsigned long last = lastPulseUs;
  interrupts();

  unsigned long sinceLast = micros() - last;
  if (p == 0 || sinceLast > 500000UL) rawRPM = 0.0;
  else rawRPM = 60000000.0 / (PULSES_PER_REV * (double)p);

  currentRPM += RPM_FILTER * (rawRPM - currentRPM);
}

// =====================================================
//  Sorting
// =====================================================
void setGate(byte pos)
{
  if (pos != gatePos)
  {
    sortingGate.write(pos);
    gatePos = pos;
    qTime();
    if (pos == GATE_REJECT)
    {
      rejectCount++;
      qP(F("GATE -> REJECT (height "));
      qNum(objectHeight, 0, 1);
      qP(F(" cm, total rejected ")); qNum(rejectCount, 0, 0); qP(F(")\n"));
    }
    else
    {
      qP(F("GATE -> PASS\n"));
    }
  }
}

float readUltrasonic()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long start = micros();
  while (digitalRead(ECHO_PIN) == LOW)
  {
    softPWM();
    if (micros() - start > ECHO_TIMEOUT_US) return -1.0;
  }
  unsigned long t0 = micros();
  while (digitalRead(ECHO_PIN) == HIGH)
  {
    softPWM();
    if (micros() - t0 > ECHO_TIMEOUT_US) return -1.0;
  }
  unsigned long duration = micros() - t0;
  return (duration * 0.0343) / 2.0;
}

void updateSorting(unsigned long now)
{
  if (now - lastUltraTime >= ULTRA_MS)
  {
    lastUltraTime = now;
    lastDistance = readUltrasonic();
    objectHeight = (lastDistance < 0) ? 0.0 : (SENSOR_TO_BELT_DIST - lastDistance);

    if (objectHeight >= HEIGHT_THRESHOLD)
    {
      if (tallCount < 255) tallCount++;
      if (tallCount >= 2) divertUntil = now + DIVERT_HOLD_MS;
    }
    else
    {
      tallCount = 0;
    }
  }
  setGate((long)(divertUntil - now) > 0 ? GATE_REJECT : GATE_PASS);
}

// =====================================================
//  Logging
// =====================================================
void logRow(double actual)
{
  if (csvMode)
  {
    qNum(millis() / 1000.0, 0, 2); qPut(',');
    qNum(setpoint, 0, 1); qPut(',');
    qNum(actual, 0, 1); qPut(',');
    qNum(error, 0, 1); qPut(',');
    qNum(pTerm, 0, 1); qPut(',');
    qNum(iTerm, 0, 1); qPut(',');
    qNum(dTerm, 0, 1); qPut(',');
    qNum(pwmOutput, 0, 0); qPut(',');
    qNum(loadPct, 0, 0); qPut(',');
    qNum(objectHeight, 0, 1); qPut('\n');
    return;
  }

  qNum(millis() / 1000.0, 6, 1); qPut(' ');
  qNum(setpoint, 6, 1); qPut(' ');
  qNum(actual, 6, 1); qPut(' ');
  qNum(error, 6, 1); qPut(' ');
  qNum(pTerm, 6, 1); qPut(' ');
  qNum(iTerm, 6, 1); qPut(' ');
  qNum(dTerm, 6, 1); qPut(' ');
  qNum(pwmOutput, 3, 0); qPut(' ');
  qNum(loadPct, 3, 0); qPut(' ');
  qNum(objectHeight, 5, 1); qPut(' ');
  qStr(gatePos == GATE_REJECT ? "REJ" : "PASS");
  if (pwmOutput >= 255) qP(F(" SAT"));
  qPut('\n');
}

void startStep()
{
  stepActive = true;
  stepStartMs = millis();
  inBandSince = 0;
  stepFrom = currentRPM;
  stepSpan = fabs(setpoint - currentRPM);
  stepDir = (setpoint >= currentRPM) ? 1 : -1;
  stepExtreme = currentRPM;
  t10 = t90 = -1;
}

void trackStep(unsigned long now)
{
  if (!stepActive)
  {
    if (fabs(error) > 10.0)
    {
      qTime(); qP(F("Large error, tracking new transient\n"));
      startStep();
    }
    return;
  }

  long elapsed = (long)(now - stepStartMs);
  if (stepDir > 0 ? currentRPM > stepExtreme : currentRPM < stepExtreme) stepExtreme = currentRPM;

  if (stepSpan > 5.0)
  {
    double progress = stepDir * (currentRPM - stepFrom);
    if (t10 < 0 && progress >= 0.1 * stepSpan) t10 = elapsed;
    if (t90 < 0 && progress >= 0.9 * stepSpan) t90 = elapsed;
  }

  if (fabs(error) <= SETTLE_BAND)
  {
    if (inBandSince == 0) inBandSince = now;
    else if (now - inBandSince >= SETTLE_HOLD_MS)
    {
      lastSettle = (inBandSince - stepStartMs) / 1000.0;
      lastRise = (t10 >= 0 && t90 >= 0) ? (t90 - t10) / 1000.0 : -1;
      double ov = stepDir * (stepExtreme - setpoint);
      lastOver = (ov > 0.0) ? ov : 0.0;
      lastOverPct = (stepSpan > 5.0) ? 100.0 * lastOver / stepSpan : 0.0;

      qTime(); qP(F("STEP DONE target ")); qNum(setpoint, 0, 1); qP(F(" RPM | rise 10-90%: "));
      if (lastRise >= 0) { qNum(lastRise, 0, 2); qP(F(" s")); } else qP(F("n/a"));
      qP(F(" | settled ")); qNum(lastSettle, 0, 2);
      qP(F(" s | overshoot ")); qNum(lastOver, 0, 1); qP(F(" RPM (")); qNum(lastOverPct, 0, 1); qP(F("%)\n"));

      stepActive = false;
      ssStart = now; ssN = ssIn = 0; ssSumErr = ssMaxAbs = ssSumPwm = 0;
    }
  }
  else inBandSince = 0;
}

void trackSteady(unsigned long now)
{
  if (stepActive) return;
  ssN++;
  ssSumErr += error;
  ssSumPwm += pwmOutput;
  if (fabs(error) > ssMaxAbs) ssMaxAbs = fabs(error);
  if (fabs(error) <= SETTLE_BAND) ssIn++;

  if (now - ssStart >= SS_WINDOW_MS && ssN > 0)
  {
    qTime(); qP(F("STEADY "));
    qNum(SS_WINDOW_MS / 1000.0, 0, 0); qP(F("s | mean err ")); qNum(ssSumErr / ssN, 0, 2);
    qP(F(" | max err ")); qNum(ssMaxAbs, 0, 2);
    qP(F(" | in band ")); qNum(100.0 * ssIn / ssN, 0, 0);
    qP(F("% | avg PWM ")); qNum(ssSumPwm / ssN, 0, 0); qPut('\n');
    ssStart = now; ssN = ssIn = 0; ssSumErr = ssMaxAbs = ssSumPwm = 0;
  }
}

void printReport()
{
  qTime(); qP(F("REPORT\n"));
  qP(F("  setpoint ")); qNum(setpoint, 0, 1); qP(F(" RPM | PV ")); qNum(currentRPM, 0, 1);
  qP(F(" | error ")); qNum(error, 0, 2); qP(F(" | PWM ")); qNum(pwmOutput, 0, 0); qPut('\n');
  qP(F("  gains kp=")); qNum(kp, 0, 3); qP(F(" ki=")); qNum(ki, 0, 3); qP(F(" kd=")); qNum(kd, 0, 3); qPut('\n');
  qP(F("  last step: "));
  if (lastSettle >= 0)
  {
    qP(F("settled in ")); qNum(lastSettle, 0, 2); qP(F(" s, overshoot ")); qNum(lastOver, 0, 1);
    qP(F(" RPM (")); qNum(lastOverPct, 0, 1); qP(F("%)"));
  }
  else qP(F("none completed yet"));
  qPut('\n');
  qP(F("  load ")); qNum(loadPct, 0, 0); qP(F(" % (size ")); qNum(loadSet, 0, 0); qP(F(", source ")); qNum(loadMode, 0, 0); qP(F(")\n"));
  qP(F("  gate: ")); qStr(gatePos == GATE_REJECT ? "REJECT" : "PASS");
  qP(F(" | rejected objects: ")); qNum(rejectCount, 0, 0); qPut('\n');
}

// =====================================================
//  Terminal commands
// =====================================================
void handleSerial()
{
  static char buf[16];
  static byte n = 0;
  while (Serial.available())
  {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n')
    {
      if (n > 0)
      {
        buf[n] = 0;
        float v = atof(buf + 1);
        switch (buf[0])
        {
          case 's':
            setpoint = constrain(v, 0.0, 1000.0);
            qTime(); qP(F("CMD setpoint = ")); qNum(setpoint, 0, 1); qP(F(" RPM\n"));
            if (setpoint > MAX_RPM) { qP(F("  WARNING: above the motor's ~")); qNum(MAX_RPM, 0, 0); qP(F(" RPM limit\n")); }
            startStep();
            break;
          case 'p': kp = v; resetPID(); qTime(); qP(F("CMD kp = ")); qNum(kp, 0, 3); qPut('\n'); break;
          case 'i': ki = v; resetPID(); qTime(); qP(F("CMD ki = ")); qNum(ki, 0, 3); qPut('\n'); break;
          case 'd': kd = v; resetPID(); qTime(); qP(F("CMD kd = ")); qNum(kd, 0, 3); qPut('\n'); break;
          case 'l':
            loadSet = (byte)constrain(v, 0, 90);
            qTime(); qP(F("CMD load size = ")); qNum(loadSet, 0, 0); qP(F(" %\n"));
            if (loadMode == 0) setLoad(loadSet);
            break;
          case 'm':
            loadMode = (byte)constrain(v, 0, 3);
            loadToggleMs = millis();
            qTime(); qP(F("CMD load source = "));
            qStr(loadMode == 0 ? "terminal" : loadMode == 1 ? "pot A0" : loadMode == 2 ? "switch D6" : "auto pulse");
            qPut('\n');
            break;
          case 'r': printReport(); break;
          case 'c':
            csvMode = !csvMode;
            qTime(); qP(F("CMD output mode: ")); qStr(csvMode ? "CSV" : "table"); qPut('\n');
            if (csvMode) qP(F("t_s,SP,PV,ERR,P,I,D,PWM,LOAD,H\n")); else printHeader();
            break;
          case 'h': printHelp(); printHeader(); break;
          default:
            qTime(); qP(F("CMD unknown, type h for help\n"));
        }
      }
      n = 0;
    }
    else if (n < 15) buf[n++] = ch;
  }
}

// END OF SKETCH