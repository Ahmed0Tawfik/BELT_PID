// =====================================================
//  Ziegler-Nichols relay tuner with SOFTWARE PWM (Simulino-safe)
//  Finds Ku and Tu, then prints gains for:
//    CLASSIC_PID     : Kp = 0.6 Ku, Ti = 0.5 Tu,  Td = 0.125 Tu
//    PESSEN_INTEGRAL : Kp = 0.7 Ku, Ti = 0.4 Tu,  Td = 0.15 Tu
//  Printed ki = Kp/Ti and kd = Kp*Td for:
//    output = kp*error + ki*integral + kd*derivative   (output in PWM counts 0..255)
//
//  D2 <- encoder | D3 -> IN2 | D4 -> IN1 | D5 -> EN1 | 9600 baud
// =====================================================

const byte IR_PULSE_PIN = 2;
const byte IN2_PIN = 3;
const byte IN1_PIN = 4;
const byte ENA_PIN = 5;

// ---------- Settings ----------
double setpoint = 150.0;             // RPM to oscillate around (motor max is ~340)
const double RELAY_D = 40.0;         // relay swing in PWM counts (bias +/- this)
const double HYST = 3.0;             // RPM hysteresis (ignores measurement noise)
const double ASSUMED_K = 1.33;       // RPM per PWM count (from your sweep): only for the starting guess
const unsigned long SAMPLE_MS = 20;
const byte SKIP_CYCLES = 2;          // ignore the first cycles (transient)
const byte USE_CYCLES = 5;           // cycles averaged for Ku and Tu

// ---------- Software PWM (analogWrite does not work in Simulino) ----------
const unsigned long SW_PERIOD_US = 10000UL;   // 100 Hz
byte duty = 0;                                // 0..255

void softPWM()
{
  unsigned long t = micros() % SW_PERIOD_US;
  unsigned long onTime = (SW_PERIOD_US * (unsigned long)duty) / 255UL;
  digitalWrite(ENA_PIN, t < onTime ? HIGH : LOW);
}

void setMotor(int pwm)
{
  duty = (byte)constrain(pwm, 0, 255);
}

// ---------- Speed measurement ----------
const float PULSES_PER_REV = 24.0;
const float RPM_FILTER = 0.5;
volatile unsigned long lastPulseUs = 0;
volatile unsigned long periodUs = 0;
double rawRPM = 0.0, currentRPM = 0.0;

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

// ---------- Tuner state ----------
enum State { FIND_BIAS, RELAY, DONE };
State state = FIND_BIAS;

int pwm = 0;
double bias = 0, d = 0;
unsigned long lastSample = 0, stateStart = 0, settleStart = 0, lastPrint = 0;
byte biasDiv = 0;

bool relayHigh = true;
double maxPV, minPV, peakMax = 0, troughMin = 0;
unsigned long lastRise = 0;
byte cycle = 0, used = 0;
double sumPeriod = 0, sumAmp = 0;

void printGains(const char *name, double kp, double tiF, double tdF, double Tu)
{
  double Ti = tiF * Tu, Td = tdF * Tu;
  Serial.print(name);
  Serial.print("  kp="); Serial.print(kp, 3);
  Serial.print("  ki="); Serial.print(kp / Ti, 3);
  Serial.print("  kd="); Serial.println(kp * Td, 3);
}

void finish(bool ok, double Ku, double Tu)
{
  setMotor(0);
  state = DONE;
  if (!ok)
  {
    Serial.println("TUNE FAILED (no stable oscillation). Check setpoint, RELAY_D, wiring.");
    return;
  }
  Serial.println("=== RESULT ===");
  Serial.print("Ku="); Serial.print(Ku, 3);
  Serial.print("  Tu="); Serial.print(Tu, 3); Serial.println(" s");
  printGains("CLASSIC_PID    ", 0.6 * Ku, 0.5, 0.125, Tu);
  printGains("PESSEN_INTEGRAL", 0.7 * Ku, 0.4, 0.15, Tu);
}

void setup()
{
  Serial.begin(9600);
  pinMode(IR_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PULSE_PIN), countPulse, RISING);
  pinMode(ENA_PIN, OUTPUT); pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  pwm = constrain((int)(setpoint / ASSUMED_K), 0, 255);   // starting guess
  setMotor(pwm);
  Serial.println("=== ZN TUNER START: finding bias PWM ===");
  stateStart = millis();
}

void loop()
{
  softPWM();                                   // must run every pass
  unsigned long now = millis();
  if (state == DONE || now - lastSample < SAMPLE_MS) return;
  lastSample = now;
  updateRPM();

  if (state == FIND_BIAS)
  {
    // Trim the PWM until the speed holds at the setpoint for 3 s
    if (++biasDiv >= 5)
    {
      biasDiv = 0;
      if (currentRPM < setpoint - 1.0) pwm++;
      else if (currentRPM > setpoint + 1.0) pwm--;
      pwm = constrain(pwm, 0, 255);
    }
    setMotor(pwm);

    if (fabs(currentRPM - setpoint) < 3.0)
    {
      if (settleStart == 0) settleStart = now;
      else if (now - settleStart >= 3000)
      {
        bias = pwm;
        d = min(RELAY_D, min(bias, 255.0 - bias));
        relayHigh = true;
        maxPV = minPV = currentRPM;
        stateStart = now;
        state = RELAY;
        Serial.print("Bias PWM="); Serial.print(bias, 0);
        Serial.print("  relay +/-"); Serial.println(d, 0);
        Serial.println("=== RELAY oscillation ===");
      }
    }
    else settleStart = 0;

    if (now - stateStart > 40000UL) { finish(false, 0, 0); return; }
  }
  else if (state == RELAY)
  {
    if (currentRPM > maxPV) maxPV = currentRPM;
    if (currentRPM < minPV) minPV = currentRPM;

    if (relayHigh && currentRPM > setpoint + HYST)
    {
      relayHigh = false;
      peakMax = maxPV;
      minPV = currentRPM;
    }
    else if (!relayHigh && currentRPM < setpoint - HYST)
    {
      relayHigh = true;
      troughMin = minPV;
      if (lastRise != 0)
      {
        cycle++;
        if (cycle > SKIP_CYCLES)
        {
          sumPeriod += (now - lastRise) / 1000.0;
          sumAmp += (peakMax - troughMin) / 2.0;
          used++;
        }
      }
      lastRise = now;
      maxPV = currentRPM;

      if (used >= USE_CYCLES)
      {
        double Tu = sumPeriod / used;
        double a = sumAmp / used;
        if (a <= HYST) { finish(false, 0, 0); return; }
        double Ku = 4.0 * d / (3.14159265 * sqrt(a * a - HYST * HYST));
        finish(true, Ku, Tu);
        return;
      }
    }

    setMotor((int)(relayHigh ? bias + d : bias - d));
    if (now - stateStart > 60000UL) { finish(false, 0, 0); return; }
  }

  if (now - lastPrint >= 250)
  {
    lastPrint = now;
    Serial.print(state == FIND_BIAS ? "BIAS  " : "RELAY ");
    Serial.print("SP:"); Serial.print(setpoint, 1);
    Serial.print(" PV:"); Serial.print(currentRPM, 1);
    Serial.print(" PWM:"); Serial.println(duty);
  }
}
