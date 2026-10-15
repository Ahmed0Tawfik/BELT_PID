// PWM sweep using SOFTWARE PWM (slow, 50 ms period) on the EN pin.
// Tests whether the Proteus motor responds to duty cycle when analogWrite doesn't.
// D2 <- encoder | D3 -> IN2 | D4 -> IN1 | D5 -> EN1 | 9600 baud

const byte IR_PULSE_PIN = 2;
const byte IN2_PIN = 3;
const byte IN1_PIN = 4;
const byte ENA_PIN = 5;

const float PULSES_PER_REV = 24.0;
const float RPM_FILTER = 0.5;
const int STEP = 15;                        // duty step (0..255)
const unsigned long HOLD_MS = 4000;         // time at each level
const unsigned long SW_PERIOD_US = 50000UL; // software PWM period (50 ms = 20 Hz)

volatile unsigned long lastPulseUs = 0, periodUs = 0;
double rawRPM = 0, currentRPM = 0;

void countPulse()
{
  unsigned long t = micros();
  periodUs = t - lastPulseUs;
  lastPulseUs = t;
}

void updateRPM()
{
  noInterrupts();
  unsigned long p = periodUs, last = lastPulseUs;
  interrupts();
  if (p == 0 || micros() - last > 500000UL) rawRPM = 0;
  else rawRPM = 60000000.0 / (PULSES_PER_REV * (double)p);
  currentRPM += RPM_FILTER * (rawRPM - currentRPM);
}

int duty = 255;   // 0..255
bool goingDown = true, finished = false;
unsigned long stepStart = 0, lastSample = 0;

void softPWM()
{
  unsigned long t = micros() % SW_PERIOD_US;
  unsigned long onTime = (SW_PERIOD_US * (unsigned long)duty) / 255UL;
  digitalWrite(ENA_PIN, t < onTime ? HIGH : LOW);
}

void setup()
{
  Serial.begin(9600);
  pinMode(IR_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PULSE_PIN), countPulse, RISING);
  pinMode(ENA_PIN, OUTPUT); pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  Serial.println("=== SOFT-PWM SWEEP: DOWN from 255 ===");
  stepStart = millis();
}

void loop()
{
  if (finished) { digitalWrite(ENA_PIN, LOW); return; }
  unsigned long now = millis();
  softPWM();
  if (now - lastSample >= 20) { lastSample = now; updateRPM(); }

  if (now - stepStart >= HOLD_MS)
  {
    Serial.print(goingDown ? "DOWN" : "UP  ");
    Serial.print("  duty "); Serial.print(duty);
    Serial.print("  ->  RPM "); Serial.println(currentRPM, 1);

    if (goingDown)
    {
      if (duty == 0) { goingDown = false; Serial.println("=== now going UP from 0 ==="); }
      else duty = max(0, duty - STEP);
    }
    else
    {
      if (duty >= 255) { finished = true; Serial.println("=== DONE ==="); return; }
      duty = min(255, duty + STEP);
    }
    stepStart = now;
  }
}
