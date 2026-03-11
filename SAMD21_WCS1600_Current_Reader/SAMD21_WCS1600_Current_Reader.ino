/*
XIAO SAMD21 + WCS1600 current reader
*/

const int CURRENT_PIN = A0;

const float ADC_REF = 3.3f;
const int ADC_MAX = 4095;

// Tune these two constants from bench data
const float ZERO_VOLTAGE = 1.745f;
const float SENSOR_SENSITIVITY_V_PER_A = 0.066f;

// Keep this if 3 A was still a little high
const float CURRENT_GAIN = 3.3;

const uint32_t SAMPLE_INTERVAL_US = 2000;
uint32_t lastSample = 0;

float readVoltage(int samples)
{
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++)
  {
    sum += analogRead(CURRENT_PIN);
  }

  float adc = (float)sum / samples;
  return adc * ADC_REF / ADC_MAX;
}

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop()
{
  uint32_t now = micros();

  if (now - lastSample >= SAMPLE_INTERVAL_US)
  {
    lastSample = now;

    float voltage = readVoltage(4);
    float current = (voltage - ZERO_VOLTAGE) / SENSOR_SENSITIVITY_V_PER_A;
    current *= CURRENT_GAIN;

    Serial.println(current, 3);
  }
}