#define LDR_PIN 32          // ADC1 pin
#define NUM_CLASSES 4
#define SAMPLES_PER_READ 20  // averaging window for noise reduction

const char* classNames[NUM_CLASSES] = {"DARK", "DIM", "BRIGHT", "VERY_BRIGHT"};
float centroids[NUM_CLASSES] = {0, 0, 0, 0};
bool calibrated[NUM_CLASSES] = {false, false, false, false};

int lastClass = -1;
int stableCount = 0;
const int STABLE_THRESHOLD = 5; // readings needed before confirming a class change

int readSmoothed() {
  long sum = 0;
  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    sum += analogRead(LDR_PIN);
    delay(2);
  }
  return sum / SAMPLES_PER_READ;
}

int classify(int reading) {
  int best = -1;
  float bestDist = 1e9;
  for (int i = 0; i < NUM_CLASSES; i++) {
    if (!calibrated[i]) continue;
    float dist = abs(reading - centroids[i]);
    if (dist < bestDist) {
      bestDist = dist;
      best = i;
    }
  }
  return best;
}

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db); // full 0-3.3V range
  Serial.println("Commands: 0-3 to calibrate that class, 's' to show status, anything else to classify.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '3') {
      int idx = c - '0';
      Serial.printf("Calibrating %s... hold steady\n", classNames[idx]);
      int reading = readSmoothed();
      centroids[idx] = reading;
      calibrated[idx] = true;
      Serial.printf("%s centroid = %.0f\n", classNames[idx], centroids[idx]);
    } else if (c == 's') {
      for (int i = 0; i < NUM_CLASSES; i++) {
        Serial.printf("%s: %s (%.0f)\n", classNames[i],
          calibrated[i] ? "OK" : "not calibrated", centroids[i]);
      }
    }
  }

  bool allCalibrated = true;
  for (int i = 0; i < NUM_CLASSES; i++) allCalibrated &= calibrated[i];

  if (allCalibrated) {
    int reading = readSmoothed();
    int result = classify(reading);

    if (result == lastClass) {
      stableCount++;
    } else {
      stableCount = 0;
      lastClass = result;
    }

    if (stableCount == STABLE_THRESHOLD) {
      Serial.printf("Class: %s (raw=%d)\n", classNames[result], reading);
    }
  }

  delay(50);
}