#define BattVoltPin 1

//battery voltage
float get_Vbatt(){
  uint32_t Vbatt = 0;
  pinMode(BattVoltPin, INPUT);
  for(int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(BattVoltPin); // Read and accumulate ADC voltage
    delay(1);
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;     // Adjust for 1:2 divider and convert to volts
  Serial.print("Vbatt: ");
  Serial.println(Vbattf, 3);                  // Output voltage to 3 decimal places
  return Vbattf;
}

uint8_t estimateSoC(float voltage) {
  // Stützpunkte: Spannung (V) → Ladezustand (%)
  const float voltages[] = {4.20, 4.10, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.40, 3.30, 3.20};
  const uint8_t socs[]    = {100,  90,   80,   70,   60,   50,   40,   30,   20,   10,    0};
  const int numPoints = sizeof(voltages) / sizeof(voltages[0]);

  // Begrenzen auf gültigen Bereich
  if (voltage >= voltages[0]) return 100;
  if (voltage <= voltages[numPoints - 1]) return 0;

  // Lineare Interpolation zwischen den Stützpunkten
  for (int i = 0; i < numPoints - 1; i++) {
    float v1 = voltages[i];
    float v2 = voltages[i + 1];
    if (voltage <= v1 && voltage >= v2) {
      float soc1 = socs[i];
      float soc2 = socs[i + 1];
      float fraction = (voltage - v2) / (v1 - v2);
      float result = soc2 + fraction * (soc1 - soc2);

      // Rundung und Begrenzung auf 0–100 %
      if (result < 0) result = 0;
      if (result > 100) result = 100;
      return (uint8_t)(result + 0.5);
    }
  }

  return 0; // sollte nie erreicht werden
}