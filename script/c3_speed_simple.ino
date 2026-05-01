

// Pin
#define TACHO_PIN 3          // Pin untuk sensor tachometer (optocoupler)

volatile unsigned long pulseCount = 0;
unsigned long lastMillis = 0;
float rpm = 0.0;
float holes = 13.00;  // no of holes in your wheel

// Fungsi ISR yang Dipanggil oleh Tacho Sensor
void IRAM_ATTR onTacho() {

pulseCount++;

}

void setup() {
  Serial.begin(115200);
 
  pinMode(TACHO_PIN, INPUT_PULLUP);
  // Pastikan Anda memilih mode interupsi yang tepat (RISING/FALLING/CHANGE)
  // RISING: Menghitung saat sinyal naik (LOW -> HIGH)
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), onTacho, RISING); 

 

  Serial.println("Timer and Interrupts Started.");
}

void loop() {
    unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 500) {
    noInterrupts();
    rpm = (pulseCount / holes) * 30.0;
    pulseCount = 0;
    lastMillis = currentMillis;
    interrupts();
 
    pulseCount = 0;
    lastMillis = millis();
 

    // Cetak informasi ke Serial Monitor
    Serial.print("RPM: ");
    Serial.println(rpm, 2); 

  }
}
