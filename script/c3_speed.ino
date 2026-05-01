// --- PID Motor trainer ---
// by Nyoman Yudi Kurniawan
// www.aisi555.com

#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// --- KONFIGURASI PENTING ---
#define PULSES_PER_ROTATION 13    // <<< GANTI: Jumlah pulsa (lubang/magnet) per satu putaran.
                                 //             SESUAIKAN NILAI INI dengan sensor Anda!
#define TIMER_FREQ 1000000       // Frekuensi 1 MHz (1,000,000 Hz). 1 tick = 1 us.
// --- KONFIGURASI PENTING ---

// ... (Deklarasi Variabel Global tetap sama)
const char* ssid = "Nama Wifi";
const char* password = "klengcinot";

#define TACHO_PIN 3          
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long pulseCountForRPM = 0;   
volatile unsigned long lastValidPulseTime = 0; 
volatile float rpm = 0;                      
volatile unsigned long lastUpdate = 0;         
volatile unsigned long pulsesUsedForLastRPM = 0; 
volatile unsigned long totalTachoTriggers = 0; 

#define TIMER_INTERVAL_US 100000    // 100 ms (0.1 detik)
#define IDLE_TIMEOUT_MS 2000        // 2 detik
#define DEBOUNCE_TIME_MS 2          // 2ms

// Fungsi ISR Timer (tetap menggunakan ARDUINO_ISR_ATTR)
void ARDUINO_ISR_ATTR onTimer() {
  unsigned long currentTime = millis();
  portENTER_CRITICAL_ISR(&timerMux);

  unsigned long pulsesThisInterval = pulseCountForRPM;
  pulsesUsedForLastRPM = pulsesThisInterval;
  pulseCountForRPM = 0; 

  float intervalSec = (float)TIMER_INTERVAL_US / 1000000.0;
  
  float rotationsThisInterval = 0.0;
  if (PULSES_PER_ROTATION > 0) {
      rotationsThisInterval = (float)pulsesThisInterval / PULSES_PER_ROTATION;
  }
  
  // Hitung RPM: (Rotations per second) * 60
  rpm = (rotationsThisInterval * 60.0) / intervalSec;

  if (currentTime - lastUpdate > IDLE_TIMEOUT_MS) {
    rpm = 0.0;
  } else if (pulsesThisInterval > 0) {
    lastUpdate = currentTime; 
  }

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

// Fungsi ISR Tacho (tetap menggunakan IRAM_ATTR)
void IRAM_ATTR onTacho() {
  portENTER_CRITICAL_ISR(&timerMux);
  unsigned long currentTime = millis();
  totalTachoTriggers++; 
  if ((currentTime - lastValidPulseTime) > DEBOUNCE_TIME_MS) {
    pulseCountForRPM++;
    lastValidPulseTime = currentTime;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting RPM Monitor (Minimal Timer Init Style)...");
  Serial.printf("PULSES_PER_ROTATION (PPR) set to: %d\n", PULSES_PER_ROTATION);

  // Setup pin tachometer
  pinMode(TACHO_PIN, INPUT_PULLUP);
  // Pastikan RISING/FALLING sudah sesuai dengan karakteristik sinyal sensor Anda.
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), onTacho, RISING); 

  // --- Setup Timer (Minimal/Implisit Style - Core v3.1.0+) ---
  timerSemaphore = xSemaphoreCreateBinary();

  // 1. Configure Timer: Set frequency to 1 MHz (1 tick = 1 us)
  // Library akan memilih Timer ID dan Prescaler terbaik.
  timer = timerBegin(TIMER_FREQ); 

  // 2. Attach the onTimer function to our timer
  timerAttachInterrupt(timer, &onTimer); 

  // 3. Set alarm to call onTimer function every 100,000 us (100ms)
  // Penting: Menggunakan TIMER_INTERVAL_US yang 100,000 us, bukan 1000 us (1ms)
  timerAlarm(timer, TIMER_INTERVAL_US, true,0);


  Serial.println("Timer and Interrupts Started successfully.");
}

void loop() {
  // Reset Counters via Serial
  if (Serial.available()) {
    Serial.read(); 
    portENTER_CRITICAL(&timerMux);
    totalTachoTriggers = 0;
    pulseCountForRPM = 0; 
    pulsesUsedForLastRPM = 0; 
    rpm = 0.0;
    lastUpdate = millis(); 
    lastValidPulseTime = millis(); 
    portEXIT_CRITICAL(&timerMux);
    Serial.println("--- Counters Reset ---");
  }

  // Cek Semaphore (hasil dari onTimer)
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    portENTER_CRITICAL(&timerMux);
    float currentRPM = rpm;
    unsigned long currentTotalTriggers = totalTachoTriggers;
    unsigned long pulsesUsedForRPMThisInterval = pulsesUsedForLastRPM;
    portEXIT_CRITICAL(&timerMux);

    // Cetak informasi ke Serial Monitor
    Serial.print("RPM: ");
    Serial.print(currentRPM, 2); 
    Serial.print(" | Total Triggers: ");
    Serial.print(currentTotalTriggers);
    Serial.print(" | Pulses Used (100ms): ");
    Serial.println(pulsesUsedForRPMThisInterval);
  }
}
