// --- PID Motor trainer ---
// by Nyoman Yudi Kurniawan
// www.aisi555.com

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp32-hal-timer.h" 
//#include "driver/ledc.h"

// --- KONFIGURASI JARINGAN & PIN ---
const char* ssid = "WesGakUsah";     // <<< GANTI INI
const char* password = "1234567890";   // <<< GANTI INI

#define TACHO_PIN 3                      // Pin GPIO untuk input sensor Tacho
#define MOTOR_PWM_PIN 10                 // Pin GPIO untuk output kontrol PWM motor (PENTING!)

// --- KONFIGURASI TACHO & TIMER ---
#define PULSES_PER_ROTATION 20           // Pulsa per putaran (sesuai kalibrasi Anda)
#define TIMER_FREQ 1000000               // 1 MHz (sumber clock untuk timer)
#define TIMER_INTERVAL_US 100000         // 100 ms (Interval untuk menghitung pulsa)
#define IDLE_TIMEOUT_MS 2000             // 2 detik (Waktu sebelum RPM dianggap 0)
#define DEBOUNCE_TIME_MS 2               // 2ms (Debouncing untuk sinyal Tacho)

// --- KONFIGURASI PWM MOTOR ---
#define PWM_CHANNEL 0
#define PWM_FREQ 20000                   // 20 kHz (frekuensi yang baik untuk motor)
#define PWM_RESOLUTION 10                 // Resolusi 10 bit (range 0 - 1024)

// --- KONFIGURASI KONTROL PID ---
#define TARGET_RPM 400.0                 // Setpoint RPM (TARGET)

// Nilai PID awal (Perlu Disesuaikan/Tuning di dunia nyata)
#define KP 1                           // Proportional Gain
#define KI 0.1                          // Integral Gain
#define KD 0.01                         // Derivative Gain
#define PID_TASK_DELAY_MS 50             // Frekuensi loop kontrol (50ms)

// WebServer
AsyncWebServer server(80);

// --- Variabel Global Tacho & Timer (Critical Section) ---
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long pulseCountForRPM = 0;   
volatile unsigned long lastValidPulseTime = 0; 
volatile float rpm = 0;                      
volatile unsigned long lastUpdate = 0;         
volatile unsigned long pulsesUsedForLastRPM = 0; 
volatile unsigned long totalTachoTriggers = 0; 

// --- Variabel Global PID ---
volatile float currentPWM = 0; // PWM Output saat ini (0-1023)
float integralError = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;

// Fungsi ISR Timer (Idle Lock Fix)
void ARDUINO_ISR_ATTR onTimer() {
  unsigned long currentTime = millis();
  portENTER_CRITICAL_ISR(&timerMux);

  // 1. Hitung RPM
  unsigned long pulsesThisInterval = pulseCountForRPM;
  pulsesUsedForLastRPM = pulsesThisInterval;
  pulseCountForRPM = 0; 
  
  float intervalSec = (float)TIMER_INTERVAL_US / 1000000.0;
  float rotationsThisInterval = 0.0;
  if (PULSES_PER_ROTATION > 0) {
      rotationsThisInterval = (float)pulsesThisInterval / PULSES_PER_ROTATION;
  }
  
  rpm = (rotationsThisInterval * 60.0) / intervalSec;

  // 2. Logika Idle Timeout (FIXED)
  if (pulsesThisInterval > 0) {
      lastUpdate = currentTime; 
  } else if (currentTime - lastUpdate > IDLE_TIMEOUT_MS) {
      rpm = 0.0; 
  } 

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

// Fungsi ISR Tacho
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

/**
 * Fungsi Kontrol PID yang berjalan di tugas FreeRTOS terpisah.
 * Menghitung PWM berdasarkan error RPM dan menyesuaikan motor.
 */
void pidControl(void *pvParameters) {
  while (1) {
    unsigned long now = millis();
    float timeChange = (float)(now - lastPIDTime) / 1000.0; // Waktu dalam detik
    lastPIDTime = now;

    // Ambil RPM saat ini dari variabel volatile
    float currentRPM;
    portENTER_CRITICAL(&timerMux);
    currentRPM = rpm; 
    portEXIT_CRITICAL(&timerMux);

    // 1. Hitung Error
    float error = TARGET_RPM - currentRPM;

    // 2. Proportional Term (P)
    float proportionalTerm = KP * error;

    // 3. Integral Term (I)
    integralError += error * timeChange;
    // Batasi integral error (Anti-Windup)
    float integralMax = 1023.0 / KI; 
    if (integralError > integralMax) integralError = integralMax;
    if (integralError < -integralMax) integralError = -integralMax;
    float integralTerm = KI * integralError;

    // 4. Derivative Term (D)
    float derivativeTerm = KD * (error - lastError) / timeChange;
    lastError = error;

    // 5. Hitung Output PWM (PID Output)
    float outputPWM = proportionalTerm + integralTerm + derivativeTerm;
    
    // 6. Batasi Output ke Range PWM (0-1023)
    if (outputPWM > 1023) outputPWM = 1023;
    else if (outputPWM < 0) outputPWM = 0;

    // 7. Terapkan Output ke Motor
    ledcWrite(MOTOR_PWM_PIN, (int)outputPWM);
   // analogWrite(PWM_CHANNEL, (int)outputPWM); 
 
    
    // Simpan nilai PWM saat ini untuk ditampilkan di Web
    currentPWM = outputPWM; 

    // Tunggu sebentar sebelum iterasi berikutnya
    vTaskDelay(pdMS_TO_TICKS(PID_TASK_DELAY_MS));
  }
}

// Handler untuk endpoint data JSON
void handleData(AsyncWebServerRequest *request) {
    portENTER_CRITICAL(&timerMux);
    float currentRPM = rpm; 
    float currentPWMDuty = currentPWM;
    portEXIT_CRITICAL(&timerMux);

    String json = "{";
    json += "\"rpm\": " + String(currentRPM, 2) + ",";
    json += "\"pwm\": " + String(currentPWMDuty, 0) + ",";
    json += "\"target\": " + String(TARGET_RPM, 0) + ",";
    json += "\"kp\": " + String(KP, 3) + ",";
    json += "\"ki\": " + String(KI, 3) + ",";
    json += "\"kd\": " + String(KD, 3);
    json += "}";
    request->send(200, "application/json", json);
}

// --- HTML Dashboard (Gauge + Kontrol Status) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
    <title>ESP32 RPM PID Controller</title>

    <script src="https://cdn.jsdelivr.net/npm/canvas-gauges@2.1.7/gauge.min.js"></script>
    <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>

    <style>
        body {
            background: #222;
            font-family: Arial, sans-serif;
            padding: 10px;
            color: white;
            text-align: center;
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            max-width: 600px; 
            margin: 10px auto;
            background: #333;
            border-radius: 15px;
            box-shadow: 0 8px 20px rgba(0,0,0,0.5);
            padding: 20px;
        }
        h2 {
            color: #FFEB3B;
            margin-bottom: 5px;
        }
        .status-box {
            margin-top: 20px;
            padding: 15px;
            background: #444;
            border-radius: 10px;
            width: 100%;
            text-align: left;
        }
        .status-item {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 1.1em;
            border-bottom: 1px solid #555;
            padding-bottom: 5px;
        }
        .status-value {
            font-weight: bold;
            color: #FFEB3B;
        }
    </style>
</head>
<body>

    <h2>⚙️ Kontrol Kecepatan Motor (PID)</h2>
    <p>Target RPM: <span id="targetRpmDisplay" class="status-value">400</span></p>

    <div class="container">
        <!-- RPM Gauge -->
        <div class="gauge-box">
            <canvas id="gaugeRPM" data-type="radial-gauge"
                data-width="350"
                data-height="350"
                data-min-value="0"
                data-max-value="1200" 
                data-major-ticks="0,200,400,600,800,1000,1200"
                data-minor-ticks="10"
                data-units="RPM"
                data-title="RPM AKTUAL"
                data-value="0"
                data-color-value="#FFEB3B" 
                data-border-shadow-width="0"
                data-glow="true"
                data-font-value="sans-serif"
                data-value-box="true"
                data-value-int="4"
                data-animation-rule="elastic">
            </canvas>
        </div>

        <!-- Status Kontrol -->
        <div class="status-box">
            <h3>Status Kontrol PID</h3>
            <div class="status-item"><span>Output PWM (0-1023):</span><span id="pwmDisplay" class="status-value">0</span></div>
            <div class="status-item"><span>Error (Target - Aktual):</span><span id="errorDisplay" class="status-value">0.00</span></div>
            <div class="status-item"><span>Kp:</span><span id="kpDisplay" class="status-value">0.00</span></div>
            <div class="status-item"><span>Ki:</span><span id="kiDisplay" class="status-value">0.00</span></div>
            <div class="status-item"><span>Kd:</span><span id="kdDisplay" class="status-value">0.00</span></div>
        </div>
    </div>

    <script>
        let gaugeRPM;
        
        const fetchPIDData = () => { 
            fetch('/data')
            .then(response => {
                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }
                return response.json();
            })
            .then(data => {
                const currentRPM = parseFloat(data.rpm) || 0;
                const currentPWM = parseInt(data.pwm) || 0;
                const targetRPM = parseFloat(data.target) || 0;
                
                // Update Gauge
                if (gaugeRPM) {
                    gaugeRPM.value = currentRPM;
                }
                
                // Update Status Display
                $('#pwmDisplay').text(currentPWM);
                $('#targetRpmDisplay').text(targetRPM);
                $('#errorDisplay').text((targetRPM - currentRPM).toFixed(2));
                $('#kpDisplay').text(parseFloat(data.kp).toFixed(3));
                $('#kiDisplay').text(parseFloat(data.ki).toFixed(3));
                $('#kdDisplay').text(parseFloat(data.kd).toFixed(3));
            })
            .catch(error => {
                console.error("❌ Gagal mengambil data PID:", error);
                if (gaugeRPM) gaugeRPM.value = 0;
            });
        };

        $(document).ready(function() {
            // Tunggu sebentar agar canvas-gauges selesai inisialisasi
            setTimeout(() => {
                const gauges = window.gauges || [];
                if (gauges.length > 0) {
                    gaugeRPM = gauges[0];
                } else {
                    console.error("Gauge RPM tidak terinisialisasi.");
                    return;
                }

                // Mulai ambil data
                fetchPIDData();

                // Jadwalkan pembaruan data setiap 500 ms (0.5 detik)
                setInterval(fetchPIDData, 500); 

            }, 500); 
        });
    </script>
</body>
</html>
)rawliteral";

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting ESP32 RPM PID WebServer...");

    // *** 1. SETUP PWM MOTOR ***
    //ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_PWM_PIN,PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_PWM_PIN, 0); // Pastikan motor mati saat startup

   //analogWriteResolution(PWM_RESOLUTION);
    //analogWriteFreq(PWM_FREQ); //biar gak bunyi ngiiing di motor 
    //analogWrite(PWM_CHANNEL,0);

    
    // *** 2. INISIALISASI WAKTU TACHO *ledcAttach(MOTOR_PWM_PIN,PWM_FREQ, PWM_RESOLUTION);**
    unsigned long currentTime = millis();
    portENTER_CRITICAL(&timerMux);
    lastUpdate = currentTime; // Mencegah Idle Lock saat boot pertama
    lastValidPulseTime = currentTime;
    portEXIT_CRITICAL(&timerMux);
    
    // *** 3. SETUP PIN TACHO ***
    pinMode(TACHO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TACHO_PIN), onTacho, RISING); 

    // *** 4. SETUP TIMER TACHO ***
    timerSemaphore = xSemaphoreCreateBinary();
    timer = timerBegin(TIMER_FREQ); 
    timerAttachInterrupt(timer, &onTimer); 
    timerAlarm(timer, TIMER_INTERVAL_US, true,0); 
    Serial.println("Timer and Interrupts Started.");

    // *** 5. SETUP FREE RTOS TASK PID ***
    xTaskCreate(
      pidControl,      // Fungsi yang akan dijalankan
      "PID Control",   // Nama tugas
      4096,            // Ukuran stack (bytes)
      NULL,            // Parameter tugas
      1,               // Prioritas (1 = lebih rendah dari task sistem)
      NULL             // Handle tugas
    );
    Serial.println("PID Control Task Started.");

    // *** 6. SETUP WIFI & WEBSERVER ***
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("Web Server Address: http://");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    server.on("/data", HTTP_GET, handleData);
    server.begin();
}

void loop() {
    // Loop utama dibiarkan kosong karena semua tugas penting (Tacho, Timer, PID, WebServer) 
    // ditangani oleh Interrupt dan FreeRTOS Tasks.
}
