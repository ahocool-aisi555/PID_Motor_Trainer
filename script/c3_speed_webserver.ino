// --- PID Motor trainer ---
// by Nyoman Yudi Kurniawan
// www.aisi555.com

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp32-hal-timer.h" 

// --- KONFIGURASI PENTING ---
const char* ssid = "Nama Wifi";      // <<< GANTI INI
const char* password = "klengcinot";    // <<< GANTI INI

#define PULSES_PER_ROTATION 13    // <<< GANTI INI
#define TACHO_PIN 3              // Pin GPIO
#define TIMER_FREQ 1000000       // 1 MHz
#define TIMER_INTERVAL_US 100000 // 100 ms
#define IDLE_TIMEOUT_MS 2000     // 2 detik
#define DEBOUNCE_TIME_MS 2       // 2ms
// --- KONFIGURASI PENTING ---

// WebServer
AsyncWebServer server(80);

// --- Variabel RPM & Timer ---
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long pulseCountForRPM = 0;   
volatile unsigned long lastValidPulseTime = 0; 
volatile float rpm = 0;                      
volatile unsigned long lastUpdate = 0;         
volatile unsigned long pulsesUsedForLastRPM = 0; 
volatile unsigned long totalTachoTriggers = 0; 

// Fungsi ISR Timer (Logika RPM Tetap)
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
  
  rpm = (rotationsThisInterval * 60.0) / intervalSec;

  // *** BAGIAN YANG DIUBAH ***
  if (pulsesThisInterval > 0) {
      // Jika ada pulsa, kita aktif. Update waktu terakhir.
      lastUpdate = currentTime; 
  } else if (currentTime - lastUpdate > IDLE_TIMEOUT_MS) {
      // Jika tidak ada pulsa dan sudah melewati timeout
      rpm = 0.0; 
      // JANGAN update lastUpdate di sini. Biarkan ia tetap di waktu terakhir pulsa.
  } 
  // *** AKHIR BAGIAN YANG DIUBAH ***

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}
// Fungsi ISR Tacho (Logika Tacho Tetap)
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

// Fungsi WebServer (Logika Data Tetap)
void handleData(AsyncWebServerRequest *request) {
    portENTER_CRITICAL(&timerMux);
    float currentRPM = rpm; 
    portEXIT_CRITICAL(&timerMux);
    String json = "{\"rpm\": " + String(currentRPM, 2) + "}";
    request->send(200, "application/json", json);
}

// --- HTML Dashboard (Gauge + 7 Segment) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
    <title>ESP32 RPM Dashboard</title>

    <script src="https://cdn.jsdelivr.net/npm/canvas-gauges@2.1.7/gauge.min.js"></script>
    
    <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
    <script src="https://code.jquery.com/ui/1.13.2/jquery-ui.min.js"></script>

    <style>
        body {
            background: #222;
            font-family: Arial, sans-serif;
            padding: 20px;
            color: white;
            text-align: center;
        }
        .dashboard-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            max-width: 450px;
            margin: 20px auto;
            background: #333;
            border-radius: 15px;
            box-shadow: 0 8px 20px rgba(0,0,0,0.5);
            padding: 20px;
        }
        h2 {
            color: #FFEB3B;
            margin-bottom: 5px;
        }
        .label {
            margin-top: 15px;
            font-size: 18px;
            color: #ccc;
        }

    </style>
</head>
<body>

    <h2>⚙️ Real-Time Engine RPM</h2>
    <p>Diukur oleh ESP32-C3</p>

    <div class="dashboard-container">
        <div class="gauge-box">
            <canvas id="gaugeRPM" data-type="radial-gauge"
                data-width="500"
                data-height="500"
                data-min-value="0"
                data-max-value="4000"
                data-major-ticks="0,500,1000,1500,2000,2500,3000,3500,4000"
                data-minor-ticks="10"
                data-units="RPM"
                data-title="PUTARAN MESIN"
                data-value="0"
                data-color-value="#E91E63"
                data-border-shadow-width="0"
                data-glow="true">
            </canvas>
        </div>

    </div>

    <script>
        let gaugeRPM;
        
        const fetchRPMData = () => { 
            fetch('/data')
            .then(response => {
                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }
                return response.json();
            })
            .then(data => {
                const currentRPM = parseFloat(data.rpm) || 0;

                // Update Gauge
                if (gaugeRPM) {
                    gaugeRPM.value = currentRPM;
                }
                
                
            })
            .catch(error => {
                console.error("❌ Gagal mengambil data RPM:", error);
            });
        };

        $(document).ready(function() {


            // 2. Tunggu agar canvas-gauges selesai inisialisasi
            setTimeout(() => {
                const gauges = window.gauges || [];
                if (gauges.length > 0) {
                    gaugeRPM = gauges[0];
                } else {
                    console.error("Gauge RPM tidak terinisialisasi.");
                    return;
                }

                // 3. Mulai ambil data
                fetchRPMData();

                // 4. Jadwalkan pembaruan data setiap 1000 ms (1 detik)
                setInterval(fetchRPMData, 1000); 

            }, 500); 
        });
    </script>
</body>
</html>
)rawliteral";



void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting ESP32 RPM WebServer...");
    Serial.printf("PPR set to: %d\n", PULSES_PER_ROTATION);

    // 1. Setup Pin Tacho
    pinMode(TACHO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TACHO_PIN), onTacho, RISING); 

    // 2. Setup Timer
    timerSemaphore = xSemaphoreCreateBinary();
    timer = timerBegin(TIMER_FREQ); 
    timerAttachInterrupt(timer, &onTimer); 
    timerAlarm(timer, TIMER_INTERVAL_US, true,0); 
    Serial.println("Timer and Interrupts Started.");

    // 3. Setup WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("Web Server Address: http://");
    Serial.println(WiFi.localIP());
    Serial.println("----------------------------------------");

    // 4. Setup WebServer Routing
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    server.on("/data", HTTP_GET, handleData);

    server.begin();
}

void loop() {
 

  //kosong aja semua di handle interrupt
}
