#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp32-hal-timer.h" 

// --- KONFIGURASI JARINGAN & PIN ---
const char* ssid = "WesGakUsah";     // <<< GANTI INI
const char* password = "1234567890";   // <<< GANTI INI

#define TACHO_PIN 3                      // Pin GPIO untuk input sensor Tacho
#define MOTOR_PWM_PIN 10               // Pin GPIO untuk output kontrol PWM motor (PENTING!)

// --- KONFIGURASI TACHO & TIMER ---
#define PULSES_PER_ROTATION 22           // Pulsa per putaran (sesuai kalibrasi Anda)
#define TIMER_FREQ 1000000               // 1 MHz (sumber clock untuk timer)
#define TIMER_INTERVAL_US 100000         // 100 ms (Interval untuk menghitung pulsa)
#define IDLE_TIMEOUT_MS 2000             // 2 detik (Waktu sebelum RPM dianggap 0)
#define DEBOUNCE_TIME_MS 1               // 2ms (Debouncing untuk sinyal Tacho)

// --- KONFIGURASI PWM MOTOR (untuk analogWrite) ---
#define PWM_RESOLUTION 10                // Resolusi 10 bit (range 0 - 1023)
#define MAX_PWM_VALUE 1023.0             // Nilai maksimum PWM
#define PWM_FREQ 20000                   // 20 kHz (frekuensi yang baik untuk motor)

// --- VARIABEL KONTROL PID YANG INTERAKTIF ---
volatile float TARGET_RPM = 450.0;        // Setpoint RPM (TARGET)
volatile float DEADBAND_RPM = 20.0;       // Toleransi RPM (+/- RPM)
volatile float KP = 0.8;                  // Proportional Gain (Awal)
volatile float KI = 0.05;                 // Integral Gain (Awal)
volatile float KD = 0.01;                // Derivative Gain (Awal)
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
float integralError = 0; // TIDAK VOLATILE karena hanya diakses dalam satu task (pidControl)
float lastError = 0;
unsigned long lastPIDTime = 0;

// Fungsi ISR Timer (Logika RPM)
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
 */
void pidControl(void *pvParameters) {
  while (1) {
    unsigned long now = millis();
    float timeChange = (float)(now - lastPIDTime) / 1000.0; // Waktu dalam detik
    lastPIDTime = now;

    // Ambil nilai kontrol saat ini dari variabel global (Volatile)
    float currentTargetRPM = TARGET_RPM;
    float currentDeadbandRPM = DEADBAND_RPM;
    float currentKP = KP;
    float currentKI = KI;
    float currentKD = KD;

    float currentRPM;
    portENTER_CRITICAL(&timerMux);
    currentRPM = rpm; 
    portEXIT_CRITICAL(&timerMux);

    // 1. Hitung Error Awal
    float error = currentTargetRPM - currentRPM;

    // 2. Terapkan DEADBAND: Jika error di dalam batas, abaikan.
    if (abs(error) < currentDeadbandRPM) {
        error = 0; 
    }

    // 3. Proportional Term (P)
    float proportionalTerm = currentKP * error;

    // 4. Integral Term (I)
    // Penumpukan Integral hanya terjadi jika error tidak nol
    if (error != 0) {
        integralError += error * timeChange;
    }
    
    // Anti-Windup: Membatasi akumulasi Integral (Hanya jika KI > 0)
    if (currentKI > 0) {
      float integralMax = MAX_PWM_VALUE / currentKI; 
      if (integralError > integralMax) integralError = integralMax;
      if (integralError < -integralMax) integralError = -integralMax;
    } else {
        integralError = 0; // Jika KI=0, reset integral
    }
    
    float integralTerm = currentKI * integralError;

    // 5. Derivative Term (D)
    float derivativeTerm = 0;
    if (timeChange > 0) {
        derivativeTerm = currentKD * (error - lastError) / timeChange;
    }
    lastError = error;

    // 6. Hitung Output PWM (PID Output)
    float outputPWM = proportionalTerm + integralTerm + derivativeTerm;
    
    // 7. Batasi Output ke Range PWM (0-1023)
    if (outputPWM > MAX_PWM_VALUE) outputPWM = MAX_PWM_VALUE; 
    else if (outputPWM < 0) outputPWM = 0;

    // 8. Terapkan Output ke Motor menggunakan ledc
    ledcWrite(MOTOR_PWM_PIN, (int)outputPWM);

    currentPWM = outputPWM; 

    vTaskDelay(pdMS_TO_TICKS(PID_TASK_DELAY_MS));
  }
}

// Handler untuk endpoint data JSON (GET)
void handleData(AsyncWebServerRequest *request) {
    portENTER_CRITICAL(&timerMux);
    float currentRPM = rpm; 
    float currentPWMDuty = currentPWM;
    portEXIT_CRITICAL(&timerMux);

    String json = "{";
    json += "\"rpm\": " + String(currentRPM, 2) + ",";
    json += "\"pwm\": " + String(currentPWMDuty, 0) + ",";
    // Ambil nilai variabel global saat ini
    json += "\"target\": " + String(TARGET_RPM, 0) + ",";
    json += "\"deadband\": " + String(DEADBAND_RPM, 0) + ",";
    json += "\"kp\": " + String(KP, 3) + ",";
    json += "\"ki\": " + String(KI, 3) + ",";
    json += "\"kd\": " + String(KD, 3);
    json += "}";
    request->send(200, "application/json", json);
}

// Handler yang diperbaiki untuk menerima parameter PID baru (POST)
void handleSetPID(AsyncWebServerRequest *request) {
    bool updated = false;
    
    // Iterasi melalui semua argumen yang dikirim (lebih robust untuk POST form data)
    for (int i = 0; i < request->args(); i++) {
        String name = request->argName(i);
        float value = request->arg(i).toFloat();

        // Validasi: Pastikan nilai yang dimasukkan non-negatif
        if (value >= 0) {
            if (name == "target") {
                TARGET_RPM = value;
                updated = true;
            } else if (name == "kp") {
                KP = value;
                updated = true;
            } else if (name == "ki") {
                KI = value;
                updated = true;
            } else if (name == "kd") {
                KD = value;
                updated = true;
            } else if (name == "deadband") {
                DEADBAND_RPM = value;
                updated = true;
            }
        }
    }

    if (updated) {
        // Reset integral error saat parameter PID diubah
        integralError = 0; 
        lastError = 0;
        Serial.printf("PID Updated: Target=%.0f, Kp=%.3f, Ki=%.3f, Kd=%.3f, Deadband=%.0f\n", 
                      TARGET_RPM, KP, KI, KD, DEADBAND_RPM);
        request->send(200, "text/plain", "PID parameters updated successfully.");
    } else {
        request->send(400, "text/plain", "Error: No valid PID parameters received.");
    }
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
            font-family: 'Inter', sans-serif;
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
        .status-box, .control-box {
            margin-top: 20px;
            padding: 15px;
            background: #444;
            border-radius: 10px;
            width: 100%;
            text-align: left;
        }
        .status-item, .input-group {
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
        .target-zone {
            color: #4CAF50; /* Hijau */
        }
        
        .input-group label {
            min-width: 100px;
            line-height: 2;
        }
        .input-group input[type="number"] {
            width: 150px;
            padding: 5px;
            border-radius: 5px;
            border: 1px solid #555;
            background-color: #333;
            color: white;
            text-align: right;
        }
        #updateButton {
            width: 100%;
            padding: 10px;
            margin-top: 15px;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-size: 1.2em;
            transition: background-color 0.3s, opacity 0.3s;
        }
        #updateButton:disabled {
            background-color: #666;
            cursor: not-allowed;
            opacity: 0.7;
        }
        #updateButton:hover:not(:disabled) {
            background-color: #45a049;
        }
    </style>
</head>
<body>

    <h2>⚙️ Kontrol Kecepatan Motor (PID)</h2>
    <p>Target RPM: <span id="targetRpmDisplay" class="status-value target-zone">400</span> (Toleransi: <span id="deadbandDisplay"></span>)</p>

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

        <!-- Formulir Kontrol PID -->
        <div class="control-box">
            <h3>🛠️ Ubah Parameter PID</h3>
            <form id="pidForm">
                <div class="input-group">
                    <label for="target">Target RPM:</label>
                    <input type="number" id="target" name="target" step="10" min="0" required>
                </div>
                <div class="input-group">
                    <label for="kp">Gain Proportional (Kp):</label>
                    <input type="number" id="kp" name="kp" step="0.001" min="0" required>
                </div>
                <div class="input-group">
                    <label for="ki">Gain Integral (Ki):</label>
                    <input type="number" id="ki" name="ki" step="0.001" min="0" required>
                </div>
                <div class="input-group">
                    <label for="kd">Gain Derivative (Kd):</label>
                    <input type="number" id="kd" name="kd" step="0.001" min="0" required>
                </div>
                <div class="input-group">
                    <label for="deadband">Deadband RPM:</label>
                    <input type="number" id="deadband" name="deadband" step="1" min="0" required>
                </div>
                <button type="submit" id="updateButton">UBAH PARAMETER PID</button>
            </form>
        </div>

        <!-- Status Kontrol -->
        <div class="status-box">
            <h3>Status Kontrol PID Saat Ini</h3>
            <div class="status-item"><span>Output PWM (0-1023):</span><span id="pwmDisplay" class="status-value">0</span></div>
            <div class="status-item"><span>Error (Target - Aktual):</span><span id="errorDisplay" class="status-value">0.00</span></div>
            <div class="status-item"><span>Kp:</span><span id="kpDisplay" class="status-value">0.00</span></div>
            <div class="status-item"><span>Ki:</span><span id="kiDisplay" class="status-value">0.00</span></div>
            <div class="status-item"><span>Kd:</span><span id="kdDisplay" class="status-value">0.00</span></div>
        </div>
    </div>

    <script>
        let gaugeRPM;
        let isTyping = false; // Flag baru untuk mendeteksi input pengguna
        
        // Fungsi untuk mengambil data RPM dan PID, dan juga mengisi formulir
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
                const deadband = parseFloat(data.deadband) || 0;
                
                // --- 1. Update Gauge & Status Display ---
                if (gaugeRPM) {
                    gaugeRPM.value = currentRPM;
                }
                
                $('#pwmDisplay').text(currentPWM);
                $('#targetRpmDisplay').text(targetRPM);
                $('#deadbandDisplay').text(`± ${deadband} RPM`);
                $('#errorDisplay').text((targetRPM - currentRPM).toFixed(2));
                $('#kpDisplay').text(parseFloat(data.kp).toFixed(3));
                $('#kiDisplay').text(parseFloat(data.ki).toFixed(3));
                $('#kdDisplay').text(parseFloat(data.kd).toFixed(3));

                // --- 2. Isi Formulir dengan Nilai Saat Ini (Hanya jika pengguna TIDAK sedang mengetik) ---
                if (!isTyping) {
                    $('#target').val(targetRPM.toFixed(0));
                    $('#deadband').val(deadband.toFixed(0));
                    $('#kp').val(parseFloat(data.kp).toFixed(3));
                    $('#ki').val(parseFloat(data.ki).toFixed(3));
                    $('#kd').val(parseFloat(data.kd).toFixed(3));
                }

            })
            .catch(error => {
                console.error("❌ Gagal mengambil data PID:", error);
                if (gaugeRPM) gaugeRPM.value = 0;
            });
        };

        // Fungsi untuk mengirimkan data PID yang baru ke ESP32
        const sendPIDData = (event) => {
            event.preventDefault(); // Mencegah reload halaman
            const form = $('#pidForm');
            const data = form.serialize(); // Mengambil data form dalam format query string

            // Tampilkan loading/disable tombol
            const button = $('#updateButton');
            const originalText = button.text();
            button.prop('disabled', true).text('Mengirim & Validasi...');

            // Hapus fokus dari input (agar isTyping = false) sebelum mengirim
            $('input[type="number"]').blur(); 

            fetch('/set_pid', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded' // Tipe data form
                },
                body: data
            })
            .then(response => {
                if (!response.ok) {
                    return response.text().then(text => { throw new Error(text || 'Error tidak diketahui dari server.'); });
                }
                return response.text();
            })
            .then(() => {
                button.prop('disabled', false).text('BERHASIL DIUBAH!');
                // Ambil data lagi setelah berhasil untuk verifikasi dan update tampilan form
                fetchPIDData(); 
                setTimeout(() => {
                    button.text(originalText);
                }, 3000); 
            })
            .catch(error => {
                console.error("❌ Error POST data:", error);
                button.prop('disabled', false).text(`GAGAL! (${error.message || 'Cek koneksi'})`);
                setTimeout(() => {
                    button.text(originalText);
                }, 3000); 
            });
        };

        $(document).ready(function() {
            setTimeout(() => {
                const gauges = window.gauges || [];
                if (gauges.length > 0) {
                    gaugeRPM = gauges[0];
                } else {
                    console.error("Gauge RPM tidak terinisialisasi.");
                    return;
                }

                // --- Logika Baru: Deteksi Fokus Input ---
                $('input[type="number"]').on('focus', function() {
                    isTyping = true;
                }).on('blur', function() {
                    // Beri jeda singkat sebelum mengatur isTyping=false
                    // untuk menghindari konflik jika data fetch terjadi tepat setelah blur
                    setTimeout(() => {
                        isTyping = false;
                    }, 100); 
                });
                // --- Akhir Logika Deteksi Fokus Input ---


                // Attach event listener ke form submission
                $('#pidForm').on('submit', sendPIDData);

                // Mulai ambil data dan update status/form
                fetchPIDData();
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
    ledcAttach(MOTOR_PWM_PIN,PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_PWM_PIN, 0); // Pastikan motor mati saat startup 

    // *** 2. INISIALISASI WAKTU TACHO ***
    unsigned long currentTime = millis();
    portENTER_CRITICAL(&timerMux);
    lastUpdate = currentTime; 
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

    // *** 5. SETUP FREE RTOS TASK PID ***
    xTaskCreate(
      pidControl,      
      "PID Control",   
      4096,            
      NULL,            
      1,               
      NULL             
    );

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
    // Endpoint baru untuk menerima data PID
    server.on("/set_pid", HTTP_POST, handleSetPID); 
    
    server.begin();
}

void loop() {
    // Loop utama dibiarkan kosong
}
