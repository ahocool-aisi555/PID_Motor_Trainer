# PID Motor Trainer
Belajar konsep PID kontrol dengan menggunakan bahan yang ada di sekitar

<img width="1000" height="473" alt="ESPC3-PID" src="https://github.com/user-attachments/assets/0fef9a0b-11b2-404e-8ac4-e8a950cb1a15" />

# Awal Mula
- Setelah 20 tahun lebih mendapatkan pelajaran kontrol PID di kampus, dipaksa untuk memahaminya kembali akibat salah satu mata kuliah kontrol motor DC yang saya ajarkan.
- AI chat bot menjadi rujukan dan berpikir kenapa tidak bikin saja simulasinya ?
- Memanfaatkan bahan-bahan bekas yang ada di gudang tanpa butuh beli komponen lagi


# Bahan-bahan

1. Motor DC dan gear bekas dari penyemprot parfum yang sudah kacau timernya
2. ESP32 - versi yang saya punya C3 mini
3. Transistor BD139 dan resistor 1K sebagai driver PWM motor
4. Sensor photodiode atau modul jadi yang biasa dipake untuk counter / deteksi terhalang

<img width="311" height="298" alt="enkoder" src="https://github.com/user-attachments/assets/65d98b17-c4e7-4a62-ad63-4aed5c82c7aa" /><br>
5. Power supply 12 Volt untuk motor DC
# Coding PID 

Coding rumus PID ini full dibantu oleh AI chat bot, berikut potongannya :

```c++
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

    // 8. Terapkan Output ke Motor menggunakan analogWrite
    ledcWrite(MOTOR_PWM_PIN, (int)outputPWM);

    currentPWM = outputPWM; 

    vTaskDelay(pdMS_TO_TICKS(PID_TASK_DELAY_MS));
  }
}


```
# Hasil Output di Web Browser

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/3ZN0tLfTw6U/0.jpg)]([https://www.youtube.com/watch?v=OqzBcb2V9yw](https://www.youtube.com/watch?v=3ZN0tLfTw6U))
<br> Klik pada gambar untuk ke youtubenya <br>
Praktek ini diajarkan di kelas Elektronika Daya - Teknik Listrik - Fak Vokasi Unesa Surabaya
