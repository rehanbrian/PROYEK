#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAPPING PIN (TP1=A0, TP2=A1, TP3=A2)
const int pin_R_high[3] = {8, 10, 12};
const int pin_R_low[3]  = {7, 9, 11};
const int pin_ADC[3]    = {A0, A1, A2};
const int pin_Button = A3;

// PIN RGB
// CATATAN: redPin dan greenPin disesuaikan dengan wiring fisik PCB
const int redPin = 5;    // Pin 5 = kabel MERAH secara fisik
const int greenPin = 3;  // Pin 3 = kabel HIJAU secara fisik
const int bluePin = 6;

const int ledBrightness = 30;

// --- KONSTANTA FISIKA ---
const float R_LOW_REF = 680.0;
const float R_HIGH_REF = 470000.0;
const float PIN_R_INTERNAL = 22.0;
const float BREADBOARD_R = 1.5;
const int   ADC_OVERHEAD_US = 82;
const float C_ELCO_CALIBRATION = 0.875;

// Faktor koreksi per pasang pin (TP1-TP2, TP1-TP3, TP2-TP3) untuk menyamakan nilai
const float LOW_R_PAIR_CAL[3] = {0.9866, 0.9879, 0.9910};
const float HIGH_R_PAIR_CAL[3] = {1.0074, 1.0063, 1.0341};
const float CAP_PAIR_CAL[3] = {1.1154, 1.1155, 1.1155};
const float ADC_TO_VOLT = 5.0 / 1023.0;

// ==========================================
// KONSTANTA BATAS DETEKSI KERUSAKAN
// ==========================================
//
// HANYA Vbe dan hFE yang bisa diandalkan di rangkaian ini.
// Leakage dan junction symmetry DIHAPUS karena:
//   - Base mengambang + R_high 470k → noise pickup → false positif selalu
//   - Bahkan 1µA noise = drop 0.47V di 470kΩ → ADC drop besar → false RUSAK
//
// hFE normal: hampir semua BJT silicon 5 - 2000
// Jika hFE < 5: transistor sangat rusak (gain hampir hilang)
const float HFE_MIN_NORMAL      = 5.0;
const float HFE_MAX_NORMAL      = 2000.0;
//
// Vbe normal silicon BJT: 0.50V - 0.80V (diukur saat R_low bias ~6mA)
// < 0.40V: junction short/terbakar   → tegangan dioda turun drastis
// > 1.05V: junction terbuka/abnormal → tegangan terlalu tinggi
// CATATAN: BJT hFE tinggi (seperti BD139 hFE>200) cepat saturasi saat
// diukur dengan R_low 680Ω (~6mA base). Dalam saturasi, Vbe bisa
// mencapai 0.85-0.95V — masih NORMAL. Threshold 1.05V memberi ruang.
const float VBE_MIN_NORMAL      = 0.40;
const float VBE_MAX_NORMAL      = 1.05;
//
// MOSFET: perubahan ADC saat gate float vs ON
// Jika delta besar = gate bocor (oxide rusak, tidak bisa menyimpan muatan)
// 250 = toleransi cukup lebar agar tidak false positif
const int   MOSFET_FLOAT_DELTA  = 250;

// ==========================================
// Kode alasan kerusakan (gunakan integer, bukan String)
// ==========================================
#define RUSAK_OK            0
#define RUSAK_HFE_RENDAH    1
#define RUSAK_HFE_ABNORMAL  2
#define RUSAK_VBE_SHORT     3
#define RUSAK_VBE_OPEN      4
#define RUSAK_GATE_BOCOR    8

// Fungsi Prototype
void resetPins();
void kurasTeganganTotal();
void dischargeIfNeeded();
int getPairIndex(int i, int j);
void workingLED();
void playScanningAnimation();
void ledNormal();
void ledRusak();
void ledGagal();
void startMeasuringLED();
void stopLEDs();
float readADC_Avg(int pin, int samples);
void prosesPengukuran();
void prosesUkurKapasitor();
void prosesUkurInduktor();
bool prosesUkurTransistor();
void tampilkanHasil(int p1, int p2, float r_val);
void tampilkanHasilKapasitor(int p1, int p2, float c_val);
void tampilkanHasilInduktor(int p1, int p2, float l_val_uH);
void tampilkanHasilTransistor(bool isNPN, int b, int c, int e, float vbe, float hfe, uint8_t kodeRusak);
void tampilkanHasilMOSFET(bool isNMOS, int g, int d, int s, uint8_t kodeRusak);
void drawScanningProgress(int stage);
void jalankanPengukuran();
void formatFloat(float val, int decimals, char* buf);
void delayWithAnimation(int durationMs, int stage);

// ==========================================
// Variabel global untuk efek breathing LED standby (non-blocking)
unsigned long lastFadeTime = 0;
int fadeAmount = 2;
int currentBrightness = 0;

// Variabel untuk fitur Auto-Sleep
unsigned long lastActivityTime = 0;
bool isSleeping = false;
const unsigned long SLEEP_TIMEOUT = 30000; // 30 detik timeout
unsigned long lastHeartbeatTime = 0;       // Penanda kedipan sleep

void drawScanningProgress(int stage) {
  display.clearDisplay();
  
  // Draw outer frame
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  // Elegant corner brackets
  display.drawLine(4, 4, 12, 4, SH110X_WHITE);
  display.drawLine(4, 4, 4, 12, SH110X_WHITE);
  display.drawLine(123, 4, 115, 4, SH110X_WHITE);
  display.drawLine(123, 4, 123, 12, SH110X_WHITE);
  display.drawLine(4, 59, 12, 59, SH110X_WHITE);
  display.drawLine(4, 59, 4, 51, SH110X_WHITE);
  display.drawLine(123, 59, 115, 59, SH110X_WHITE);
  display.drawLine(123, 59, 123, 51, SH110X_WHITE);
  
  // Scanning... text at the top
  display.setCursor(32, 12);
  display.print(F("SCANNING..."));
  
  // Step Tracker Bar in the middle (Y=24)
  const char* labels[] = {"BJT", "RES", "CAP", "IND"};
  int centers[] = {25, 51, 77, 103};
  for (int i = 0; i < 4; i++) {
    int cx = centers[i];
    if (i == stage) {
      display.fillRect(cx - 11, 24, 22, 11, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }
    display.setCursor(cx - 9, 26);
    display.print(labels[i]);
  }
  display.setTextColor(SH110X_WHITE); // Restore color
  
  // Draw scanner box at the bottom
  display.drawRect(14, 40, 100, 20, SH110X_WHITE);
  
  // Sweeping laser with motion trail
  int scanX = 16 + ((millis() / 15) % 96);
  display.drawLine(scanX, 42, scanX, 58, SH110X_WHITE);
  if (scanX > 18) {
    display.drawLine(scanX - 2, 44, scanX - 2, 56, SH110X_WHITE);
  }
  if (scanX > 20) {
    display.drawLine(scanX - 4, 46, scanX - 4, 54, SH110X_WHITE);
  }
  
  display.display();
}

void updateScanner(int stage) {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 25) {
    lastUpdate = millis();
    drawScanningProgress(stage);
  }
}

void delayWithAnimation(int durationMs, int stage) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    updateScanner(stage);
    delay(2);
  }
}

void setup() {
  if (!display.begin(0x3C, true)) for(;;);
  pinMode(pin_Button, INPUT);
  pinMode(redPin, OUTPUT); pinMode(greenPin, OUTPUT); pinMode(bluePin, OUTPUT);
  resetPins();

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  
  // Outer frame
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  // Elegant corner brackets
  display.drawLine(4, 4, 12, 4, SH110X_WHITE);
  display.drawLine(4, 4, 4, 12, SH110X_WHITE);
  display.drawLine(123, 4, 115, 4, SH110X_WHITE);
  display.drawLine(123, 4, 123, 12, SH110X_WHITE);
  display.drawLine(4, 59, 12, 59, SH110X_WHITE);
  display.drawLine(4, 59, 4, 51, SH110X_WHITE);
  display.drawLine(123, 59, 115, 59, SH110X_WHITE);
  display.drawLine(123, 59, 123, 51, SH110X_WHITE);
  
  display.setTextSize(1);
  display.setCursor(14, 16);
  display.print(F("SMART LCR-T METER"));
  
  // Elegant divider line
  display.drawLine(16, 28, 112, 28, SH110X_WHITE);
  
  // Banner READY terbalik (simple rect saves memory)
  display.fillRect(29, 36, 70, 18, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(35, 37);
  display.setTextSize(2);
  display.print(F("READY"));
  
  // Kembalikan warna teks default
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.display();
  
  lastActivityTime = millis();
}

void resetPins() {
  for (int i = 0; i < 3; i++) {
    pinMode(pin_R_high[i], INPUT);
    pinMode(pin_R_low[i], INPUT);
    pinMode(pin_ADC[i], INPUT);
  }
}

void dischargeChargedCapacitor() {
  for (int i = 0; i < 3; i++) {
    pinMode(pin_ADC[i], INPUT);
    pinMode(pin_R_high[i], OUTPUT); digitalWrite(pin_R_high[i], LOW);
    pinMode(pin_R_low[i], OUTPUT);  digitalWrite(pin_R_low[i], LOW);
  }
  delayMicroseconds(500);
  
  int v0 = analogRead(A0); int v1 = analogRead(A1); int v2 = analogRead(A2);
  int startV = max(v0, max(v1, v2));
  
  if (startV > 15) { // Hanya jika ada tegangan yang signifikan
    unsigned long startKuras = millis();
    bool oledUpdated = false;
    int confirm_count = 0;
    
    while (millis() - startKuras < 5000) { // Max 5 detik untuk Elco raksasa
      int c0 = analogRead(A0); int c1 = analogRead(A1); int c2 = analogRead(A2);
      int currentV = max(c0, max(c1, c2));
      
      if (currentV < 3) {
        confirm_count++;
        if (confirm_count >= 5) break;
      } else {
        confirm_count = 0;
      }
      
      if (!oledUpdated) {
        oledUpdated = true;
        stopLEDs(); // Matikan LED scan sejenak
      }
      
      static unsigned long lastAnimTime = 0;
      if (millis() - lastAnimTime > 50) {
        lastAnimTime = millis();
        
        display.clearDisplay();
        display.drawRect(0, 0, 128, 64, SH110X_WHITE);
        display.drawLine(4, 4, 12, 4, SH110X_WHITE);
        display.drawLine(4, 4, 4, 12, SH110X_WHITE);
        display.drawLine(123, 4, 115, 4, SH110X_WHITE);
        display.drawLine(123, 4, 123, 12, SH110X_WHITE);
        display.drawLine(4, 59, 12, 59, SH110X_WHITE);
        display.drawLine(4, 59, 4, 51, SH110X_WHITE);
        display.drawLine(123, 59, 115, 59, SH110X_WHITE);
        display.drawLine(123, 59, 123, 51, SH110X_WHITE);
        
        display.fillRect(1, 1, 126, 11, SH110X_WHITE);
        display.setTextColor(SH110X_BLACK);
        display.setCursor(20, 3); display.print(F("DISCHARGING ELCO"));
        display.setTextColor(SH110X_WHITE);
        
        display.drawLine(44, 30, 58, 30, SH110X_WHITE);
        display.drawLine(58, 20, 58, 40, SH110X_WHITE);
        display.drawLine(62, 20, 62, 40, SH110X_WHITE);
        display.drawLine(62, 30, 76, 30, SH110X_WHITE);
        
        int sparkFrame = (millis() / 150) % 3;
        if (sparkFrame == 0) {
          display.drawLine(38, 18, 42, 22, SH110X_WHITE);
          display.drawLine(82, 42, 78, 38, SH110X_WHITE);
        } else if (sparkFrame == 1) {
          display.drawLine(82, 18, 78, 22, SH110X_WHITE);
          display.drawLine(38, 42, 42, 38, SH110X_WHITE);
        }
        
        int barX = 16;
        int barY = 48;
        int barW = 96;
        int barH = 8;
        display.drawRect(barX, barY, barW, barH, SH110X_WHITE);
        
        int maxFillW = barW - 4;
        int fillW = 0;
        if (startV > 3) {
          fillW = map(currentV, 3, startV, 0, maxFillW);
        }
        fillW = constrain(fillW, 0, maxFillW);
        display.fillRect(barX + 2, barY + 2, fillW, barH - 4, SH110X_WHITE);
        
        display.display();
      }
      delay(5);
    }
  }
  resetPins();
}

void kurasTeganganTotal() {
  for (int i = 0; i < 3; i++) {
    pinMode(pin_ADC[i], INPUT);
    pinMode(pin_R_high[i], OUTPUT); digitalWrite(pin_R_high[i], LOW);
    pinMode(pin_R_low[i], OUTPUT);  digitalWrite(pin_R_low[i], LOW);
  }
  
  unsigned long startKuras = millis();
  while (millis() - startKuras < 10) { // Maksimum 10ms (sangat cepat!)
    int c0 = analogRead(A0); int c1 = analogRead(A1); int c2 = analogRead(A2);
    int currentV = max(c0, max(c1, c2));
    if (currentV < 3) break;
    delayMicroseconds(500);
  }
  resetPins();
}


void dischargeIfNeeded() {
  // Hubungkan semua pin ke GND melalui R_low (680 ohm) agar muatan kapasitor ter-referensi ke GND
  for (int k = 0; k < 3; k++) {
    pinMode(pin_R_low[k], OUTPUT);
    digitalWrite(pin_R_low[k], LOW);
  }
  delayMicroseconds(500); // Beri waktu transient sesaat
  int v0 = analogRead(A0); int v1 = analogRead(A1); int v2 = analogRead(A2);
  
  if (v0 > 4 || v1 > 4 || v2 > 4) {
    // Ada muatan! Lakukan pengosongan aktif
    unsigned long start = millis();
    while (millis() - start < 1000) {
      int c0 = analogRead(A0); int c1 = analogRead(A1); int c2 = analogRead(A2);
      if (max(c0, max(c1, c2)) < 3) break;
      delay(2);
    }
  }
  resetPins();
}

int getPairIndex(int i, int j) {
  int sum = i + j;
  if (sum == 1) return 0; // TP1-TP2
  if (sum == 2) return 1; // TP1-TP3
  return 2;               // TP2-TP3
}

void workingLED() {
  // Kosong, warna LED dikendalikan oleh modul scanning masing-masing
}

void playScanningAnimation() {
  // Animasi radar kotak melingkar halus dari tepi tulisan (simple rect saves memory)
  for (int h = 14; h <= 54; h += 4) {
    display.clearDisplay();
    
    int w = h + 42;
    int x = 64 - w / 2;
    int y = 32 - h / 2;
    
    display.drawRect(x, y, w, h, SH110X_WHITE);
    
    display.setCursor(38, 28);
    display.setTextSize(1);
    display.print(F("SCANNING"));
    display.display();
    
    int br = map(h, 14, 54, 3, 20);
    analogWrite(redPin, br);
    analogWrite(bluePin, br + 5);
    delay(40);
  }
  stopLEDs();
}

void startMeasuringLED() {
  analogWrite(redPin, 20);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 25);
}

void stopLEDs() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

float readADC_Avg(int pin, int samples) {
  unsigned long sum = 0;
  for (int i = 0; i < samples; i++) { sum += analogRead(pin); delay(1); }
  return (float)sum / samples;
}

void formatFloat(float val, int decimals, char* buf) {
  float rounding = 0.5;
  for (int i = 0; i < decimals; i++) rounding /= 10.0;
  
  long multiplier = 1;
  for (int i = 0; i < decimals; i++) multiplier *= 10;
  
  long total = (long)(val * multiplier + (val >= 0.0 ? rounding * multiplier : -rounding * multiplier));
  if (total < 0) {
    *buf++ = '-';
    total = -total;
  }
  long ipart = total / multiplier;
  long fpart = total % multiplier;
  
  ltoa(ipart, buf, 10);
  if (decimals > 0) {
    int len = strlen(buf);
    buf[len] = '.';
    char fbuf[10];
    ltoa(fpart, fbuf, 10);
    int flen = strlen(fbuf);
    int idx = len + 1;
    for (int i = 0; i < decimals - flen; i++) {
      buf[idx++] = '0';
    }
    strcpy(buf + idx, fbuf);
  }
}

void jalankanPengukuran() {
  if (isSleeping) {
    isSleeping = false;
    display.oled_command(0xAF); // Bangunkan OLED
  } else {
    stopLEDs();
  }
  
  playScanningAnimation();
  dischargeChargedCapacitor();
  kurasTeganganTotal();
  if (!prosesUkurTransistor()) prosesPengukuran();
  
  stopLEDs();
  lastActivityTime = millis();
  
  display.setTextSize(1);
  while (digitalRead(pin_Button) == LOW);
  
  currentBrightness = 0;
  fadeAmount = 2;
}

void loop() {
  if (isSleeping) {
    if (millis() - lastHeartbeatTime > 5000) {
      lastHeartbeatTime = millis();
      analogWrite(bluePin, 15);
      delay(15);
      analogWrite(bluePin, 0);
    }

    if (digitalRead(pin_Button) == LOW) {
      delay(50);
      if (digitalRead(pin_Button) == LOW) {
        jalankanPengukuran();
      }
    }
    return;
  }

  if (millis() - lastFadeTime > 25) {
    lastFadeTime = millis();
    currentBrightness += fadeAmount;
    if (currentBrightness <= 0 || currentBrightness >= 25) {
      fadeAmount = -fadeAmount;
    }
    analogWrite(bluePin, max(0, min(currentBrightness, 25)));
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
  }

  if (millis() - lastActivityTime > SLEEP_TIMEOUT) {
    isSleeping = true;
    lastHeartbeatTime = millis();
    stopLEDs();
    display.oled_command(0xAE);
    return;
  }

  if (digitalRead(pin_Button) == LOW) {
    delay(50);
    if (digitalRead(pin_Button) == LOW) {
      jalankanPengukuran();
    }
  }
}

// ==========================================
// 1. RESISTOR
// ==========================================
void prosesPengukuran() {
  // LED Kuning (Lembut)
  analogWrite(redPin, 20); analogWrite(greenPin, 10); analogWrite(bluePin, 0);
  delayWithAnimation(300, 1); // stage 1 = RES
  dischargeIfNeeded(); // Pastikan kapasitor dikosongkan sebelum loop dimulai

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) continue;
      updateScanner(1);

      dischargeIfNeeded(); // Kosongkan di setiap kombinasi agar tidak ada muatan sisa
      pinMode(pin_R_low[i], OUTPUT); digitalWrite(pin_R_low[i], HIGH);
      pinMode(pin_R_low[j], OUTPUT); digitalWrite(pin_R_low[j], LOW);

      delay(2);
      float v_cek1 = readADC_Avg(pin_ADC[i], 5);
      delay(2);
      float v_cek2 = readADC_Avg(pin_ADC[i], 5);
      if (abs(v_cek2 - v_cek1) > 2.0) continue;

      analogRead(pin_ADC[i]); analogRead(pin_ADC[i]);
      float val_high = readADC_Avg(pin_ADC[i], 15);
      analogRead(pin_ADC[j]); analogRead(pin_ADC[j]);
      float val_low  = readADC_Avg(pin_ADC[j], 15);

      if (val_high < 1020.0 && val_low > 2.0 && (val_high - val_low) > 2.0) {
        float current_R = (R_LOW_REF + PIN_R_INTERNAL) * (val_high - val_low) / val_low;
        current_R *= LOW_R_PAIR_CAL[getPairIndex(i, j)];
        if (current_R < 100.0 && current_R > BREADBOARD_R) current_R -= BREADBOARD_R;
        if (current_R > 0.5 && current_R < 15000.0) {
          tampilkanHasil(i + 1, j + 1, current_R);
          resetPins(); return;
        }
      }

      dischargeIfNeeded();

      pinMode(pin_R_high[i], OUTPUT); digitalWrite(pin_R_high[i], HIGH);
      pinMode(pin_R_low[j], OUTPUT);  digitalWrite(pin_R_low[j], LOW);

      delay(2);
      if (analogRead(pin_ADC[i]) < 12) continue;

      // Tunggu lebih lama sebelum cek stabilitas (settling time untuk R besar)
      delay(15);
      v_cek1 = readADC_Avg(pin_ADC[i], 6);
      delay(30);
      v_cek2 = readADC_Avg(pin_ADC[i], 6);
      // Jika sinyal tidak stabil (masih charging = kapasitor) lewati
      if ((v_cek2 - v_cek1) > 5.0) continue;

      delay(10);
      analogRead(pin_ADC[i]); analogRead(pin_ADC[i]);
      val_high = readADC_Avg(pin_ADC[i], 40);
      analogRead(pin_ADC[j]); analogRead(pin_ADC[j]);
      val_low  = readADC_Avg(pin_ADC[j], 40);

      if (val_high < 1022.0 && (val_high - val_low) > 0.5) {
        float current_R = R_HIGH_REF * (val_high - val_low) / (1023.0 - val_high);
        int pairIdx = getPairIndex(i, j);
        current_R *= HIGH_R_PAIR_CAL[pairIdx];

        if (current_R >= 15000.0 && current_R <= 2000000.0) { // Naikkan ke 2M ohm agar resistor 1M yang terbaca sedikit tinggi tidak terbuang
          dischargeIfNeeded();

          pinMode(pin_R_high[j], OUTPUT); digitalWrite(pin_R_high[j], HIGH);
          pinMode(pin_R_low[i], OUTPUT);  digitalWrite(pin_R_low[i], LOW);
          delay(10);

          analogRead(pin_ADC[j]); analogRead(pin_ADC[j]);
          float rev_val_high = readADC_Avg(pin_ADC[j], 40);
          analogRead(pin_ADC[i]); analogRead(pin_ADC[i]);
          float rev_val_low  = readADC_Avg(pin_ADC[i], 40);

          float rev_R = R_HIGH_REF * (rev_val_high - rev_val_low) / (1023.0 - rev_val_high);
          rev_R *= HIGH_R_PAIR_CAL[pairIdx];

          // R > 700kΩ: toleransi cross-check diperlebar ke 40%
          // karena micro-leakage pin ADC menyebabkan deviasi
          // proporsional lebih besar di impedansi tinggi
          float tol = (current_R > 700000.0) ? 0.40 : 0.20;
          if (abs(current_R - rev_R) / current_R < tol) {
            tampilkanHasil(i + 1, j + 1, (current_R + rev_R) / 2.0);
            resetPins(); return;
          }
        }
      }
    }
  }
  prosesUkurKapasitor();
}

// ==========================================
// 2. KAPASITOR
// ==========================================
void prosesUkurKapasitor() {
  // LED Pink/Rose (Lembut)
  analogWrite(redPin, 20); analogWrite(greenPin, 0); analogWrite(bluePin, 10);
  delayWithAnimation(300, 2); // stage 2 = CAP
  unsigned long startTime, elapsedTime;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) continue;
      updateScanner(2);
      dischargeIfNeeded();

      pinMode(pin_ADC[j], OUTPUT); digitalWrite(pin_ADC[j], LOW);
      startTime = micros();
      pinMode(pin_R_low[i], OUTPUT); digitalWrite(pin_R_low[i], HIGH);

      while (analogRead(pin_ADC[i]) < 647) {
        if ((micros() - startTime) > 2000000UL) break;
      }
      elapsedTime = micros() - startTime;
      resetPins();

      if (elapsedTime < 1990000UL) {
        if (elapsedTime > (unsigned long)ADC_OVERHEAD_US) elapsedTime -= ADC_OVERHEAD_US;
        if (elapsedTime > 300 && elapsedTime < 2000000UL) {
          float c_val_uF = ((float)elapsedTime / R_LOW_REF) * C_ELCO_CALIBRATION;
          c_val_uF *= CAP_PAIR_CAL[getPairIndex(i, j)];
          tampilkanHasilKapasitor(i + 1, j + 1, c_val_uF);
          return;
        }
      }

      dischargeIfNeeded();

      pinMode(pin_ADC[j], OUTPUT); digitalWrite(pin_ADC[j], LOW);
      startTime = micros();
      pinMode(pin_R_high[i], OUTPUT); digitalWrite(pin_R_high[i], HIGH);

      while (analogRead(pin_ADC[i]) < 647) {
        if ((micros() - startTime) > 500000UL) break;
      }
      elapsedTime = micros() - startTime;
      resetPins();

      if (elapsedTime < 495000UL) {
        if (elapsedTime > (unsigned long)ADC_OVERHEAD_US) elapsedTime -= ADC_OVERHEAD_US;
        if (elapsedTime > 300 && elapsedTime < 500000UL) {
          float c_val_uF = (float)elapsedTime / R_HIGH_REF;
          tampilkanHasilKapasitor(i + 1, j + 1, c_val_uF);
          return;
        }
      }
    }
  }
  prosesUkurInduktor();
}

// ==========================================
// 3. INDUKTOR
// ==========================================
void prosesUkurInduktor() {
  // LED Putih (Lembut)
  analogWrite(redPin, 15); analogWrite(greenPin, 15); analogWrite(bluePin, 20);
  delayWithAnimation(500, 3); // stage 3 = IND
  unsigned int timerTicks = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) continue;
      updateScanner(3);
      resetPins();

      pinMode(pin_ADC[i], OUTPUT); digitalWrite(pin_ADC[i], LOW);
      pinMode(pin_ADC[j], OUTPUT); digitalWrite(pin_ADC[j], LOW);
      delay(10);
      pinMode(pin_ADC[i], INPUT);

      ADCSRA &= ~(1 << ADEN);
      ADCSRB |= (1 << ACME);

      int channel = 0;
      if (pin_ADC[i] == A1) channel = 1;
      if (pin_ADC[i] == A2) channel = 2;
      ADMUX = channel;

      ACSR = (1 << ACBG);
      delayMicroseconds(100);

      pinMode(pin_ADC[j], OUTPUT); digitalWrite(pin_ADC[j], LOW);

      TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
      noInterrupts();
      pinMode(pin_R_low[i], OUTPUT); digitalWrite(pin_R_low[i], HIGH);
      TCCR1B = (1 << CS10);

      while ((ACSR & (1 << ACO)) == 0) {
        if (TCNT1 > 60000) break;
      }
      timerTicks = TCNT1;
      interrupts();

      TCCR1B = 0;
      pinMode(pin_R_low[i], INPUT);
      ADCSRA |= (1 << ADEN);

      if (timerTicks > 10 && timerTicks < 60000) {
        float l_val_uH = timerTicks * 20.62;
        if (l_val_uH > 20.0) {
          tampilkanHasilInduktor(i + 1, j + 1, l_val_uH);
          resetPins(); return;
        }
      }
    }
  }

  resetPins();
  stopLEDs();
  delay(400); // Jeda mati sebelum hasil kosong/gagal muncul
  display.clearDisplay();
  
  // Frame luar premium
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  
  // Header bar terbalik
  display.fillRect(1, 1, 126, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(6, 3); display.setTextSize(1);
  display.print(F("WARNING / ERROR"));
  display.setTextColor(SH110X_WHITE); // Kembalikan warna default
  
  // Kotak dialog peringatan di dalam
  display.drawRect(8, 16, 112, 42, SH110X_WHITE);
  display.setCursor(22, 24); display.print(F("Komponen TIDAK"));
  display.setCursor(16, 36); display.print(F("TERDETEKSI/RUSAK"));
  
  display.display();
  ledGagal();
}

// ==========================================
// 4. SEMIKONDUKTOR
// ==========================================



bool prosesUkurTransistor() {
  // LED Cyan (Lembut)
  analogWrite(redPin, 0); analogWrite(greenPin, 15); analogWrite(bluePin, 20);
  delayWithAnimation(300, 0); // stage 0 = BJT
 
  float best_hfe = 0;
  bool  best_isNPN = true;
  int   best_b = -1, best_c = -1, best_e = -1;
  float best_vbe = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) continue;
      for (int k = 0; k < 3; k++) {
        if (k == i || k == j) continue;
        updateScanner(0);

        // ---- N-MOSFET ----
        kurasTeganganTotal();
        pinMode(pin_ADC[k], OUTPUT); digitalWrite(pin_ADC[k], LOW); // Source ke GND
        pinMode(pin_R_low[j], OUTPUT); digitalWrite(pin_R_low[j], HIGH); // Drain pulled HIGH
        pinMode(pin_ADC[i], OUTPUT); digitalWrite(pin_ADC[i], LOW); // Gate = LOW
        delay(2); int n_off1 = analogRead(pin_ADC[j]);
        
        digitalWrite(pin_ADC[i], HIGH); // Gate = HIGH
        delay(2); int n_on = analogRead(pin_ADC[j]);
        
        pinMode(pin_ADC[i], INPUT); // Float Gate
        delayMicroseconds(100);
        int n_float = analogRead(pin_ADC[j]);
        
        pinMode(pin_ADC[i], OUTPUT); digitalWrite(pin_ADC[i], LOW); // Gate = LOW
        delay(2); int n_off2 = analogRead(pin_ADC[j]);

        if (n_off1 > 700 && n_on < 650 && n_float < 650 && n_off2 > 700) {
          tampilkanHasilMOSFET(true, i+1, j+1, k+1, RUSAK_OK);
          resetPins(); return true;
        }

        // ---- P-MOSFET ----
        kurasTeganganTotal();
        pinMode(pin_ADC[k], OUTPUT); digitalWrite(pin_ADC[k], HIGH); // Source ke 5V
        pinMode(pin_R_low[j], OUTPUT); digitalWrite(pin_R_low[j], LOW); // Drain pulled LOW
        pinMode(pin_ADC[i], OUTPUT); digitalWrite(pin_ADC[i], HIGH); // Gate = HIGH
        delay(2); int p_off1 = analogRead(pin_ADC[j]);
        
        digitalWrite(pin_ADC[i], LOW); // Gate = LOW
        delay(2); int p_on = analogRead(pin_ADC[j]);
        
        pinMode(pin_ADC[i], INPUT); // Float Gate
        delayMicroseconds(100);
        int p_float = analogRead(pin_ADC[j]);
        
        pinMode(pin_ADC[i], OUTPUT); digitalWrite(pin_ADC[i], HIGH); // Gate = HIGH
        delay(2); int p_off2 = analogRead(pin_ADC[j]);

        if (p_off1 < 350 && p_on > 350 && p_float > 350 && p_off2 < 350) {
          tampilkanHasilMOSFET(false, i+1, j+1, k+1, RUSAK_OK);
          resetPins(); return true;
        }

        // ---- NPN BJT ----
        kurasTeganganTotal();
        pinMode(pin_R_low[i], OUTPUT); digitalWrite(pin_R_low[i], HIGH);
        pinMode(pin_ADC[k], OUTPUT);   digitalWrite(pin_ADC[k], LOW);
        pinMode(pin_R_low[j], OUTPUT); digitalWrite(pin_R_low[j], HIGH);
        delay(2);

        int b_val = analogRead(pin_ADC[i]);
        int c_val = analogRead(pin_ADC[j]);

        if (b_val > 50 && b_val < 400 && c_val < b_val) {
          float vbe = b_val * ADC_TO_VOLT;
          pinMode(pin_R_low[i], INPUT);
          pinMode(pin_R_high[i], OUTPUT); digitalWrite(pin_R_high[i], HIGH);
          delay(2);
          int b_new = analogRead(pin_ADC[i]);
          int c_new = analogRead(pin_ADC[j]);
          float denom = (float)(1023 - b_new) * R_LOW_REF;
          float hfe = 0;
          if (denom > 0) {
            hfe = ((float)(1023 - c_new) * R_HIGH_REF) / denom;
          }
          if (hfe >= 0.0 && hfe > best_hfe) {
            best_hfe = hfe; best_isNPN = true;
            best_b = i+1; best_c = j+1; best_e = k+1; best_vbe = vbe;
          }
        }

        // ---- PNP BJT ----
        kurasTeganganTotal();
        pinMode(pin_R_low[i], OUTPUT); digitalWrite(pin_R_low[i], LOW);
        pinMode(pin_ADC[k], OUTPUT);   digitalWrite(pin_ADC[k], HIGH);
        pinMode(pin_R_low[j], OUTPUT); digitalWrite(pin_R_low[j], LOW);
        delay(2);

        b_val = analogRead(pin_ADC[i]);
        c_val = analogRead(pin_ADC[j]);

        if (b_val > 600 && b_val < 970 && c_val > b_val) {
          float vbe = 5.0 - (b_val * ADC_TO_VOLT);
          pinMode(pin_R_low[i], INPUT);
          pinMode(pin_R_high[i], OUTPUT); digitalWrite(pin_R_high[i], LOW);
          delay(2);
          int b_new = analogRead(pin_ADC[i]);
          int c_new = analogRead(pin_ADC[j]);
          float denom = (float)b_new * R_LOW_REF;
          float hfe = 0;
          if (denom > 0) {
            hfe = ((float)c_new * R_HIGH_REF) / denom;
          }
          if (hfe >= 0.0 && hfe > best_hfe) {
            best_hfe = hfe; best_isNPN = false;
            best_b = i+1; best_c = j+1; best_e = k+1; best_vbe = vbe;
          }
        }
      }
    }
  }

  // ============================================================
  // Evaluasi kerusakan BJT
  // Hanya 2 parameter yang reliabel di rangkaian ini:
  //   1. hFE (penguatan) - drop drastis = transistor rusak
  //   2. Vbe (tegangan junction) - keluar range = junction rusak
  // Leakage dan junction symmetry TIDAK digunakan (terlalu banyak
  // false positif karena base floating + R_high besar = noise pickup)
  // ============================================================
  if (best_hfe > 0) {
    uint8_t kode = RUSAK_OK;

    if      (best_hfe < HFE_MIN_NORMAL)  kode = RUSAK_HFE_RENDAH;   // gain hilang
    else if (best_hfe > HFE_MAX_NORMAL)  kode = RUSAK_HFE_ABNORMAL; // tidak wajar
    else if (best_vbe < VBE_MIN_NORMAL)  kode = RUSAK_VBE_SHORT;    // junction short
    else if (best_vbe > VBE_MAX_NORMAL)  kode = RUSAK_VBE_OPEN;     // junction terbuka
    // else: RUSAK_OK - transistor normal

    tampilkanHasilTransistor(best_isNPN, best_b, best_c, best_e,
                              best_vbe, best_hfe, kode);
    resetPins();
    return true;
  }

  // Jika gagal terdeteksi sebagai BJT/MOSFET normal, lakukan cek apakah ini semikonduktor rusak/short
  // (menghantarkan arus pada >= 2 pasang pin)
  bool pair_conducts[3] = {false, false, false};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) continue;
      dischargeIfNeeded(); // Kosongkan muatan sisa sebelum pengetesan pasangan pin ini
      resetPins();
      pinMode(pin_R_low[i], OUTPUT); digitalWrite(pin_R_low[i], HIGH);
      pinMode(pin_R_low[j], OUTPUT); digitalWrite(pin_R_low[j], LOW);
      delay(5); // Jeda dinaikkan ke 5ms agar kapasitor sempat terisi penuh dan arus transient turun ke 0
      int v_high = analogRead(pin_ADC[i]);
      int v_low = analogRead(pin_ADC[j]);
      if (v_high < 1000 || v_low > 20) {
        if ((v_high - v_low) < 900) { // Harus hantaran DC kontinu (diode/short), bukan pengisian kapasitor
          pair_conducts[getPairIndex(i, j)] = true; 
        }
      }
    }
  }
  int num_conducting_pairs = 0;
  for (int p = 0; p < 3; p++) {
    if (pair_conducts[p]) num_conducting_pairs++;
  }
  if (num_conducting_pairs >= 2) {
    stopLEDs();
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SH110X_WHITE);
    
    display.fillRect(1, 1, 126, 11, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(6, 3); display.setTextSize(1);
    display.print(F("WARNING / ERROR"));
    display.setTextColor(SH110X_WHITE);
    
    display.drawRect(8, 16, 112, 42, SH110X_WHITE);
    display.setCursor(25, 24); display.print(F("Semikonduktor"));
    display.setCursor(25, 36); display.print(F("RUSAK / SHORT"));
    display.display();
    
    ledRusak();
    resetPins();
    return true; // Return true agar tidak lanjut ke prosesPengukuran()
  }

  return false;
}

// ==========================================
// 5. SIMBOL KOMPONEN
// ==========================================
void drawResistorSymbol(int x, int y) {
  display.drawLine(x,      y+10, x+10,  y+10, SH110X_WHITE);
  display.drawLine(x+10,  y+10, x+12,  y+5,  SH110X_WHITE);
  display.drawLine(x+12,  y+5,  x+16,  y+15, SH110X_WHITE);
  display.drawLine(x+16,  y+15, x+20,  y+5,  SH110X_WHITE);
  display.drawLine(x+20,  y+5,  x+24,  y+15, SH110X_WHITE);
  display.drawLine(x+24,  y+15, x+28,  y+5,  SH110X_WHITE);
  display.drawLine(x+28,  y+5,  x+30,  y+10, SH110X_WHITE);
  display.drawLine(x+30,  y+10, x+40,  y+10, SH110X_WHITE);
}

void drawCapacitorSymbol(int x, int y) {
  display.drawLine(x,     y+10, x+16, y+10, SH110X_WHITE);
  display.drawLine(x+16,  y+2,  x+16, y+18, SH110X_WHITE);
  display.drawLine(x+20,  y+2,  x+20, y+18, SH110X_WHITE);
  display.drawLine(x+20,  y+10, x+36, y+10, SH110X_WHITE);
}

void drawInductorSymbol(int x, int y_center) {
  display.drawLine(x, y_center, x+5, y_center, SH110X_WHITE);
  int cx0 = x + 5;
  for (int i = 0; i < 4; i++) {
    int o = cx0 + i*8;
    display.drawLine(o, y_center, o+2, y_center-4, SH110X_WHITE);
    display.drawLine(o+2, y_center-4, o+5, y_center-4, SH110X_WHITE);
    display.drawLine(o+5, y_center-4, o+8, y_center, SH110X_WHITE);
  }
  int end_x = cx0 + 32;
  display.drawLine(end_x, y_center, end_x+5, y_center, SH110X_WHITE);
}

void tampilkanHasil(int p1, int p2, float r_val) {
  stopLEDs();
  delay(400); // Jeda mati sebelum hasil muncul
  display.clearDisplay();
  
  // Frame luar premium
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  
  // Header bar terbalik
  display.fillRect(1, 1, 126, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(6, 3); display.setTextSize(1);
  display.print(F("RESISTOR"));
  display.setTextColor(SH110X_WHITE); // Kembalikan warna default
  
  // Diagram Grafis Koneksi Fisik: [TP1] --[==]-- [TP2]
  // TP1 Box (simple rect saves memory)
  display.drawRect(12, 14, 28, 12, SH110X_WHITE);
  display.setCursor(18, 16); display.print(F("TP")); display.print(p1);
  
  // Connection line left
  display.drawLine(40, 20, 44, 20, SH110X_WHITE);
  // Resistor symbol
  drawResistorSymbol(44, 10);
  // Connection line right
  display.drawLine(84, 20, 88, 20, SH110X_WHITE);
  
  // TP2 Box (simple rect saves memory)
  display.drawRect(88, 14, 28, 12, SH110X_WHITE);
  display.setCursor(94, 16); display.print(F("TP")); display.print(p2);
  
  // Divider line
  display.drawLine(8, 29, 120, 29, SH110X_WHITE);
  
  // Hitung Nilai Resistansi Centered tanpa String object
  char valBuf[16];
  char unitChar = ' ';
  if (r_val >= 1000000.0) {
    formatFloat(r_val / 1000000.0, 2, valBuf);
    unitChar = 'M';
  } else if (r_val >= 1000.0) {
    formatFloat(r_val / 1000.0, 2, valBuf);
    unitChar = 'k';
  } else {
    formatFloat(r_val, 1, valBuf);
  }
  
  int len = strlen(valBuf);
  int textLen = len + 2; // +1 unitChar/spasi, +1 ohm symbol
  int startX = (128 - (textLen * 12)) / 2;
  display.setTextSize(2);
  display.setCursor(startX, 33);
  display.print(valBuf);
  if (unitChar != ' ') {
    display.print(unitChar);
  } else {
    display.print(' ');
  }
  display.write(233); // Tulis simbol Ohm
  
  // Badge warning "Bandingkan dg gelang" di dasar layar (simple rect saves memory)
  display.fillRect(2, 51, 124, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setTextSize(1);
  display.setCursor(1, 53);
  display.print(F("Bandingkan dgn gelang"));
  display.setTextColor(SH110X_WHITE);
  
  display.display();
  ledNormal();
}

void tampilkanHasilKapasitor(int p1, int p2, float c_val_uF) {
  stopLEDs();
  delay(400); // Jeda mati sebelum hasil muncul
  display.clearDisplay();
  
  // Frame luar premium
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  
  // Header bar terbalik
  display.fillRect(1, 1, 126, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(6, 3); display.setTextSize(1);
  display.print(F("KAPASITOR"));
  display.setTextColor(SH110X_WHITE); // Kembalikan warna default
  
  // Diagram Grafis Koneksi Fisik: [TP1] --||-- [TP2]
  // TP1 Box (simple rect saves memory)
  display.drawRect(12, 14, 28, 12, SH110X_WHITE);
  display.setCursor(18, 16); display.print(F("TP")); display.print(p1);
  
  // Connection line left
  display.drawLine(40, 20, 46, 20, SH110X_WHITE);
  // Capacitor symbol
  drawCapacitorSymbol(46, 10);
  // Connection line right
  display.drawLine(82, 20, 88, 20, SH110X_WHITE);
  
  // TP2 Box (simple rect saves memory)
  display.drawRect(88, 14, 28, 12, SH110X_WHITE);
  display.setCursor(94, 16); display.print(F("TP")); display.print(p2);
  
  // Divider line
  display.drawLine(8, 29, 120, 29, SH110X_WHITE);
  
  // Hitung Nilai Kapasitansi Centered tanpa String object
  char valBuf[16];
  const char* unitStr = "";
  if (c_val_uF >= 1000.0) {
    formatFloat(c_val_uF / 1000.0, 2, valBuf);
    unitStr = " mF";
  } else if (c_val_uF >= 1.0) {
    formatFloat(c_val_uF, 2, valBuf);
    unitStr = " uF";
  } else if (c_val_uF >= 0.001) {
    formatFloat(c_val_uF * 1000.0, 1, valBuf);
    unitStr = " nF";
  } else {
    formatFloat(c_val_uF * 1000000.0, 0, valBuf);
    unitStr = " pF";
  }
  
  int textLen = strlen(valBuf) + strlen(unitStr);
  int startX = (128 - (textLen * 12)) / 2;
  display.setTextSize(2);
  display.setCursor(startX, 33);
  display.print(valBuf);
  display.print(unitStr);
  
  // Cetak info di dasar layar (Tanpa warning text)
  display.setTextSize(1);
  display.setCursor(7, 53);
  display.print(F("Komponen Terdeteksi"));
  
  display.display();
  ledNormal();
}

void tampilkanHasilInduktor(int p1, int p2, float l_val_uH) {
  stopLEDs();
  delay(400); // Jeda mati sebelum hasil muncul
  display.clearDisplay();
  
  // Frame luar premium
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  
  // Header bar terbalik
  display.fillRect(1, 1, 126, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(6, 3); display.setTextSize(1);
  display.print(F("INDUKTOR"));
  display.setTextColor(SH110X_WHITE); // Kembalikan warna default
  
  // Diagram Grafis Koneksi Fisik: [TP1] --@@-- [TP2]
  // TP1 Box (simple rect saves memory)
  display.drawRect(12, 14, 28, 12, SH110X_WHITE);
  display.setCursor(18, 16); display.print(F("TP")); display.print(p1);
  
  // Connection line left
  display.drawLine(40, 20, 43, 20, SH110X_WHITE);
  // Inductor symbol
  drawInductorSymbol(43, 20);
  // Connection line right
  display.drawLine(85, 20, 88, 20, SH110X_WHITE);
  
  // TP2 Box (simple rect saves memory)
  display.drawRect(88, 14, 28, 12, SH110X_WHITE);
  display.setCursor(94, 16); display.print(F("TP")); display.print(p2);
  
  // Divider line
  display.drawLine(8, 29, 120, 29, SH110X_WHITE);
  
  // Hitung Nilai Induktansi Centered tanpa String object
  char valBuf[16];
  const char* unitStr = "";
  if (l_val_uH >= 1000.0) {
    formatFloat(l_val_uH / 1000.0, 2, valBuf);
    unitStr = " mH";
  } else {
    formatFloat(l_val_uH, 1, valBuf);
    unitStr = " uH";
  }
  
  int textLen = strlen(valBuf) + strlen(unitStr);
  int startX = (128 - (textLen * 12)) / 2;
  display.setTextSize(2);
  display.setCursor(startX, 33);
  display.print(valBuf);
  display.print(unitStr);
  
  // Label info di dasar layar
  display.setTextSize(1);
  display.setCursor(7, 53);
  display.print(F("Komponen Terdeteksi"));
  
  display.display();
  ledNormal();
}

// Helper: LED indikator status
void ledNormal() {
  stopLEDs();
  analogWrite(greenPin, 25); // Hijau lembut
  delay(3500); // Tahan selama 3.5 detik agar jelas terbaca
  stopLEDs();
  delay(1500); // Jeda mati 1.5 detik sebelum standby biru menyala kembali
}

void ledRusak() {
  stopLEDs();
  // Merah berkedip 5x (Total durasi ~3.2 detik)
  for (int i = 0; i < 5; i++) {
    analogWrite(redPin, 30); // Merah lembut
    delay(400); 
    analogWrite(redPin, 0); 
    delay(250);
  }
  delay(1500); // Jeda mati 1.5 detik sebelum standby biru menyala kembali
}

void ledGagal() {
  stopLEDs();
  // Jingga/Orange berkedip 4x (Total durasi ~3.2 detik)
  for (int i = 0; i < 4; i++) {
    analogWrite(redPin, 30); // Jingga lembut
    analogWrite(greenPin, 5);
    delay(500);
    stopLEDs();
    delay(300);
  }
  delay(1500); // Jeda mati 1.5 detik sebelum standby biru menyala kembali
}

// ============================================================
// TAMPILAN BJT
// ============================================================
void tampilkanHasilTransistor(bool isNPN, int b, int c, int e,
                               float vbe, float hfe, uint8_t kodeRusak) {
  stopLEDs();
  delay(400); // Jeda mati sebelum hasil muncul
  display.clearDisplay();
  
  // Frame luar premium
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  
  // Header bar terbalik
  display.fillRect(1, 1, 126, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(6, 3); display.setTextSize(1);
  display.print(F("BJT "));
  display.print(isNPN ? F("NPN") : F("PNP"));
  
  // Badge RUSAK jika rusak
  if (kodeRusak != RUSAK_OK) {
    display.fillRect(80, 2, 44, 9, SH110X_BLACK);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(85, 3);
    display.print(F("RUSAK"));
  }
  display.setTextColor(SH110X_WHITE); // Kembalikan default
  
  // Pembatas panel kiri (vertikal)
  display.drawLine(58, 12, 58, 62, SH110X_WHITE);
  
  // Pemetaan kaki bergaya soket fisik di kiri atas
  char pin1 = '?', pin2 = '?', pin3 = '?';
  if      (b == 1) pin1 = 'B'; else if (c == 1) pin1 = 'C'; else pin1 = 'E';
  if      (b == 2) pin2 = 'B'; else if (c == 2) pin2 = 'C'; else pin2 = 'E';
  if      (b == 3) pin3 = 'B'; else if (c == 3) pin3 = 'C'; else pin3 = 'E';

  // Soket Fisik (simple rect saves memory)
  display.drawRect(4, 14, 15, 11, SH110X_WHITE);
  display.setCursor(9, 16); display.print(F("1"));
  display.drawRect(21, 14, 15, 11, SH110X_WHITE);
  display.setCursor(26, 16); display.print(F("2"));
  display.drawRect(38, 14, 15, 11, SH110X_WHITE);
  display.setCursor(43, 16); display.print(F("3"));

  // Label di bawah soket
  display.setCursor(9, 27); display.print(pin1);
  display.setCursor(26, 27); display.print(pin2);
  display.setCursor(43, 27); display.print(pin3);

  // Pembatas horizontal kiri
  display.drawLine(4, 39, 54, 39, SH110X_WHITE);

  // Parameter (panel kiri bawah)
  display.setCursor(4, 42);
  display.print(F("hFE:")); display.print((int)hfe);
  display.setCursor(4, 53);
  if (kodeRusak == RUSAK_OK) {
    char vbeBuf[10];
    formatFloat(vbe, 2, vbeBuf);
    display.print(F("Vbe:")); display.print(vbeBuf); display.print(F("V"));
  } else {
    display.print(F("Sts:"));
    switch (kodeRusak) {
      case RUSAK_HFE_RENDAH:   display.print(F("hFE<5")); break;
      case RUSAK_HFE_ABNORMAL: display.print(F("hFEab")); break;
      case RUSAK_VBE_SHORT:    display.print(F("Short")); break;
      case RUSAK_VBE_OPEN:     display.print(F("Open"));  break;
      default:                 display.print(F("Err"));   break;
    }
  }
  
  // === Simbol BJT (panel kanan) ===
  const int cx = 92, cy = 37, r = 14;
  
  display.drawLine(cx-22, cy, cx-r, cy, SH110X_WHITE); // Base line
  display.drawLine(cx-r, cy-10, cx-r, cy+10, SH110X_WHITE); // Base bar
  display.setCursor(60, 34); display.print(F("B")); // Label B
  
  display.drawLine(cx-r, cy-5,  cx+2, cy-14, SH110X_WHITE); // Collector diag
  display.drawLine(cx+2, cy-14, cx+2, cy-25, SH110X_WHITE); // Collector vert
  display.setCursor(98, 14); display.print(F("C"));
  
  display.drawLine(cx-r, cy+5,  cx+2, cy+14, SH110X_WHITE); // Emitter diag
  display.drawLine(cx+2, cy+14, cx+2, cy+25, SH110X_WHITE); // Emitter vert
  display.setCursor(82, 49); display.print(F("E"));
  
  if (isNPN) {
    display.drawLine(cx+2, cy+21, cx-2, cy+16, SH110X_WHITE);
    display.drawLine(cx+2, cy+21, cx+6, cy+16, SH110X_WHITE);
  } else {
    display.drawLine(cx+2, cy+15, cx-2, cy+20, SH110X_WHITE);
    display.drawLine(cx+2, cy+15, cx+6, cy+20, SH110X_WHITE);
  }

  display.display();
  if (kodeRusak != RUSAK_OK) ledRusak(); else ledNormal();
}

// ============================================================
// TAMPILAN MOSFET
// ============================================================
void tampilkanHasilMOSFET(bool isNMOS, int g, int d, int s, uint8_t kodeRusak) {
  stopLEDs();
  delay(400); // Jeda mati sebelum hasil muncul
  display.clearDisplay();
  
  // Frame luar premium
  display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  
  // Header bar terbalik
  display.fillRect(1, 1, 126, 11, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(6, 3); display.setTextSize(1);
  display.print(isNMOS ? F("N-MOSFET") : F("P-MOSFET"));
  
  // Badge RUSAK jika rusak
  if (kodeRusak != RUSAK_OK) {
    display.fillRect(80, 2, 44, 9, SH110X_BLACK);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(85, 3);
    display.print(F("RUSAK"));
  }
  display.setTextColor(SH110X_WHITE); // Kembalikan default
  
  // Pembatas panel kiri (vertikal)
  display.drawLine(58, 12, 58, 62, SH110X_WHITE);
  
  // Pemetaan kaki bergaya soket fisik di kiri atas
  char pin1 = '?', pin2 = '?', pin3 = '?';
  if      (g == 1) pin1 = 'G'; else if (d == 1) pin1 = 'D'; else pin1 = 'S';
  if      (g == 2) pin2 = 'G'; else if (d == 2) pin2 = 'D'; else pin2 = 'S';
  if      (g == 3) pin3 = 'G'; else if (d == 3) pin3 = 'D'; else pin3 = 'S';

  // Soket Fisik (simple rect saves memory)
  display.drawRect(4, 14, 15, 11, SH110X_WHITE);
  display.setCursor(9, 16); display.print(F("1"));
  display.drawRect(21, 14, 15, 11, SH110X_WHITE);
  display.setCursor(26, 16); display.print(F("2"));
  display.drawRect(38, 14, 15, 11, SH110X_WHITE);
  display.setCursor(43, 16); display.print(F("3"));

  // Label di bawah soket
  display.setCursor(9, 27); display.print(pin1);
  display.setCursor(26, 27); display.print(pin2);
  display.setCursor(43, 27); display.print(pin3);

  // Pembatas horizontal kiri
  display.drawLine(4, 39, 54, 39, SH110X_WHITE);

  // Parameter (panel kiri bawah)
  display.setCursor(4, 42);
  display.print(F("Gate:"));
  display.setCursor(4, 53);
  if (kodeRusak == RUSAK_OK) {
    display.print(F("OK"));
  } else {
    display.print(F("Leak"));
  }
  
  // === Simbol MOSFET (panel kanan) ===
  const int cx = 92, cy = 37, r = 14;
  
  display.drawLine(cx-22, cy, cx-10, cy, SH110X_WHITE); // Gate line
  display.setCursor(60, 34); display.print(F("G")); // Label G
  display.drawLine(cx-10, cy-12, cx-10, cy+12, SH110X_WHITE); // Gate bar
  
  display.drawLine(cx-6, cy-11, cx-6, cy-5,  SH110X_WHITE);
  display.drawLine(cx-6, cy-2,  cx-6, cy+2,  SH110X_WHITE);
  display.drawLine(cx-6, cy+5,  cx-6, cy+11, SH110X_WHITE);
  
  display.drawLine(cx-6, cy-8, cx+4, cy-8, SH110X_WHITE);  // Drain horiz
  display.drawLine(cx+4, cy-8, cx+4, cy-25, SH110X_WHITE); // Drain vert
  display.setCursor(100, 14); display.print(F("D"));
  
  display.drawLine(cx-6, cy+8, cx+4, cy+8, SH110X_WHITE);  // Source horiz
  display.drawLine(cx+4, cy+8, cx+4, cy+25, SH110X_WHITE); // Source vert
  display.setCursor(100, 50); display.print(F("S"));
  
  display.drawLine(cx-6, cy, cx+4, cy, SH110X_WHITE); // Bulk
  display.drawLine(cx+4, cy, cx+4, cy+8, SH110X_WHITE);
  
  if (isNMOS) {
    display.drawLine(cx-6, cy, cx-2, cy-3, SH110X_WHITE);
    display.drawLine(cx-6, cy, cx-2, cy+3, SH110X_WHITE);
  } else {
    display.drawLine(cx-2, cy, cx-6, cy-3, SH110X_WHITE);
    display.drawLine(cx-2, cy, cx-6, cy+3, SH110X_WHITE);
  }

  display.display();
  if (kodeRusak != RUSAK_OK) ledRusak(); else ledNormal();
}
