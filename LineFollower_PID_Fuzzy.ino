#include <EEPROM.h>

// --- EEPROM Address ---
const int EEPROM_SIGNATURE = 0;  // 1 byte signature to verify data validity
const int EEPROM_THRESHOLD = 1;  // 8 bytes for threshold values (1-8)
const int EEPROM_KP = 9;         // 4 bytes for Kp (float)
const int EEPROM_KD = 13;        // 4 bytes for Kd (float)
const int EEPROM_KI = 17;  // 4 bytes untuk Ki (setelah Kd)

// --- Tombol ---
const int btnRightUp = 11;
const int btnRightDown = 12;
const int btnLeftUp = 3;
const int btnLeftDown = 2;

// --- LED Arah ---
const int ledLeft = 7;
const int ledRight = 8;

// --- Motor ---
const int motorKiri = 9;
const int motorKanan = 6;

// --- Sensor IR ---
const int sensorPin[] = { A4, A5, A6, A7, A0, A1, A2, A3 };    // 8 sensor
int threshold[] = { 438, 442, 432, 418, 421, 399, 427, 427 };  // Default

// --- PID & Speed ---
float Kp = 12.0;
float Ki = 5.0;
float Kd = 2.0;
int baseSpeed = 200;
int LastError = 0;
float errorSum = 0;                 // Untuk perhitungan integral error
const float MAX_ERROR_SUM = 100.0;  // Batasan maksimum integral error (anti-windup)

// --- Variabel Waktu ---
unsigned long now = 0;
unsigned long lastTime = 0;
float timeChange = 0;

// --- State Control ---
int systemState = 0;  // Kontrol state program

// --- Blinking LED ---
unsigned long lastBlinkTime = 0;
bool ledState = false;
const int blinkInterval = 100;

// --- Timer Monitoring ---
unsigned long lastMonitorTime = 0;
const int monitorInterval = 200; // interval tampil data (ms)

int kecepatanNow = 150;
unsigned long lastFuzzyTime = 0;
const int intervalFuzzy = 2000;


int fuzzySpeed(int error) {
  int absError = abs(error);

  if (absError == 0) {
    return 200;  // lurus → ngebut
  }
  else if (absError <= 2 || absError <= -2) {
    return 200;  // belok dikit
  }
  else if (absError <= 3 || absError <= -3) {
    return 180;  // belok sedang
  }
  else if (absError <= 4 || absError <= -4) {
    return 160;  // belok sedang
  }
  else if (absError <= 5 || absError <= -5) {
    return 140;  // belok sedang
  }
  else {
    return 100;  // belok tajam → pelan
  }
}

void nyalakanLED500ms() {
  digitalWrite(ledLeft, HIGH);
  digitalWrite(ledRight, HIGH);
  delay(500);
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
}

void setup() {
  Serial.begin(9600);

  pinMode(btnRightUp, INPUT_PULLUP);
  pinMode(btnRightDown, INPUT_PULLUP);
  pinMode(btnLeftUp, INPUT_PULLUP);
  pinMode(btnLeftDown, INPUT_PULLUP);

  pinMode(ledLeft, OUTPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(motorKiri, OUTPUT);
  pinMode(motorKanan, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPin[i], INPUT);
  }

  // Baca data kalibrasi dari EEPROM jika ada
  bacaDataEEPROM();

  // Tampilkan data kalibrasi ke Serial Monitor
  tampilkanDataKalibrasi();

  Serial.println("Sistem Siap. Kontrol dengan tombol:");
  Serial.println("- Left Down: Kalibrasi & simpan ke EEPROM (State 1)");
  Serial.println("- Left Up: Jalankan dengan konfigurasi EEPROM (State 2)");
  Serial.println("- Saat robot berjalan:");
  Serial.println("  - Left Down: Kurangi Kp (State 3)");
  Serial.println("  - Left Up: Tambah Kp (State 4)");
  Serial.println("  - Right Down: Kurangi Kd (State 5)");
  Serial.println("  - Right Up: Tambah Kd (State 6)");
  Serial.println("  - Left Down + Left Up: Keluar ke menu utama");
}

void loop() {
  // Baca status tombol
  boolean leftDownPressed = (digitalRead(btnLeftDown) == LOW);
  boolean leftUpPressed = (digitalRead(btnLeftUp) == LOW);
  boolean rightDownPressed = (digitalRead(btnRightDown) == LOW);
  boolean rightUpPressed = (digitalRead(btnRightUp) == LOW);

  // Menjalankan program berdasarkan state
  switch (systemState) {
    case 0:  // Menu Utama
      // State 1: Kalibrasi dan simpan ke EEPROM (Left Down)
      if (leftDownPressed) {
        systemState = 1;
        Serial.println("\n>> State 1: Kalibrasi Sensor & Simpan EEPROM <<");
        delay(300);
      }

      // State 2: Jalankan robot dengan konfigurasi EEPROM (Left Up)
      if (leftUpPressed) {
        systemState = 2;
        errorSum = 0;
        Serial.println("\n>> State 2: Mode Line Follower dengan Konfigurasi EEPROM <<");
        delay(300);
      }
      break;

    case 1:  // Kalibrasi dan simpan ke EEPROM
      kalibrasiSensor();
      simpanDataEEPROM();
      tampilkanDataKalibrasi();
      systemState = 0;  // Kembali ke menu utama
      Serial.println(">> Kembali ke Menu Utama <<");
      delay(500);
      break;

    case 2:  // Mode Line Follower
      // Mulai menggerakkan robot
      jalanRobot();

      // State 3: Kurangi Kp (Left Down)
      if (leftDownPressed) {
        systemState = 3;
        Kp -= 1.0;
        if (Kp < 0) Kp = 0;
        Serial.print("\n>> State 3: Kp Dikurangi: ");
        Serial.println(Kp);
        simpanDataEEPROM();
        delay(300);
        systemState = 2;  // Kembali ke mode line follower
      }

      // State 4: Tambah Kp (Left Up)
      if (leftUpPressed) {
        systemState = 4;
        Kp += 1.0;
        Serial.print("\n>> State 4: Kp Ditambah: ");
        Serial.println(Kp);
        simpanDataEEPROM();
        delay(300);
        systemState = 2;  // Kembali ke mode line follower
      }

      // State 5: Kurangi Kd (Right Down)
      if (rightDownPressed) {
        systemState = 5;
        Kd -= 1.0;
        if (Kd < 0) Kd = 0;
        Serial.print("\n>> State 5: Kd Dikurangi: ");
        Serial.println(Kd);
        simpanDataEEPROM();
        delay(300);
        systemState = 2;  // Kembali ke mode line follower
      }

      // State 6: Tambah Kd (Right Up)
      if (rightUpPressed) {
        systemState = 6;
        Kd += 1.0;
        Serial.print("\n>> State 6: Kd Ditambah: ");
        Serial.println(Kd);
        simpanDataEEPROM();
        delay(300);
        systemState = 2;  // Kembali ke mode line follower
      }

      // State 7: Tambah Ki (Left Up + Right Up)
      if (leftUpPressed && rightUpPressed) {
        systemState = 7;

        Serial.println("\n>> State 7: Tambah Ki <<");

        nyalakanLED500ms();   // LED nyala 500 ms

        Ki += 1.0;
        errorSum = 0;

        Serial.print("Ki sekarang: ");
        Serial.println(Ki);

        simpanDataEEPROM();

        delay(300);
        systemState = 2;
      }

      // State 8: Kurangi Ki (Left Down + Right Down)
      if (leftDownPressed && rightDownPressed) {
        systemState = 8;

        Serial.println("\n>> State 8: Kurangi Ki <<");

        nyalakanLED500ms();   // LED nyala 500 ms

        Ki -= 1.0;
        if (Ki < 0) Ki = 0;
        errorSum = 0;

        Serial.print("Ki sekarang: ");
        Serial.println(Ki);

        simpanDataEEPROM();

        delay(300);
        systemState = 2;
      }

      // Keluar dari mode line follower (tekan Left Down + Left Up bersamaan)
      if (leftDownPressed && leftUpPressed) {
        stopMotor();
        Serial.println("\n>> Robot Dihentikan - Kembali ke Menu Utama <<");
        systemState = 0;
        delay(300);
      }

    delay(10);
    break;
  }
}

// Fungsi untuk menyimpan data kalibrasi ke EEPROM
void simpanDataEEPROM() {
  // Simpan nilai threshold
  for (int i = 0; i < 8; i++) {
    EEPROM.put(EEPROM_THRESHOLD + (i * sizeof(int)), threshold[i]);
  }
  // Simpan nilai Kp dan Kd
  EEPROM.put(EEPROM_KP, Kp);
  EEPROM.put(EEPROM_KD, Kd);
  EEPROM.put(EEPROM_KI, Ki);

    // Simpan signature untuk memastikan data valid
  EEPROM.write(EEPROM_SIGNATURE, 0xAA);

  Serial.println(">> Data Kalibrasi Tersimpan di EEPROM <<");
}

// Fungsi untuk membaca data kalibrasi dari EEPROM
void bacaDataEEPROM() {

  byte signature = EEPROM.read(EEPROM_SIGNATURE);

  if (signature == 0xAA) {

    for (int i = 0; i < 8; i++) {
      EEPROM.get(EEPROM_THRESHOLD + (i * sizeof(int)), threshold[i]);
    }

    EEPROM.get(EEPROM_KP, Kp);
    EEPROM.get(EEPROM_KD, Kd);
    EEPROM.get(EEPROM_KI, Ki);

    // VALIDASI
    bool dataValid = true;

    for (int i = 0; i < 8; i++) {
      if (threshold[i] < 100 || threshold[i] > 900) {
        dataValid = false;
      }
    }

    if (!dataValid) {
      Serial.println(">> Data EEPROM Rusak! Gunakan Default <<");

      int defaultThreshold[8] = {438, 442, 432, 418, 421, 399, 427, 427};

      for (int i = 0; i < 8; i++) {
        threshold[i] = defaultThreshold[i];
      }

      Kp = 12;
      Kd = 2;
    }

    Serial.println(">> Data EEPROM Dibaca <<");

  } else {
    Serial.println(">> EEPROM Kosong, Gunakan Default <<");
  }
}
// Tampilkan data kalibrasi ke Serial Monitor
void tampilkanDataKalibrasi() {
  Serial.println("\n=== DATA KALIBRASI SENSOR ===");
  for (int i = 0; i < 8; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" Threshold: ");
    Serial.println(threshold[i]);
  }

  Serial.println("\n=== PARAMETER PID ===");
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.println("============================\n");
}

void kalibrasiSensor() {
  Serial.println(">> Kalibrasi Dimulai <<");
  int minVal[8], maxVal[8];

  // Inisialisasi
  for (int i = 0; i < 8; i++) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  // Loop pembacaan
  for (int t = 0; t < 100; t++) {
    // Kedipkan LED selama kalibrasi
    blinkLEDs();

    for (int i = 0; i < 8; i++) {
      int nilai = analogRead(sensorPin[i]);
      if (nilai < minVal[i]) minVal[i] = nilai;
      if (nilai > maxVal[i]) maxVal[i] = nilai;
    }

    delay(50);
  }

  // Hitung threshold & tampilkan
  for (int i = 0; i < 8; i++) {
    threshold[i] = (minVal[i] + maxVal[i]) / 2;
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" Threshold: ");
    Serial.println(threshold[i]);
    delay(50);
  }

  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
  Serial.println(">> Kalibrasi Selesai <<\n");
}

// Fungsi untuk mengedipkan LED saat kalibrasi
void blinkLEDs() {
  unsigned long currentTime = millis();
  if (currentTime - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = currentTime;
    ledState = !ledState;

    digitalWrite(ledLeft, ledState);
    digitalWrite(ledRight, ledState);
  }
}

String bacaSensor() {
  String kondisi = "";
  for (int i = 0; i < 8; i++) {
    int nilai = analogRead(sensorPin[i]);
    kondisi += (nilai > threshold[i]) ? "1" : "0";
  }
  return kondisi;
}

int hitungError(String kondisi) {
  if (kondisi == "10000000") return 7;
  else if (kondisi == "11000000") return 6;
  else if (kondisi == "01000000") return 5;
  else if (kondisi == "01100000") return 4;
  else if (kondisi == "00100000") return 3;
  else if (kondisi == "00110000") return 2;
  else if (kondisi == "00010000") return 1;
  else if (kondisi == "00011000") return 0;
  else if (kondisi == "00001000") return -1;
  else if (kondisi == "00001100") return -2;
  else if (kondisi == "00000100") return -3;
  else if (kondisi == "00000110") return -4;
  else if (kondisi == "00000010") return -5;
  else if (kondisi == "00000011") return -6;
  else if (kondisi == "00000001") return -7;
  else return LastError;
}

void jalanRobot() {
  String kondisi = bacaSensor();
  int error = hitungError(kondisi);

  hitungWaktu();  // panggil fungsi waktu

  // Integral
  errorSum += error * timeChange;

  if (errorSum > MAX_ERROR_SUM) errorSum = MAX_ERROR_SUM;
  if (errorSum < -MAX_ERROR_SUM) errorSum = -MAX_ERROR_SUM;

  if (error == 0) {
    errorSum *= 0.5;
  }

  int deltaError = error - LastError;
  int output = (Kp * error) + (Ki * errorSum) + (Kd * deltaError);

  // 🔥 FUZZY SPEED CONTROL
  kecepatanNow = fuzzySpeed(error);

  // PID tetap dipakai untuk arah
  int kiri = constrain(kecepatanNow + output, 0, 255);
  int kanan = constrain(kecepatanNow - output, 0, 255);

  analogWrite(motorKiri, kiri);
  analogWrite(motorKanan, kanan);

  // LED indikator
  unsigned long currentTime = millis();
  if (currentTime - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = currentTime;
    ledState = !ledState;

    if (error > 0) {
      digitalWrite(ledLeft, ledState);
      digitalWrite(ledRight, LOW);
    } else if (error < 0) {
      digitalWrite(ledLeft, LOW);
      digitalWrite(ledRight, ledState);
    } else {
      digitalWrite(ledLeft, LOW);
      digitalWrite(ledRight, LOW);
    }
  }

  monitorSerial(error, kiri, kanan, kondisi);

  if (millis() - lastFuzzyTime >= intervalFuzzy) {
  lastFuzzyTime = millis();

  kecepatanNow = fuzzySpeed(error);

  Serial.print("Fuzzy Speed: ");
  Serial.println(kecepatanNow);
}

  LastError = error;
}

void monitorSerial(int error, int kiri, int kanan, String kondisi) {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMonitorTime >= monitorInterval) {
    lastMonitorTime = currentMillis;

    Serial.print("Time(ms): ");
    Serial.print(currentMillis);

    Serial.print(" | Sensor: ");
    Serial.print(kondisi);

    Serial.print(" | Error: ");
    Serial.print(error);

    Serial.print(" | Kp: ");
    Serial.print(Kp);

    Serial.print(" | Ki: ");
    Serial.print(Ki);

    Serial.print(" | Kd: ");
    Serial.print(Kd);

    Serial.print(" | Motor Kiri: ");
    Serial.print(kiri);

    Serial.print(" | Motor Kanan: ");
    Serial.print(kanan);

    Serial.print(" | dt: ");
    Serial.print(timeChange, 4);

    Serial.println();
  }
}

void hitungWaktu() {
  now = millis();
  timeChange = (now - lastTime) / 1000.0;  // konversi ke detik
  lastTime = now;
}

void stopMotor() {
  analogWrite(motorKiri, 0);
  analogWrite(motorKanan, 0);
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
}