#include "sensors.h"
#include "config.h"
#include "display.h"
#include "motors.h"
#include <Preferences.h>

static Preferences sensorPrefs;

static uint16_t sensorMin[8] = {300, 300, 300, 300, 300, 300, 300, 300};
static uint16_t sensorMax[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
static uint32_t sensorAvg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static bool lineIsDark[8]    = {true, true, true, true, true, true, true, true};
static bool sensorEnabled[8] = {true, true, true, true, true, true, true, true}; // All enabled by default
static bool calibrated       = false; // Chỉ bật lên true sau khi nạp NVS thành công hoặc Calib xong
static int lastNormValues[8] = {0};
static int8_t lastGoodLineDir = 1;  // +1 = Line on Right, -1 = Line on Left

int8_t sensorsGetLastGoodDir() {
    return lastGoodLineDir;
}

void sensorsGetNormValues(int outValues[8]) {
    for (int i = 0; i < 8; i++) {
        outValues[i] = lastNormValues[i];
    }
}

// ----------------------------------------------------
// MUX Channel Switching & ADC Sampling (Direct GPIO Register Write ~15ns)
// ----------------------------------------------------
static inline void setMux(uint8_t ch) {
    uint32_t setMask = 0;
    uint32_t clrMask = 0;

    if (ch & 0x01) setMask |= (1UL << PIN_MUX_S0); else clrMask |= (1UL << PIN_MUX_S0);
    if (ch & 0x02) setMask |= (1UL << PIN_MUX_S1); else clrMask |= (1UL << PIN_MUX_S1);
    if (ch & 0x04) setMask |= (1UL << PIN_MUX_S2); else clrMask |= (1UL << PIN_MUX_S2);

    if (setMask) GPIO.out_w1ts = setMask;
    if (clrMask) GPIO.out_w1tc = clrMask;
}

uint16_t sensorsReadChannel(uint8_t ch) {
    setMux(ch);
    delayMicroseconds(MUX_SWITCH_DELAY_US);
    return (uint16_t)analogRead(PIN_MUX_SIG); // Đọc trực tiếp cảm biến không qua lọc trễ
}

// ----------------------------------------------------
// NVS Flash Persistence (Preferences.h)
// ----------------------------------------------------
bool sensorsLoadCalibration() {
    if (!sensorPrefs.begin("sensors", true)) { // Read-only
        return false;
    }
    if (!sensorPrefs.isKey("valid") || !sensorPrefs.getBool("valid", false)) {
        sensorPrefs.end();
        return false;
    }

    uint16_t loadedMin[8] = {0};
    uint16_t loadedMax[8] = {0};
    uint8_t darkBytes[8] = {0};
    uint8_t enBytes[8] = {0};

    size_t readMin  = sensorPrefs.getBytes("min", loadedMin, sizeof(loadedMin));
    size_t readMax  = sensorPrefs.getBytes("max", loadedMax, sizeof(loadedMax));
    size_t readDark = sensorPrefs.getBytes("dark", darkBytes, sizeof(darkBytes));
    size_t readEn   = sensorPrefs.getBytes("en", enBytes, sizeof(enBytes));
    sensorPrefs.end();

    if (readMin != sizeof(sensorMin) || readMax != sizeof(sensorMax) ||
        readDark != sizeof(darkBytes) || readEn != sizeof(enBytes)) {
        return false;
    }

    for (int i = 0; i < 8; i++) {
        sensorMin[i]     = loadedMin[i];
        sensorMax[i]     = loadedMax[i];
        lineIsDark[i]    = (darkBytes[i] != 0);
        int span = (int)loadedMax[i] - (int)loadedMin[i];
        sensorEnabled[i] = (enBytes[i] != 0) || (span >= 150);
    }
    calibrated = true;

    Serial.println("\n[NVS FLASH] Đã nạp thành công thông số cảm biến từ Flash:");
    for (int i = 0; i < 8; i++) {
        int span = sensorMax[i] - sensorMin[i];
        Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u | Line=%s [%s]\n",
            i, sensorMin[i], sensorMax[i], span,
            lineIsDark[i] ? "DARK" : "LIGHT",
            sensorEnabled[i] ? "OK" : "TẮT");
    }
    Serial.println();
    return true;
}

void sensorsSaveCalibration() {
    if (!sensorPrefs.begin("sensors", false)) { // Read-write
        Serial.println("[NVS FLASH] LỖI: Không thể mở Flash để lưu!");
        return;
    }
    uint8_t darkBytes[8];
    uint8_t enBytes[8];
    for (int i = 0; i < 8; i++) {
        darkBytes[i] = lineIsDark[i] ? 1 : 0;
        enBytes[i]   = sensorEnabled[i] ? 1 : 0;
    }
    sensorPrefs.putBytes("min", sensorMin, sizeof(sensorMin));
    sensorPrefs.putBytes("max", sensorMax, sizeof(sensorMax));
    sensorPrefs.putBytes("dark", darkBytes, sizeof(darkBytes));
    sensorPrefs.putBytes("en", enBytes, sizeof(enBytes));
    sensorPrefs.putBool("valid", true);
    sensorPrefs.end();

    Serial.println("[NVS FLASH] Đã lưu thông số cảm biến vào Flash thành công!");
}

void sensorsInit() {
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_SIG, INPUT);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MUX_SIG, ADC_11db); // Full 0V - ~3.1V dynamic range

    // Try loading saved calibration from ESP32 Flash
    if (!sensorsLoadCalibration()) {
        Serial.println("[NVS FLASH] Chưa có dữ liệu calib trong Flash. Dùng thông số mặc định.");
        Serial.println("            (Bấm SW1 để quét 10s và lưu vĩnh viễn vào Flash!)");
    }
}

void sensorsCalibrate(uint32_t durationMs) {
    motorsStop(); // Ensure motors are completely off during calibration
    Serial.println("\n==========================================");
    Serial.printf("[CALIB] Bắt đầu Calibrate trong %u giây...\n", durationMs / 1000);
    Serial.println("  Trượt đều toàn bộ xe qua lại vạch đen để cả 8 mắt đều quét qua!");
    Serial.println("==========================================");
    delay(800);

    for (int i = 0; i < 8; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
        sensorAvg[i] = 0;
    }

    uint32_t sampleCount = 0;
    uint32_t startMs = millis();
    uint32_t lastDotMs = 0;
    while (millis() - startMs < durationMs) {
        uint32_t elapsed = millis() - startMs;
        displaySetCalibProgress(elapsed, durationMs);
        for (uint8_t i = 0; i < 8; i++) {
            uint16_t val = sensorsReadChannel(i);
            if (val < sensorMin[i]) sensorMin[i] = val;
            if (val > sensorMax[i]) sensorMax[i] = val;
            sensorAvg[i] += val;
        }
        sampleCount++;
        if (millis() - lastDotMs > 500) {
            lastDotMs = millis();
            Serial.print(".");
        }
        delay(2);
    }
    Serial.println(" XONG!\n");

    if (sampleCount == 0) sampleCount = 1;

    // Xác định phân cực màu vạch dựa trên 2 mắt ngoài cùng (Ch0 và Ch7 ít nằm trên vạch nhất)
    int darkVotes = 0;
    int lightVotes = 0;
    const uint8_t outerChannels[4] = {0, 1, 6, 7};
    for (int k = 0; k < 4; k++) {
        uint8_t i = outerChannels[k];
        int span = sensorMax[i] - sensorMin[i];
        if (span >= 180) {
            uint32_t backgroundMean = sensorAvg[i] / sampleCount;
            uint16_t midpoint = (sensorMin[i] + sensorMax[i]) / 2;
            // Nếu nền sân có giá trị ADC > midpoint -> Vạch có giá trị ADC thấp (lineIsDark = true theo công thức (Max - raw)/span)
            if (backgroundMean > midpoint) darkVotes++;
            else lightVotes++;
        }
    }
    bool uniformLineIsDark = (darkVotes >= lightVotes);

    Serial.println("[CALIB] Kết quả Calibrate:");
    for (int i = 0; i < 8; i++) {
        int span = sensorMax[i] - sensorMin[i];
        lineIsDark[i] = uniformLineIsDark;

        if (span < 180) {
            sensorEnabled[i] = false;
            Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u [TẮT - Không đủ tương phản (Cần >= 180)!]\n",
                i, sensorMin[i], sensorMax[i], span);
        } else {
            sensorEnabled[i] = true;
            Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u | Line=%s [OK]\n",
                i, sensorMin[i], sensorMax[i], span, uniformLineIsDark ? "DARK" : "LIGHT");
        }
    }
    calibrated = true;
    sensorsSaveCalibration();
    displayShowMessage("CALIBRATION", "SAVED TO FLASH!", 1800);
    Serial.println("==========================================\n");
}

bool sensorsIsCalibrated() {
    return calibrated;
}

LineDetection sensorsProcess() {
    LineDetection det;
    det.positionMm     = 0.0f;
    det.positionNorm   = 0.0f;
    det.confidence     = 0.0f;
    det.lineDetected   = false;
    det.isExtrapolated = false;
    det.turn90         = 0;

    uint32_t startUs = micros();

    // 1. Đọc 8 kênh MUX ADC
    uint16_t raw[8];
    for (uint8_t i = 0; i < 8; i++) {
        raw[i] = sensorsReadChannel(i);
    }
    det.adcDurationUs = (uint32_t)(micros() - startUs);

    // 2. Chuẩn hóa analog S_i [0.0, 1.0]
    float S[8];
    float sMax = 0.0f;
    float sSum = 0.0f;

    for (uint8_t i = 0; i < 8; i++) {
        if (!sensorEnabled[i]) {
            S[i] = 0.0f;
            lastNormValues[i] = 0;
            continue;
        }
        int span = (int)sensorMax[i] - (int)sensorMin[i];
        if (span < 150) {
            S[i] = 0.0f;
            lastNormValues[i] = 0;
            continue;
        }

        float val = 0.0f;
        if (lineIsDark[i]) {
            val = ((float)sensorMax[i] - (float)raw[i]) / (float)span;
        } else {
            val = ((float)raw[i] - (float)sensorMin[i]) / (float)span;
        }
        val = constrain(val, 0.0f, 1.0f);

        // Cắt sàn nhiễu 0.08 (giữ trọn độ nhạy phản xạ nhanh khi chạy tốc độ cao)
        if (val < 0.08f) val = 0.0f;
        else val = (val - 0.08f) / 0.92f;

        S[i] = val;
        lastNormValues[i] = (int)roundf(val * 1000.0f);

        if (val > sMax) sMax = val;
        sSum += val;
    }

    // ----------------------------------------------------
    // ----------------------------------------------------
    // 3. NHẬN DIỆN GÓC CUA VUÔNG 90 ĐỘ (90-DEGREE TURN DETECTION)
    // Thực hiện TRƯỚC TIÊN trên tín hiệu gốc (chưa qua triệt tiêu) để không bị bỏ sót:
    // Khi cụm mắt ngoài cùng ở một bên bắt vạch đậm (S[5..7] hoặc S[0..2] > 0.80),
    // trong khi mắt ngoài cùng bên đối diện là nền trắng (< 0.25) -> Chắc chắn là góc vuông 90°!
    // ----------------------------------------------------
    float leftSideSum  = S[0] + S[1] + S[2] + S[3];
    float rightSideSum = S[4] + S[5] + S[6] + S[7];
    float rightOuter   = S[5] + S[6] + S[7];
    float leftOuter    = S[0] + S[1] + S[2];

    if ((rightOuter > 0.80f || (rightSideSum > 1.20f && (S[6] > 0.25f || S[7] > 0.25f))) &&
        (S[0] < 0.25f && S[1] < 0.35f)) {
        det.turn90      = 1;  // Góc vuông 90° bên PHẢI
        lastGoodLineDir = 1;
        // Triệt tiêu ngay toàn bộ cánh đối diện để không làm lệch trọng tâm
        S[0] = 0.0f; S[1] = 0.0f; S[2] = 0.0f; S[3] = 0.0f;
    } else if ((leftOuter > 0.80f || (leftSideSum > 1.20f && (S[0] > 0.25f || S[1] > 0.25f))) &&
               (S[7] < 0.25f && S[6] < 0.35f)) {
        det.turn90      = -1; // Góc vuông 90° bên TRÁI
        lastGoodLineDir = -1;
        // Triệt tiêu ngay toàn bộ cánh đối diện để không làm lệch trọng tâm
        S[4] = 0.0f; S[5] = 0.0f; S[6] = 0.0f; S[7] = 0.0f;
    } else {
        // ----------------------------------------------------
        // 3.5. KHẮC PHỤC TRIỆT TIÊU LỰC CUA CHO VẠCH CONG THƯỜNG (ANTI-CANCELLATION)
        // Chỉ áp dụng khi KHÔNG PHẢI góc vuông 90 độ:
        // ----------------------------------------------------
        float leftWing  = S[0] + S[1];
        float rightWing = S[6] + S[7];

        if ((leftWing > 0.30f && rightWing > 0.30f) || 
            (leftSideSum > 0.60f && rightSideSum > 0.60f && (S[0] > 0.20f || S[7] > 0.20f))) {
            // Ưu tiên bên có tổng năng lượng tín hiệu vượt trội (> 1.5 lần):
            if (rightSideSum > leftSideSum * 1.5f) {
                S[0] = 0.0f; S[1] = 0.0f; S[2] = 0.0f;
                lastGoodLineDir = 1;
            } else if (leftSideSum > rightSideSum * 1.5f) {
                S[5] = 0.0f; S[6] = 0.0f; S[7] = 0.0f;
                lastGoodLineDir = -1;
            } else {
                // Tín hiệu 2 bên tương đương: Dùng hướng cua gần nhất làm mốc phân giải
                if (lastGoodLineDir > 0) {
                    S[0] = 0.0f; S[1] = 0.0f; S[2] = 0.0f;
                } else {
                    S[5] = 0.0f; S[6] = 0.0f; S[7] = 0.0f;
                }
            }
        }
    }

    // Cập nhật lại sSum, sMax sau khi phân giải triệt tiêu vạch:
    sSum = 0.0f;
    sMax = 0.0f;
    for (uint8_t i = 0; i < 8; i++) {
        if (S[i] > sMax) sMax = S[i];
        sSum += S[i];
    }

    // 4. Tính trọng tâm vạch (Centroid)
    // Ngưỡng phát hiện vạch sMax >= 0.18 và sSum >= 0.18:
    // Đảm bảo bắt vạch tức thời ngay cả khi xe đang xoay tốc độ cao để tìm vạch
    if (sMax >= 0.18f && sSum >= 0.18f) {
        float num = 0.0f;
        float den = 0.0f;
        for (uint8_t i = 0; i < 8; i++) {
            if (S[i] > 0.0f) {
                num += S[i] * SENSOR_WEIGHTS[i];
                den += S[i];
            }
        }

        if (den > 0.01f) {
            float rawPos = num / den;
            det.lineDetected   = true;
            det.confidence     = constrain(sMax, 0.0f, 1.0f);
            det.isExtrapolated = false;

            // Nếu phát hiện góc vuông 90 độ, ép vị trí ra sát biên để PD bẻ lái tối đa ngay lập tức!
            if (det.turn90 == 1) {
                det.positionNorm   = SENSOR_EXTRAPOLATE_POS; // +4.25 (+50.4 mm)
                det.isExtrapolated = true;
                lastGoodLineDir    = 1;
            } else if (det.turn90 == -1) {
                det.positionNorm   = -SENSOR_EXTRAPOLATE_POS; // -4.25 (-50.4 mm)
                det.isExtrapolated = true;
                lastGoodLineDir    = -1;
            } else if (S[7] > 0.15f && (S[7] >= S[6] || S[5] < 0.20f)) {
                // Vạch trượt hẳn về phía mắt mép Ch7
                float edgeSlide = constrain((S[7] - S[6] + 0.2f) / 0.8f, 0.0f, 1.0f);
                det.positionNorm   = 3.5f + edgeSlide * (SENSOR_EXTRAPOLATE_POS - 3.5f);
                det.isExtrapolated = true;
                lastGoodLineDir    = 1;
            } else if (S[0] > 0.15f && (S[0] >= S[1] || S[2] < 0.20f)) {
                // Vạch trượt hẳn về phía mắt mép Ch0
                float edgeSlide = constrain((S[0] - S[1] + 0.2f) / 0.8f, 0.0f, 1.0f);
                det.positionNorm   = -3.5f - edgeSlide * (SENSOR_EXTRAPOLATE_POS - 3.5f);
                det.isExtrapolated = true;
                lastGoodLineDir    = -1;
            } else {
                det.positionNorm   = rawPos + (SENSOR_CENTER_TRIM / SENSOR_PITCH_MM);
                if (det.positionNorm > 0.35f) lastGoodLineDir = 1;
                else if (det.positionNorm < -0.35f) lastGoodLineDir = -1;
            }

            det.positionMm     = det.positionNorm * SENSOR_PITCH_MM;
            return det;
        }
    }

    // 5. Ngoại suy khi vạch trượt hẳn ra mép ngoài (Mắt ngoài cùng bắt được tín hiệu >= 0.14)
    if (S[7] >= 0.14f && S[7] >= S[0]) {
        det.lineDetected   = true;
        det.isExtrapolated = true;
        det.positionNorm   = SENSOR_EXTRAPOLATE_POS;
        det.positionMm     = SENSOR_EXTRAPOLATE_MM;
        det.confidence     = S[7];
        lastGoodLineDir    = 1;
        return det;
    } else if (S[0] >= 0.14f && S[0] >= S[7]) {
        det.lineDetected   = true;
        det.isExtrapolated = true;
        det.positionNorm   = -SENSOR_EXTRAPOLATE_POS;
        det.positionMm     = -SENSOR_EXTRAPOLATE_MM;
        det.confidence     = S[0];
        lastGoodLineDir    = -1;
        return det;
    }

    // Mất line hoàn toàn (tất cả các mắt đều là nền trắng)
    det.lineDetected   = false;
    det.isExtrapolated = false;
    det.confidence     = 0.0f;
    det.positionNorm   = 0.0f;
    det.positionMm     = 0.0f;
    return det;
}

float sensorsGetError(float lastKnownError, bool &lineDetected, int &turn90) {
    LineDetection det = sensorsProcess();
    lineDetected = det.lineDetected;
    turn90 = det.turn90;
    if (det.lineDetected) {
        return det.positionMm;
    }
    return lastKnownError;
}



void sensorsPrintDebug(float currentError) {
    Serial.print("Sensors: [");
    for (uint8_t i = 0; i < 8; i++) {
        int norm = lastNormValues[i];
        if (!sensorEnabled[i]) {
            Serial.print("x"); // Disabled/dead sensor marker
        } else if (norm > 600) {
            Serial.print("#");
        } else if (norm > 180) {
            Serial.print("+");
        } else {
            Serial.print(".");
        }
        if (i < 7) Serial.print(" ");
    }
    Serial.printf("] Err:%+5.2f (Ch3:%3d Ch4:%3d)\n", currentError, lastNormValues[3], lastNormValues[4]);
}

void sensorsPrintRaw() {
    Serial.print("  Raw ADC: [");
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t r = sensorsReadChannel(i);
        Serial.printf(" %u:%4u", i, r);
        if (i < 7) Serial.print(" |");
    }
    Serial.println(" ]");
}
