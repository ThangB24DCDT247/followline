#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "encoders.h"
#include "display.h"

enum AppMode {
    MODE_IDLE,
    MODE_RUNNING,
    MODE_LINE_HOLD,
    MODE_MENU
};

static AppMode appMode = MODE_IDLE;

// Flash NVS Configuration
static CarConfig activeConfig;
static CarConfig menuConfig;
static int menuSelectedIndex = 0;
static bool menuIsEditing = false;

// Button States
static bool lastSw1 = HIGH;
static bool lastSw2 = HIGH;
static bool lastSw3 = HIGH;

// Timing & State Variables
static uint32_t lastCtrlUs = 0;
static uint32_t lastDebugPrintMs = 0;
static uint32_t brakeHoldUntilMs = 0;
static uint32_t lineLostStartTime = 0;
static float lastRunError = 0.0f;
static float lastHoldError = 0.0f;
static float filteredRunDeltaErr = 0.0f;
static float holdDeltaErr = 0.0f;
static float currentRunRate = 0.0f;
static float currentHoldRate = 0.0f;
static bool runFirstStep = true;
static bool holdFirstStep = true;
static int8_t lastKnownDir = 1;
static int8_t lastMotionState = 0; // -1: Rẽ trái, 0: Đi thẳng, +1: Rẽ phải

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.flush();

    pinMode(PIN_BTN_SW1, INPUT_PULLUP);
    pinMode(PIN_BTN_SW2, INPUT_PULLUP);
    pinMode(PIN_BTN_SW3, INPUT_PULLUP);

    configLoad(activeConfig);
    motorsSetActiveBrake(activeConfig.activeBrakeEn);

    sensorsInit();
    motorsInit();
    encodersInit();
    displayInit();

    Serial.println("\n=======================================================");
    Serial.println("[ESP32 LINE FOLLOWER READY (Pure Direct PD)]");
    Serial.printf("  Config: Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f ABS:%s\n",
        activeConfig.boostSpeed, activeConfig.baseSpeed, activeConfig.cornerSpeed,
        activeConfig.kpGain, activeConfig.kdGain, activeConfig.activeBrakeEn ? "ON" : "OFF");
    Serial.println("=======================================================\n");
}

void loop() {
    uint32_t now = millis();

    // --- 1. Pushbutton Polling ---
    bool sw1 = digitalRead(PIN_BTN_SW1);
    bool sw2 = digitalRead(PIN_BTN_SW2);
    bool sw3 = digitalRead(PIN_BTN_SW3);

    // ========================================================
    // MODE: OLED TUNING MENU
    // ========================================================
    if (appMode == MODE_MENU) {
        motorsStop();

        // SW1: UP in navigation / [+] in edit
        if (sw1 == LOW && lastSw1 == HIGH) {
            if (!menuIsEditing) {
                menuSelectedIndex = (menuSelectedIndex + TOTAL_MENU_ITEMS - 1) % TOTAL_MENU_ITEMS;
            } else {
                switch (menuSelectedIndex) {
                    case MENU_ITEM_BOOST_SPD:    menuConfig.boostSpeed  = constrain(menuConfig.boostSpeed + 5.0f, CFG_BOOST_SPD_MIN, CFG_BOOST_SPD_MAX); break;
                    case MENU_ITEM_BASE_SPD:     menuConfig.baseSpeed   = constrain(menuConfig.baseSpeed + 5.0f, CFG_BASE_SPD_MIN, CFG_BASE_SPD_MAX); break;
                    case MENU_ITEM_CORNER_SPD:   menuConfig.cornerSpeed = constrain(menuConfig.cornerSpeed + 5.0f, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX); break;
                    case MENU_ITEM_KP_GAIN:      menuConfig.kpGain      = constrain(menuConfig.kpGain + 0.5f, CFG_KP_MIN, CFG_KP_MAX); break;
                    case MENU_ITEM_KD_GAIN:      menuConfig.kdGain      = constrain(menuConfig.kdGain + 1.0f, CFG_KD_MIN, CFG_KD_MAX); break;
                    case MENU_ITEM_KC_GAIN:      menuConfig.kCurveGain  = constrain(menuConfig.kCurveGain + 0.5f, CFG_KC_MIN, CFG_KC_MAX); break;
                    case MENU_ITEM_ACTIVE_BRAKE: 
                        menuConfig.activeBrakeEn = !menuConfig.activeBrakeEn;
                        motorsSetActiveBrake(menuConfig.activeBrakeEn);
                        break;
                }
            }
            delay(150);
        }
        lastSw1 = sw1;

        // SW2: DOWN in navigation / [-] in edit
        if (sw2 == LOW && lastSw2 == HIGH) {
            if (!menuIsEditing) {
                menuSelectedIndex = (menuSelectedIndex + 1) % TOTAL_MENU_ITEMS;
            } else {
                switch (menuSelectedIndex) {
                    case MENU_ITEM_BOOST_SPD:    menuConfig.boostSpeed  = constrain(menuConfig.boostSpeed - 5.0f, CFG_BOOST_SPD_MIN, CFG_BOOST_SPD_MAX); break;
                    case MENU_ITEM_BASE_SPD:     menuConfig.baseSpeed   = constrain(menuConfig.baseSpeed - 5.0f, CFG_BASE_SPD_MIN, CFG_BASE_SPD_MAX); break;
                    case MENU_ITEM_CORNER_SPD:   menuConfig.cornerSpeed = constrain(menuConfig.cornerSpeed - 5.0f, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX); break;
                    case MENU_ITEM_KP_GAIN:      menuConfig.kpGain      = constrain(menuConfig.kpGain - 0.5f, CFG_KP_MIN, CFG_KP_MAX); break;
                    case MENU_ITEM_KD_GAIN:      menuConfig.kdGain      = constrain(menuConfig.kdGain - 1.0f, CFG_KD_MIN, CFG_KD_MAX); break;
                    case MENU_ITEM_KC_GAIN:      menuConfig.kCurveGain  = constrain(menuConfig.kCurveGain - 0.5f, CFG_KC_MIN, CFG_KC_MAX); break;
                    case MENU_ITEM_ACTIVE_BRAKE: 
                        menuConfig.activeBrakeEn = !menuConfig.activeBrakeEn;
                        motorsSetActiveBrake(menuConfig.activeBrakeEn);
                        break;
                }
            }
            delay(150);
        }
        lastSw2 = sw2;

        // SW3: SELECT / TOGGLE EDIT / EXECUTE
        if (sw3 == LOW && lastSw3 == HIGH) {
            if (menuIsEditing) {
                menuIsEditing = false;
            } else {
                if (menuSelectedIndex <= MENU_ITEM_ACTIVE_BRAKE) {
                    menuIsEditing = true;
                } else if (menuSelectedIndex == MENU_ITEM_SAVE_EXIT) {
                    if (menuConfig.baseSpeed < menuConfig.cornerSpeed) menuConfig.baseSpeed = menuConfig.cornerSpeed;
                    if (menuConfig.boostSpeed < menuConfig.baseSpeed) menuConfig.boostSpeed = menuConfig.baseSpeed;
                    configSave(menuConfig);
                    activeConfig = menuConfig;
                    motorsSetActiveBrake(activeConfig.activeBrakeEn);
                    displayShowMessage("SAVED", "CONFIG SAVED TO NVS", 1000);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == MENU_ITEM_CANCEL) {
                    menuConfig = activeConfig;
                    motorsSetActiveBrake(activeConfig.activeBrakeEn);
                    displayShowMessage("CANCEL", "CHANGES DISCARDED", 800);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == MENU_ITEM_RESTORE_DEF) {
                    configResetDefaults(menuConfig);
                    motorsSetActiveBrake(menuConfig.activeBrakeEn);
                    displayShowMessage("RESET", "RESTORED DEFAULTS", 800);
                } else if (menuSelectedIndex == MENU_ITEM_TEST_LINE_HOLD) {
                    appMode = MODE_LINE_HOLD;
                    holdFirstStep = true;
                    holdDeltaErr = 0.0f;
                    currentHoldRate = 0.0f;
                    motorsStop();
                    displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
                }
            }
            delay(200);
        }
        lastSw3 = sw3;

        MenuDisplayData mData;
        mData.selectedIndex  = menuSelectedIndex;
        mData.isEditing      = menuIsEditing;
        mData.boostSpeed     = menuConfig.boostSpeed;
        mData.baseSpeed      = menuConfig.baseSpeed;
        mData.cornerSpeed    = menuConfig.cornerSpeed;
        mData.kpGain         = menuConfig.kpGain;
        mData.kdGain         = menuConfig.kdGain;
        mData.kCurveGain     = menuConfig.kCurveGain;
        mData.activeBrakeEn  = menuConfig.activeBrakeEn;
        displayUpdateMenu(mData);

        delay(15);
        return;
    }

    // --- Pushbutton Actions ---
    // SW1: Calibrate (Idle) / Emergency Stop (Active)
    if (sw1 == LOW && lastSw1 == HIGH) {
        if (appMode == MODE_IDLE) {
            motorsStop();
            sensorsCalibrate(10000);
            // Refresh button states to reality after 10-second blocking calibration
            lastSw1 = digitalRead(PIN_BTN_SW1);
            lastSw2 = digitalRead(PIN_BTN_SW2);
            lastSw3 = digitalRead(PIN_BTN_SW3);
            return;
        } else {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("E-STOP", "EMERGENCY STOP!", 1200);
        }
    }
    lastSw1 = sw1;

    // SW2: Tap = RUN/STOP, Hold >= 450ms = LINE-HOLD
    static uint32_t sw2PressStartTime = 0;
    static bool sw2HoldTriggered = false;
    static AppMode sw2ModeOnPress = MODE_IDLE;

    if (sw2 == LOW && lastSw2 == HIGH) {
        sw2ModeOnPress = appMode;
        if (appMode == MODE_RUNNING || appMode == MODE_LINE_HOLD) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("STOPPED", "CAR IN IDLE", 800);
        } else if (appMode == MODE_IDLE) {
            sw2PressStartTime = now;
            sw2HoldTriggered = false;
        }
    } else if (sw2 == LOW && !sw2HoldTriggered) {
        if (sw2ModeOnPress == MODE_IDLE && now - sw2PressStartTime >= 450) {
            sw2HoldTriggered = true;
            appMode = MODE_LINE_HOLD;
            holdFirstStep = true;
            holdDeltaErr = 0.0f;
            currentHoldRate = 0.0f;
            motorsStop();
            displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
        }
    } else if (sw2 == HIGH && lastSw2 == LOW) {
        if (sw2ModeOnPress == MODE_IDLE && !sw2HoldTriggered) {
            appMode = MODE_RUNNING;
            runFirstStep = true;
            filteredRunDeltaErr = 0.0f;
            currentRunRate = 0.0f;
            encodersResetTicks();
            brakeHoldUntilMs = 0;
            lineLostStartTime = 0;
            lastMotionState = 0;
            lastCtrlUs = micros();
            displayShowMessage("START", "FULL SPEED RUN", 1000);
        }
    }
    lastSw2 = sw2;

    // SW3: Open Menu (Idle) / Stop (Active)
    if (sw3 == LOW && lastSw3 == HIGH) {
        if (appMode == MODE_IDLE) {
            appMode = MODE_MENU;
            motorsStop();
            menuConfig = activeConfig;
            menuSelectedIndex = 0;
            menuIsEditing = false;
            displayClearMessage();

            MenuDisplayData mData;
            mData.selectedIndex  = 0;
            mData.isEditing      = false;
            mData.boostSpeed     = menuConfig.boostSpeed;
            mData.baseSpeed      = menuConfig.baseSpeed;
            mData.cornerSpeed    = menuConfig.cornerSpeed;
            mData.kpGain         = menuConfig.kpGain;
            mData.kdGain         = menuConfig.kdGain;
            mData.kCurveGain     = menuConfig.kCurveGain;
            mData.activeBrakeEn  = menuConfig.activeBrakeEn;
            displayUpdateMenu(mData);
            return;
        } else if (appMode == MODE_RUNNING || appMode == MODE_LINE_HOLD) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("IDLE", "CAR IN IDLE", 800);
        }
    }
    lastSw3 = sw3;

    // ========================================================
    // MODE 1: IDLE
    // ========================================================
    if (appMode == MODE_IDLE) {
        if (Serial.available()) {
            char cmd = (char)Serial.read();
            if (cmd == 'h' || cmd == 'H') {
                appMode = MODE_LINE_HOLD;
                holdFirstStep = true;
                holdDeltaErr = 0.0f;
                currentHoldRate = 0.0f;
                motorsStop();
                displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
                return;
            }
        }

        if (now < brakeHoldUntilMs) {
            motorsBrake();
        } else {
            motorsStop();
        }

        if (now - lastDebugPrintMs > 100) {
            lastDebugPrintMs = now;
            bool dummyLine;
            int dummy90;
            float currentErr = sensorsGetError(0.0f, dummyLine, dummy90);

            sensorsPrintDebug(currentErr);
            sensorsPrintRaw();
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_IDLE,
                currentErr,
                dummyLine,
                norm,
                encodersGetLeftSpeedMmS(),
                encodersGetRightSpeedMmS(),
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                0, 0, 0.0f,
                sensorsIsCalibrated(),
                activeConfig.baseSpeed
            );
        }
        delay(5);
        return;
    }

    // ========================================================
    // MODE 2: LINE-HOLD BENCH TEST (Speed = 0)
    // ========================================================
    if (appMode == MODE_LINE_HOLD) {
        uint32_t nowUs = micros();
        if (nowUs - lastCtrlUs < (CONTROL_LOOP_MS * 1000UL)) return;
        lastCtrlUs = nowUs;

        encodersUpdateSpeed();
        LineDetection det = sensorsProcess();
        int leftPwm = 0;
        int rightPwm = 0;

        if (det.lineDetected) {
            float error = det.positionMm;
            float rawDeltaErr = holdFirstStep ? 0.0f : (error - lastHoldError);
            lastHoldError = error;
            float rawDeltaErrClamped = constrain(rawDeltaErr, -6.0f, 6.0f);
            holdDeltaErr = holdFirstStep ? 0.0f : (0.75f * rawDeltaErrClamped + 0.25f * holdDeltaErr);
            currentHoldRate = rawDeltaErr / (CONTROL_LOOP_MS * 0.001f);
            holdFirstStep = false;

            // Vùng chết chuyển tiếp liên tục 0.6mm quanh tâm
            float effError = (fabsf(error) <= 0.6f) ? 0.0f : ((error > 0.0f) ? (error - 0.6f) : (error + 0.6f));
            float normErr = effError / SENSOR_PITCH_MM;

            float pTerm = activeConfig.kpGain * effError;
            float dTerm = activeConfig.kdGain * holdDeltaErr;
            float cTerm = activeConfig.kCurveGain * (normErr * fabsf(normErr));

            int steer = (int)constrain(pTerm + dTerm + cTerm, -200.0f, 200.0f);
            leftPwm  = steer;
            rightPwm = -steer;
            motorsSet(leftPwm, rightPwm);
        } else {
            motorsStop();
            holdFirstStep = true;
            holdDeltaErr = 0.0f;
            currentHoldRate = 0.0f;
        }

        if (now - lastDebugPrintMs > 80) {
            lastDebugPrintMs = now;
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_LINE_HOLD,
                det.positionMm,
                det.lineDetected,
                norm,
                encodersGetLeftSpeedMmS(),
                encodersGetRightSpeedMmS(),
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                leftPwm,
                rightPwm,
                currentHoldRate,
                sensorsIsCalibrated()
            );
        }
        return;
    }

    // ========================================================
    // MODE 3: RUNNING (250 Hz Pure PD + Dashed Line & 90° Turn)
    // ========================================================
    uint32_t nowUs = micros();
    if (nowUs - lastCtrlUs < (CONTROL_LOOP_MS * 1000UL)) return;
    lastCtrlUs += (CONTROL_LOOP_MS * 1000UL);
    if (nowUs - lastCtrlUs > (CONTROL_LOOP_MS * 2000UL)) lastCtrlUs = nowUs;

    // Biến lưu trạng thái góc cua 90 độ chuyên biệt
    static uint32_t turn90StartTime    = 0;
    static uint32_t turn90LatchUntilMs = 0;
    static int8_t   turn90LatchDir     = 0;

    // 1. Encoders & Odometry
    encodersUpdateSpeed();
    float vMeasLMmS = encodersGetLeftSpeedMmS();
    float vMeasRMmS = encodersGetRightSpeedMmS();

    // 2. Đọc cảm biến
    LineDetection det = sensorsProcess();
    float error = 0.0f;

    // ========================================================
    // QUẢN LÝ GÓC CUA VUÔNG 90 ĐỘ CHUYÊN BIỆT (DEDICATED 90° CONTROLLER)
    // ========================================================
    if (det.turn90 != 0) {
        if (turn90LatchDir == 0) {
            turn90StartTime    = now;
            turn90LatchUntilMs = now + 350; // Timeout bảo vệ tối đa 350ms
            turn90LatchDir     = (int8_t)det.turn90;
            lastKnownDir       = turn90LatchDir;
            lastMotionState    = turn90LatchDir;
        }
    }

    // Kiểm tra điều kiện HOÀN THÀNH GÓC VUÔNG (Dynamic Exit Trigger):
    // Sau 60ms đầu (để mũi xe kịp vung khỏi vạch cũ):
    // Khi vạch mới đã quét lọt vào khu vực mắt trung tâm (|det.positionMm| <= 15mm):
    // NGẮT CHẾ ĐỘ 90 ĐỘ TỨC KHẮC! Để thuật toán PD khóa ngay tâm vạch mới, triệt tiêu 100% hiện tượng quay lố!
    if (turn90LatchDir != 0) {
        uint32_t turn90Elapsed = now - turn90StartTime;
        if (turn90Elapsed >= 60) {
            bool lineCentered = det.lineDetected && (fabsf(det.positionMm) <= 15.0f);
            if (lineCentered || now >= turn90LatchUntilMs) {
                turn90LatchDir     = 0;
                turn90LatchUntilMs = 0;
                turn90StartTime    = 0;
                runFirstStep       = true;
                filteredRunDeltaErr = 0.0f;
            }
        }
    }

    // THỰC THI QUAY GÓC VUÔNG 90 ĐỘ (Controlled High-Torque Pivot Turn):
    // Trong khi góc 90 độ đang diễn ra, thực thi lệnh quay vi sai tại chỗ
    // với mô-men mạnh (+140..175 / -125..155), bỏ qua PD thông thường
    if (turn90LatchDir != 0) {
        lineLostStartTime = 0; // Đang quay cua 90° chủ động, không tính là mất line

        int pivotForwardPwm = (int)constrain(activeConfig.cornerSpeed * 1.65f, 140.0f, 175.0f);
        int pivotReversePwm = (int)roundf(pivotForwardPwm * 0.90f);

        int leftPwm  = (turn90LatchDir > 0) ? pivotForwardPwm : -pivotReversePwm;
        int rightPwm = (turn90LatchDir > 0) ? -pivotReversePwm : pivotForwardPwm;
        motorsSet(leftPwm, rightPwm);

        runFirstStep = true;
        filteredRunDeltaErr = 0.0f;
        currentRunRate = 0.0f;

        // Cập nhật OLED
        static uint32_t last90DispMs = 0;
        if (now - last90DispMs > 80) {
            last90DispMs = now;
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_RUNNING,
                (turn90LatchDir > 0) ? SENSOR_EXTRAPOLATE_MM : -SENSOR_EXTRAPOLATE_MM,
                true,
                norm,
                vMeasLMmS,
                vMeasRMmS,
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                leftPwm,
                rightPwm,
                0.0f,
                sensorsIsCalibrated(),
                activeConfig.cornerSpeed
            );
        }
        return;
    }

    if (det.lineDetected) {
        error = det.positionMm;
        lineLostStartTime = 0;

        // Ghi nhớ trạng thái chuyển động / ngã rẽ cuối cùng trước khi mất line:
        // - Góc vuông 90° hoặc lệch nhiều sang trái (error <= -9.0mm) -> RẼ TRÁI (-1)
        // - Góc vuông 90° hoặc lệch nhiều sang phải (error >= +9.0mm) -> RẼ PHẢI (+1)
        // - Nằm trong dải trung tâm (|error| <= 6.0mm) và không có tín hiệu góc vuông -> LINE THẲNG (0)
        if (det.turn90 == -1 || error <= -9.0f) {
            lastMotionState = -1; // Ngã rẽ trái
            lastKnownDir    = -1;
        } else if (det.turn90 == 1 || error >= 9.0f) {
            lastMotionState = 1;  // Ngã rẽ phải
            lastKnownDir    = 1;
        } else if (fabsf(error) <= 6.0f && det.turn90 == 0) {
            lastMotionState = 0;  // Line thẳng
        }
    } else {
        // ====================================================
        // MẤT LINE: XỬ LÝ THEO TRẠNG THÁI GẦN NHẤT (OVERSHOOT & GAP RECOVERY)
        // 1. Nếu lần cuối là LINE THẲNG (0): Đi tiếp thẳng để vượt nét đứt / khoảng trống
        // 2. Nếu lần cuối là RẼ TRÁI (-1): Xoay gắt sang TRÁI để bắt lại vạch bị vượt quá
        // 3. Nếu lần cuối là RẼ PHẢI (+1): Xoay gắt sang PHẢI để bắt lại vạch bị vượt quá
        // 4. Ngay khi chạm lại line: Lập tức thoát ra và đi tiếp theo PD!
        // ====================================================
        if (lineLostStartTime == 0) {
            lineLostStartTime = now;
        }
        uint32_t lostElapsed = now - lineLostStartTime;

        // Quá thời gian tìm kiếm tối đa (LINE_LOST_TIMEOUT_MS = 3000ms): Dừng an toàn
        if (lostElapsed > LINE_LOST_TIMEOUT_MS) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("SAFETY STOP", "LINE LOST >3S", 1500);
            return;
        }

        int leftPwm  = 0;
        int rightPwm = 0;

        // Mô-men xoay tìm vạch mạnh mẽ (165 - 195 PWM) để thắng hoàn toàn ma sát bánh xe
        int searchPwm = (int)constrain(activeConfig.cornerSpeed * 1.85f, 165.0f, 195.0f);
        int revPwm    = (int)roundf(searchPwm * 0.90f);

        if (lastMotionState == 0) {
            // ------------------------------------------------
            // TRƯỜNG HỢP 1: LẦN CUỐI CÙNG LÀ LINE THẲNG
            // -> Đi tiếp thẳng về phía trước để vượt qua đoạn nét đứt / khoảng hở vạch
            // ------------------------------------------------
            if (lostElapsed < 220) {
                // Trong 220ms đầu (~10-15cm): Giữ thẳng lái phóng tới
                int straightPwm = (int)activeConfig.baseSpeed;
                leftPwm  = straightPwm;
                rightPwm = straightPwm;
            } else if (lostElapsed < 670) {
                // Sau 220ms vẫn chưa thấy vạch: Bắt đầu xoay tìm kiếm theo hướng lastKnownDir
                int sDir = (lastKnownDir != 0) ? lastKnownDir : 1;
                leftPwm  = (sDir > 0) ? searchPwm : -revPwm;
                rightPwm = (sDir > 0) ? -revPwm : searchPwm;
            } else if (lostElapsed < 1120) {
                // Đảo chiều quét bán cầu đối diện
                int sDir = (lastKnownDir != 0) ? -lastKnownDir : -1;
                leftPwm  = (sDir > 0) ? searchPwm : -revPwm;
                rightPwm = (sDir > 0) ? -revPwm : searchPwm;
            } else {
                // Quét qua lại tuần hoàn mỗi 450ms
                uint32_t cycle = (lostElapsed - 1120) / 450;
                int sDir = (cycle % 2 == 0) ? lastKnownDir : -lastKnownDir;
                if (sDir == 0) sDir = 1;
                leftPwm  = (sDir > 0) ? searchPwm : -revPwm;
                rightPwm = (sDir > 0) ? -revPwm : searchPwm;
            }
        } else {
            // ------------------------------------------------
            // TRƯỜNG HỢP 2 & 3: LẦN CUỐI LÀ RẼ TRÁI (-1) HOẶC RẼ PHẢI (+1)
            // Xe đi quá đà văng khỏi cua -> Lập tức xoay gắt về phía cua đó để tóm lại vạch!
            // ------------------------------------------------
            int turnDir = (lastMotionState > 0) ? 1 : -1;

            if (lostElapsed < 450) {
                // PHA 1 (0 .. 450ms): Xoay gắt theo đúng hướng cua bị vượt quá
                // (Rẽ trái -> Xoay trái; Rẽ phải -> Xoay phải)
                leftPwm  = (turnDir > 0) ? searchPwm : -revPwm;
                rightPwm = (turnDir > 0) ? -revPwm : searchPwm;
            } else if (lostElapsed < 1050) {
                // PHA 2 (450 .. 1050ms): Đảo chiều quay ngược lại quét 360° dự phòng
                leftPwm  = (turnDir > 0) ? -revPwm : searchPwm;
                rightPwm = (turnDir > 0) ? searchPwm : -revPwm;
            } else {
                // PHA 3 (> 1050ms): Đảo chiều tuần hoàn mỗi 450ms
                uint32_t cycle = (lostElapsed - 1050) / 450;
                int sDir = (cycle % 2 == 0) ? turnDir : -turnDir;
                leftPwm  = (sDir > 0) ? searchPwm : -revPwm;
                rightPwm = (sDir > 0) ? -revPwm : searchPwm;
            }
        }

        motorsSet(leftPwm, rightPwm);

        runFirstStep = true;
        filteredRunDeltaErr = 0.0f;
        currentRunRate = 0.0f;

        // Cập nhật màn hình OLED trạng thái SEARCH để quan sát trực quan
        static uint32_t lastSearchDispMs = 0;
        if (now - lastSearchDispMs > 80) {
            lastSearchDispMs = now;
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_RUNNING,
                0.0f,
                false,
                norm,
                vMeasLMmS,
                vMeasRMmS,
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                leftPwm,
                rightPwm,
                0.0f,
                sensorsIsCalibrated(),
                activeConfig.baseSpeed
            );
        }
        return;
    }

    // 3. PD Steering với lọc vi phân phản xạ nhanh
    float rawDeltaErr = runFirstStep ? 0.0f : (error - lastRunError);
    lastRunError = error;

    // Giữ trọn lực bẻ lái D khi vào cua gấp
    float maxDelta = 30.0f;
    float rawDeltaErrClamped = constrain(rawDeltaErr, -maxDelta, maxDelta);
    filteredRunDeltaErr = runFirstStep ? 0.0f : (0.75f * rawDeltaErrClamped + 0.25f * filteredRunDeltaErr);
    currentRunRate = rawDeltaErr / (CONTROL_LOOP_MS * 0.001f);
    runFirstStep = false;

    // Vùng chết chuyển tiếp liên tục 0.5mm quanh tâm
    float effError = (fabsf(error) <= 0.5f) ? 0.0f : ((error > 0.0f) ? (error - 0.5f) : (error + 0.5f));
    float normErr  = effError / SENSOR_PITCH_MM;

    float pTerm = activeConfig.kpGain * effError;

    // TRIỆT TIÊU HIỆN TƯỢNG "BỊ CHẶN LẠI KHI CUA NHANH" (ANTI-DAMPING CHOKE):
    // Khi đang trong góc cua (|effError| > 6.0mm):
    // Nếu sai số đang giảm (effError * filteredRunDeltaErr < 0, tức xe đang xoay đầu vào vạch cua),
    // khâu D sinh lực cản âm cực lớn chống lại việc bẻ lái làm xe bị "sượng" / "chặn lại".
    // Tại đây, ta giảm 70% lực cản vi phân để giải phóng hoàn toàn mô-men bẻ lái!
    float effectiveKd = activeConfig.kdGain;
    if (fabsf(effError) > 6.0f && (effError * filteredRunDeltaErr < 0.0f)) {
        effectiveKd *= 0.30f;
    }
    float dTerm = effectiveKd * filteredRunDeltaErr;

    // Tăng lực bẻ lái phi tuyến cTerm khi vào cua để ép bánh xe xoay gắt
    float cTerm = activeConfig.kCurveGain * (normErr * fabsf(normErr) * 3.5f);
    float steerPwm = constrain(pTerm + dTerm + cTerm, -255.0f, 255.0f);

    // 4. Điều phối tốc độ tiến (Speed Coordination & Rate-Sensitive Deceleration)
    float absErr = fabsf(error);
    float basePwm = activeConfig.baseSpeed;

    // Tính độ khẩn cấp vào cua từ cả độ lệch vạch (absErr) và tốc độ trôi vạch (filteredRunDeltaErr):
    float cornerUrgency = 0.0f;
    if (absErr > 4.0f) {
        cornerUrgency = (absErr - 4.0f) / 14.0f; // 4mm -> 18mm
    }
        float rateUrgency = fabsf(filteredRunDeltaErr) / 4.0f; // Nhận diện cua từ tốc độ trôi
        if (rateUrgency > cornerUrgency) cornerUrgency = rateUrgency;
        cornerUrgency = constrain(cornerUrgency, 0.0f, 1.0f);

        if (cornerUrgency > 0.0f) {
            basePwm = activeConfig.boostSpeed - cornerUrgency * (activeConfig.boostSpeed - activeConfig.cornerSpeed);
            if (absErr > 18.0f) {
                // Cua rất gắt: hạ tiếp basePwm xuống tối đa 50% cornerSpeed để bánh ngoài luôn có lực đẩy tới mạnh
                float extremeRatio = constrain((absErr - 18.0f) / 15.0f, 0.0f, 1.0f);
                basePwm = activeConfig.cornerSpeed - extremeRatio * (activeConfig.cornerSpeed * 0.50f);
            }
        } else {
            basePwm = activeConfig.boostSpeed;
        }

    // 5. Phân bổ công suất bánh xe
    int leftPwm  = (int)roundf(basePwm + steerPwm);
    int rightPwm = (int)roundf(basePwm - steerPwm);

    leftPwm  = constrain(leftPwm,  -255, 255);
    rightPwm = constrain(rightPwm, -255, 255);
    motorsSet(leftPwm, rightPwm);

    // 6. Telemetry & Display
    if (now - lastDebugPrintMs >= 1000) {
        lastDebugPrintMs = now;
        if (Serial.availableForWrite() >= 96) {
            Serial.printf("[PD RUN] e:%+5.1f mm | Spd:%.0f | L:%+4d R:%+4d | Rt:%+4.0f\n",
                error, basePwm, leftPwm, rightPwm, currentRunRate);
        }
    }

    static uint32_t lastRunDispMs = 0;
    if (now - lastRunDispMs > 80) {
        lastRunDispMs = now;
        int norm[8];
        sensorsGetNormValues(norm);
        displayUpdateSnapshot(
            DISP_MODE_RUNNING,
            error,
            det.lineDetected,
            norm,
            vMeasLMmS,
            vMeasRMmS,
            encodersGetLeftDistanceMm(),
            encodersGetRightDistanceMm(),
            leftPwm,
            rightPwm,
            currentRunRate,
            sensorsIsCalibrated(),
            activeConfig.baseSpeed
        );
    }
}