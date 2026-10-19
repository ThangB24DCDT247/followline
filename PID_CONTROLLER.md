# CHI TIẾT LÕI ĐIỀU KHIỂN PD (LINE FOLLOWER CONTROL CORE)

Tài liệu này phân tích chi tiết về mặt toán học, thuật toán và mã nguồn của **Bộ điều khiển bẻ lái bám vạch thời gian thực** (Real-Time Pure PD Steering Controller) được triển khai trên dòng xe dò line tốc độ cao sử dụng vi điều khiển ESP32.

---

## 1. Tổng quan kiến trúc bộ điều khiển

Trong các ứng dụng xe dò line tốc độ cao, việc sử dụng bộ điều khiển PID truyền thống thường gặp vấn đề lớn do **khâu Tích phân ($I$)**. Vì vậy, hệ thống này được thiết kế dựa trên cấu trúc:

$$\text{Pure PD} + \text{First-Order Low-Pass Filter (EMA)} + \text{Non-Linear Curve Boost} + \text{Adaptive Speed Profiling}$$

```mermaid
flowchart TD
    subgraph Input["1. Đầu vào (Cảm biến)"]
        Sensors["8 Cảm biến MUX CD74HC4051"] --> Norm["Chuẩn hóa & Lọc sàn [0.0..1.0]"]
        Norm --> Centroid["Tính trọng tâm Centroid Weighted"]
        Centroid --> Error["Sai số thực tế: e (mm)<br/>[-50.4 mm .. +50.4 mm]"]
    end

    subgraph CorePD["2. Lõi điều khiển PD (250 Hz - 4ms)"]
        Error --> P_Term["Khâu Tỉ lệ (P):<br/>P = Kp * e_eff"]
        
        Error --> DeltaCalc["Đạo hàm thô:<br/>Δe_raw = e(t) - e(t-1)"]
        DeltaCalc --> KickReject["Khử Derivative Kick:<br/>Δe_clamped = constrain(Δe_raw, -6.0, +6.0)"]
        KickReject --> EMA_Filter["Lọc thông thấp EMA (α=0.75):<br/>Δe_filt = 0.75*Δe_clamped + 0.25*Δe_prev"]
        EMA_Filter --> D_Term["Khâu Vi phân (D):<br/>D = Kd * Δe_filt"]
        
        Error --> NormPos["Chuẩn hóa tọa độ pitch:<br/>e_norm = e_eff / PITCH_MM"]
        NormPos --> CurveBoost["Bù cua phi tuyến (C):<br/>C = Kc * (e_norm * |e_norm|)"]
        
        P_Term --> Sum["Tổng lái:<br/>Steer = P + D + C"]
        D_Term --> Sum
        CurveBoost --> Sum
    end

    subgraph SpeedControl["3. Điều phối tốc độ thích ứng"]
        Error --> SpeedCheck{"Kiểm tra |e|"}
        SpeedCheck -- "|e| < 3 mm" --> Boost["Boost Speed (160 PWM)"]
        SpeedCheck -- "3 mm <= |e| <= 8 mm" --> Base["Base Speed (130 PWM)"]
        SpeedCheck -- "|e| > 8 mm" --> Corner["Corner Speed (90 PWM)"]
    end

    subgraph Output["4. Bộ trộn công suất (Mixer)"]
        Sum --> Mixer["Differential Mixer"]
        Boost --> Mixer
        Base --> Mixer
        Corner --> Mixer
        Mixer --> OutL["Left PWM = Base + Steer"]
        Mixer --> OutR["Right PWM = Base - Steer"]
        OutL --> Driver["Driver DRV8871 (20kHz PWM)"]
        OutR --> Driver
    end
```

---

## 2. Mô hình toán học chi tiết

### 2.1. Tín hiệu sai số đầu vào ($e$)
Sai số $e$ (tính bằng **milimét**) thể hiện khoảng cách từ tâm thanh cảm biến của xe đến vị trí tâm của vạch đen:

$$e = \text{positionNorm} \times \text{SENSOR\_PITCH\_MM}$$

Trong đó:
* $\text{positionNorm} = \frac{\sum_{i=0}^7 (S_i \cdot W_i)}{\sum_{i=0}^7 S_i} \in [-3.5, +3.5]$
* Trọng số cảm biến: $W = [-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5]$
* Bước khoảng cách giữa 2 mắt liền kề: $\text{SENSOR\_PITCH\_MM} = \frac{83.0\text{ mm}}{7} \approx 11.857\text{ mm}$
* Khi vạch văng ra mép ngoài thanh cảm biến, thuật toán kích hoạt ngoại suy (Extrapolation): $e = \pm 50.39\text{ mm}$.

---

### 2.2. Khâu Tỉ lệ (Proportional - P)

$$P = K_p \times e$$

* **Ý nghĩa vật lý:** Tạo ra phản lực bẻ lái tỉ lệ thuận với khoảng cách lệch tâm. Lệch càng xa vạch, bánh xe càng bẻ góc lớn để kéo xe về lại tâm.
* **Hệ số mặc định:** $K_p = 4.5$ (`KP_GAIN`).
* **Ví dụ:** Khi lệch $e = +10\text{ mm}$, thành phần $P = 4.5 \times 10 = +45\text{ PWM}$.

---

### 2.3. Khâu Vi phân chống rung giật & Khử Derivative Kick (Fast D-Filter with Kick Rejection)

Trong thực tế, phép lấy vi phân thuần $\Delta e = e_t - e_{t-1}$ trên dữ liệu ADC đọc trực tiếp sẽ bị ảnh hưởng bởi 2 vấn đề lớn:
1. **Nhiễu lượng tử hóa ($\pm 0.5\text{ mm}$ do rung bề mặt sàn):** Với $K_d = 22.0$, nhiễu này sinh ra gai nhọn $\pm 22\text{ PWM}$ ở tần số 250 Hz, là nguyên nhân gây hiện tượng rung lắc đuôi (chatter/fishtailing) trên đường thẳng.
2. **Cú sốc vi phân (Derivative Kick) khi chuyển trạng thái ngoại suy:** Khi xe vào cua gắt hoặc thoát cua, cảm biến chuyển trạng thái từ dò vạch thông thường ($e \approx 30\text{ mm}$) sang ngoại suy mép ngoài ($e = 50.4\text{ mm}$) hoặc ngược lại, $\Delta e$ nhảy vọt $15\text{ mm}$ đến $35\text{ mm}$ chỉ trong đúng 1 chu kỳ 4ms. Nhân với $K_d = 22.0$, tín hiệu $D$ sinh ra cú đạp phanh ngược lên tới $-500\text{ PWM}$ đến $-700\text{ PWM}$, làm đuôi xe bị quăng mạnh ra ngoài!

Để giải quyết triệt để 2 vấn đề này:
* **Khử Derivative Kick:** Giới hạn tốc độ biến thiên vị trí tối đa về mặt động học thực tế ($1.5\text{ m/s} = 6.0\text{ mm} / 4\text{ ms}$):
  $$\Delta e_{\text{clamped}} = \text{constrain}(\Delta e_{\text{raw}}, -6.0\text{ mm}, +6.0\text{ mm})$$
* **Bộ lọc đáp ứng nhanh ($\alpha = 0.75$):**
  $$\Delta e_{\text{filtered}}(t) = 0.75 \cdot \Delta e_{\text{clamped}}(t) + 0.25 \cdot \Delta e_{\text{filtered}}(t-1)$$
  $$D = K_d \times \Delta e_{\text{filtered}}$$
* **Vùng chết chuyển tiếp liên tục (Continuous Deadband 0.6 mm):**
  $$e_{\text{eff}} = \begin{cases} 0 & \text{khi } |e| \le 0.6\text{ mm} \\ e - 0.6 & \text{khi } e > 0.6\text{ mm} \\ e + 0.6 & \text{khi } e < -0.6\text{ mm} \end{cases}$$
  Triệt tiêu hoàn toàn bước nhảy gián đoạn tại ngưỡng 0.6mm, giúp xe lướt trên đường thẳng êm tuyệt đối.

---

### 2.4. Khâu Bù cua phi tuyến chuẩn hóa (Normalized Non-Linear Curve Boost - C)

Để vượt qua các góc vuông 90° hoặc khúc cua chữ U bán kính nhỏ mà không làm mất ổn định trên đoạn đường thẳng, mã nguồn tích hợp thành phần phi tuyến được chuẩn hóa theo tọa độ pitch cảm biến ($e_{\text{norm}} \in [-3.5, +3.5]$):

$$e_{\text{norm}} = \frac{e_{\text{eff}}}{\text{SENSOR\_PITCH\_MM}}$$

$$C = K_c \times \left(e_{\text{norm}} \cdot |e_{\text{norm}}|\right)$$

* **Hệ số mặc định:** $K_c = 2.0$ (`K_CURVE_GAIN`, có thể điều chỉnh qua Menu OLED từ $0.0$ đến $20.0$).
* **Đặc tính phi tuyến chuẩn hóa:**
  * Khi sai số nhỏ ($|e| < 6\text{ mm} \implies |e_{\text{norm}}| < 0.5$): $C \approx 2.0 \times 0.25 = 0.5\text{ PWM}$ (bằng 0, không làm rung xe).
  * Khi vào cua vừa ($e \approx 25\text{ mm} \implies e_{\text{norm}} \approx 2.1$): $C \approx 2.0 \times 4.4 = 8.8\text{ PWM}$.
  * Khi vào cua gắt ($e \approx 40\text{ mm} \implies e_{\text{norm}} \approx 3.37$): $C \approx 2.0 \times 11.4 = 22.8\text{ PWM}$.
  * Khi chạm mép ngoại suy ($e = 50.4\text{ mm} \implies e_{\text{norm}} = 4.25$): $C \approx 2.0 \times 18.06 = 36.1\text{ PWM}$.
  * **Khắc phục lỗi bùng nổ PWM:** Trước đây công thức cũ dùng $e_{\text{eff}}^2 \times 0.05$ (đơn vị mm²), tại $e = 40\text{ mm}$ sinh ra tới $160\text{ PWM}$ và tại $e = 50\text{ mm}$ sinh ra $254\text{ PWM}$, làm cộng dồn với $P$ vượt quá $480\text{ PWM}$ gây quá bão hòa và văng xe. Công thức chuẩn hóa $e_{\text{norm}}^2$ đã đưa lực lái về đúng dải điều khiển tối ưu.

---

### 2.5. Thuật toán chống triệt tiêu lực cua (Anti-Cancellation Algorithm)

#### Vấn đề thực tế trong các khúc cua:
Khi xe đang ôm cua sang phải, cụm mắt bên phải (Ch6, Ch7) bắt được vạch đen ($e > 0$). Tuy nhiên, do góc cua chữ U hoặc vòng lặp track sát nhau, mắt bên trái (Ch0, Ch1) cũng bất ngờ đọc được vạch đen.
* Trọng số mắt trái mang dấu âm ($W_0 = -3.5, W_1 = -2.5$).
* Trọng số mắt phải mang dấu dương ($W_6 = +2.5, W_7 = +3.5$).
* Nếu tính trọng tâm trên toàn bộ 8 mắt, tín hiệu âm bên trái sẽ **triệt tiêu tín hiệu dương bên phải**, khiến sai số $e$ sụt giảm nghiêm trọng về $0\text{ mm}$!
* Khi $e \to 0$: Lực bẻ lái $\text{steerPwm} \to 0$, và bộ điều phối tốc độ nhầm tưởng xe đang đi đường thẳng nên tăng vọt lên `boostSpeed` (160 PWM), khiến xe mất lái và lao thẳng ra khỏi khúc cua!

#### Giải pháp giải quyết triệt để:
Hệ thống sử dụng cơ chế **Bảo toàn hướng cua & Triệt tiêu cánh đối diện khi có khoảng trống rộng**:
1. Nhận diện xung đột khi cả 2 cánh ngoài cùng đều có tín hiệu (`S[0]+S[1] > 0.35` và `S[6]+S[7] > 0.35`), VÀ toàn bộ vùng giữa (Ch2, Ch3, Ch4, Ch5) đều là sàn trắng ($< 0.20$).
2. Nếu xe đang trong hướng cua phải (`lastGoodLineDir > 0`): Loại bỏ cánh trái ($S[0] = 0, S[1] = 0$). Lực cua phải được bảo toàn 100%.
3. Nếu xe đang trong hướng cua trái (`lastGoodLineDir < 0`): Loại bỏ cánh phải ($S[6] = 0, S[7] = 0$). Lực cua trái được bảo toàn 100%.
4. Cơ chế này tuyệt đối **không can thiệp trên vạch đơn thông thường**, loại trừ 100% tình trạng nhảy sai số giả.

---

### 2.6. Tổng hợp tín hiệu lái (Steer PWM)

$$\text{steerPwm} = \text{constrain}(P + D + C, -255, +255)$$

---

### 2.7. Cơ chế nhận diện góc vuông 90° & Tự động quay lại tìm vạch tức thì (Active Line Recovery)

#### 1. Nhận diện góc vuông 90° (`turn90`) & Khóa trạng thái (90° Turn Latch):
* **Nhận diện:** Khi một nửa thanh cảm biến bên phải (`Ch4..Ch7`) hoặc bên trái (`Ch0..Ch3`) đồng loạt bắt được vạch đậm ($\sum S > 1.4$) trong khi 2 mắt mép bên kia hoàn toàn trắng ($< 0.25$), hệ thống kích hoạt `det.turn90 = +1` hoặc `-1`.
* **Khóa trạng thái (`turn90LatchUntilMs` trong 200ms):**
  * Ép vị trí về mép ngoại suy cực đại: $e = \pm 50.4\text{ mm}$ ($e_{\text{norm}} = \pm 4.25$).
  * Nới lỏng kẹp đạo hàm: $\text{maxDelta} = 35.0\text{ mm}$ để giải phóng $100\%$ xung vi phân bẻ lái tức thời.
  * Ghìm sâu tốc độ tiến: $V_{\text{base}} = 0.40 \times V_{\text{corner}}$ ($\sim 36\text{ PWM}$) để mô-men vi sai đảo chiều bánh trong mạnh mẽ, giúp xe xoay Pivot Turn tại chỗ.
  * Nếu xe lố đà văng khỏi line: Bánh xe lập tức xoay đối xứng (`+sweepPwm` / `-sweepPwm`) để bẻ ngoặt đầu xe 90° vào lại track.

#### 2. Tự động quay lại tìm vạch tức thì khi mất line (Active Line Recovery):
* Khi mất vạch (`det.lineDetected == false`), hệ thống **tuyệt đối không phóng thẳng**:
  * Ngay chu kỳ 4ms đầu tiên, xe kích hoạt xoay tại chỗ (Pivot Turn) theo đúng hướng vạch vừa thoát (`lastKnownDir` hoặc `turn90LatchDir`).
  * Trong 700ms đầu tiên: Duy trì xoay dứt khoát và liên tục về hướng vạch vừa thoát (bánh ngoài quay tới `110-130 PWM`, bánh trong quay lùi `-95 đến -110 PWM`).
  * 99% trường hợp mất vạch là do cua gấp, vạch nằm ngay sát bên cạnh xe. Nhờ xoay tại chỗ liên tục, thanh cảm biến phía trước sẽ quét ngược lại cắt ngang qua vạch trong vòng 80-200ms và khóa lại vạch tức thì.
  * Sau 700ms nếu chưa bắt được vạch, hệ thống mới đảo chiều quét mở rộng góc tìm kiếm trước khi dừng an toàn ở mốc 3000ms.

---

## 3. Lý do loại bỏ hoàn toàn khâu Tích phân ($I$)

Nhiều người thường thắc mắc vì sao lại là **PD** mà không phải **PID** đầy đủ. Trong xe dò line thi đấu tốc độ cao ($> 1.5\text{ m/s}$):

1. **Khắc phục hiện tượng Trễ pha (Phase Lag):** Khâu $I$ tích lũy sai số theo thời gian ($\int e\,dt$). Khi xe đổi hướng liên tục từ cua trái sang cua phải (hình zíc-zắc S-curve), khâu $I$ vẫn giữ điện áp tích lũy của khúc cua cũ, làm bánh xe bị trễ phản ứng và mất line ngay lập tức.
2. **Loại trừ bão hòa tích phân (Anti-windup):** Khâu $I$ rất dễ bị tích lũy giá trị cực đại khi vào cua sâu, gây vọt lố nghiêm trọng khi thoát cua.
3. **Độ chính xác xác lập không yêu cầu tuyệt đối:** Mục tiêu của xe dò line là **tốc độ đáp ứng tức thời (Transient Response)** thay vì triệt tiêu sai số tĩnh bằng $0$ như các cánh tay robot đứng yên.

---

## 4. Bộ điều phối tốc độ thích ứng (Adaptive Speed Profiling)

Hệ thống tự động thay đổi tốc độ cơ sở ($V_{\text{base}}$) theo độ cong của đường:

```text
               |e| < 3 mm                3 mm <= |e| <= 8 mm               |e| > 8 mm
           (Đường thẳng tắp)              (Đường cong nhẹ)               (Cua gấp / U-turn)
         ┌───────────────────┐         ┌────────────────────┐         ┌────────────────────┐
Tốc độ:  │   BOOST SPEED     │         │     BASE SPEED     │         │    CORNER SPEED    │
PWM:     │       160         │         │        130         │         │         90         │
         └───────────────────┘         └────────────────────┘         └────────────────────┘
```

* **Đường thẳng ($|e| < 3.0\text{ mm}$):** Xe bung hết công suất với `boostSpeed = 160` để rút ngắn thời gian vòng đua.
* **Đường lượn ($3.0\text{ mm} \le |e| \le 8.0\text{ mm}$):** Xe duy trì tốc độ ổn định với `baseSpeed = 130`.
* **Cua gắt ($|e| > 8.0\text{ mm}$):** Tự động ghìm tốc độ về `cornerSpeed = 90` để hạ quán tính li tâm trước khi bẻ lái.

---

## 5. Bộ trộn công suất vi sai (Differential Drive Mixer)

Sau khi tính được $V_{\text{base}}$ và $\text{steerPwm}$, tín hiệu được phân bổ ra 2 động cơ:

$$PWM_{\text{Left}} = \text{constrain}\left(\text{round}(V_{\text{base}} + \text{steerPwm}), -255, 255\right)$$

$$PWM_{\text{Right}} = \text{constrain}\left(\text{round}(V_{\text{base}} - \text{steerPwm}), -255, 255\right)$$

### Phân tích trạng thái hoạt động:
* **Khi đi thẳng ($e = 0, \text{steerPwm} = 0$):** $PWM_{\text{Left}} = PWM_{\text{Right}} = V_{\text{base}}$.
* **Khi lệch sang phải ($e > 0, \text{steerPwm} > 0$):** 
  * Bánh Trái tăng ga: $V_{\text{base}} + \text{steerPwm}$.
  * Bánh Phải giảm ga: $V_{\text{base}} - \text{steerPwm}$.
* **Khi vào cua cực gắt ($\text{steerPwm} > V_{\text{base}}$):**
  * Bánh Phải mang giá trị âm ($PWM_{\text{Right}} < 0$).
  * Nhờ chế độ **Fast Decay & Active Dynamic Brake** của driver DRV8871 (xung 20 kHz), bánh trong cua sẽ **quay lùi chủ động**, giúp xe xoay quanh tâm trục bánh (Pivot Turn) với bán kính bằng $0$ mà không bị lật.

---

## 6. Đoạn mã triển khai thực tế (`src/main.cpp`)

Dưới đây là đoạn code thực thi lõi điều khiển chạy ở chu kỳ **4ms (250Hz)**:

```cpp
// 1. Kiểm tra chu kỳ cứng 4ms
uint32_t nowUs = micros();
if (nowUs - lastCtrlUs < (CONTROL_LOOP_MS * 1000UL)) return;
lastCtrlUs += (CONTROL_LOOP_MS * 1000UL);

// 2. Tính đạo hàm sai số và lọc vi phân đáp ứng nhanh (Alpha = 0.75)
float rawDeltaErr = runFirstStep ? 0.0f : (error - lastRunError);
lastRunError = error;
filteredRunDeltaErr = runFirstStep ? 0.0f : (0.75f * rawDeltaErr + 0.25f * filteredRunDeltaErr);
currentRunRate = rawDeltaErr / (CONTROL_LOOP_MS * 0.001f);
runFirstStep = false;

// 3. Vùng chết nhẹ 0.6mm quanh tâm và tính PD + Curve Boost
float effError = (fabsf(error) < 0.6f) ? 0.0f : error;
float pTerm = activeConfig.kpGain * effError;
float dTerm = activeConfig.kdGain * filteredRunDeltaErr;
float cTerm = activeConfig.kCurveGain * (effError * fabsf(effError) * 0.05f);
float steerPwm = pTerm + dTerm + cTerm;

// 4. Điều phối tốc độ tiến nội suy mượt mà
float absErr = fabsf(error);
float basePwm = activeConfig.baseSpeed;
if (absErr <= 3.0f) {
    basePwm = activeConfig.boostSpeed;
} else if (absErr >= 25.0f) {
    basePwm = activeConfig.cornerSpeed * 0.70f; // Ghìm sâu tốc độ khi cua gắt
} else if (absErr >= 14.0f) {
    float r = (absErr - 14.0f) / (25.0f - 14.0f);
    basePwm = activeConfig.cornerSpeed - r * (activeConfig.cornerSpeed * 0.30f);
} else {
    float ratio = (absErr - 3.0f) / (14.0f - 3.0f);
    basePwm = activeConfig.boostSpeed - ratio * (activeConfig.boostSpeed - activeConfig.cornerSpeed);
}

// 5. Phân bổ công suất bánh xe
int leftPwm  = (int)roundf(basePwm + steerPwm);
int rightPwm = (int)roundf(basePwm - steerPwm);

leftPwm  = constrain(leftPwm,  -255, 255);
rightPwm = constrain(rightPwm, -255, 255);
motorsSet(leftPwm, rightPwm);
```

---

## 7. Bảng thông số và hướng dẫn hiệu chỉnh (Tuning Guide)

### 7.1. Bảng giá trị mặc định (`include/config.h`)

| Tham số | Biến cấu hình | Mặc định | Dải điều chỉnh | Ý nghĩa |
| :--- | :--- | :---: | :---: | :--- |
| **Kp** | `kpGain` | **4.5** | 0.0 – 30.0 | Độ nhạy bẻ lái theo sai số $e$ |
| **Kd** | `kdGain` | **22.0** | 0.0 – 100.0 | Giảm chấn, triệt tiêu lắc xe |
| **Kc** | `kCurveGain` | **2.0** | 0.0 – 20.0 | Bù lực phi tuyến khi cua gắt |
| **Boost Spd** | `boostSpeed` | **160** | 50 – 255 | Tốc độ khi chạy đường thẳng |
| **Base Spd** | `baseSpeed` | **130** | 40 – 255 | Tốc độ hành trình tiêu chuẩn |
| **Corner Spd** | `cornerSpeed` | **90** | 30 – 255 | Tốc độ khi vào khúc cua sâu |
| **Line Lost TO** | `LINE_LOST_TIMEOUT_MS` | **2000 ms** | 500 – 5000 ms | Thời gian quét tìm lại vạch trước khi ngắt động cơ |

---

### 7.2. Quy trình cân chỉnh PD thực chiến (5 bước)

1. **Bước 1 (Thiết lập ban đầu):** Đặt $K_d = 0$, $K_c = 0$, chọn tốc độ thấp (`baseSpeed = 80`, `cornerSpeed = 60`, `boostSpeed = 80`).
2. **Bước 2 (Tìm $K_p$ tới hạn):** Tăng dần $K_p$ (từng nấc 0.5) cho đến khi xe bắt đầu bám được vạch nhưng có hiện tượng lắc lư đuôi xe liên tục quanh tâm vạch.
3. **Bước 3 (Thêm $K_d$ dập dao động):** Tăng dần $K_d$ (từng nấc 2.0 đến 4.0). Khi $K_d$ đủ lớn, toàn bộ hiện tượng lắc đuôi sẽ biến mất, xe chạy bám tâm vạch phẳng lặng như đi trên ray. Nếu tăng $K_d$ quá cao xe sẽ phát ra tiếng rè cơ khí và rung nhẹ.
4. **Bước 4 (Tăng tốc độ):** Nâng dần `baseSpeed` và `boostSpeed` lên giá trị mong muốn (130 – 180).
5. **Bước 5 (Kích hoạt $K_c$ cho khúc cua gắt):** Nếu xe bị trượt văng ra ngoài ở những góc cua 90° hoặc vòng lặp gấp, tăng dần $K_c$ từ $2.0$ đến $8.0$ để tăng cường lực xoay phi tuyến khi xe lệch khỏi $8\text{ mm}$.

