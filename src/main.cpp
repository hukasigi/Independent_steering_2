#include "AnglePID.h"
#include "SpeedPID.h"
#include <Arduino.h>
#include <CAN.h>
#include <ESP32Encoder.h>
#include <PS4Controller.h>

constexpr int8_t PIN_STEER_MOTOR_PWM = 22;
constexpr int8_t PIN_STEER_MOTOR_DIR = 23;
constexpr int8_t PIN_ENC_A           = 27;
constexpr int8_t PIN_ENC_B           = 14;
constexpr int8_t PIN_ENC_Z           = 25;

constexpr int8_t W1_CH = 0;

constexpr double ENC_RESOLUTION                  = 4096.;
constexpr double STEER_GEAR_RATIO_MOTOR_TO_STEER = 2.0; // 1:2（ステア角に対しモータ2回転）

constexpr int16_t PWM_LIMIT    = 255;
constexpr int     PWM_DEADBAND = 8;

constexpr int      HOMING_PWM        = 70;
constexpr uint32_t HOMING_TIMEOUT_MS = 8000;

constexpr int16_t MOTOR_CURRENT_LIMIT = 2000;

// グローバル変数
ESP32Encoder encoder;
AnglePID     anglePID_STEER_1(5., 0.0, 0.0, -300.0, 300.0, 360.0, -10.0, 10.0);
SpeedPID     speedPID_STEER_1(1., 20., 0.00, -PWM_LIMIT, PWM_LIMIT);
SpeedPID     speedPID_DRIVE_1(1., 20., 0.00, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);

volatile long encoderCount      = 0;
volatile bool zeroPointDetected = false;

bool homingDone  = false;
bool homingError = false;

volatile int16_t angle   = 0;
volatile int16_t speed   = 0;
volatile int16_t current = 0;
volatile uint8_t temp    = 0;

volatile int16_t prev_raw_angle       = 0;
volatile int32_t cumulative_raw_angle = 0;
volatile double  output_angle         = 0.0;

void onReceive(int packetSize) {
    if (CAN.packetId() != 0x202) {
        while (CAN.available())
            CAN.read();
        return;
    }

    if (packetSize < 8) {
        while (CAN.available())
            CAN.read();
        return;
    }

    uint8_t data[8] = {0};
    for (int i = 0; i < 8 && CAN.available(); i++) {
        data[i] = (uint8_t)CAN.read();
    }

    int16_t raw_angle = (int16_t)((data[0] << 8) | data[1]);
    speed             = (int16_t)((data[2] << 8) | data[3]);
    current           = (int16_t)((data[4] << 8) | data[5]);
    temp              = data[6];
}

// Z相割り込みハンドラ
void IRAM_ATTR onZPhase() {
    encoderCount      = 0;
    zeroPointDetected = true;
}

// signは、モータの配線や取り付け向きの補正
void setMotor(int8_t dirPin, int pwmCh, int sign, int pwm_signed) {
    int duty = sign * pwm_signed;
    duty     = constrain(duty, -PWM_LIMIT, PWM_LIMIT);

    if (abs(duty) < PWM_DEADBAND) {
        ledcWrite(pwmCh, 0);
        return;
    }
    digitalWrite(dirPin, (duty > 0) ? HIGH : LOW);
    ledcWrite(pwmCh, abs(duty));
}

static inline long steerDegToEncCount(double steer_deg) {
    // ステア角[deg] -> エンコーダカウント（ギア比考慮）
    return lroundf((steer_deg * STEER_GEAR_RATIO_MOTOR_TO_STEER * ENC_RESOLUTION) / 360.0);
}

void stopSteerMotor() {
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, 0);
}

bool runSteerHoming() {
    // 1) ロリコン(エンコーダ)をリセットし現在位置を0
    noInterrupts();
    zeroPointDetected = false;
    interrupts();
    encoder.clearCount();
    const long plus180  = steerDegToEncCount(+180.0); // +180°(ステア)
    const long minus180 = steerDegToEncCount(-180.0); // -180°(ステア)

    // 2) +180°回転開始
    uint32_t t0 = millis();
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, +HOMING_PWM);

    while (encoder.getCount() < plus180) {
        if (zeroPointDetected) {
            // 3) 途中でZ相検出 -> リセットして停止
            stopSteerMotor();
            encoder.clearCount();
            noInterrupts();
            zeroPointDetected = false;
            interrupts();
            Serial.println("[HOMING] Z detected on + sweep. Homing OK.");
            return true;
        }

        if (millis() - t0 > HOMING_TIMEOUT_MS) break;
        delay(2);
    }

    // +180までで未検出 -> 4) 逆回転して -180 まで（合計 -360）
    t0 = millis();
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, -HOMING_PWM);

    while (encoder.getCount() > minus180) {
        if (zeroPointDetected) {
            // 途中でZ相検出 -> リセットして停止
            stopSteerMotor();
            encoder.clearCount();
            noInterrupts();
            zeroPointDetected = false;
            interrupts();
            Serial.println("[HOMING] Z detected on - sweep. Homing OK.");
            return true;
        }
        if (millis() - t0 > HOMING_TIMEOUT_MS) break;
        delay(2);
    }

    // 5) -180まで行っても未検出 -> 回路エラー
    stopSteerMotor();
    Serial.println("[HOMING][ERROR] Z phase not detected. Check encoder/Z wiring.");
    return false;
}
unsigned long last = micros();

void setup() {
    Serial.begin(115200);

    // モーターピン初期化
    pinMode(PIN_STEER_MOTOR_DIR, OUTPUT);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
    pinMode(PIN_ENC_Z, INPUT);
    encoder.clearCount();
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_Z), onZPhase, RISING);

    ledcSetup(W1_CH, 12800, 8);
    ledcAttachPin(PIN_STEER_MOTOR_PWM, W1_CH);

    PS4.begin("e4:65:b8:7e:05:4a");

    CAN.setPins(4, 5); // RX, TX

    if (!CAN.begin(1000000)) {
        Serial.println("CAN init failed");
        while (1)
            ;
    }

    CAN.onReceive(onReceive);

    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;

    homingDone  = runSteerHoming();
    homingError = !homingDone;

    last = micros();

    Serial.println("Setup complete");
}

constexpr int32_t CONTROL_CYCLE = 5000;

static double normalizeAngleDeg(double a) {
    while (a > 180.0)
        a -= 360.0;
    while (a < -180.0)
        a += 360.0;
    return a;
}

void loop() {
    if (homingError) {
        static uint32_t last = 0;
        if (millis() - last > 1000) {
            last = millis();
            Serial.println("[HOMING ERROR] paused. power-cycle after fixing wiring.");
        }
        stopSteerMotor();
        delay(10);
        return;
    }

    if (!PS4.isConnected()) {
        stopSteerMotor();
        delay(10);
        return;
    }

    unsigned long now = micros();
    if (now - last < CONTROL_CYCLE) return;

    double dt = (now - last) * 1.e-6;
    last      = now;

    int    rx            = -PS4.RStickX();
    int    ry            = PS4.RStickY();
    double stickAngleDeg = atan2((double)ry, (double)rx) * 180.0 / M_PI;
    // スティック上を0度にするために90度シフト
    stickAngleDeg   = normalizeAngleDeg(stickAngleDeg - 90.0);
    double stickMag = hypot((double)rx, (double)ry);

    // デッドゾーン処理
    const double DEADZONE   = 15.0;
    double       moveOutput = 0.0;
    if (stickMag > DEADZONE) {
        moveOutput = (stickMag - DEADZONE) / (127.0 - DEADZONE) * MOTOR_CURRENT_LIMIT;
        moveOutput = constrain(moveOutput, 0.0, (double)MOTOR_CURRENT_LIMIT);
    }

    double currentAngleDeg = (encoder.getCount() * 360.0 / ENC_RESOLUTION) / STEER_GEAR_RATIO_MOTOR_TO_STEER;
    currentAngleDeg        = normalizeAngleDeg(currentAngleDeg);

    // スティックを戻したら、現在の目標角度を保持
    static bool   targetInitialized  = false;
    static double holdAngleTargetDeg = 0.0;
    if (!targetInitialized) {
        holdAngleTargetDeg = currentAngleDeg; // 起動直後の急な飛びを防止
        targetInitialized  = true;
    }

    if (stickMag > DEADZONE) {
        holdAngleTargetDeg = normalizeAngleDeg(stickAngleDeg);
    }
    double angleTargetDeg = holdAngleTargetDeg;

    // 90度最適化: 目標角と現在角の差が ±90° を超える場合は目標に ±180° を足して
    // ステアを最小化し、進行方向を反転する
    double angleError = normalizeAngleDeg(angleTargetDeg - currentAngleDeg);
    double driveSign  = 1.0;
    if (angleError > 90.0) {
        angleTargetDeg = normalizeAngleDeg(angleTargetDeg - 180.0);
        driveSign      = -1.0;
    } else if (angleError < -90.0) {
        angleTargetDeg = normalizeAngleDeg(angleTargetDeg + 180.0);
        driveSign      = -1.0;
    }

    double targetSpeedRPM = anglePID_STEER_1.update(angleTargetDeg, currentAngleDeg, dt);

    static long prev_enc = 0;
    long        enc_now  = encoder.getCount();
    long        delta    = enc_now - prev_enc;
    prev_enc             = enc_now;

    double motorRpm = (((double)delta / ENC_RESOLUTION) / dt * 60.0);

    double steerMotorOutput = speedPID_STEER_1.update(targetSpeedRPM, motorRpm, dt);

    // デバッグ出力（追加）: 符号・値の確認用
    Serial.printf("angleT:%.1f angle:%.1f  rpm:%.1f ,speed:%d\n", angleTargetDeg, currentAngleDeg, motorRpm, speed);

    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, 1, (int)steerMotorOutput);

    // driveSign を適用して前後反転を反映
    int16_t motor_current = speedPID_DRIVE_1.update(moveOutput * driveSign, speed, dt);

    CAN.beginPacket(0x200);
    for (int i = 0; i < 4; i++) {
        CAN.write(motor_current >> 8);
        CAN.write(motor_current & 0xFF);
    }
    CAN.endPacket();
}
