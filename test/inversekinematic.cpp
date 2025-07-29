#include <Arduino.h>
#include <math.h>


// Geometry constants - giữ nguyên từ code gốc
const float R = 35.0;
const float r = 20.0;
const float h = 14.6;
const float Ln = sqrt((R - r) * (R - r) + h * h);

// Joystick pins và thông số
const int joystickX = A6;
const int joystickY = A7;
const int centerValue = 512;
const int minValue = 0;
const int maxValue = 1023;
const int DEAD_ZONE = 100;
const int SAMPLES = 10;
const int SAMPLE_DELAY = 1;

// Giới hạn góc
const float YAW_LIMIT = 10;
const float PITCH_LIMIT = 10;

// Function để đọc giá trị analog có lọc nhiễu
int readAnalogFiltered(int pin) {
    long sum = 0;
    for(int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);
        delay(SAMPLE_DELAY);
    }
    return sum / SAMPLES;
}

// Function để xử lý dead zone
int applyDeadZone(int value, int center, int threshold) {
    if(abs(value - center) < threshold) {
        return center;
    }
    return value;
}

void calculateRotationMatrix(float alpha, float beta, float T[3][3]) {
    T[0][0] = cos(beta);
    T[0][1] = sin(alpha) * sin(beta);
    T[0][2] = cos(alpha) * sin(beta);
    T[1][0] = 0;
    T[1][1] = cos(alpha);
    T[1][2] = -sin(alpha);
    T[2][0] = -sin(beta);
    T[2][1] = sin(alpha) * cos(beta);
    T[2][2] = cos(alpha) * cos(beta);
}

float calculateCylinderLength(int i, float alpha, float beta) {
    float Pb[3][3] = {
        {0, sqrt(3) * R / 2, -sqrt(3) * R / 2},
        {R, -R / 2, -R / 2},
        {0, 0, 0}
    };
    float Pt[3][3] = {
        {0, sqrt(3) * r / 2, -sqrt(3) * r / 2},
        {r, -r / 2, -r / 2},
        {0, 0, 0}
    };

    float T[3][3];
    calculateRotationMatrix(alpha, beta, T);

    float Bt[3];
    for (int j = 0; j < 3; j++) {
        Bt[j] = T[j][0] * Pt[0][i] + T[j][1] * Pt[1][i] + T[j][2] * Pt[2][i];
    }

    float Po[3] = {0, 0, h};
    float deltaX = (Bt[0] + Po[0]) - Pb[0][i];
    float deltaY = (Bt[1] + Po[1]) - Pb[1][i];
    float deltaZ = (Bt[2] + Po[2]) - Pb[2][i];

    float L = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    return round(((L - Ln) * 10)+50);
}

void setup() {
    Serial.begin(115200);  // Tăng tốc độ giao tiếp Serial
    pinMode(joystickX, INPUT);
    pinMode(joystickY, INPUT);

}

void loop() {
    // Đọc và xử lý giá trị joystick
    int xValue = readAnalogFiltered(joystickX);
    int yValue = readAnalogFiltered(joystickY);
    
    // Áp dụng dead zone
    xValue = applyDeadZone(xValue, centerValue, DEAD_ZONE);
    yValue = applyDeadZone(yValue, centerValue, DEAD_ZONE);
    
    // Tính toán góc đã chuẩn hóa
    float xNormalized = ((float)(xValue - centerValue) / (maxValue - centerValue));
    float yNormalized = ((float)(yValue - centerValue) / (maxValue - centerValue));
    
    // Tính và giới hạn góc
    float yaw = constrain(xNormalized * YAW_LIMIT, 0, YAW_LIMIT);
    float pitch = constrain(yNormalized * PITCH_LIMIT, 0, PITCH_LIMIT);

    // Tính độ dài của các xi lanh
    float cylinder1 = calculateCylinderLength(0, radians(pitch), radians(yaw));
    float cylinder2 = calculateCylinderLength(1, radians(pitch), radians(yaw));
    float cylinder3 = calculateCylinderLength(2, radians(pitch), radians(yaw));

    // In kết quả theo định dạng dễ đọc
    Serial.print(xValue);
    Serial.print(",");
    Serial.print(yValue);
    Serial.print(" | ");
    Serial.print(yaw, 1);
    Serial.print(",");
    Serial.print(pitch, 1);
    Serial.print(" | ");
    Serial.print(cylinder1, 2);
    Serial.print(",");
    Serial.print(cylinder2, 2);
    Serial.print(",");
    Serial.println(cylinder3, 2);

    delay(100);  // Delay để không in quá nhanh
}