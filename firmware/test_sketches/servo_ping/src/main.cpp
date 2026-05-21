#include <Arduino.h>

#define SERVO_TX   17
#define SERVO_RX   16
#define SERVO_BAUD 1000000UL

// Sends a PING packet to servo ID and returns true if a valid response arrives.
// SCS protocol: 0xFF 0xFF [ID] [LEN=2] [0x01=PING] [CHECKSUM]
// Response:     0xFF 0xFF [ID] [2]     [ERR]       [CHECKSUM]
static bool ping_servo(uint8_t id) {
    while (Serial2.available()) Serial2.read();  // flush RX

    uint8_t pkt[6];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 2;                                    // length
    pkt[4] = 0x01;                                 // PING instruction
    pkt[5] = ~(id + 2 + 0x01) & 0xFF;             // checksum

    Serial2.write(pkt, 6);
    Serial2.flush();  // wait for TX to drain before listening

    // Drain any echo bytes that appear on the half-duplex bus
    delayMicroseconds(300);  // 6 bytes @ 1Mbps = 60µs; 300µs gives margin
    while (Serial2.available()) Serial2.read();

    uint8_t resp[6];
    uint8_t idx = 0;
    uint32_t deadline = micros() + 10000;  // 10ms timeout
    while (micros() < deadline && idx < 6) {
        if (Serial2.available()) resp[idx++] = Serial2.read();
    }

    if (idx < 6) return false;
    if (resp[0] != 0xFF || resp[1] != 0xFF) return false;
    if (resp[2] != id) return false;
    return true;
}

static void scan_all() {
    const uint8_t ids[]   = {0x01, 0x02, 0x03, 0x04, 0x05};
    const char*   names[] = {"J0  Base     ", "J1a Shoulder ",
                              "J1b Shoulder ", "J2  Elbow    ", "J3  Gripper  "};
    int found = 0;
    for (int i = 0; i < 5; i++) {
        bool ok = ping_servo(ids[i]);
        Serial.printf("  ID 0x%02X (%s): %s\n", ids[i], names[i], ok ? "FOUND" : "missing");
        if (ok) found++;
        delay(50);
    }
    Serial.printf("\n  %d/5 servos found — %s\n\n", found, found == 5 ? "PASS" : "FAIL (check wiring)");
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
    delay(200);
    Serial.println("\n=== Servo Ping Test ===");
    scan_all();
}

void loop() {
    delay(3000);
    Serial.println("--- re-scan ---");
    scan_all();
}
