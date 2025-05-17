//
// Created by gary on 5/12/25.
//


#include "imu_spy.h"

RTC_DATA_ATTR uint8_t oldRegs[REG_COUNT]; // Save register values over deep sleep to diff changes size shutdown.
static const char *regNames[REG_COUNT];
static uint8_t fifoBuffer[1024] = {0};
static int doDumpFifoBuffer = 0;
static int sampleRates[] ={4, 5, 10, 20, 50, 100, 125, 200, 250, 500,1000};
static int sampleRate = SAMPLE_RATE;
static uint16_t fifo_count = 0;

void setSampleRate(int sample_rate);

void setNames() {
    memset(regNames, 0, sizeof(regNames));

    regNames[0x00] = "RW   XG_OFFS_TC";
    regNames[0x01] = "RW   YG_OFFS_TC";
    regNames[0x02] = "RW   ZG_OFFS_TC";
    regNames[0x03] = "RW   X_FINE_GAIN";
    regNames[0x04] = "RW   Y_FINE_GAIN";
    regNames[0x05] = "RW   Z_FINE_GAIN";
    regNames[0x06] = "RW   XA_OFFS_H";
    regNames[0x07] = "RW   XA_OFFS_L_TC";
    regNames[0x08] = "RW   YA_OFFS_H";
    regNames[0x09] = "RW   YA_OFFS_L_TC";
    regNames[0x0A] = "RW   ZA_OFFS_H";
    regNames[0x0B] = "RW   ZA_OFFS_L_TC";
    regNames[0x0D] = "RW   SELF_TEST_X";
    regNames[0x0E] = "RW   SELF_TEST_Y";
    regNames[0x0F] = "RW   SELF_TEST_Z";
    regNames[0x10] = "RW   SELF_TEST_A";
    regNames[0x13] = "RW   XG_OFFS_USRH";
    regNames[0x14] = "RW   XG_OFFS_USRL";
    regNames[0x15] = "RW   YG_OFFS_USRH";
    regNames[0x16] = "RW   YG_OFFS_USRL";
    regNames[0x17] = "RW   ZG_OFFS_USRH";
    regNames[0x18] = "RW   ZG_OFFS_USRL";
    regNames[0x19] = "RW   SMPLRT_DIV";
    regNames[0x1A] = "RW   CONFIG";
    regNames[0x1B] = "RW   GYRO_CONFIG";
    regNames[0x1C] = "RW   ACCEL_CONFIG";
    regNames[0x1D] = "RW   FF_THR";
    regNames[0x1E] = "RW   FF_DUR";
    regNames[0x1F] = "RW   MOT_THR";
    regNames[0x20] = "RW   MOT_DUR";
    regNames[0x21] = "RW   ZRMOT_THR";
    regNames[0x22] = "RW   ZRMOT_DUR";
    regNames[0x23] = "RW   FIFO_EN";
    regNames[0x24] = "RW   I2C_MST_CTRL";
    regNames[0x25] = "RW   I2C_SLV0_ADDR";
    regNames[0x26] = "RW   I2C_SLV0_REG";
    regNames[0x27] = "RW   I2C_SLV0_CTRL";
    regNames[0x28] = "RW   I2C_SLV1_ADDR";
    regNames[0x29] = "RW   I2C_SLV1_REG";
    regNames[0x2A] = "RW   I2C_SLV1_CTRL";
    regNames[0x2B] = "RW   I2C_SLV2_ADDR";
    regNames[0x2C] = "RW   I2C_SLV2_REG";
    regNames[0x2D] = "RW   I2C_SLV2_CTRL";
    regNames[0x2E] = "RW   I2C_SLV3_ADDR";
    regNames[0x2F] = "RW   I2C_SLV3_REG";
    regNames[0x30] = "RW   I2C_SLV3_CTRL";
    regNames[0x31] = "RW   I2C_SLV4_ADDR";
    regNames[0x32] = "RW   I2C_SLV4_REG";
    regNames[0x33] = "RW   I2C_SLV4_DO";
    regNames[0x34] = "RW   I2C_SLV4_CTRL";
    regNames[0x35] = "R    I2C_SLV4_DI";
    regNames[0x36] = "R    I2C_MST_STATUS";
    regNames[0x37] = "RW   INT_PIN_CFG";
    regNames[0x38] = "RW   INT_ENABLE";
    regNames[0x39] = "R    DMP_INT_STATUS";
    regNames[0x3A] = "R    INT_STATUS";
    regNames[0x3B] = "R    ACCEL_XOUT_H";
    regNames[0x3C] = "R    ACCEL_XOUT_L";
    regNames[0x3D] = "R    ACCEL_YOUT_H";
    regNames[0x3E] = "R    ACCEL_YOUT_L";
    regNames[0x3F] = "R    ACCEL_ZOUT_H";
    regNames[0x40] = "R    ACCEL_ZOUT_L";
    regNames[0x41] = "R    TEMP_OUT_H";
    regNames[0x42] = "R    TEMP_OUT_L";
    regNames[0x43] = "R    GYRO_XOUT_H";
    regNames[0x44] = "R    GYRO_XOUT_L";
    regNames[0x45] = "R    GYRO_YOUT_H";
    regNames[0x46] = "R    GYRO_YOUT_L";
    regNames[0x47] = "R    GYRO_ZOUT_H";
    regNames[0x48] = "R    GYRO_ZOUT_L";
    regNames[0x49] = "R    EXT_SENS_DATA_00";
    regNames[0x4A] = "R    EXT_SENS_DATA_01";
    regNames[0x4B] = "R    EXT_SENS_DATA_02";
    regNames[0x4C] = "R    EXT_SENS_DATA_03";
    regNames[0x4D] = "R    EXT_SENS_DATA_04";
    regNames[0x4E] = "R    EXT_SENS_DATA_05";
    regNames[0x4F] = "R    EXT_SENS_DATA_06";
    regNames[0x50] = "R    EXT_SENS_DATA_07";
    regNames[0x51] = "R    EXT_SENS_DATA_08";
    regNames[0x52] = "R    EXT_SENS_DATA_09";
    regNames[0x53] = "R    EXT_SENS_DATA_10";
    regNames[0x54] = "R    EXT_SENS_DATA_11";
    regNames[0x55] = "R    EXT_SENS_DATA_12";
    regNames[0x56] = "R    EXT_SENS_DATA_13";
    regNames[0x57] = "R    EXT_SENS_DATA_14";
    regNames[0x58] = "R    EXT_SENS_DATA_15";
    regNames[0x59] = "R    EXT_SENS_DATA_16";
    regNames[0x5A] = "R    EXT_SENS_DATA_17";
    regNames[0x5B] = "R    EXT_SENS_DATA_18";
    regNames[0x5C] = "R    EXT_SENS_DATA_19";
    regNames[0x5D] = "R    EXT_SENS_DATA_20";
    regNames[0x5E] = "R    EXT_SENS_DATA_21";
    regNames[0x5F] = "R    EXT_SENS_DATA_22";
    regNames[0x60] = "R    EXT_SENS_DATA_23";
    regNames[0x61] = "R    MOT_DETECT_STATS";
    regNames[0x63] = "RW   I2C_SLV0_DO";
    regNames[0x64] = "RW   I2C_SLV1_DO";
    regNames[0x65] = "RW   I2C_SLV2_DO";
    regNames[0x66] = "RW   I2C_SLV3_DO";
    regNames[0x67] = "RW   I2C_MST_DLY_CTRL";
    regNames[0x68] = "RW   SIGNAL_PATH_RST";
    regNames[0x69] = "RW   MOT_DETECT_CTRL";
    regNames[0x6A] = "RW   USER_CTRL";
    regNames[0x6B] = "RW   PWR_MGMT_1";
    regNames[0x6C] = "RW   PWR_MGMT_2";
    regNames[0x6D] = "RW   BANK_SEL";
    regNames[0x6E] = "RW   MEM_START_ADDR";
    regNames[0x72] = "RW   FIFO_COUNTH";
    regNames[0x73] = "RW   FIFO_COUNTL";
    regNames[0x74] = "RW   FIFO_R_W";
    regNames[0x75] = "R    WHO_AM_I";

    for (int i = 0; i < REG_COUNT; i++) {
        if (regNames[i] == 0) {
            regNames[i] = " -   - ";
            Serial.printf("WRITING BLANK TO %d", i);
        }
    }
}


void readRegisters(uint8_t *data, uint8_t start, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(start);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t) MPU6050_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.available() ? Wire.read() : 0xFF;
    }
}

uint16_t readTwoBytes(uint8_t address) {
    uint16_t value = 0;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(address);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t) MPU6050_ADDR, (uint8_t) 2);
    if (Wire.available() == 2) {
        value = (Wire.read() << 8) | Wire.read();
    }
    return value;
}

void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    int result = Wire.endTransmission();
    if (result != 0) {
        Serial.printf("I2C write error: %d on reg %02X\n", result, reg);
    }
}



uint16_t getFifoCount() {
    fifo_count = readTwoBytes(0x72); // Read the FIFO count
    return fifo_count;
}

struct OHLC {
    int16_t first, high, low, last;

    void update(int16_t v, bool firstly, bool lastly) {
        if (firstly) { first = high = low = last = v; }
        if (lastly) last = v;
        else {
            if (v > high) high = v;
            if (v < low) low = v;
        }
    }
};

const char *labels[7] = {"AccX", "AccY", "AccZ", "Temp", "GyroX", "GyroY", "GyroZ"};
const size_t PKT = 14, WIDTH = 50;


void printOHLCChart(const uint8_t *buf, size_t len, int row_num, int column) {
    OHLC stats[7] = {};
    size_t frames = len / PKT;
    if (frames == 0) return;
    if (len < PKT) return; // Prevent out-of-bounds

    for (size_t i = 0; i < frames; ++i) {
        int endOffset = i * PKT + PKT;

        if (endOffset > len) {
            Serial.printf(TEXT_RED " len=%d, offset=%d", len, endOffset);
            while (true) {
                delay(100);
            }
        }

        bool first = i == 0;
        bool last = i == frames - 1;
        const uint8_t *frame = buf + (i * PKT);

        for (int ch = 0; ch < 7; ++ch) {
            int16_t v = int16_t(frame[2 * ch] << 8 | frame[2 * ch + 1]);
            stats[ch].update(v, first, last);
        }
    }

    for (int ch = 0; ch < 7; ++ch) {
        auto &o = stats[ch];

        int pf = ((o.first + 32768) * WIDTH) / 65536;
        int pe = ((o.last + 32768) * WIDTH) / 65536;
        int ph = ((o.high + 32768) * WIDTH) / 65536;
        int pl = ((o.low + 32768) * WIDTH) / 65536;

        Serial.printf("\033[%d;%dH\033[0m", row_num + ch, column);

        Serial.printf(TEXT_RESET
                      "%-5s | L=%6d O=%6d C=%6d H=%6d | ",
                      labels[ch], o.low, o.first, o.last, o.high
        );

        for (int x = 0; x <= WIDTH; ++x) {
            char c =
                x == pf && x == pe ? '*'
              : x == pf ? 'F'
              : x == pe ? 'L'
              : x < pl  ? ' '
              : x > ph  ? ' '
              : '-';
            Serial.print(c);
        }
        Serial.println("|" ERASE_EOL);
    }
}


int dumpFifoBuffer(int row_num) {
    int n = 0;
    int packets_per_buffer = 1024 / 14;

    // How long to fill a 1024 byte buffer with 14 byte packets at a sample rate of (n)Hz?
    double ms_per_packet = 1000.0 / SAMPLE_RATE;
    int ms_per_buffer = ms_per_packet * packets_per_buffer;
#define FIFO_GRACE_TIME_MS 200

    // Check if FIFO buffer is full, or about to overflow
    if (getFifoCount() >= 1000) {
        Serial.printf("FIFO buffer overflow %d. Resetting...", fifo_count);
        writeRegister(0x6A, 0x04); // Reset FIFO
        writeRegister(0x6A, 0x40); // Re-enable FIFO

        // read imu int status
        uint8_t int_status = readTwoBytes(0x3A);
        Serial.printf("INT STATUS: %02X" TEXT_RESET ERASE_EOL "\n", int_status);
        // Wait a bit then process the buffer
        //delay(ms_per_buffer - FIFO_GRACE_TIME_MS );

        getFifoCount();
    }

    // Determine the delay time to wait according to SAMPLE_RATE, current FiFo buffer size(bytes) before FIFO exceeds 400 bytes
    double time_used_in_buffer = fifo_count / 14 * ms_per_packet;
    int packets_in_buffer = fifo_count / 14;
    int packets_left = packets_per_buffer - packets_in_buffer - FIFO_GRACE_TIME_MS / ms_per_packet;

    double fifo_time_remaining = max(0.0, ms_per_buffer - time_used_in_buffer - FIFO_GRACE_TIME_MS);
    double expected_fifoCount = (time_used_in_buffer + fifo_time_remaining) * 14 / ms_per_packet;

    Serial.printf(TEXT_RESET "FIFO buffer: %d bytes, (%d/%d) used/remaining packets, %d ms used, %d ms left, buffer capacity ms = %d ",
                  fifo_count, packets_in_buffer, packets_left , (int) time_used_in_buffer, (int) fifo_time_remaining,  ms_per_buffer);

    if (fifo_time_remaining > 200) {
//        return 0;
    }

    //delay((int)timeRemaining);

    Serial.printf("Buffer size: exp: %0.2f, act:%d --" TEXT_RESET ERASE_EOL, expected_fifoCount, getFifoCount());
    int bytesRemaining = fifo_count;

    Serial.printf("\033[%d;1H\033[0m", row_num);

    // Unload FIFO packets if buffer size < 1024
    while (bytesRemaining >= 14 && (n + 14) <= 1024) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x74); // FIFO_R_W register
        Wire.endTransmission(false);
        Wire.requestFrom((uint16_t) MPU6050_ADDR, (uint8_t) 14);

        for (int i = 0; i < 14; i++) {
            if (n >= 1024) break; // Prevent overflow
            fifoBuffer[n] = Wire.available() ? Wire.read() : 0xFF;
            bytesRemaining--;
            n++;
        }
    }

    if (doDumpFifoBuffer) {
        int column = 0;

        for (int i = 0; i < fifo_count; i++) {
            if (i % 14 == 0 && i > 0) {
                column++;
                if (column == 8) {
                    Serial.print(TEXT_RESET ERASE_EOL);
                    column = 0;
                    row_num++;
                    Serial.printf("\033[%d;1H\033[0m", row_num);
                } else {
                    Serial.printf(" - ");
                }
            }
            Serial.printf("%02X", fifoBuffer[i]);
            if (i & 1) {
                Serial.print(" ");
            }
        }
    }
    //Serial.println(TEXT_RESET ERASE_EOL);
    return n;
}

uint32_t next_print_time = 0;

void printRegisters() {
    uint8_t regs[REG_COUNT];
    readRegisters(regs, 0x00, REG_COUNT);

    if (millis() < next_print_time) {
        return;
    }

    next_print_time = millis() + 200;

    for (int row = 0; row < ROWS_PER_COL; row++) {
        Serial.printf("\033[%d;1H\033[0m", row + 1);

        for (int col = 0; col < (REG_COUNT + ROWS_PER_COL - 1) / ROWS_PER_COL; col++) {
            int idx = col * ROWS_PER_COL + row;

            if (idx >= REG_COUNT) break;

            char newValueText[6] = {0}; // Initialize with zeros

            if (regs[idx] != oldRegs[idx]) {
                Serial.print("\033[7m" TEXT_YELLOW);
                snprintf(newValueText, 6, " %02x ", regs[idx]);
            } else {
                Serial.print(TEXT_RESET);
                snprintf(newValueText, 6, "    "); // Keep it blank if no change
            }

            Serial.printf("| %02x:", idx);
            const char *regReadWrite = regNames[idx];
            const char *registerName = regNames[idx] + 4;

            regReadWrite = regReadWrite == nullptr ? "???" : regReadWrite;
            registerName = registerName == nullptr ? "???" : registerName;

            Serial.printf("%18s ", registerName);
            Serial.printf("%.2s ", regReadWrite);
            Serial.printf("%02x ", oldRegs[idx]);
            Serial.printf("%s ", newValueText);

            for (int b = 7; b >= 0; b--) {
                Serial.printf("%c", (regs[idx] & (1 << b)) ? '1' : '0');
            }

            Serial.print(" ");
        }
        Serial.print(TEXT_RESET);
    }

    Serial.print(ERASE_EOL);
    Serial.flush(true);

    int pin34 = digitalRead(GPIO_NUM_34);

    Serial.printf("\033[%d;1H\033[0m", ROWS_PER_COL + 1);
    Serial.printf("PIN 34: %d  ", pin34);

    int n = dumpFifoBuffer(ROWS_PER_COL + 2);
    memcpy(oldRegs, regs, REG_COUNT);

    /*
    for (int i = 0; i < 5; i++) {
        Serial.printf("\033[%d;1H\033[0m" TEXT_RESET ERASE_EOL, ROWS_PER_COL + 8 + i);
    }
    */

    printOHLCChart(fifoBuffer, n, ROWS_PER_COL + 8, 1); // Print the chart starting from row 16

    Serial.flush();
}


void enterDeepSleep() {
    Serial.println("Entering deep sleep for 10 seconds max, or wake on GPIO34 HIGH - IMU INT");
    uint8_t intStatus = readTwoBytes(0x3A);

    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1);
    esp_deep_sleep(10 * 1000000); // Sleep for 10 seconds
}

void setup() {
    delay(1000);

    Wire.end(); // Deinitialize the I²C bus
    Wire.setBufferSize(1024);
    Wire.begin(GPIO_NUM_21, GPIO_NUM_22, 400000); // Reinitialize the I²C bus

    Serial.setTxBufferSize(16384);
    Serial.begin(SERIAL_BAUD_RATE);

    pinMode(GPIO_NUM_34, INPUT_PULLUP);
    setNames();

    Serial.print("\033[0;1H"); // lear screen and move cursor to top left
    //Serial.print("\033[2J\033[0;1H"); // Clear screen and move cursor to top left
    Serial.flush();

    delay(10); // Allow time for serial / wire background tasks to start

    printRegisters();
}

void init() {
    Serial.println("Initialising MPU6050 Registers:"); // MPU6050 register definitions

    writeRegister(PWR_MGMT_1, 0x01); // Wake up device (clear sleep bit)

    delay(10);

    writeRegister(MOT_THR, 0x01); // Set motion threshold to 40 mg (20 * 2 mg/LSB)
    writeRegister(MOT_DUR, 0x01); // Set motion duration to 1 ms
    writeRegister(MOT_DETECT_CTRL, 0x15); // Configure motion detection: logic + filters
    writeRegister(INT_PIN_CFG, 0x20); // Latch interrupt until cleared by reading INT_STATUS
    writeRegister(INT_ENABLE, 0x40); // Enable motion interrupt

    Serial.println("MPU6050 motion interrupt configured.");

    writeRegister(RA_GYRO_CONFIG, 0b00010000); // Configure gyro, DLPF and accel
    writeRegister(RA_ACCEL_CONFIG, 0b00000001); // Configure gyro, DLPF and accel
    writeRegister(RA_CONFIG, 0b00000001); // Configure DLPF (Digital Low Pass Filter)
    writeRegister(RA_FIFO_EN, 0b11111000); // Enable specific FIFO features: Temp, XG, YG, ZG, and ACCEL

    writeRegister(RA_USER_CTRL, 0x0C); // Reset FIFO and DMP
    writeRegister(RA_USER_CTRL, 0x40); // Set USER_CTRL to enable FIFO

    setSampleRate(sampleRate);
}

void setSampleRate(int sample_rate) {

    switch (sample_rate) {
        case 1000:
            writeRegister(RA_SMPLRT_DIV, 0);   // 1000 Hz
            break;
        case 500:
            writeRegister(RA_SMPLRT_DIV,1);
            break;
        case 250:
            writeRegister(RA_SMPLRT_DIV,3);
            break;
        case 200:
            writeRegister(RA_SMPLRT_DIV,4);
            break;
        case 125:
            writeRegister(RA_SMPLRT_DIV,7);
            break;
        case 100:
            writeRegister(RA_SMPLRT_DIV,9);
            break;
        case 50:
            writeRegister(RA_SMPLRT_DIV,19);
            break;
        case 20:
            writeRegister(RA_SMPLRT_DIV,49);
            break;
        case 10:
            writeRegister(RA_SMPLRT_DIV,99);
            break;
        case 5:
            writeRegister(RA_SMPLRT_DIV,199);
            break;
        case 4:
            writeRegister(RA_SMPLRT_DIV,249);
            break;
        default:
            Serial.printf("Invalid sampling rate %d.", sample_rate);
            abort();
    }
    sampleRate = sample_rate;
    Serial.printf("Set sample rate to %d Hz.\n", sampleRate);
}

void handleCommand() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if (cmd == "") {
            printRegisters();
        }

        if (cmd.startsWith("R ")) {
            int reg = cmd.substring(2).toInt();
            if (reg < REG_COUNT) {
                uint8_t value = 0;
                readRegisters(&value, reg, 1);
                Serial.printf("Reg %02X: %02X\r\n", reg, value);
            } else {
                Serial.println("Invalid register number");
            }
        }

        if (cmd == "I") {
            init();
            printRegisters();
        }

        if (cmd == "F") {
            doDumpFifoBuffer = 1 - doDumpFifoBuffer;
            Serial.printf("FIFO dump is now [%s]\r\n", doDumpFifoBuffer ? "enabled" : "disabled");
        }

        if (cmd.startsWith("X ")) {
            int n = cmd.substring(2).toInt();
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    printRegisters();
                    if (Serial.available()) {
                        Serial.println("INTERRUPTED");
                        break;
                    }
                }
            } else {
                Serial.println("Invalid X command");
            }
            return;
        }

        if (cmd.startsWith("S")) {
            int new_sample_rate = cmd.substring(2).toInt();
            int sample_rate_index = -1;
            for (int i = 0; i < sizeof(sampleRates) / sizeof(sampleRates[0]); i++) {
                if (sampleRates[i] == new_sample_rate) {
                    setSampleRate(new_sample_rate);
                    sample_rate_index = i;
                    break;;
                }
            }
            if (sample_rate_index == -1) {
                Serial.println("Invalid sample rate");
            }
            return;
        }

        if (cmd.startsWith("E ")) {
            int reg, val;
            if (sscanf(cmd.c_str(), "E %x %x", &reg, &val) == 2 && reg < REG_COUNT && val < 0x100) {
                writeRegister((uint8_t) reg, (uint8_t) val);
                Serial.printf("Wrote %02X to reg %02X\r\n", val, reg);
            } else {
                Serial.println("Invalid E command");
            }
            return;
        }
        if (cmd == "DS") {
            enterDeepSleep();
        }

        if (cmd == "?") {
            Serial.println("Available commands:");
            Serial.println("N <n>         - Read and print registers <n> times");
            Serial.println("E <reg> <val> - Write <val> to register <reg>");
            Serial.println("F             - Toggle dump of FIFO buffer");
            Serial.println("DS            - Enter deep sleep mode");
            Serial.println("S <rate>      - Set FIFO Sample Rate: 5, 10, 20, 50, 100, 125, 200, 250, 500, 1000");
            Serial.println("<enter>       - Read and print all registers");
            Serial.println("I             - Init MPU6050");
            Serial.println("?             - Show this help message");
        } else {
            Serial.println("Unknown command");
        }

        Serial.println("Enter command:");
    }
}

void loop() {
    handleCommand();
}
