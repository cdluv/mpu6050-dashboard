//
// Created by gary on 5/12/25.
//

#ifndef IMU_SPY_H
#define IMU_SPY_H



#include <Arduino.h>
#include "imu_spy.h"

#include <Wire.h>
#include <esp_sleep.h>

#define TEXT_BLACK   "\033[30m"
#define TEXT_RED     "\033[31m"
#define TEXT_GREEN   "\033[32m"
#define TEXT_YELLOW  "\033[33m"
#define TEXT_BLUE    "\033[34m"
#define TEXT_MAGENTA "\033[35m"
#define TEXT_CYAN    "\033[36m"
#define TEXT_WHITE   "\033[37m"

#define BG_BLACK     "\033[40m"
#define BG_RED       "\033[41m"
#define BG_GREEN     "\033[42m"
#define BG_YELLOW    "\033[43m"
#define BG_BLUE      "\033[44m"
#define BG_MAGENTA   "\033[45m"
#define BG_CYAN      "\033[46m"
#define BG_WHITE     "\033[47m"
#define TEXT_RESET   "\033[0m"
#define ERASE_EOL    "\033[K"

#define MPU6050_ADDR 0x68
#define REG_COUNT 0x7F

#define SAMPLE_RATE 4

#define ROWS_PER_COL 32

#define PWR_MGMT_1         0x6B
#define INT_ENABLE         0x38
#define INT_PIN_CFG        0x37
#define MOT_THR            0x1F
#define MOT_DUR            0x20
#define MOT_DETECT_CTRL    0x69

#define RA_GYRO_CONFIG       0x1B
#define RA_ACCEL_CONFIG      0x1C
#define RA_USER_CTRL         0x6A
#define RA_CONFIG            0x1A
#define RA_FIFO_EN           0x23
#define RA_SMPLRT_DIV        0x19

#endif //IMU_SPY_H
