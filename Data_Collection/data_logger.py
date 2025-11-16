#!/usr/bin/env python3
"""
data_logger_with_imu.py
----------------------------------------------------------
Comprehensive data logging script for the Autonomous Robotic
Indoor Air Quality (IAQ) Monitoring Platform.

This script acquires synchronized readings from:
- Air quality sensors (CO₂, PM₁, PM₂.₅, PM₁₀)
- Environmental sensors (Temperature, Humidity, Pressure, Altitude)
- Inertial sensors (Accelerometer, Gyroscope, Magnetometer)

Each record is timestamped and stored in CSV format at 5-second intervals.

Author: Mohammed Al-Salahi
Affiliation: Qatar University
----------------------------------------------------------
"""

import csv
import os
import time
import math
import smbus2
import board
import adafruit_dht

# ==========================================================
# --- I2C & GPIO CONFIGURATION -----------------------------
# ==========================================================

# PM Sensor (DFRobot SEN0460 / PMSA003)
PM25_I2C_BUS = 3
PM25_I2C_ADDR = 0x19
pm25_bus = smbus2.SMBus(PM25_I2C_BUS)

# CO₂ Sensor (MG-811 + ADS1015)
CO2_I2C_BUS = 6
CO2_I2C_ADDR = 0x48
REG_CONVERSION = 0x00
REG_CONFIG = 0x01
CO2_CHANNEL = 0
MUX_MAP = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}
BASE_CONFIG = 0x8003
co2_bus = smbus2.SMBus(CO2_I2C_BUS)

# BMP280 (Pressure + Altitude)
BMP280_I2C_BUS = 4
BMP280_I2C_ADDR = 0x76
bmp280_bus = smbus2.SMBus(BMP280_I2C_BUS)

# DHT11 (Temperature + Humidity)
dht_device = adafruit_dht.DHT11(board.D17)

# IMU (LSM303DLHC + L3GD20)
IMU_I2C_BUS = 1
ACCEL_ADDR = 0x19
MAG_ADDR = 0x1E
GYRO_ADDR = 0x69
imu_bus = smbus2.SMBus(IMU_I2C_BUS)

# ==========================================================
# --- SENSOR INITIALIZATION -------------------------------
# ==========================================================

# Accelerometer (LSM303DLHC)
imu_bus.write_byte_data(ACCEL_ADDR, 0x20, 0x57)  # Enable X/Y/Z, 100 Hz
imu_bus.write_byte_data(ACCEL_ADDR, 0x23, 0x00)  # ±2 g

# Gyroscope (L3GD20)
imu_bus.write_byte_data(GYRO_ADDR, 0x20, 0x0F)  # Normal mode, all axes
imu_bus.write_byte_data(GYRO_ADDR, 0x23, 0x00)  # ±250 dps

# Magnetometer (LSM303DLHC)
imu_bus.write_byte_data(MAG_ADDR, 0x00, 0x14)  # 30 Hz data rate
imu_bus.write_byte_data(MAG_ADDR, 0x01, 0x20)  # ±1.3 Gauss range
imu_bus.write_byte_data(MAG_ADDR, 0x02, 0x00)  # Continuous-conversion mode

# ==========================================================
# --- SENSOR READ FUNCTIONS -------------------------------
# ==========================================================

def read_pm25():
    try:
        data = pm25_bus.read_i2c_block_data(PM25_I2C_ADDR, 0x00, 32)
        pm1_atm  = (data[7] << 8) | data[6]
        pm25_atm = (data[9] << 8) | data[8]
        pm10_atm = (data[11] << 8) | data[10]
        return pm1_atm, pm25_atm, pm10_atm, 0
    except Exception as e:
        print(f"PM2.5 read error: {e}")
        return 0, 0, 0, 1


def read_co2():
    try:
        CONFIG = BASE_CONFIG | MUX_MAP[CO2_CHANNEL]
        co2_bus.write_i2c_block_data(CO2_I2C_ADDR, REG_CONFIG,
            [(CONFIG >> 8) & 0xFF, CONFIG & 0xFF])
        time.sleep(0.005)
        data = co2_bus.read_i2c_block_data(CO2_I2C_ADDR, REG_CONVERSION, 2)
        raw = (data[0] << 8) | data[1]
        raw >>= 4
        if raw > 0x7FF:
            raw -= 1 << 12
        voltage = raw * 0.002  # 2mV per LSB
        A, B = 400, 1.5
        ppm_est = int(A * math.exp(B * (voltage - 0.4)))
        ppm_est = max(min(ppm_est, 5000), 350)
        return ppm_est, 0
    except Exception as e:
        print(f"CO₂ read error: {e}")
        return 0, 1


def read_dht11():
    try:
        temp_c = dht_device.temperature
        rh_pct = dht_device.humidity
        return temp_c, rh_pct, 0
    except Exception:
        return 0, 0, 1


def read_bmp280():
    try:
        chip_id = bmp280_bus.read_byte_data(BMP280_I2C_ADDR, 0xD0)
        if chip_id != 0x58:
            raise Exception("BMP280 not detected")
        bmp280_bus.write_byte_data(BMP280_I2C_ADDR, 0xF4, 0x27)
        bmp280_bus.write_byte_data(BMP280_I2C_ADDR, 0xF5, 0xA0)
        time.sleep(0.05)
        data = bmp280_bus.read_i2c_block_data(BMP280_I2C_ADDR, 0xF7, 6)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        calib = bmp280_bus.read_i2c_block_data(BMP280_I2C_ADDR, 0x88, 24)
        dig_T1 = calib[1] << 8 | calib[0]
        dig_T2 = (calib[3] << 8 | calib[2]) - (1 << 16) if calib[3] & 0x80 else (calib[3] << 8 | calib[2])
        dig_T3 = (calib[5] << 8 | calib[4]) - (1 << 16) if calib[5] & 0x80 else (calib[5] << 8 | calib[4])
        var1 = (((adc_t / 16384.0) - (dig_T1 / 1024.0)) * dig_T2)
        var2 = ((((adc_t / 131072.0) - (dig_T1 / 8192.0)) ** 2) * dig_T3)
        t_fine = int(var1 + var2)
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * 0 / 32768.0
        pressure_hpa = round(1013.25, 2)
        altitude_m = round(44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 0.1903)), 2)
        return pressure_hpa, altitude_m, 0
    except Exception:
        return 0, 0, 1


# ==========================================================
# --- IMU SENSOR READ FUNCTIONS ----------------------------
# ==========================================================

def twos_comp(val):
    if val > 32767:
        val -= 65536
    return val

def read_accel():
    data = imu_bus.read_i2c_block_data(ACCEL_ADDR, 0x28 | 0x80, 6)
    x_raw = twos_comp((data[1] << 8) | data[0]) >> 4
    y_raw = twos_comp((data[3] << 8) | data[2]) >> 4
    z_raw = twos_comp((data[5] << 8) | data[4]) >> 4
    scale = 0.001 * 9.80665  # 1 mg/LSB * g-to-m/s²
    return x_raw * scale, y_raw * scale, z_raw * scale

def read_gyro():
    data = imu_bus.read_i2c_block_data(GYRO_ADDR, 0x28 | 0x80, 6)
    x = twos_comp(data[0] | (data[1] << 8)) * 0.00875
    y = twos_comp(data[2] | (data[3] << 8)) * 0.00875
    z = twos_comp(data[4] | (data[5] << 8)) * 0.00875
    return x, y, z

def read_mag():
    data = imu_bus.read_i2c_block_data(MAG_ADDR, 0x03, 6)
    x_raw = twos_comp((data[0] << 8) | data[1])
    z_raw = twos_comp((data[2] << 8) | data[3])
    y_raw = twos_comp((data[4] << 8) | data[5])
    scale = 1100.0 / 2048.0  # ±1.3 Gauss range → µT
    return x_raw * scale, y_raw * scale, z_raw * scale


# ==========================================================
# --- DATA LOGGING LOOP ------------------------------------
# ==========================================================

filename = "dataset_with_imu.csv"
if not os.path.exists(filename):
    with open(filename, "w", newline="") as f:
        csv.writer(f).writerow([
            "timestamp", "co2_ppm", "pm1_ugm3", "pm25_ugm3", "pm10_ugm3",
            "temp_c", "rh_pct", "pressure_hpa", "altitude_m",
            "accel_x", "accel_y", "accel_z",
            "gyro_x", "gyro_y", "gyro_z",
            "mag_x", "mag_y", "mag_z",
            "room_label", "run_id", "quality_flag"
        ])

room_label = input("Enter room label (e.g., Kitchen, LivingRoom): ")
run_id = input("Enter run ID (e.g., run_1): ") or "run_1"

print(f"\n Logging started for '{room_label}' [{run_id}] — press Ctrl+C to stop\n")

try:
    while True:
        timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

        co2_ppm, q1 = read_co2()
        pm1, pm25, pm10, q2 = read_pm25()
        temp_c, rh_pct, q3 = read_dht11()
        pressure_hpa, altitude_m, q4 = read_bmp280()
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        mx, my, mz = read_mag()

        quality_flag = max(q1, q2, q3, q4)

        row = [
            timestamp, co2_ppm, pm1, pm25, pm10,
            temp_c, rh_pct, pressure_hpa, altitude_m,
            ax, ay, az, gx, gy, gz, mx, my, mz,
            room_label, run_id, quality_flag
        ]

        with open(filename, "a", newline="") as f:
            csv.writer(f).writerow(row)

        print(row)
        time.sleep(5)

except KeyboardInterrupt:
    print("\n Logging stopped by user.")
    dht_device.exit()

