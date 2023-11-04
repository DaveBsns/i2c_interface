#include <stdio.h>
#include <util/delay.h>
#include <i2cmaster.h>

#define BMP180_I2C_ADDRESS 0x77

int main() {
    uint8_t data[22];  // Data buffer to store calibration and measurement data
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2;
    int32_t MB, MC, MD;
    int32_t B5;
    int32_t UT, UP;
    double temperature, pressure;
    
    // Initialize I2C communication
    i2c_init();

    // Read calibration data from BMP180
    i2c_start_wait((BMP180_I2C_ADDRESS << 1) | I2C_WRITE);
    i2c_write(0xAA); // Start reading calibration data
    i2c_rep_start((BMP180_I2C_ADDRESS << 1) | I2C_READ);
    for (int i = 0; i < 22; i++) {
        data[i] = i2c_readAck();
    }
    i2c_stop();

    // Parse calibration data
    AC1 = (int16_t)((data[0] << 8) | data[1]);
    AC2 = (int16_t)((data[2] << 8) | data[3]);
    AC3 = (int16_t)((data[4] << 8) | data[5]);
    AC4 = (uint16_t)((data[6] << 8) | data[7]);
    AC5 = (uint16_t)((data[8] << 8) | data[9]);
    AC6 = (uint16_t)((data[10] << 8) | data[11]);
    B1 = (int16_t)((data[12] << 8) | data[13]);
    B2 = (int16_t)((data[14] << 8) | data[15]);
    MB = (int32_t)((data[16] << 8) | data[17]);
    MC = (int32_t)((data[18] << 8) | data[19]);
    MD = (int32_t)((data[20] << 8) | data[21]);

    // Read temperature
    i2c_start_wait((BMP180_I2C_ADDRESS << 1) | I2C_WRITE);
    i2c_write(0xF4); // Control register for temperature measurement
    i2c_write(0x2E); // Start temperature measurement
    i2c_rep_start((BMP180_I2C_ADDRESS << 1) | I2C_READ);
    UT = ((int32_t)i2c_readAck() << 8) | i2c_readNak();
    i2c_stop();

    // Read pressure
    i2c_start_wait((BMP180_I2C_ADDRESS << 1) | I2C_WRITE);
    i2c_write(0xF4); // Control register for pressure measurement
    i2c_write(0x34 + (0x00 << 6)); // Start pressure measurement (oversampling = 0)
    i2c_stop();

    _delay_ms(5); // Wait for the measurement to complete

    i2c_start_wait((BMP180_I2C_ADDRESS << 1) | I2C_WRITE);
    i2c_write(0xF6); // Read pressure data
    i2c_rep_start((BMP180_I2C_ADDRESS << 1) | I2C_READ);
    UP = ((int32_t)i2c_readAck() << 16) | ((int32_t)i2c_readAck() << 8) | i2c_readNak();
    i2c_stop();

    // Calculate temperature
    int32_t X1 = (UT - AC6) * AC5 >> 15;
    int32_t X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    temperature = (B5 + 8) >> 4;
    temperature /= 10.0;

    // Calculate pressure
    int32_t B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6 >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((int32_t)(AC1 * 4 + X3) << 0) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * (B6 * B6 >> 12)) >> 16;
    int32_t X4 = (X1 + X2) + 2;
    int32_t B4 = (AC4 * (uint32_t)(X4 + 32768)) >> 15;
    int32_t B7 = ((UP - B3) * (50000 >> 0));
    if (B7 < 0x80000000) {
        pressure = (B7 << 1) / B4;
    } else {
        pressure = (B7 / B4) << 1;
    }
    int32_t X5 = (pressure - 12000) * (X3 >> 11);
    X1 = (X1 >> 3) * (X1 >> 3);
    X2 = (X2 >> 13) * (X2 >> 13);
    X3 = (X3 >> 2) * X3;
    X4 = (X4 >> 3) * (X4 >> 3);
    X5 = (X5 >> 11) * (X5 >> 11);
    int32_t Y1 = X1 + X2 + X3 + X4 + X5;
    Y1 = Y1 >> 2;
    pressure = B4 * (Y1 >> 2);
    pressure /= 100.0;
    for(int i = 0; i<20; i++){
        printf("Temperature: %.2f °C\n", temperature);
        printf("Pressure: %.2f hPa\n", pressure);
    }
    

    return 0;
}