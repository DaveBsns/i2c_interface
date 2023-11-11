#include <stdio.h>
#include <util/delay.h>
#include <lib/hd44780pcf8574.h>
#include <compat/twi.h>
#include <stdbool.h>
#include <library.h>

#define F_CPU 4000000UL

/* I2C clock in Hz */
#define SCL_CLOCK  100000L

// Adress of the i2c BMP180 sensor
#define BMP180_ADDRESS 0x77 // (0x77 << 1) //With bitshifting it end in adress 0xEE

#define BMP180_ADDRESS_WRITE 0xEE // (0x77 << 1) //With bitshifting it end in adress 0xEE
#define BMP180_ADDRESS_READ 0xEF // (0x77 << 1) //With bitshifting it end in adress 0xEE

#define BMP180_TEMPDATA_ADRESS 0xF6

#define BMP180_CONTROL 0xF4

#define BMP180_READTEMPCMD 0x2E

// Adress of the i2c LCD Display
#define LCD_ADDRESS 0x27

#define I2C_READ 1

#define I2C_WRITE 0

#define AC1_ADRESS 0xAA
#define AC2_ADRESS 0xAC
#define AC3_ADRESS 0xAE
#define AC4_ADRESS 0xB0
#define AC5_ADRESS 0xB2
#define AC6_ADRESS 0xB4

#define B1_ADRESS 0xB6
#define B2_ADRESS 0xB8

#define MB_ADRESS 0xBA
#define MC_ADRESS 0xBC
#define MD_ADRESS 0xBE

// Temperature calibration values
short ac1;;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;
long ut;

bool lcd_initialized = false;

void display_init(char addr) {
    HD44780_PCF8574_Init(addr);
    HD44780_PCF8574_DisplayOn(addr);
    lcd_initialized = true;
}

void display_text(char addr, int number){
    if(!lcd_initialized) {
        display_init(addr);
    }
    HD44780_PCF8574_DisplayClear(addr);
    //HD44780_PCF8574_CursorOn(addr);
    //HD44780_PCF8574_CursorBlink(addr);

    // Separate the first two digits and the last two digits
    int first_two = number / 10;
    int last_two = (number % 100) % 10;

    HD44780_PCF8574_DrawString(addr, "Temperature:");
    HD44780_PCF8574_PositionXY(addr, 0, 1);
    char number_str[16];
    sprintf(number_str, "%d.%0d0 C", first_two, last_two);
    HD44780_PCF8574_DrawString(addr, number_str);
}

uint16_t bmp180_get_cal_param(char addr){
    unsigned char MSB;
    unsigned char LSB;
    uint16_t combinedValue;
    
    //Request Calibration value register
    i2c_start_wait(BMP180_ADDRESS_WRITE);       // module adress 0x77 = 1110111 || + Write Bit (0) = 0xEE = 11101110 || + Acknowledge Bit (0) = 111011100
    i2c_write(addr);                            // Register of the calibration value to read
    i2c_stop();                                 // Stop Bit (1)

    //Read calibration value split into MSB and LSB
    i2c_start_wait(BMP180_ADDRESS_READ);        // module adress 0x77 = 1110111 || + Read Bit (1) = 0xEE = 11101111 || + Acknowledge Bit (0) = 111011100
    MSB = i2c_readAck();                        // response || + Acknowledge Master (0)
    LSB = i2c_readNak();                        // response || + Not Acknowledge Master (1)
    i2c_stop();                                 // Stop Bit (1)

    combinedValue = (uint16_t)MSB << 8 | LSB;

    return combinedValue;
}

uint16_t bmp180_get_ut(){
    unsigned char MSB;
    unsigned char LSB;
    uint16_t ut;

    // Start Temperature reading
    i2c_start_wait(BMP180_ADDRESS_WRITE);       // module adress 0x77 = 1110111 || + Write Bit (0) = 0xEE = 11101110 || + Acknowledge Bit (0) = 111011100
    i2c_write(BMP180_CONTROL);                  // control register adress 0xF4 = 11110100 || + Acknowledge Bit (0) = 111101000              
    i2c_write(BMP180_READTEMPCMD);              // Temperature Read Command Register 0x2E = 00101110 || + Acknowledge Bit (0) = 001011100       
    i2c_stop();                                 // Stop Bit (1)

    //Request Temperature value
    i2c_start_wait(BMP180_ADDRESS_WRITE);       // module adress 0x77 = 1110111 || + Write Bit (0) = 0xEE = 11101110 || + Acknowledge Bit (0) = 111011100
    i2c_write(BMP180_TEMPDATA_ADRESS);          // Temperature Data Register 0xF6 = 11110110 || + Acknowledge Bit (0) = 111101100
    i2c_stop();                                 // Stop Bit (1)
    //_delay_ms(5);                                 
    //Read Temperature split into MSB and LSB
    i2c_start_wait(BMP180_ADDRESS_READ);        // module adress 0x77 = 1110111 || + Read Bit (1) = 0xEE = 11101111 || + Acknowledge Bit (0) = 111011100
    MSB = i2c_readAck();                        // response || + Acknowledge Master (0)
    LSB = i2c_readNak();                        // response || + Not Acknowledge Master (1)
    i2c_stop();                                 // Stop Bit (1)
    
    // Combine MSB and LSB into a 16-bit value
    ut = (uint16_t)MSB << 8 | LSB;

    return ut;
}

int32_t computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

int main(void) {
    for(;;) {
    i2c_initialize();  
    ac1 = bmp180_get_cal_param(AC1_ADRESS);
    ac2 = bmp180_get_cal_param(AC2_ADRESS);
    ac3 = bmp180_get_cal_param(AC3_ADRESS);
    ac4 = bmp180_get_cal_param(AC4_ADRESS);
    ac5 = bmp180_get_cal_param(AC5_ADRESS);
    ac6 = bmp180_get_cal_param(AC6_ADRESS);
    b1 = bmp180_get_cal_param(B1_ADRESS);
    b2 = bmp180_get_cal_param(B2_ADRESS);
    mb = bmp180_get_cal_param(MB_ADRESS);
    mc = bmp180_get_cal_param(MC_ADRESS);
    md = bmp180_get_cal_param(MD_ADRESS);
    ut = bmp180_get_ut();
    int32_t rawTemp = computeB5(ut);
    int32_t temperature = (rawTemp + 8) / (1 << 4);
    display_text(LCD_ADDRESS, temperature);
    _delay_ms(500);
    }
    return 0;
}

void blinki_blink(){
    DDRB |= (1<< DDB5);
    while(1){
        _delay_ms(200);
        PORTB |= (1 << PORTB5);
        _delay_ms(200);
        PORTB &= ~(1 << PORTB5);
    }
}