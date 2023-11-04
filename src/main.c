#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
// #include <lcd.h>
#include <lib/hd44780pcf8574.h>

// Adress of the i2c BMP180 sensor
#define BMP180_ADDRESS (0x77 < 1)

// Adress of the i2c LCD Display
#define LCD_ADDRESS 0x27

//#define I2C_READ 1

//#define i2c_read(ack) (ack) ? i2c_readAck() : i2c_readNak();

void display_text(char addr, int number){
    // Initialize the LCD display
    HD44780_PCF8574_Init(addr);
    HD44780_PCF8574_DisplayOn(addr);
    HD44780_PCF8574_DisplayClear(addr);
    HD44780_PCF8574_CursorOn(addr);
    HD44780_PCF8574_CursorBlink(addr);

    char number_str[16];
    sprintf(number_str, "Temp: %d C", number);
    HD44780_PCF8574_DrawString(addr, number_str);
}


void read_bmp180(char addr){
    
    //i2c_rep_start(addr+I2C_READ);
    
    
    
    //i2c_start_wait(BMP180_ADDRESS);
    //str = i2c_read(0x2E);
    //i2c_write(0xF4); // Control register for temperature reading
    
    // i2c_read(addr);
    
    i2c_stop();

}

int main(void) {
    unsigned char str;

    i2c_init();
    str  = i2c_start(BMP180_ADDRESS);
    if(str) {
        DDRB |= (1<< DDB5);
        while(1){
            _delay_ms(200);
            PORTB |= (1 << PORTB5);
            _delay_ms(200);
            PORTB &= ~(1 << PORTB5);
        }
    }else {
        DDRB |= (1<< DDB5);
        while(1){
            _delay_ms(500);
            PORTB |= (1 << PORTB5);
            _delay_ms(500);
            PORTB &= ~(1 << PORTB5);
        }
    }

    
    
    
    
    
    
    
    //read_bmp180(BMP180_ADDRESS);

    //display_text(LCD_ADDRESS, 100);
    


    

    
    
    

    /*
    i2c_start_wait(BMP180_ADDRESS + I2C_WRITE);
    i2c_write(0xF4); // Control register for temperature reading
    i2c_rep_start(BMP180_ADDRESS + I2C_READ);
    i2c_stop();
    */
    
    
    
    // Initialize the BMP180 sensor

    /*
    // Calibration coefficients
    int16_t ac1 = 0xAA; 
    int16_t ac2 = 0xAC; 
    int16_t ac3 = 0xAE; 
    int16_t ac4 = 0xB0; 
    int16_t ac5 = 0xB2; 
    int16_t ac6 = 0xB4; 
    int16_t mb = 0xBA;  
    int16_t mc = 0xBC;  
    int16_t md = 0xBE; 

    // Calculate temperature in degrees Celsius
    int32_t x1 = ((int32_t)temperature - ac6) * ac5 >> 15;
    int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
    int32_t b5 = x1 + x2;
    int16_t temperature_celsius = ((b5 + 8) >> 4) / 10;
    */

    return 0;
}




