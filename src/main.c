#include <stdio.h>
#include <util/delay.h>
// #include <i2cmaster.h>
#include <lib/hd44780pcf8574.h>
#include <compat/twi.h>
#include <stdbool.h>
#include "library.c"


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
// use datasheet numbers!

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

lcd_initialized = false;

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
    int last_two = number % 100;

    HD44780_PCF8574_DrawString(addr, "Temperature:");
    HD44780_PCF8574_PositionXY(addr, 0, 1);
    char number_str[16];
    sprintf(number_str, "%d.%02d C", first_two, last_two);
    HD44780_PCF8574_DrawString(addr, number_str);
    
}

void i2c_initialize(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
}

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWI = Two-Wire Interface
                                                // TWCR = Two-Wire Control Register
                                                // TWINT = TWI Interrupt Flag
                                                // TWSTA = TWI Start Condition Bit
                                                // TWEN = TWI Enable Bit

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
    
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

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

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}/* i2c_readAck */

void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
        
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
        
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }
}

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
    
	if( twst != TW_MT_DATA_ACK) return 1;
    //blinki_blink();
    
	return 0;

}/* i2c_write */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
    // blinki_blink();
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;
    

}/* i2c_readNak */

int32_t computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
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
    //Read Temperature split into MSB and LSB
    i2c_start_wait(BMP180_ADDRESS_READ);        // module adress 0x77 = 1110111 || + Read Bit (1) = 0xEE = 11101111 || + Acknowledge Bit (0) = 111011100
    MSB = i2c_readAck();                        // response || + Acknowledge Master (0)
    LSB = i2c_readNak();                        // response || + Not Acknowledge Master (1)
    i2c_stop();                                 // Stop Bit (1)
    
    // Combine MSB and LSB into a 16-bit value
    ut = (uint16_t)MSB << 8 | LSB;

    return ut;
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
    int32_t tempVal = computeB5(ut);
    int32_t temperature = (tempVal + 8) / (1 << 4);
    display_text(LCD_ADDRESS, temperature);
    _delay_ms(500);
    }
    
    /*
    unsigned char MSB;
    unsigned char LSB;
    uint16_t combinedValue;
    
    for (;;) {
        // Start Temperature reading
        i2c_initialize();                           // Oszilosneering comments
        i2c_start_wait(BMP180_ADDRESS_WRITE);       // module adress 0x77 = 1110111 || + Write Bit (0) = 0xEE = 11101110 || + Acknowledge Bit (0) = 111011100
        i2c_write(BMP180_CONTROL);                  // control register adress 0xF4 = 11110100 || + Acknowledge Bit (0) = 111101000              
        i2c_write(BMP180_READTEMPCMD);              // Temperature Read Command Register 0x2E = 00101110 || + Acknowledge Bit (0) = 001011100       
        i2c_stop();                                 // Stop Bit (1)
        

        //Request Temperature value
        i2c_start_wait(BMP180_ADDRESS_WRITE);       // module adress 0x77 = 1110111 || + Write Bit (0) = 0xEE = 11101110 || + Acknowledge Bit (0) = 111011100
        i2c_write(BMP180_TEMPDATA_ADRESS);          // Temperature Data Register 0xF6 = 11110110 || + Acknowledge Bit (0) = 111101100
        i2c_stop();                                 // Stop Bit (1)
        //Read Temperature split into MSB and LSB
        i2c_start_wait(BMP180_ADDRESS_READ);        // module adress 0x77 = 1110111 || + Read Bit (1) = 0xEE = 11101111 || + Acknowledge Bit (0) = 111011100
        MSB = i2c_readAck();                        // response || + Acknowledge Master (0)
        LSB = i2c_readNak();                        // response || + Not Acknowledge Master (1)
        i2c_stop();                                 // Stop Bit (1)
        
        // Combine MSB and LSB into a 16-bit value
        combinedValue = (uint16_t)MSB << 8 | LSB;
        int32_t tempVal = computeB5(combinedValue);
        
        display_text(LCD_ADDRESS, tempVal);
        
        //Keep the display readable by limiting temp updates to every 200ms
        _delay_ms(200);
      
    }
    */

    return 0;
}








