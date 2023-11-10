#include <stdio.h>
#include <util/delay.h>
// #include <i2cmaster.h>
// #include <lcd.h>
#include <lib/hd44780pcf8574.h>
#include <compat/twi.h>
#include <stdbool.h>


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

// use datasheet numbers!
UP = 23843;
ac6 = 23153;
ac5 = 32757;
mc = -8711;
md = 2868;
b1 = 6190;
b2 = 4;
ac3 = -14383;
ac2 = -72;
ac1 = 408;
ac4 = 32741;
oversampling = 0;

//#define i2c_read(ack) (ack) ? i2c_readAck() : i2c_readNak();

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
    int first_two = number / 100;
    int last_two = number % 100;

    HD44780_PCF8574_DrawString(addr, "Temperatur:");
    HD44780_PCF8574_PositionXY(addr, 0, 1);
    char number_str[16];
    sprintf(number_str, "%d.%02d C", first_two, last_two);
    HD44780_PCF8574_DrawString(addr, number_str);
    
}


void read_bmp180(char addr){
    
    //i2c_rep_start(addr+I2C_READ);
    
    
    
    //i2c_start_wait(BMP180_ADDRESS);
    //str = i2c_read(0x2E);
    //i2c_write(0xF4); // Control register for temperature reading
    
    // i2c_read(addr);
    
    //i2c_stop();

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

/*
int main(void) {
    unsigned char MSB;
    unsigned char LSB;
    i2c_initialize();
    i2c_start_wait(BMP180_ADDRESS_WRITE);
    i2c_write(BMP180_CONTROL);
    i2c_write(BMP180_READTEMPCMD);
    i2c_stop();

    i2c_start_wait(BMP180_ADDRESS_WRITE);
    i2c_write(BMP180_TEMPDATA_ADRESS);
    i2c_stop();

    i2c_start_wait(BMP180_ADDRESS_READ);
    i2c_write(BMP180_ADDRESS_READ);
    MSB = i2c_readNak();
    LSB = i2c_readNak();
    i2c_stop();

    // Assuming MSB and LSB are uint8_t variables representing the received bytes
    uint16_t combinedValue = (uint16_t)MSB << 8 | LSB;



    //i2c_start_wait(BMP180_TEMPDATA_ADRESS);
    //i2c_rep_start(BMP180_TEMPDATA_ADRESS);
    //ret = i2c_readNak();
    //i2c_stop();
    
    for(;;){
        _delay_ms(2000);
        display_text(LCD_ADDRESS, MSB);
        _delay_ms(2000);
        display_text(LCD_ADDRESS, LSB);
        _delay_ms(2000);
        display_text(LCD_ADDRESS, combinedValue);
    };

    return 0;
}
*/


int main(void) {
    unsigned char MSB;
    unsigned char LSB;
    uint16_t combinedValue;

    i2c_initialize();


        

    for (;;) {
        // Start Temperature reading
        i2c_start_wait(BMP180_ADDRESS_WRITE);
        i2c_write(BMP180_CONTROL);
        i2c_write(BMP180_READTEMPCMD);
        i2c_stop();

        //Request Temperature value
        i2c_start_wait(BMP180_ADDRESS_WRITE);
        i2c_write(BMP180_TEMPDATA_ADRESS);
        i2c_stop();

        //Read Temperature split into MSB and LSB
        i2c_start_wait(BMP180_ADDRESS_READ);
        MSB = i2c_readAck();
        LSB = i2c_readNak();
        i2c_stop();

        // Combine MSB and LSB into a 16-bit value
        combinedValue = (uint16_t)MSB << 8 | LSB;
        int32_t tempVal = computeB5(combinedValue);
      
        display_text(LCD_ADDRESS, tempVal);
        
        //Keep the display readable by limiting temp updates to every 200ms
        _delay_ms(250);
    }

    return 0;
}




