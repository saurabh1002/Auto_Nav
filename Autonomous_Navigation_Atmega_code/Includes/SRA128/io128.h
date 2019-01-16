
/* io_config .h */
#ifndef IO128_H
#define IO128_H

#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>

#ifndef F_CPU
	#define F_CPU 14745600UL
#endif



//BASIC I/O FILE SETTINGS


#define INPUT 0
#define OUTPUT 1

//typedef enum _BOOL { FALSE = 0, TRUE } BOOL;
#ifndef _IO_REG
#define _IO_REG
typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 
#endif

#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt


/** I N C L U D E S **/

//ADC 	DIRECTIONS

#define ADC_PORT			PORTF
#define ADC_DIR 	DDRF


//PWM pins

//PWM0

#define PWM0_DIR  	REGISTER_BIT(DDRB,4)
#define PWM0  		REGISTER_BIT(PORTB,4)

//PWM1A

#define PWM1A_DIR  	REGISTER_BIT(DDRB,5)
#define PWM1A 		REGISTER_BIT(PORTB,5)

//PWM1B

#define PWM1B_DIR  	REGISTER_BIT(DDRB,6)
#define PWM1B 		REGISTER_BIT(PORTB,6)

//PWM1C

#define PWM1C_DIR  	REGISTER_BIT(DDRB,7)
#define PWM1C 		REGISTER_BIT(PORTB,7)

//PWM3A

#define PWM3A_DIR  	REGISTER_BIT(DDRE,3)
#define PWM3A 		REGISTER_BIT(PORTE,3)

//PWM3B

#define PWM3B_DIR  	REGISTER_BIT(DDRE,4)
#define PWM3B 		REGISTER_BIT(PORTE,4)

//PWM3C

#define PWM3C_DIR  	REGISTER_BIT(DDRE,5)
#define PWM3C 		REGISTER_BIT(PORTE,5)


//PWM2

#define PWM2_DIR  	REGISTER_BIT(DDRB,7)
#define PWM2 		REGISTER_BIT(PORTB,7)

//LCD CONNECTIONS

#define LCD_DATA A	//Port PC0-PC3 are connected to D4-D7

#define LCD_E A	//Enable OR strobe signal
#define LCD_E_POS	PA7	//Position of enable in above port

#define LCD_RS A	
#define LCD_RS_POS 	PA5

#define LCD_RW A
#define LCD_RW_POS 	PA6

//FUNCTION PROTOTYPES
void port_init(void);
void pwm0_init(void);
void pwm1_init(void);
void pwm2_init(void);
void pwm3_init(void);
void adc_init(void);
unsigned int adc_start(unsigned char channel);
void delay_sec(int x);
void delay_milisec(int n);
void delay_microsec(int n);
void check_sensors(void);
void calibrate_black(void);
void calibrate_white(void);
void set_threshold(void);
void flick (void);
void set_pwm1a(int a);
void set_pwm1b(int b);
void set_pwm1c(int c);
void set_pwm3a(int a);
void set_pwm3b(int b);
void set_pwm3c(int c);
void set_pwm0(int d);
void set_pwm2(int e);
void lcd_byte(uint8_t c,uint8_t isdata);
void lcd_busy_loop(void);
void lcd_init(uint8_t style);
void lcd_write_string(const char *msg);
void lcd_write_int(int val,unsigned int field_length);
void lcd_goto_xy(uint8_t x,uint8_t y);
void lcd_write_string_xy(int x,int y,char *msg);
void lcd_write_int_xy(int x,int y,int val,int fl);
void usart_init(void);
void usart_transmit_char( unsigned char data );
void usart_transmit_string(char *msg );
void usart_transmit_newline(void);
unsigned char usart_receive_char(void);
#endif //IO_H
