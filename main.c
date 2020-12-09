/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 02 December 2020, 09:41
 */

#define F_CPU 3333333UL

#include <stdio.h>
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>	
#include <xc.h>

#define LCD_init 0x33 
#define LCD_8bit 0x38
#define	LCD_displayOn_CursorBlink 0x0F
#define	LCD_displayOn_CursorOff 0x0C 
#define	LCD_clear 0x01
#define LCD_cursor_1_line_start 0x80		
#define LCD_cursor_2_line_start 0xC0
#define LCD_cursor_increment 0x06
#define LCD_cursor_decrement 0x04
#define delay_enable 60
#define delay_lcd_init 100
#define delay_lcd_clear 2
#define delay_lcd_set 60
#define big_delay 2000
#define small_delay 500
#define black_box 0xFF

volatile uint16_t ADC = 0;
volatile uint8_t updateLCD = 0; 

volatile unsigned int clicks;
volatile unsigned int rpm;

//set LCD-ports
void LCD_Command( unsigned char cmnd )
{	
    PORTD.OUT = cmnd;
	PORTB.OUTCLR = PIN4_bm;				/* RS=0, command reg. */
	PORTB.OUTSET = PIN3_bm;				/* Enable pulse */
	_delay_us(delay_enable);
	PORTB.OUTCLR = PIN3_bm;

	_delay_us(delay_lcd_set);
}

//set LCD-ports
void LCD_Char( unsigned char data )
{
	PORTD.OUT = data;
	PORTB.OUTSET = PIN4_bm;				/* RS=1, command reg. */
	PORTB.OUTSET = PIN3_bm;				/* Enable pulse */
	_delay_us(delay_enable);
	PORTB.OUTCLR = PIN3_bm;
	_delay_us(delay_lcd_set);
    LCD_Command(LCD_cursor_increment);
}

//Initialize LCD-screen
void LCD_Init (void)					
{
    PORTB.DIRSET = (PIN3_bm | PIN4_bm | PIN5_bm);
    PORTB.OUTSET = PIN5_bm; 
    PORTB.OUTCLR = (PIN3_bm | PIN4_bm);
	PORTD.DIR = 0xFF;				
    PORTD.OUT = 0x00;
    _delay_us(delay_lcd_set);						
	
 
	LCD_Command(LCD_init);
    LCD_Command(LCD_8bit);
	LCD_Command(LCD_displayOn_CursorOff);              	             	
	LCD_Command(LCD_clear); 
    LCD_Command(LCD_cursor_1_line_start);
	_delay_us(delay_lcd_init);    
}

//enables writing to the LCD-screen
void LCD_String (char *str)				
{
	int i;
	for(i=0;str[i]!='\0';i++)				
	{
		LCD_Char(str[i]);
	}
}

//clears LCD-screen
void LCD_Clear()
{
	LCD_Command(LCD_clear);					
	_delay_ms(delay_lcd_clear);
	LCD_Command(LCD_cursor_1_line_start);	
    _delay_ms(delay_lcd_clear);
}

//Initialize ADC
void ADC0_init(void)
{
    //Disable digital input buffer
    PORTE.PIN0CTRL &= ~PORT_ISC_gm;
    PORTE.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    
    //Disable pull-up resistor
    PORTE.PIN0CTRL &= ~PORT_PULLUPEN_bm;
    
    //CLK_PER divided by 16, Internal reference
    ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc;
    
    //ADC enabled, 10-bit mode
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
    
    //Select ADC channel
    ADC0.MUXPOS = ADC_MUXPOS_AIN8_gc;
    
    //set internal reference
    VREF.CTRLA |= VREF_ADC0REFSEL_2V5_gc; 
    
    //Enable interrupts
    ADC0.INTCTRL |= ADC_RESRDY_bm;    
}

int main(void) 
{   
    //Eet up periodic cycle
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc;
    
    //Initialize ADC
    ADC0_init();	
    
    // Enable RTC interrupt timer
    RTC.PITINTCTRL = RTC_PI_bm;
    
    // Sets LDR pin
    PORTE.DIRCLR = PIN0_bm;
    
    // Sets dc-motor pin
    PORTA.DIRSET = PIN3_bm;
    
    //let me sleep :)
    set_sleep_mode(SLPCTRL_SMODE_IDLE_gc);
    
    //Initialize lcd-screen
    cli();
    LCD_Init();    
    sei(); 

    //rtc enable
    RTC.PITCTRLA = RTC.PITCTRLA | RTC_PITEN_bm;
    
    //value for adc
    uint16_t exValue = 0;
    
    //prints rpm count.
    LCD_Clear();
    LCD_String("RPM Count:");
    
    ADC0.COMMAND = ADC_STCONV_bm;
    
    //turns on dc-motor
    //PORTA.OUT |= PIN3_bm;
    
    char string[5];
    
    while(1)
    {   
        if (updateLCD)
        {
            //näytön päivitys
            rpm = clicks * 30;
            sprintf(string, "%d", rpm);
            LCD_Command(LCD_cursor_2_line_start);
            LCD_String(string); 
            clicks = 0;
            updateLCD = 0;
        }
        
        //click counter, if over 600 adds click
        else
        {        
            if (ADC > 600 && exValue < 600)
            {
                clicks++;
                exValue = 700;
            }

            else if (ADC > 600 && exValue > 600)
            {
                exValue = 700;
            }
            else
            {
                exValue = 200;
            }     
        }
        sleep_mode();
    }    
}

//counts rpm & prints it to the LCD-screen
ISR(RTC_PIT_vect)
{      
    RTC.PITINTFLAGS = RTC_PI_bm;
    updateLCD = 5;
}

ISR(ADC0_RESRDY_vect)
{
    //set ADC value
    ADC = ADC0.RES;
    
    ADC0.COMMAND = ADC_STCONV_bm;
}