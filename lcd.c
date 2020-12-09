/*
 * File:   lcd.c
 * Author: dtek0068
 *
 * Created on 03 December 2020, 11:23
 */


#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

#define F_CPU 3333333
#define _BV(bit) (1 << (bit))

#if LCD_IO_MODE
#define LCD_EN_delay()   _delay_us(LCD_DELAY_ENABLE_PULSE)
#define LCD_EN_high()    LCD_EN_PORT.OUT  |=  _BV(LCD_EN_PIN);			// PORTC PIN6
#define LCD_EN_low()     LCD_EN_PORT.OUT  &= ~_BV(LCD_EN_PIN);
#define LCD_EN_toggle()  toggle_e()

#define LCD_RW_high()   LCD_RW_PORT.OUT |=  _BV(LCD_RW_PIN)					// PORTC PIN5
#define LCD_RW_low()    LCD_RW_PORT.OUT &= ~_BV(LCD_RW_PIN)

#define LCD_RS_high()   LCD_RS_PORT.OUT |=  _BV(LCD_RS_PIN)					// PORTC PIN7
#define LCD_RS_low()    LCD_RS_PORT.OUT &= ~_BV(LCD_RS_PIN)
#endif

//***************************************************************************************//

#if LCD_IO_MODE
#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 						// 2-LINE LCD
#endif
#endif

#if LCD_IO_MODE
static void toggle_e(void);
#endif

//***************************************************************************************//


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
#define delay(us)  _delay_us(us) 

#if LCD_IO_MODE
/* toggle Enable Pin to initiate write */
static void toggle_e(void)
{
    LCD_EN_high();
    LCD_EN_delay();
    LCD_EN_low();
}
#endif


/*************************************************************************
Low-level function to write byte to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data    
                 0: write instruction
Returns:  none
*************************************************************************/
#if LCD_IO_MODE
static void lcd_write(uint8_t data,uint8_t rs) 
{
    unsigned char dataBits ;

    if (rs) {        /* write data        (RS=1, RW=0) */
       LCD_RS_high();
    } else {         /* write instruction (RS=0, RW=0) */
       LCD_RS_low();
    }
    LCD_RW_low();    /* RW=0  write mode      */

    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT )
      && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT )
      && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
      && (LCD_DATA0_PIN == 0)
      && (LCD_DATA1_PIN == 1)
      && (LCD_DATA2_PIN == 2)
      && (LCD_DATA3_PIN == 3) )
    {
        /* configure data pins as output */
        LCD_DATA0_PORT.DIR |= 0x0F;

        /* output high nibble first */
        dataBits = LCD_DATA0_PORT.OUT & 0xF0;
        LCD_DATA0_PORT.OUT = dataBits |((data>>4)&0x0F);
        LCD_EN_toggle();

        /* output low nibble */
        LCD_DATA0_PORT.OUT = dataBits | (data&0x0F);
        LCD_EN_toggle();

        /* all data pins high (inactive) */
        LCD_DATA0_PORT.OUT = dataBits | 0x0F;
    }
    else
    {
        /* configure data pins as output */
        LCD_DATA0_PORT.DIR |= _BV(LCD_DATA0_PIN);
        LCD_DATA1_PORT.DIR |= _BV(LCD_DATA1_PIN);
        LCD_DATA2_PORT.DIR |= _BV(LCD_DATA2_PIN);
        LCD_DATA3_PORT.DIR |= _BV(LCD_DATA3_PIN);
        
        /* output high nibble first */
        LCD_DATA3_PORT.OUT &= ~_BV(LCD_DATA3_PIN);
        LCD_DATA2_PORT.OUT &= ~_BV(LCD_DATA2_PIN);
        LCD_DATA1_PORT.OUT &= ~_BV(LCD_DATA1_PIN);
        LCD_DATA0_PORT.OUT &= ~_BV(LCD_DATA0_PIN);
        
      if(data & 0x80) LCD_DATA3_PORT.OUT |= _BV(LCD_DATA3_PIN);
      if(data & 0x40) LCD_DATA2_PORT.OUT |= _BV(LCD_DATA2_PIN);
      if(data & 0x20) LCD_DATA1_PORT.OUT |= _BV(LCD_DATA1_PIN);
      if(data & 0x10) LCD_DATA0_PORT.OUT |= _BV(LCD_DATA0_PIN);   
        LCD_EN_toggle();
        
        /* output low nibble */
        LCD_DATA3_PORT.OUT &= ~_BV(LCD_DATA3_PIN);
        LCD_DATA2_PORT.OUT &= ~_BV(LCD_DATA2_PIN);
        LCD_DATA1_PORT.OUT &= ~_BV(LCD_DATA1_PIN);
        LCD_DATA0_PORT.OUT &= ~_BV(LCD_DATA0_PIN);
      if(data & 0x08) LCD_DATA3_PORT.OUT |= _BV(LCD_DATA3_PIN);
      if(data & 0x04) LCD_DATA2_PORT.OUT |= _BV(LCD_DATA2_PIN);
      if(data & 0x02) LCD_DATA1_PORT.OUT |= _BV(LCD_DATA1_PIN);
      if(data & 0x01) LCD_DATA0_PORT.OUT |= _BV(LCD_DATA0_PIN);
        LCD_EN_toggle();        
        
        /* all data pins high (inactive) */
        LCD_DATA0_PORT.OUT |= _BV(LCD_DATA0_PIN);
        LCD_DATA1_PORT.OUT |= _BV(LCD_DATA1_PIN);
        LCD_DATA2_PORT.OUT |= _BV(LCD_DATA2_PIN);
        LCD_DATA3_PORT.OUT |= _BV(LCD_DATA3_PIN);
    }
}
#else
#define lcd_write(d,rs) if (rs) *(volatile uint8_t*)(LCD_IO_DATA) = d; else *(volatile uint8_t*)(LCD_IO_FUNCTION) = d;
/* rs==0 -> write instruction to LCD_IO_FUNCTION */
/* rs==1 -> write data to LCD_IO_DATA */
#endif


/*************************************************************************
Low-level function to read byte from LCD controller
Input:    rs     1: read data    
                 0: read busy flag / address counter
Returns:  byte read from LCD controller
*************************************************************************/
#if LCD_IO_MODE
static uint8_t lcd_read(uint8_t rs) 
{
    uint8_t data;
    
    
    if (rs)
        LCD_RS_high();                       /* RS=1: read data      */
    else
        LCD_RS_low();                        /* RS=0: read busy flag */
    LCD_RW_high();                           /* RW=1  read mode      */
    
    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
      && ( LCD_DATA0_PIN == 0 )&& (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) )
    {
        LCD_DATA0_PORT.DIR &= 0xF0;         /* configure data pins as input */
        
        LCD_EN_high();
        LCD_EN_delay();        
        data = LCD_DATA0_PORT.IN << 4;     /* read high nibble first */
        LCD_EN_low();
        
        LCD_EN_delay();                       /* Enable 500ns low       */
        
        LCD_EN_high();
        LCD_EN_delay();
        data |= LCD_DATA0_PORT.IN &0x0F;    /* read low nibble        */
        LCD_EN_low();
    }
    else
    {
        /* configure data pins as input */
        LCD_DATA0_PORT.DIR &= ~_BV(LCD_DATA0_PIN);
        LCD_DATA1_PORT.DIR &= ~_BV(LCD_DATA1_PIN);
        LCD_DATA2_PORT.DIR &= ~_BV(LCD_DATA2_PIN);
        LCD_DATA3_PORT.DIR &= ~_BV(LCD_DATA3_PIN);
                
        /* read high nibble first */
        LCD_EN_high();
        LCD_EN_delay();        
        data = 0;
        if ( LCD_DATA0_PORT.IN & _BV(LCD_DATA0_PIN) ) data |= 0x10;
        if ( LCD_DATA1_PORT.IN & _BV(LCD_DATA1_PIN) ) data |= 0x20;
        if ( LCD_DATA2_PORT.IN & _BV(LCD_DATA2_PIN) ) data |= 0x40;
        if ( LCD_DATA3_PORT.IN & _BV(LCD_DATA3_PIN) ) data |= 0x80;
        LCD_EN_low();

        LCD_EN_delay();                       /* Enable 500ns low       */
    
        /* read low nibble */    
        LCD_EN_high();
        LCD_EN_delay();
        if ( LCD_DATA0_PORT.IN & _BV(LCD_DATA0_PIN) ) data |= 0x01;
        if ( LCD_DATA1_PORT.IN & _BV(LCD_DATA1_PIN) ) data |= 0x02;
        if ( LCD_DATA2_PORT.IN & _BV(LCD_DATA2_PIN) ) data |= 0x04;
        if ( LCD_DATA3_PORT.IN & _BV(LCD_DATA3_PIN) ) data |= 0x08;        
        LCD_EN_low();
    }
    return data;
}
#else
#define lcd_read(rs) (rs) ? *(volatile uint8_t*)(LCD_IO_DATA+LCD_IO_READ) : *(volatile uint8_t*)(LCD_IO_FUNCTION+LCD_IO_READ)
/* rs==0 -> read instruction from LCD_IO_FUNCTION */
/* rs==1 -> read data from LCD_IO_DATA */
#endif


/*************************************************************************
loops while lcd is busy, returns address counter
*************************************************************************/
static uint8_t lcd_waitbusy(void)

{
    register uint8_t c;
    
    /* wait until busy flag is cleared */
    while ( (c=lcd_read(0)) & (1<<LCD_BUSY)) {}
    
    /* the address counter is updated 4us after the busy flag is cleared */
    delay(LCD_DELAY_BUSY_FLAG);

    /* now read the address counter */
    return (lcd_read(0));  // return address counter
    
}/* lcd_waitbusy */


/*************************************************************************
Move cursor to the start of next line or to the first line if the cursor 
is already on the last line.
*************************************************************************/
static inline void lcd_newline(uint8_t pos)
{
    register uint8_t addressCounter;


#if LCD_LINES==1
    addressCounter = 0;
#endif
#if LCD_LINES==2
    if ( pos < (LCD_START_LINE2) )
        addressCounter = LCD_START_LINE2;
    else
        addressCounter = LCD_START_LINE1;
#endif
    lcd_command((1<<LCD_DDRAM)+addressCounter);

}/* lcd_newline */


/*
** PUBLIC FUNCTIONS 
*/

/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_command(uint8_t cmd)
{
    lcd_waitbusy();
    lcd_write(cmd,0);
}


/*************************************************************************
Send data byte to LCD controller 
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_data(uint8_t data)
{
    lcd_waitbusy();
    lcd_write(data,1);
}


/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
#if LCD_LINES==1
    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
#endif
#if LCD_LINES==2
    if ( y==0 ) 
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
#endif
#if LCD_LINES==4
    if ( y==0 )
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else if ( y==1)
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
    else if ( y==2)
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE3+x);
    else /* y==3 */
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE4+x);
#endif

}/* lcd_gotoxy */


/*************************************************************************
*************************************************************************/
int lcd_getxy(void)
{
    return lcd_waitbusy();
}


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_command(1<<LCD_CLR);
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);
}


/*************************************************************************
Display character at current cursor position 
Input:    character to be displayed                                       
Returns:  none
*************************************************************************/
void lcd_putc(char c)
{
    uint8_t pos;

    pos = lcd_waitbusy();   // read busy-flag and address counter
    if (c=='\n')
    {
        lcd_newline(pos);
    }
    else
    {
#if LCD_WRAP_LINES==1

#if LCD_LINES==1
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }

#elif LCD_LINES==2
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ){
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#endif
        lcd_waitbusy();
#endif
        lcd_write(c, 1);
    }

}/* lcd_putc */


/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */

/*************************************************************************
Initialize display and select type of cursor 
Input:    dispAttr LCD_DISP_OFF            display off
                   LCD_DISP_ON             display on, cursor off
                   LCD_DISP_ON_CURSOR      display on, cursor on
                   LCD_DISP_CURSOR_BLINK   display on, cursor on flashing
Returns:  none
*************************************************************************/
void lcd_init(uint8_t dispAttr)
{
#if LCD_IO_MODE
    /*
     *  Initialize LCD to 4 bit I/O mode
     */
    /*configure all port bits as output (LCD data and control lines on different ports */
    LCD_RS_PORT.DIR   |= _BV(LCD_RS_PIN);
    LCD_RW_PORT.DIR    |= _BV(LCD_RW_PIN);
    LCD_EN_PORT.DIR     |= _BV(LCD_EN_PIN);
    LCD_DATA0_PORT.DIR |= _BV(LCD_DATA0_PIN);
    LCD_DATA1_PORT.DIR |= _BV(LCD_DATA1_PIN);
    LCD_DATA2_PORT.DIR |= _BV(LCD_DATA2_PIN);
    LCD_DATA3_PORT.DIR |= _BV(LCD_DATA3_PIN);

    delay(LCD_DELAY_BOOTUP);             /* wait 16ms or more after power-on       */
    
    /* initial write to lcd is 8bit */
    LCD_DATA1_PORT.OUT |= _BV(LCD_DATA1_PIN);    // LCD_FUNCTION>>4;
    LCD_DATA0_PORT.OUT |= _BV(LCD_DATA0_PIN);    // LCD_FUNCTION_8BIT>>4;
    LCD_EN_toggle();
    delay(LCD_DELAY_INIT);               /* delay, busy flag can't be checked here */
   
    /* repeat last command */ 
    LCD_EN_toggle();      
    delay(LCD_DELAY_INIT_REP);           /* delay, busy flag can't be checked here */
    
    /* repeat last command a third time */
    LCD_EN_toggle();      
    delay(LCD_DELAY_INIT_REP);           /* delay, busy flag can't be checked here */

    /* now configure for 4bit mode */
    LCD_DATA0_PORT.OUT &= ~_BV(LCD_DATA0_PIN);   // LCD_FUNCTION_4BIT_1LINE>>4
    LCD_EN_toggle();
    delay(LCD_DELAY_INIT_4BIT);          /* some displays need this additional delay */
    
    /* from now the LCD only accepts 4 bit I/O, we can use lcd_command() */    
#endif
}/* lcd_init */