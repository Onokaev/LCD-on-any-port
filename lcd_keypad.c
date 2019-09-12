#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#define pass(void);
#include <avr/interrupt.h>

//buttons
#define pass(void);
#define BIT_IS_SET(byte, bit) (byte & (1 << bit))
#define BIT_IS_UNSET(byte, bit) (!(byte & (1 << bit)))

volatile uint8_t flag1 = 0;
volatile uint8_t flag2 = 0;


//LCD
#define LCD_Dir DDRD					/* Define LCD data port direction */
#define LCD_Port PORTD					/* Define LCD data port */
#define RS PC1							/* Define Register Select (data reg./command reg.) signal pin */
#define EN PC0 							/* Define Enable signal pin */
const int delay_scan = 60;


//4 by 3 keypad
#define KP_R0 PC4
#define KP_R1 PB1
#define KP_R2 PB2
#define KP_R3 PB4

#define KP_C0 PB0
#define KP_C1 PB5
#define KP_C2 PB3


unsigned char keypad_read(void);
unsigned char keypad_wait(void);
void keying_in(void);

//lcd
void LCD_Command( unsigned volatile char cmnd );
void LCD_Char( unsigned volatile char data );
void LCD_Init (void);
void LCD_String (char *str);
void LCD_String_xy (char row, char pos, char *str);
void LCD_Clear(void);
void LCD_Welcome(void);
void itoa(int _val, char* _s, int _radix);

void intialization(void);


int main(void)
{
	intialization();
	LCD_Init();
	LCD_Welcome();
	
	while(1)
	{	
		if (flag1 == 1 && flag2 == 0)
		{
			LCD_Init();
			PORTD |= (1 << PORTD6);
			keying_in();		
		}
		
		else if (flag1 == 1 && flag2 ==1 )
		{
			LCD_Clear();
			_delay_ms(100);
			LCD_String(" Cleared ");
			PORTB |=  (1 << PORTB0) | (1 << PORTB5) | (1 << PORTB3);
	//		PORTD &= ~(1 << PORTD6);
			flag2 = 0;
			_delay_ms(500);
			LCD_Clear();
			keying_in();
		}	
	}
}

void intialization(void)
{
	DDRB |= (1 << DDB0) | (1 << DDB3) | (1 << DDB5);   // columns of the keypad are outputs
	DDRD |= 0;
	DDRD |= (1 << DDD6);   //led pin is an output
	DDRC |= 0;           //active low and active high buttons are connected here as inputs
	
	PORTB |=  (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB4);            //rows of the keypad. they are maintained high. when a key is pressed, a row is connected to ground through the column
	PORTD |= (1 << PORTD7);
	DDRC |= (1 << DDC0) | (1 << DDC1);    // register select and enable pin of the lcd
	
	//setting up interrupt for the buttons
	PORTD &= ~(1 << PORTD6);              //ensure led is low
	sei();
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);         //enable pin change interrupt on pc2 and pc3	
}

void keying_in(void)
{
	char snum[10];
	
	while(flag2 != 1) 
	{
		int key = keypad_wait();
				
		if (key	 == '*')
		{
			LCD_String("*");
		}
		else if (key == '#')
		{
			LCD_String("#");
		}
		else
		{
			itoa(key, snum, 10);  //10 is base which in this case is decimal
			LCD_String(snum);
		}
				
	}
	LCD_Clear();
	
}

void LCD_Welcome(void)
{
	int shift = 1;
	for (int i = 0; i< shift ; i++)
	{
		LCD_String("  OCTRINSIC!!  ");
		_delay_ms(600);
		LCD_Clear();
		_delay_ms(300);
	}
	
	LCD_String("1.Press Button 1 ! ");
	LCD_Command(0xC0);
	_delay_ms(400);
	LCD_String("2.Press keypad ");
	_delay_ms(1500);
	LCD_Clear();
	LCD_String(" to enter values ");
	_delay_ms(1000);
	LCD_Clear();	
	
	
	char play[4];
	
	for (int i = 3; i != 0; i-- )
	{
		itoa(i, play, 10);
		LCD_String(play);
		_delay_ms(800);
		LCD_Clear();
	}
	LCD_String(" Go!! ");
	
}

unsigned char keypad_read(void)
{
	//start of the scanning process
	_delay_ms(20);
	PORTB &= ~(1 << PORTB0);
 	PORTB |= (1 << PORTB5);
 	PORTB |= (1 << PORTB3);
	
	_delay_ms(delay_scan);
 	if (!(PIND & (1 << PIND7))) return 1;   //key 1 is pressed
 	if (!(PINB & (1 << PINB1))) return 4;   //key 4 pressed
 	if (!(PINB & (1 << PINB2))) return 7;   //key 7 pressed
 	if (!(PINB & (1 << PINB4))) return '*';//key * pressed. Sort this. It is an issue
	
	
	
	PORTB |= (1 << PORTB0);
	PORTB &= ~(1 << PORTB5);
	PORTB |= (1 << PORTB3);
	_delay_ms(delay_scan);
	if (!(PIND & (1 << PIND7))) return 2;   //key 1 is pressed
    if (!(PINB & (1 << PINB1))) return 5;   //key 4 pressed
	if (!(PINB & (1 << PINB2))) return 8;   //key 7 pressed
	if (!(PINB & (1 << PINB4))) return 0; //key * pressed. Sort this. It is an issue
	
	
	
 	PORTB |= (1 << PORTB0);
 	PORTB |= (1 << PORTB5);
	PORTB &= ~(1 << PORTB3);
	_delay_ms(delay_scan);
	if (!(PIND & (1 << PIND7))) return 3;   //key 1 is pressed
	if (!(PINB & (1 << PINB1))) return 6;   //key 4 pressed
	if (!(PINB & (1 << PINB2))) return 9;   //key 7 pressed
	if (!(PINB & (1 << PINB4))) return '#'; //key * pressed. Sort this. It is an issue
		
	else	 
	return 0xFF;
}

unsigned char keypad_wait(void)
{
	unsigned char c_pressed_key = 0xFF;
	
	do 
	{
		c_pressed_key = keypad_read();
	} while (c_pressed_key == 0xFF);
	
	while(keypad_read() != 0xFF)
	{
		return c_pressed_key;
	}
	return c_pressed_key;
	
}


//interrupt service routine for the active low and active high buttons
ISR(PCINT1_vect)
{
	if (!(PINC & (1 << PINC2)))
	{
		flag1 = 1;
		return;
	}

	
	if (PINC & (1 << PINC3))
	{
		flag2 = 1;
		return;
	}
	
}


void LCD_Command( unsigned volatile char cmnd )
{

	LCD_Port = (LCD_Port & 0xF0) | ((cmnd & 0xF0) >> 4); /* sending upper nibble */
	PORTC &= ~ (1<<RS);				/* RS=0, command reg. */
	PORTC |= (1<<EN);				/* Enable pulse */
	_delay_us(1);
	PORTC &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0xF0) | (cmnd & 0x0F);  /* sending lower nibble */
	PORTC |= (1<<EN);
	_delay_us(1);
	PORTC &= ~ (1<<EN);
	_delay_ms(2);
}


void LCD_Char( unsigned volatile char data )
{
	LCD_Port = (LCD_Port & 0xF0) | ((data & 0xF0) >> 4); /* sending upper nibble */
	PORTC |= (1<<RS);				/* RS=1, data reg. */
	PORTC |= (1<<EN);
	_delay_us(1);
	PORTC &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0xF0) | (data & 0x0F); /* sending lower nibble */
	PORTC |= (1<<EN);
	_delay_us(1);
	PORTC &= ~ (1<<EN);
	_delay_ms(2);
}


void LCD_Init (void)					/* LCD Initialize function */
{
	LCD_Dir |= 0x0F;						/* Make LCD command port direction as o/p */
	_delay_ms(20);						/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x33);
	LCD_Command(0x32);		    		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
	LCD_Command(0x0c);              	/* Display on cursor off*/
	LCD_Command(0x06);              	/* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}


void LCD_String (char *str)				/* Send string to LCD function */
{
	int i;
	
	for(i=0; str[i]!=0 ;i++)				/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
			
}


void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);		/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);		/* Command of first row and required position<16 */
	LCD_String(str);					/* Call LCD string function */
}

void LCD_Clear(void)
{
	LCD_Command (0x01);					/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}
