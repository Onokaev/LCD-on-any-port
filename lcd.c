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
