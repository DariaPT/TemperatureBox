#include <hd44780.h>
#include <stm32f10x_gpio.h>

	void lcd_delay_ms(uint32_t t)
	{
		unsigned long int j = 0;
		unsigned long int T = t*7200;
		for(j = 0; j <= T; j++){};
	}
	
	void lcd_send_byte(uint8_t data, uint8_t com)
	{
		GPIO_ResetBits(LCD_PORT, RW);
		GPIO_SetBits(LCD_PORT, E);

		if(com == 1)
			GPIO_SetBits(LCD_PORT, RS);
			else GPIO_ResetBits(LCD_PORT, RS);

		if(data & 0x01)
			GPIO_SetBits(LCD_PORT, D11);
			else GPIO_ResetBits(LCD_PORT, D11);

		if(data & 0x02)
			GPIO_SetBits(LCD_PORT, D12);
			else GPIO_ResetBits(LCD_PORT, D12);

		if(data & 0x04)
			GPIO_SetBits(LCD_PORT, D13);
			else GPIO_ResetBits(LCD_PORT, D13);

		if(data & 0x08)
			GPIO_SetBits(LCD_PORT, D14);
			else GPIO_ResetBits(LCD_PORT, D14);

		lcd_delay_ms(2);
		GPIO_ResetBits(LCD_PORT, E);
		lcd_delay_ms(2);

	}

	//Initialization 0x02 0x02 0x08 0x00 0x0C 0x00 0x06 0x08 0x00

	void lcd_Initialization(void)	// Инициализация дисплея
	{
		lcd_delay_ms(100);
		// Определелие параметров развёртки
		lcd_send_byte(0x02, 0);
		lcd_send_byte(0x02, 0);
		lcd_send_byte(0x08, 0);
		// Режим отображения
		lcd_send_byte(0x00, 0);
		lcd_send_byte(0x0C, 0);
		// Сдвиг курсора или экрана
		lcd_send_byte(0x00, 0);
		lcd_send_byte(0x06, 0);
		// Адрес памяти счётчика DDRAM
		lcd_send_byte(0x08, 0);
		lcd_send_byte(0x00, 0);

	}
	void lcd_digit(char sign)
		{	uint16_t dig;
			lcd_delay_ms(2);
			
			switch (sign)
			{
				case ' ': dig = 0x20; break;
				case '!': dig = 0x21; break;
				case '"': dig = 0x22; break;
				case '#': dig = 0x23; break;
				case '$': dig = 0x24; break;
				case '%': dig = 0x25; break;
				case '&': dig = 0x26; break;
				case '`': dig = 0x27; break;
				case '(': dig = 0x28; break;
				case ')': dig = 0x29; break;
				//case '': dig = 0x2A; break;
				case '+': dig = 0x2B; break;
				case ',': dig = 0x2C; break;
				case '-': dig = 0x2D; break;
				case '.': dig = 0x2E; break;
				case '/': dig = 0x2F; break;
				case '0': dig = 0x30; break;
				case '1': dig = 0x31; break;
				case '2': dig = 0x32; break;
				case '3': dig = 0x33; break;
				case '4': dig = 0x34; break;
				case '5': dig = 0x35; break;
				case '6': dig = 0x36; break;
				case '7': dig = 0x37; break;
				case '8': dig = 0x38; break;
				case '9': dig = 0x39; break;
				case ':': dig = 0x3A; break;
				case ';': dig = 0x3B; break;
				case '<': dig = 0x3C; break;
				case '=': dig = 0x3D; break;
				case '>': dig = 0x3E; break;
				case '?': dig = 0x3F; break;
				case '@': dig = 0x40; break;
				case 'A': dig = 0x41; break;
				case 'B': dig = 0x42; break;
				case 'C': dig = 0x43; break;
				case 'D': dig = 0x44; break;
				case 'E': dig = 0x45; break;
				case 'F': dig = 0x46; break;
				case 'G': dig = 0x47; break;
				case 'H': dig = 0x48; break;
				case 'I': dig = 0x49; break;
				case 'J': dig = 0x4A; break;
				case 'K': dig = 0x4B; break;
				case 'L': dig = 0x4C; break;
				case 'M': dig = 0x4D; break;
				case 'N': dig = 0x4E; break;
				case 'O': dig = 0x4F; break;
				case 'P': dig = 0x50; break;
				case 'Q': dig = 0x51; break;
				case 'R': dig = 0x52; break;
				case 'S': dig = 0x53; break;
				case 'T': dig = 0x54; break;
				case 'U': dig = 0x55; break;
				case 'V': dig = 0x56; break;
				case 'W': dig = 0x57; break;
				case 'X': dig = 0x57; break;
				case 'Y': dig = 0x59; break;
				case 'Z': dig = 0x5A; break;
				case '[': dig = 0x5B; break;
				case ']': dig = 0x5D; break;
				case '^': dig = 0x5E; break;
				case '_': dig = 0x5F; break;
				case 'a': dig = 0x61; break;
				case 'b': dig = 0x62; break;
				case 'c': dig = 0x63; break;
				case 'd': dig = 0x64; break;
				case 'e': dig = 0x65; break;
				case 'f': dig = 0x66; break;
				case 'g': dig = 0x67; break;
				case 'h': dig = 0x68; break;
				case 'i': dig = 0x69; break;
				case 'j': dig = 0x6A; break;
				case 'k': dig = 0x6B; break;
				case 'l': dig = 0x6C; break;
				case 'm': dig = 0x6D; break;
				case 'n': dig = 0x6E; break;
				case 'o': dig = 0x6F; break;
				case 'p': dig = 0x70; break;
				case 'q': dig = 0x71; break;
				case 'r': dig = 0x72; break;
				case 's': dig = 0x73; break;
				case 't': dig = 0x74; break;
				case 'u': dig = 0x75; break;
				case 'v': dig = 0x76; break;
				case 'w': dig = 0x77; break;
				case 'x': dig = 0x78; break;
				case 'y': dig = 0x79; break;
				case 'z': dig = 0x7A; break;

			}


			lcd_send_byte(dig>>4, 1);
			lcd_send_byte(dig & 0x0F, 1);
		}	
	void lcd_wright(char stroka[20])	// Запись в экран
	{
		uint8_t i = 0;
		lcd_delay_ms(2);
		while(stroka[i])
		{
			lcd_digit(stroka[i]);
			i++;
		}
		i = 0;
		lcd_delay_ms(2);
	}
	
	void lcd_clear()	// Очистка дисплея
	{
		lcd_delay_ms(2);
		lcd_send_byte(0x00, 0);
		lcd_send_byte(0x01, 0);
	}
	
	void lcd_position(uint8_t x, uint8_t y)	// Установка позиции курсора
	{
		int p = 0;

		lcd_delay_ms(2);


		if(y == 2)
		{
			p = 0xC0 + x;
			lcd_send_byte(p>>4, 0);
			lcd_send_byte(p & 0x0F, 0);
		}

		if(y == 1)
		{
			lcd_send_byte(0x08, 0);
			lcd_send_byte(x, 0);
		}
		
	}
	
	void lcd_frame_position(uint8_t y)	// Установка позиции экрана курсора
	{
		lcd_delay_ms(10);
		lcd_send_byte(0x01, 0);
		lcd_send_byte(0x0C<<y, 0);
	}
	
	





