/*
  MC		LCD
PORTx0	   4 - RS
PORTx1	   5 - R/W
PORTx2	   6 - E
PORTx3 
PORTx4	   11 - D4
PORTx5     12 - D5
PORTx6     13 - D6
PORTx7     14 - D7
*/
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#define LCD_PORT GPIOB
#define RS  GPIO_Pin_3
#define RW  GPIO_Pin_4
#define E   GPIO_Pin_6
#define D11 GPIO_Pin_7
#define D12 GPIO_Pin_8
#define D13 GPIO_Pin_9
#define D14 GPIO_Pin_10

	void lcd_Initialization(void);						// Инициализация дисплея
	void lcd_wright(char stroka[20]);					// Запись в экран
	void lcd_clear();									// Очистка дисплея
	void lcd_position(uint8_t x, uint8_t y);	// Установка позиции курсора
	void lcd_frame_position(uint8_t y);	// Установка позиции курсора
	void lcd_delay_ms(uint32_t t);
	void lcd_digit(char sign);
	void lcd_send_byte(uint8_t data, uint8_t com);
