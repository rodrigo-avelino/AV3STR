#include "stdint.h"
#include "tim.h"
#include "stm32f4xx_hal.h"


void send_to_lcd(int data, int rs){
	// RS pin (PA9)
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, rs ? GPIO_PIN_SET : GPIO_PIN_RESET);

	    // Data pins
	    // D7 -> PA8
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, ((data >> 3) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	    // D6 -> PB10
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, ((data >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	    // D5 -> PB4
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, ((data >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	    // D4 -> PB5
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, ((data >> 0) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	    // Enable pulse (E pin -> PC7)
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	    delayLCD(100);  // You can also use HAL_Delay(1) for milliseconds

	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	    delayLCD(100);
}

void lcd_send_cmd(char cmd){
	char datatosend;
	datatosend = ((cmd>>4)&0x0f);
	send_to_lcd(datatosend,0);

	datatosend = ((cmd)&0x0f);
	send_to_lcd(datatosend, 0);
}
void lcd_init(void){
	delay_ms(50);
	lcd_send_cmd(0x30);
	delay_ms(5);
	lcd_send_cmd(0x30);
	delay_ms(1);
	lcd_send_cmd(0x30);
	delay_ms(1);
	lcd_send_cmd(0x20);
	delay_ms(1);

	lcd_send_cmd(0x28);
	delay_ms(50);
	lcd_send_cmd(0x08);
	delay_ms(50);
	lcd_send_cmd(0x01);
	delay_ms(50);
	delay_ms(50);
	lcd_send_cmd(0x06);
	delay_ms(50);
	lcd_send_cmd(0x0C);
}

void lcd_send_data(char data){
	char datatosend;

	datatosend = ((data>>4)&0x0f);
	send_to_lcd(datatosend,1);

	datatosend = ((data)&0x0f);
	send_to_lcd(datatosend,1);
}

void lcd_clear (void){
	lcd_send_cmd(0x01);
	delay_ms(100);
}

void lcd_put_cur(int row, int col){
	switch(row){
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd(col);
}

void lcd_send_string(char *str){
	while (*str) lcd_send_data(*str++);
}


