/*
 * LCD1602.h
 *
 *  Created on: May 23, 2025
 *      Author: rodri
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

void send_to_lcd(int data, int rs);
void lcd_send_cmd(char cmd);
void lcd_init(void);
void lcd_send_data(char data);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_send_string(char *str);


#endif /* INC_LCD1602_H_ */
