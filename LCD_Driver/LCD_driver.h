/*
 * LCD_driver.h
 *
 *  Created on: Aug 9, 2024
 *      Author: q
 */

#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_gpio.h"

/* bsp exposed apis */
void mdelay(uint32_t cnt);
void udelay(uint32_t cnt);
void write_4_bits(uint8_t value);
void lcd_send_command(uint8_t cmd);
void lcd_display_clear(void);
void lcd_init(void);
void lcd_print_char(uint8_t data);
void lcd_display_return_home(void);
void lcd_print_string(char*);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_enable(void);
void lcd_disable(void);
/*Application configurable items */


#define LCD_GPIO_PORTA    GPIOA
#define LCD_GPIO_PORTB    GPIOB
#define LCD_GPIO_PORTC    GPIOC
#define LCD_GPIO_RS       GPIO_PIN_9
#define LCD_GPIO_EN       GPIO_PIN_7
#define LCD_GPIO_D4       GPIO_PIN_5
#define LCD_GPIO_D5       GPIO_PIN_4
#define LCD_GPIO_D6       GPIO_PIN_10
#define LCD_GPIO_D7       GPIO_PIN_8

/*LCD commands */
#define LCD_CMD_4DL_2N_5X8F  		0x28
#define LCD_CMD_DON_CURON    		0x0E
#define LCD_CMD_INCADD       		0x06
#define LCD_CMD_DIS_CLEAR    		0X01
#define LCD_CMD_DIS_RETURN_HOME  	0x02
#define LCD_CMD_CURS_OFF            0x0C
#define LCD_CMD_CURS_RECT           0x0D
#define LCD_CMD_CURS_SECLINE        0xC0

void lcd_init(void);

#endif /* LCD_DRIVER_H_ */
