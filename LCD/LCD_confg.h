/*
 * LCD_confg.h
 *
 *  Created on: Aug 9, 2024
 *      Author: q
 */

#ifndef LCD_CONFG_H_
#define LCD_CONFG_H_
/* CONFIG FOR LIBRARY USER */
#define GPIO_PORTA GPIOA
#define GPIO_PORTB GPIOB
#define GPIO_PORTC GPIOC

//4 pin mode -> pins
#define DATA5_Pin GPIO_PIN_5   // B
#define DATA6_Pin GPIO_PIN_4
#define DATA7_Pin GPIO_PIN_10
#define DATA8_Pin GPIO_PIN_8   // A

#define RS_Pin GPIO_PIN_9    // A
#define E_Pin  GPIO_PIN_7    // C

//RW Pin not used,connect to GND

//if you want to work with 8 bit mode uncomment the area which is given below

/*
#define LCD8Bit
#define DATA1_Pin GPIO_PIN_1
#define DATA2_Pin GPIO_PIN_2
#define DATA3_Pin GPIO_PIN_3
#define DATA4_Pin GPIO_PIN_4
*/


#endif /* LCD_CONFG_H_ */
