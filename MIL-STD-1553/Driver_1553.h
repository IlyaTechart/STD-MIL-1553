/*
 * Driver_1553.h
 *
 *  Created on: Jun 9, 2024
 *      Author: q
 */

#ifndef DRIVER_1553_H_
#define DRIVER_1553_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#define MIL_BASEADDR  (MIL_RegDef_t*)0x20000032
#define MIL_BASEADDR_2  (MIL_Handle_struc*)0x20000200

#define SYNC_C
#define SYNC_D

#define ABS(x) (((x) > 0) ? (x) : -(x))

extern uint32_t signal_index;
extern uint8_t init_values[17];

extern bool flag123;

extern uint32_t counter_bit_capture;
extern uint32_t num_byte;
extern uint8_t logic_decode_start;
typedef struct
{
	union
	{
		uint8_t command_word[680];

	}MIL_RegDef_struct;

	 uint8_t coder_array[31][20];
	 uint8_t command_word_manch[1360];
	 uint32_t cnt_bit;
	 uint8_t array_receive_bit[170];
	 uint8_t array_receive_byte[1360];
	 uint8_t decoder_array[31][17];
	 uint16_t data_complit[31];
	 uint8_t data_word;
	 uint8_t sync_tx[7];
	 uint8_t sync_rx[7];
	 bool flag_sync_c_TX;
	 bool flag_sync_c_RX;
	 bool flag_sync_d_TX;
	 bool flag_sync_d_RX;

}MIL_Handle_struc;

typedef struct
{
	__IO uint32_t Message;
	__IO uint32_t Error;
}MIL_Error;

typedef struct
{
	__IO uint8_t type_device;

}MIL_Config_Device;

typedef struct
{
    __IO uint8_t sync_c[3]    ;
	__IO uint8_t addr_rt[5]   ;
	__IO uint8_t wr           ;
	__IO uint8_t subaddr_cl[5];
	__IO uint8_t n_com[5]     ;
	__IO uint8_t p            ;

}MIL_Command_Word;

typedef struct
{
    __IO uint8_t sync_d[3]    ;
    __IO uint8_t data[16]     ;
	__IO uint8_t p            ;

}MIL_Data_Word;

typedef struct
{
    __IO uint8_t sync_c[3]    ;
	__IO uint8_t addr_rt[5]   ;
	__IO uint8_t a            ;
	__IO uint8_t b            ;
	__IO uint8_t c            ;
	__IO uint8_t x[3]         ;
	__IO uint8_t d            ;
	__IO uint8_t e            ;
	__IO uint8_t f            ;
	__IO uint8_t g            ;
	__IO uint8_t h            ;
	__IO uint8_t p            ;

}MIL_Response_Word;

typedef enum
{
	MIL_TRANS   =  0x00,
	MIL_NO_TRANS = 0x01

}MIL_StatusTypeDef;

MIL_StatusTypeDef MIL_Receive(MIL_Handle_struc *MIL_Addr);
void MIL_Handlig_Tx_data(MIL_Handle_struc *MIL_Addr, MIL_Command_Word *MIL_Comm, uint8_t* data, uint8_t size);
void MIL_Transmit(MIL_Handle_struc *MIL_Addr);
void ManchesterEncode(MIL_Handle_struc *MIL_Addr, uint8_t NUM );
void ParityBit(MIL_Handle_struc *MIL_Addr);
void MIL_command(MIL_Handle_struc *MIL_Addr, uint8_t command);
void ProcessCapture2(MIL_Handle_struc *MIL_Addr);
void ManchesterDecoder(MIL_Handle_struc *MIL_Addr);
uint32_t BinaryArrayToDecimal(const uint8_t* binaryArray, uint8_t length);
void DecimalToBinaryArray(uint32_t number, uint8_t* binaryArray, size_t length);
void MILError(MIL_Handle_struc *MIL_Addr, MIL_Error *MIL_Error_);
#endif /* DRIVER_1553_H_ */
