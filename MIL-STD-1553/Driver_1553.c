/*
 * Driver_1553.c
 *
 *  Created on: Jun 9, 2024
 *      Author: q
 */
#include "stm32f4xx.h"
#include "Driver_1553.h"

MIL_StatusTypeDef MIL_Receive(MIL_Handle_struc *MIL_Addr)
{
	 uint8_t status = 0;

	 HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	 while(counter_bit_capture == 0);

	 status = MIL_TRANS;

	 return status;
}

void MIL_Handlig_Tx_data(MIL_Handle_struc *MIL_Addr, MIL_Command_Word *MIL_Comm, uint8_t* data, uint8_t size)
{

	uint8_t mach_word = 0;

	if(size == 0)
	{
		for(uint8_t i = 0; i < 5 ; i++)
		{
			MIL_Addr->coder_array[0][3 + i] = MIL_Comm->addr_rt[i];
		}

		for(uint8_t i = 0; i < 5 ; i++)
		{
			MIL_Addr->coder_array[0][9 + i] = MIL_Comm->subaddr_cl[i];
		}

		MIL_Addr->coder_array[0][8] =  MIL_Comm->wr;

//		for(uint8_t i = 0; i < 4 ; i++)
//		{
//			MIL_Addr->coder_array[0][14 + i] = (uint8_t)((mach_word >> (4 - i)) & 0x1) ;
//		}

		for(uint8_t i = 0; i < 5 ; i++)
		{
			MIL_Addr->coder_array[0][14 + i] =  MIL_Comm->n_com[i] ;
		}

		MIL_Addr->coder_array[0][19] = 1;

	} else{

		for(uint32_t i = 1; i < size; i += 2)
		{
			uint8_t data1[8];
			uint8_t data2[8];

			for(uint8_t t = 0; t < 8; t++)
			{
				 data1[t] = ((data[i - 1] >> (7 - t)) & 0x1);
			}

			for(uint8_t t = 0; t < 8; t++)
			{
				 data2[t] = ((data[i] >> (7 - t)) & 0x1);
			}

			for(uint8_t t = 3; t < 11; t++)
			{
				MIL_Addr->coder_array[(i + 1) / 2][t] = data1[t - 3];
			}

			for(uint8_t t = 11; t < 19; t++)
			{
				MIL_Addr->coder_array[(i + 1) / 2][t] = data2[t - 11];
			}

			mach_word++;
		}

		for(uint8_t i = 0; i < 5 ; i++)
		{
			MIL_Addr->coder_array[0][3 + i] = MIL_Comm->addr_rt[i];
		}

		for(uint8_t i = 0; i < 5 ; i++)
		{
			MIL_Addr->coder_array[0][9 + i] = MIL_Comm->subaddr_cl[i];
		}

		MIL_Addr->coder_array[0][8] =  MIL_Comm->wr;

		for(uint8_t i = 0; i < 4 ; i++)
		{
			MIL_Addr->coder_array[0][14 + i] = (uint8_t)((mach_word >> (4 - i)) & 0x1) ;
		}
	}


	 ManchesterEncode(MIL_Addr, mach_word);

}
void MIL_Transmit(MIL_Handle_struc *MIL_Addr)
{
	__disable_irq();
	TIM1->DIER &= ~(1 << 1);
	TIM1->DIER &= ~(1 << 2);
	TIM1->CCER &= ~(1 << 0);
	TIM1->CCER &= ~(1 << 4);
	GPIOC->ODR |= (1 << 8);
	TIM2->SR &= ~(1 << 0);
	TIM2->CNT = 5;
	for(uint32_t i = 0; i < MIL_Addr->cnt_bit; i++)
	{
		if(MIL_Addr->command_word_manch[i])
		{
			GPIOC->BSRR |= (1 << 21) | (1 << 6);
			while( !(TIM2->SR & TIM_SR_UIF) );
			TIM2->SR &= ~(1 << 0);

		}else{
			GPIOC->BSRR |= (1 << 22) | (1 << 5);
			while( !(TIM2->SR & TIM_SR_UIF) );
			TIM2->SR &= ~(1 << 0);
		}
	}
	GPIOC->ODR |= (1 << 6);
	GPIOC->ODR |= (1 << 5);
	GPIOC->ODR &= ~(1 << 8);
	 TIM1->SR &= ~(1 << 0);
	 TIM1->SR &= ~(1 << 1);
	 TIM1->SR &= ~(1 << 2);
	 TIM1->SR &= ~(1 << 6);
      while( !(GPIOC->IDR & (1 << 6)) || !(GPIOC->IDR & (1 << 5)) )
      {
    		 TIM1->SR &= ~(1 << 1);
    		 TIM1->SR &= ~(1 << 2);
      }
		TIM1->DIER |= (1 << 1);
		TIM1->DIER |= (1 << 2);
		TIM1->CCER |= (1 << 0);
		TIM1->CCER |= (1 << 4);
	__enable_irq();
	 GPIOC->ODR |= (1 << 11);
	 GPIOC->ODR &= ~(1 << 11);

}

void ManchesterEncode(MIL_Handle_struc *MIL_Addr, uint8_t NUM )
{
	MIL_Addr->flag_sync_c_TX = true;

	for(uint8_t i = 3; i < 20 ; i++)
	{

        if (MIL_Addr->coder_array[0][i])
        {
            // 1 -> 10
            MIL_Addr->command_word_manch[i * 2] = 1;
            MIL_Addr->command_word_manch[i * 2 + 1] = 0;
        }
        else
        {
            // 0 -> 01
            MIL_Addr->command_word_manch[i * 2] = 0;
            MIL_Addr->command_word_manch[i * 2 + 1] = 1;
        }

        MIL_Addr->cnt_bit++;

	}

	if(NUM != 0)
	{
		for (uint32_t t = 1; t < NUM; t++)
		{
		    // Вычисляем начальное смещение для каждой строки t
		    uint32_t base_idx = t * 40;  // Каждая строка 20 бит, значит 40 манчестерских бит

		    // Синхронизация (sync)
		    MIL_Addr->command_word_manch[base_idx] = 0;
		    MIL_Addr->command_word_manch[base_idx + 1] = 0;
		    MIL_Addr->command_word_manch[base_idx + 2] = 0;
		    MIL_Addr->command_word_manch[base_idx + 3] = 1;
		    MIL_Addr->command_word_manch[base_idx + 4] = 1;
		    MIL_Addr->command_word_manch[base_idx + 5] = 1;
		    MIL_Addr->cnt_bit += 3;

		    // Проходим по 20 битам данных
		    for (uint32_t i = 3; i < 20; i++)
		    {
		        uint32_t manchester_idx = base_idx + 2 * i;  // Синхронизация занимает первые 6 бит

		        // Генерация манчестерского кода для каждого бита coder_array
		        if (MIL_Addr->coder_array[t][i])
		        {
		            // 1 -> 10
		            MIL_Addr->command_word_manch[manchester_idx] = 1;
		            MIL_Addr->command_word_manch[manchester_idx + 1] = 0;
		        }
		        else
		        {
		            // 0 -> 01
		            MIL_Addr->command_word_manch[manchester_idx] = 0;
		            MIL_Addr->command_word_manch[manchester_idx + 1] = 1;
		        }

		        MIL_Addr->cnt_bit++;
		    }
		}

	}

	MIL_Addr->cnt_bit = MIL_Addr->cnt_bit * 2 + 6;

/*
    for (uint8_t i = 0; i < 20; i++) {
        if (MIL_Addr->MIL_RegDef_struct.command_word[i]) {
            // 1 -> 10
        	MIL_Addr->command_word_manch[2 * i] = 1;
        	MIL_Addr->command_word_manch[2 * i + 1] = 0;
        } else {
            // 0 -> 01
        	MIL_Addr->command_word_manch[2 * i] = 0;
        	MIL_Addr->command_word_manch[2 * i + 1] = 1;
        }
    }

    */
    if(MIL_Addr->flag_sync_c_TX == 1)
    {
        MIL_Addr->command_word_manch[0] = 1;
        MIL_Addr->command_word_manch[1] = 1;
        MIL_Addr->command_word_manch[2] = 1;
        MIL_Addr->command_word_manch[3] = 0;
        MIL_Addr->command_word_manch[4] = 0;
        MIL_Addr->command_word_manch[5] = 0;
    }
    if(MIL_Addr->flag_sync_d_TX == 1)
    {
        MIL_Addr->command_word_manch[0] = 0;
        MIL_Addr->command_word_manch[1] = 0;
        MIL_Addr->command_word_manch[2] = 0;
        MIL_Addr->command_word_manch[3] = 1;
        MIL_Addr->command_word_manch[4] = 1;
        MIL_Addr->command_word_manch[5] = 1;
    }


    MIL_Addr->flag_sync_c_RX = false;
    MIL_Addr->flag_sync_c_TX = false;
    MIL_Addr->flag_sync_d_RX = false;
    MIL_Addr->flag_sync_d_TX = false;
}

void ParityBit(MIL_Handle_struc *MIL_Addr)
{
	uint8_t count = 0;

	for(uint8_t i = 3; i < 19; i++)
	{
		if(MIL_Addr->MIL_RegDef_struct.command_word[i])
		{
			count++;
		}
	}

	 MIL_Addr->MIL_RegDef_struct.command_word[19] = ( (count % 2) == 0) ? 1 : 0 ;

//	MIL_Addr->p = MIL_Addr->MIL_RegDef_struct.command_word[19];
}

/**Заносит в массив командного слова команду управления.
*Эта функция вносит команды благодря поэлементному устанавлению байтов массива command_word, структуры
 MIL_Addr.
 *Param 1: MIL_Handle_struc - указатель на главную структуру Милла.
  */
/*
 00000   Принять управление интерфейсом
 00001   Синхронизация
 00010   Передать ОС
 00011   Начать самоконтроль ОУ
 00100   Блокировать передатчик
 00101   Разблокировать передатчик
 00110   Блокировать признак неисправности ОУ
 00111   Разблокировать признак неисправности
 01000   Установить ОУ в исходное состояние ОУ
 10000   Передать векторное слово
 10001   Синхронизация (с СД)
 10010   Передать последнюю команду
 10011   Передать слово ВС К ОУ
 10100   Блокировать /-Й передатчик
 10101   Разблокировать /-Й передатчик
 */
void MIL_command(MIL_Handle_struc *MIL_Addr, uint8_t command)
{
  uint8_t count = 14;
  uint8_t right_shift = 0;

   switch (command) {

   case 0b00000: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00001: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00010: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00011: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00100: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00101: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00110: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b00111: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b01000: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b10000: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b10001: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b10010: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b01011: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;
  case 0b10101: while(count < 18)
  {
  MIL_Addr->MIL_RegDef_struct.command_word[count] = ( (command >> right_shift) & 0x1);
  count++;
  right_shift++;
  }
  break;

  default:
  break;
 }
}


void ProcessCapture2(MIL_Handle_struc *MIL_Addr)
{
	 static uint8_t null_cnt = 0;
	 static uint8_t one_cnt = 0;

		   if(TIM1->SR & (1 << 1))
		   {
		   TIM1->CNT = 40;
	//	   TIM1->CNT = 60
	 	   while(null_cnt <= 6)
	 	   {

	 		   for(uint8_t i = 0; i < 8; i++)
	 		     {
		 		      while( ! (TIM1->SR & TIM_SR_UIF) );
		 		      TIM1->SR &= ~(1 << 0);
	 			      if(GPIOA->IDR & (1 << 8))
	 			      {
			 		    GPIOD->ODR |= (1 << 2);
			 		    MIL_Addr->array_receive_bit[counter_bit_capture] |= ( 1 << i);
			 		    GPIOD->ODR &= ~(1 << 2);
			 		    null_cnt = 0;
	 			       }
	 			       else{
	 			    	GPIOD->ODR |= (1 << 2);
			 		    MIL_Addr->array_receive_bit[counter_bit_capture] &= ~( 1 << i);
			 		    GPIOD->ODR &= ~(1 << 2);
			 		    null_cnt++;

	 			       }
		          }
	 		  counter_bit_capture += 1;
	 	       }
		     }

		   if(TIM1->SR & (1 << 2))
		   {
			       TIM1->CNT = 40;
	    	//	   TIM1->CNT = 60;
	    	 	   while(null_cnt <= 6)
	    	 	   {
	    	 		   for(uint8_t i = 0; i < 8; i++)
	    	 		     {
	    		 		      while( ! (TIM1->SR & TIM_SR_UIF) );
	    		 		      TIM1->SR &= ~(1 << 0);
	    	 			      if(GPIOA->IDR & (1 << 9))
	    	 			      {
	    	 			    	GPIOD->ODR |= (1 << 2);
	    			 		    MIL_Addr->array_receive_bit[counter_bit_capture] |= ( 1 << i);
	    			 		    GPIOD->ODR &= ~(1 << 2);
	    			 		    null_cnt = 0;
	    	 			       }
	    	 			       else{
	    	 			    	GPIOD->ODR |= (1 << 2);
	    			 		    MIL_Addr->array_receive_bit[counter_bit_capture] &= ~( 1 << i);
	    			 		    GPIOD->ODR &= ~(1 << 2);
	    			 		    null_cnt++;

	    	 			       }
	    		          }
	    	 		  counter_bit_capture += 1;
	    	 	    }
	         }

		one_cnt = 0;
	    null_cnt = 0;
	    num_byte = counter_bit_capture;
	    counter_bit_capture = 0;

}

void ManchesterDecoder(MIL_Handle_struc *MIL_Addr)
{

	uint32_t bite_MIL = 0;
	uint8_t word_MIL = 0;
	uint8_t cnt = 0;
	uint8_t ii = 0;
	bool Error_decod = 0;

	logic_decode_start = 0;

	MIL_Addr->data_word = 0;

		for(uint8_t t = 0; t < num_byte; t ++)
		{
			for(uint8_t i = 0; i < 8; i ++)
			{
				MIL_Addr->array_receive_byte[t * 8 + i] = (MIL_Addr->array_receive_bit[t] >> i & 0x1);
			}
		}

		if(  (MIL_Addr->array_receive_byte[0] == 1) && (MIL_Addr->array_receive_byte[1] == 1) && (MIL_Addr->array_receive_byte[2] == 0)
				&& (MIL_Addr->array_receive_byte[3] == 0) && (MIL_Addr->array_receive_byte[4] == 0) )
		{
			logic_decode_start = 5;
			for(uint8_t t = 5; t < 38; t += 2)
			{

						if ((MIL_Addr->array_receive_byte[t] == 1) && (MIL_Addr->array_receive_byte[t + 1] == 0)) {
				        MIL_Addr->decoder_array[0][(t - 4) / 2] = 1; // 10 -> 1
				         } else if ((MIL_Addr->array_receive_byte[t ] == 0) && (MIL_Addr->array_receive_byte[t + 1] == 1)) {
				        MIL_Addr->decoder_array[0][(t - 4) / 2] = 0; // 01 -> 0
				         }else{
				        	 MIL_Addr->decoder_array[0][(t - 4) / 2] = 2;
				         }
			}

		}else if( (MIL_Addr->array_receive_byte[0] == 1) && (MIL_Addr->array_receive_byte[1] == 0) && (MIL_Addr->array_receive_byte[2] == 0) && (MIL_Addr->array_receive_byte[3] == 0) )
		{
			logic_decode_start = 4;
			for(uint8_t t = 5; t < 38; t += 2)
			{

						if ((MIL_Addr->array_receive_byte[t] == 1) && (MIL_Addr->array_receive_byte[t + 1] == 0)) {
				        MIL_Addr->decoder_array[0][(t - 4) / 2] = 1; // 10 -> 1
				         } else if ((MIL_Addr->array_receive_byte[t ] == 0) && (MIL_Addr->array_receive_byte[t + 1] == 1)) {
				        MIL_Addr->decoder_array[0][(t - 4) / 2] = 0; // 01 -> 0
				         }else{
				        	 MIL_Addr->decoder_array[0][(t - 4) / 2] = 2;
				         }
			}


		}else{
			Error_decod = 1;
		}

		GPIOC->ODR |= (1 << 10);
		GPIOC->ODR &= ~(1 << 10);

		if(logic_decode_start == 5)
		{
			for(uint32_t i = 39; i < (8 * num_byte); i += 2)
			{
				if((MIL_Addr->array_receive_byte[i] == 0) && (MIL_Addr->array_receive_byte[i + 1] == 0) && (MIL_Addr->array_receive_byte[i + 2] == 0)
						&& (MIL_Addr->array_receive_byte[i + 3] == 1)&& (MIL_Addr->array_receive_byte[i + 4] == 1) && (MIL_Addr->array_receive_byte[i + 5] == 1))
				{
					ii = 0;
					i += 4;
					word_MIL++;
					MIL_Addr->data_word++;
				}else{
					if ((MIL_Addr->array_receive_byte[i] == 1) && (MIL_Addr->array_receive_byte[i + 1] == 0)) {
			        MIL_Addr->decoder_array[word_MIL][ii / 2] = 1; // 10 -> 1
			         } else if ((MIL_Addr->array_receive_byte[i] == 0) && (MIL_Addr->array_receive_byte[i + 1] == 1)) {
			        MIL_Addr->decoder_array[word_MIL][ii / 2] = 0; // 01 -> 0
			         }else{
			        	 MIL_Addr->decoder_array[word_MIL][ii / 2] = 2;
			         }
					ii += 2;
				}

			}
		}else if(logic_decode_start == 4)
		{
			for(uint32_t i = 38; i < (8 * num_byte); i += 2)
			{
				if((MIL_Addr->array_receive_byte[i] == 0) && (MIL_Addr->array_receive_byte[i + 1] == 0) && (MIL_Addr->array_receive_byte[i + 2] == 0)
						&& (MIL_Addr->array_receive_byte[i + 3] == 1)&& (MIL_Addr->array_receive_byte[i + 4] == 1) && (MIL_Addr->array_receive_byte[i + 5] == 1))
				{
					ii = 0;
					i += 4;
					word_MIL++;
					MIL_Addr->data_word++;
				}else{
					if ((MIL_Addr->array_receive_byte[i] == 1) && (MIL_Addr->array_receive_byte[i + 1] == 0)) {
			        MIL_Addr->decoder_array[word_MIL][ii / 2] = 1; // 10 -> 1
			         } else if ((MIL_Addr->array_receive_byte[i] == 0) && (MIL_Addr->array_receive_byte[i + 1] == 1)) {
			        MIL_Addr->decoder_array[word_MIL][ii / 2] = 0; // 01 -> 0
			         }else{
			        	 MIL_Addr->decoder_array[word_MIL][ii / 2] = 2;
			         }
					ii += 2;
			     }
			}
		}else{
			Error_decod = 1;
		}


		for(uint8_t i = 0; i < (MIL_Addr->data_word + 1); i++)
		{
			for( uint8_t t = 0; t < 16 ; t++)
			{
				if(MIL_Addr->decoder_array[i][(15-t)])
				{
					MIL_Addr->data_complit[i] |= (1 << t);
				}else{
					MIL_Addr->data_complit[i] &= ~(1 << t);
				}
			}
		}
			MIL_Addr->flag_sync_c_RX = true;


//	RXdata = BinaryArrayToDecimal((uint8_t*)&MIL_Addr->decoder_array, 17);


}

/* перевод из двоичного кода (эллементов массива) в число*/
uint32_t BinaryArrayToDecimal(const uint8_t* binaryArray, uint8_t length)
{
    uint32_t decimalValue = 0;

    for (uint8_t i = 0; i < length; i++) {
        decimalValue = (decimalValue << 1) | binaryArray[i];
    }

    return decimalValue;
}

/*Перевод из числа в двоичный массив*/
void DecimalToBinaryArray(uint32_t number, uint8_t* binaryArray, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        binaryArray[length - 1 - i] = (number >> i) & 0x1;
    }
}

void MILError(MIL_Handle_struc *MIL_Addr, MIL_Error *MIL_Error_)
{
	for( uint8_t i = 0; i < 17 ;i++ )
	{
		if(MIL_Addr->decoder_array[i] != init_values[i])
		{
			MIL_Error_->Error++;
		}
	}
	MIL_Error_->Message++;
}
