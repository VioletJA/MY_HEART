/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : max30102.c
 * @brief          : Ѫ��������
 ******************************************************************************
 * @attention
 * 1.Ҫ�궨�� ARM_MATH_CM7,__FPU_PRESENT
 * 2.��DSP
 * 3.main������������ȫ�ֱ���
 *	uint8_t max30102_int_flag = 0; // �жϱ�־
 *	float ppg_data_cache_RED[CACHE_NUMS] = {0}; // ������
 *	float ppg_data_cache_IR[CACHE_NUMS] = {0};  // ������
 *	uint16_t cache_counter = 0; // ���������
 * 
 ******************************************************************************
 */
/* USER CODE END Header */
#include "max30102.h"
#include "max30102_fir.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
extern uint8_t max30102_int_flag;
extern float ppg_data_cache_RED[CACHE_NUMS] ; // ������
extern float ppg_data_cache_IR[CACHE_NUMS] ;  // ������
extern uint16_t cache_counter;
/**
 * @brief IIC д��
 * @retval None
 */
void max30102_i2c_write(uint8_t reg_adder, uint8_t data)
{
	uint8_t transmit_data[2];
	transmit_data[0] = reg_adder;
	transmit_data[1] = data;
	HAL_StatusTypeDef state;
	state=i2c_transmit(transmit_data, 2);
    printf("%d\r\n",state);
	
}
 
/**
 * @brief IIC ��ȡ
 * @retval None
 */
void max30102_i2c_read(uint8_t reg_adder, uint8_t *pdata, uint8_t data_size)
{
	uint8_t adder = reg_adder;
	i2c_transmit(&adder, 1);
	i2c_receive(pdata, data_size);
}
 
/**
 * @brief max30102��ʼ��
 * @retval None
 */
void max30102_init(void)
{
	uint8_t data;
 
	max30102_i2c_write(MODE_CONFIGURATION, 0x40); // reset the device
 
	delay_ms(20);
 
	max30102_i2c_write(INTERRUPT_ENABLE1, 0xE0);
	max30102_i2c_write(INTERRUPT_ENABLE2, 0x00); // interrupt enable: FIFO almost full flag, new FIFO Data Ready,
												 //                    ambient light cancellation overflow, power ready flag,
												 //						    		internal temperature ready flag
 
	max30102_i2c_write(FIFO_WR_POINTER, 0x00);
	max30102_i2c_write(FIFO_OV_COUNTER, 0x00);
	max30102_i2c_write(FIFO_RD_POINTER, 0x00); // clear the pointer
 
	max30102_i2c_write(FIFO_CONFIGURATION, 0x4F); // FIFO configuration: sample averaging(1),FIFO rolls on full(0), FIFO almost full value(15 empty data samples when interrupt is issued)
 
	max30102_i2c_write(MODE_CONFIGURATION, 0x03); // MODE configuration:SpO2 mode
 
	max30102_i2c_write(SPO2_CONFIGURATION, 0x2A); // SpO2 configuration:ACD resolution:15.63pA,sample rate control:200Hz, LED pulse width:215 us
 
	max30102_i2c_write(LED1_PULSE_AMPLITUDE, 0x2f); // IR LED
	max30102_i2c_write(LED2_PULSE_AMPLITUDE, 0x2f); // RED LED current
 
	max30102_i2c_write(TEMPERATURE_CONFIG, 0x01); // temp
 
	max30102_i2c_read(INTERRUPT_STATUS1, &data, 1);
	max30102_i2c_read(INTERRUPT_STATUS2, &data, 1); // clear status
}
 
/**
 * @brief fifo����ȡ
 * @param output_data
 * @retval None
 */
void max30102_fifo_read(float *output_data)
{
	uint8_t receive_data[6];
	uint32_t data[2];
	max30102_i2c_read(FIFO_DATA, receive_data, 6);
	data[0] = ((receive_data[0] << 16 | receive_data[1] << 8 | receive_data[2]) & 0x03ffff);
	data[1] = ((receive_data[3] << 16 | receive_data[4] << 8 | receive_data[5]) & 0x03ffff);
	*output_data = data[0];
	*(output_data + 1) = data[1];
}
 
/**
 * @brief ��ȡ����
 * @param input_data cache_nums(���������������)
 * @retval (uint16_t)����
 */
uint16_t max30102_getHeartRate(float *input_data, uint16_t cache_nums)
{
	float input_data_sum_aver = 0;
	uint16_t i, temp;
 
	for (i = 0; i < cache_nums; i++)
	{
		input_data_sum_aver += *(input_data + i);
	}
	input_data_sum_aver = input_data_sum_aver / cache_nums;
	for (i = 0; i < cache_nums; i++)
	{
		if ((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
		{
			temp = i;
			break;
		}
	}
	i++;
	for (; i < cache_nums; i++)
	{
		if ((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
		{
			temp = i - temp;
			break;
		}
	}
	if ((temp > 14) && (temp < 100))
	{
		return 3000 / temp;
	}
	else
	{
		return 0;
	}
}
 
/**
 * @brief ��ȡѪ��
 * @param input_data red_input_data cache_nums(���������������)
 * @retval (float)Ѫ��
 */
float max30102_getSpO2(float *ir_input_data, float *red_input_data, uint16_t cache_nums)
{
	float ir_max = *ir_input_data, ir_min = *ir_input_data;
	float red_max = *red_input_data, red_min = *red_input_data;
	float R;
	uint16_t i;
	for (i = 1; i < cache_nums; i++)
	{
		if (ir_max < *(ir_input_data + i))
		{
			ir_max = *(ir_input_data + i);
		}
		if (ir_min > *(ir_input_data + i))
		{
			ir_min = *(ir_input_data + i);
		}
		if (red_max < *(red_input_data + i))
		{
			red_max = *(red_input_data + i);
		}
		if (red_min > *(red_input_data + i))
		{
			red_min = *(red_input_data + i);
		}
	}
 
	R = ((ir_max + ir_min) * (red_max - red_min)) / ((red_max + red_min) * (ir_max - ir_min));
	return ((-45.060) * R * R + 30.354 * R + 94.845);
}
 
/**
 * @brief MAX30102������
 * @param HeartRate(����) SpO2(Ѫ��) max30102_data fir_output
 * @retval (uint8_t)MAX30102_DATA_OK:������ȡ  (uint8_t)!MAX30102_DATA_OK:���ڶ�ȡ
 */
uint8_t MAX30102_Get_DATA(uint16_t *HeartRate,float *SpO2,float max30102_data[2],float fir_output[2])
{
	if (max30102_int_flag) // �ж��źŲ���
	{
		max30102_int_flag = 0;
		max30102_fifo_read(max30102_data); // ��ȡ����
		ir_max30102_fir(&max30102_data[0], &fir_output[0]);
		red_max30102_fir(&max30102_data[1], &fir_output[1]);  
		//printf("%f,%f\r\n",max30102_data[0],max30102_data[1]);		// �˲�
		if ((max30102_data[0] > PPG_DATA_THRESHOLD) && (max30102_data[1] > PPG_DATA_THRESHOLD)) // ������ֵ��˵���������нӴ�
		{
			taskENTER_CRITICAL();               // �����ٽ��� 
			touch_30102=1;//�ж���ָ����
			taskEXIT_CRITICAL();                //�˳��ٽ���
			ppg_data_cache_IR[cache_counter] = fir_output[0];
			ppg_data_cache_RED[cache_counter] = fir_output[1];
			cache_counter++;
		}
		else // С����ֵ
		{
			taskENTER_CRITICAL();               // �����ٽ��� 
			touch_30102=0;//�ж���ָ����
			taskEXIT_CRITICAL();                //�˳��ٽ���
			cache_counter = 0;
		}
		if (cache_counter >= CACHE_NUMS) // �ռ���������
		{
			*HeartRate = max30102_getHeartRate(ppg_data_cache_IR, CACHE_NUMS);
			*SpO2 = max30102_getSpO2(ppg_data_cache_IR, ppg_data_cache_RED, CACHE_NUMS);
			cache_counter = 0;
			return MAX30102_DATA_OK;
		}
	}
	return !MAX30102_DATA_OK;
}
 
/**
 * @brief MAX30102���������ⲿ�жϴ���
 * @param GPIO_Pin
 * @attention cubemx�����½��� ���� �����ж�
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MAX30102_INT_Pin)
  {
    max30102_int_flag = 1;
//	  printf("�ⲿ�жϴ����ɹ�");
  }
}
 

