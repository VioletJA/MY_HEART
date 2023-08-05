#ifndef __WS2812B_H__
#define __WS2812B_H__
 
#include "main.h"
 
/*���������ļ�������CCR�ĺ궨��*/
#define CODE_1       (58)       //1�붨ʱ����������
#define CODE_0       (25)       //0�붨ʱ����������
 
/*����һ�����嵥��LED��ԭɫֵ��С�Ľṹ��*/
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color_TypeDef;
 
#define Pixel_NUM 17  //LED�����궨�壬������ʹ��һ��LED��������pixelΪ���ص���˼��
 
/*Some Static Colors------------------------------*/
extern RGB_Color_TypeDef RED;   //��ʾ��ɫRGB����
extern RGB_Color_TypeDef ORANGE;
extern RGB_Color_TypeDef YELLOW;
extern RGB_Color_TypeDef GREEN;
extern RGB_Color_TypeDef CYAN;
extern RGB_Color_TypeDef BLUE;
extern RGB_Color_TypeDef PURPLE;
extern RGB_Color_TypeDef BLACK;
extern RGB_Color_TypeDef WHITE;
extern RGB_Color_TypeDef MAGENTA ;

void RGB_SetColor(uint8_t LedId,RGB_Color_TypeDef Color);//��һ��LEDװ��24����ɫ�����루0���1�룩
void Reset_Load(void); //�ú������ڽ��������24�����ݱ�Ϊ0������RESET_code
void RGB_SendArray(void);          //������������
void RGB_RED(uint16_t Pixel_Len);  //��ʾ���
void RGB_GREEN(uint16_t Pixel_Len);//��ʾ�̵�
void RGB_BLUE(uint16_t Pixel_Len); //��ʾ����
void RGB_WHITE(uint16_t Pixel_Len);//��ʾ�׵�
 
void led_loop(void);
void rgb_show(uint32_t Pixel_Len, RGB_Color_TypeDef rgb);
void rainbowCycle(uint8_t wait);
#endif
 

