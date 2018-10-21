#include "stm32f10x.h"
#include "ds18b20.h"
#include "delay.h"

#define DelayMs(x) delay_ms(x)


#define	in  1
#define out 0

GPIO_InitTypeDef GPIO_InitStructure;



void DelayUs(u32 t)
{
    t*=16;
    while(t--);
}

void TRIS_PIN(char x)
{
    GPIO_InitStructure.GPIO_Pin = (1<<DQ_PIN);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    if(!x)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		//Output
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(DQ_GPIO, &GPIO_InitStructure);
}

void onewire_reset(void)
{
    TRIS_PIN(out);
    DQ_PIN_WRITE(0)
    DelayUs(480);
    TRIS_PIN(in);
    DelayUs(400);
    TRIS_PIN(out);
}

void onewire_write(char data)
{
    unsigned char i, bitshifter;
    bitshifter = 1;
    for (i=0; i<8; i++)
    {
        if (data & bitshifter)
        {
            TRIS_PIN(out);
            DQ_PIN_WRITE(0)
            DelayUs(3);
            TRIS_PIN(in);
            DelayUs(60);
        }
        else
        {
            TRIS_PIN(out);
            DQ_PIN_WRITE(0)
            DelayUs(60);
            TRIS_PIN(in);
        }
        bitshifter = bitshifter<<1;
    }
}

unsigned char onewire_read( void )
{
    unsigned char i;
    unsigned char data, bitshifter;
    data = 0;
    bitshifter = 1;
    for (i=0; i<8; i++)
    {
        TRIS_PIN(out);
        DQ_PIN_WRITE(0)
        DelayUs(6);
        TRIS_PIN(in);
        DelayUs(4);
        if (DQ_PIN_READ)
            data |= bitshifter;
        DelayUs(50);
        bitshifter = bitshifter<<1;
    }
    return data;
}

float ds18b20_read(void)
{
    int busy;
    int sayi1;
    int sayi2;
    float result;
    onewire_reset();
    onewire_write(0xCC);
    onewire_write(0x44);
    while(busy==0)
        busy=onewire_read();
    onewire_reset();
    onewire_write(0xCC);
    onewire_write(0xBE);
    sayi1 = onewire_read();
    sayi2 = onewire_read();
    result=sayi2*256+sayi1;
    result=result/16.0;
    DelayMs(250);
    return result;
}
