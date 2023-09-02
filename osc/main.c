/*
 * main.c
 *
 *  Created on: 2023��6��20��
 *      Author: Ming
 */
#include "init.h"
#include "spi.h"
#include <string.h>
#include "ad9958_hps.h"
// #include "uart.h"

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

extern void *periph_virtual_base;
extern uint32_t *led_pio_virtual_base;
extern uint32_t *button_pio_virtual_base;
extern uint32_t *sw_pio_virtual_base;
extern uint32_t *uart_0_virtual_base;
extern uint32_t *spi_0_virtual_base;
extern uint32_t *pio0_virtual_base;
extern uint16_t *ram_0_virtual_base; // AD data ,256
extern uint32_t *ram_1_virtual_base; // control reg, 16

void uart_putc(char c);
void uart_printf(char *str);
int uart_getc(uint32_t timeout);
int uart_scanf(char *p);
void read_ad();
void set_volt_scale(uint8_t scale); // 控制继电器换挡
void set_time_scale(uint8_t scale);
bool get_Vpp();
void auto_set();
void save();
void load();
void get_state();
void send_data(uint8_t* data);
void update(volt,time);
void single();

int trigger = 0;     // 触发阈值
uint32_t freq = 0;   // 测频
int sample_time = 0; // 采样间隔 (sample_time+1)*5ns
uint8_t time_scale, volt_scale;
uint16_t ADdata[404];
uint8_t points[404];
uint8_t points_buf[404];
float Vpp;
bool stop;
char Uart_begin[] = "addt s0.id,0,400\xff\xff\xff\0";
char Uart_end[] = "\x01\xff\xff\xff\0";
uint32_t pio0 = 0;

void uart_putc(char c)
{
    unsigned short uart_status; // 状态寄存器值
    do
    {
        uart_status = *(uart_0_virtual_base + 2); // 读取状态寄存器
    } while (!(uart_status & 0x40));              // 等待状态寄存器bit6（trdy）为1
    *(uart_0_virtual_base + 1) = c;               // 发送一个字符
    //delay_us(100);
}

// 串口字符串发送函数
void uart_printf(char *str)
{
    while (*str != '\0') // 检测当前指针指向的数是否为空字符
    {
        uart_putc(*str); // 发送一个字符
        str++;           // 字符串指针+1
    }
}
int uart_clr(void)
{
    return *(uart_0_virtual_base + 0);            // 读取一个字符并作为函数返回值返回
}
int uart_getc(uint32_t timeout)
{
    uint32_t _time=0;
    unsigned short uart_status; // 状态寄存器值
    do
    {
        _time++;
        uart_status = *(uart_0_virtual_base + 2); // 读取状态寄存器
        if(!(_time^timeout))return 256;
    } while ( !(uart_status & 0x80) );              // 等待状态寄存器bit7（rrdy）为1
    return *(uart_0_virtual_base + 0);            // 读取一个字符并作为函数返回值返回
}

int uart_scanf(char *p)
{
    int cnt = 0; // 接收个数计数器
    while (1)
    {
        *p = uart_getc(214748364); // 读取一个字符的数据
        cnt++;
        if (*p == '\n') // 判断数据是否为换行
            return cnt; // 换行则停止计数,返回当前接收的字符个数
        else
            p++; // 接收指针增 1
    }
}

void read_ad()
{
    while (*pio0_virtual_base & 0x01)
        // printf("%hhd", *pio0_virtual_base)
        ;
    pio0 |= (1 << 1);
    *pio0_virtual_base = pio0;
    delay_us(1);
    for (int i = 2; i < 402; i++)
        ADdata[i] = *(uint16_t *)(ram_0_virtual_base + i);
    pio0 &= ~(1 << 1);
    *pio0_virtual_base = pio0;

    float ratio;
    int offset;
    if (volt_scale == 2)
    { 
        float a=165,b=100;
        for (int i = 2; i < 402; i++)
            points[i]=(1.2207*ADdata[i]-2500+b)*16/a+128;
    }
    else if(volt_scale==1)
    { 
        for (int i = 2; i < 402; i++)
            points[i] = ADdata[i]/9.9-75.84;
    }
    else
    { 
        for (int i = 2; i < 402; i++)
            points[i] = 0.085*ADdata[i]-170+128;
    }

}

void set_volt_scale(uint8_t scale) // 控制继电器换挡
{
    volt_scale = scale;
    if (scale == 0) // x1   1V/div
    {
        pio0 |= (1 << 25);  // OUT25=1
        pio0 &= ~(1 << 26); // OUT26=0
    }
    else if (scale == 1) // x10   0.1V/div
    {
        pio0 &= ~(1 << 25); // OUT25=0
        pio0 |= (1 << 26);  // OUT26=1
    }
    else if (scale == 2) // x200  2mv/div
    {
        pio0 &= ~(1 << 25); // OUT26=0
        pio0 &= ~(1 << 26); // OUT26=0
    }
    *pio0_virtual_base = pio0;
}

void set_time_scale(uint8_t scale)
{
    time_scale = scale;
    if (scale == 0) //   20ms/div, 1khz
    {
        sample_time = 199999;
    }
    else if (scale == 1) //   2ms/div, 10khz
    {
        sample_time = 19999;
    }
    else if (scale == 2) //   200us/div, 100khz
    {
        sample_time = 1999;
    }
    else if (scale == 3) //   20us/div, 1Mhz
    {
        sample_time = 199;
    }
    else if (scale == 4) //   2us/div, 10Mhz
    {
        freq = *(ram_1_virtual_base + 0);
        int x = (0.000000995 * freq + 0.999);
        sample_time = (200000000.0 * x / freq) + 0.999 + 19;
    }
    else if (scale == 5) //  100ns/div, 200Mhz
    {
        freq = *(ram_1_virtual_base + 0);
        int x = (0.000000995 * freq + 0.999);
        sample_time = (200000000.0 * x / freq) + 0.999;
    }
    *(ram_1_virtual_base + 1) = sample_time;
}

bool get_Vpp()
{
    uint8_t cnt = 0;
    uint16_t _min = 4095;
    uint16_t _max = 0;
    read_ad();
    for (int i = 2; i < 402; i++)
    {
        _min = min(_min, ADdata[i]);
        _max = max(_max, ADdata[i]);
        if ((ADdata[i] < 800 || ADdata[i] > 3300)&&volt_scale==2 )
            cnt++;
        else if ((ADdata[i] < 600 || ADdata[i] > 3300)&&volt_scale==1 )
            cnt++;
    }
    //printf("cnt=%d,,%d,%d,%d\n", cnt, _min, _max, volt_scale);
    if (cnt > 10)
        return true;
    return false;
}
void single()
{
    stop=true;
}
void auto_set()
{
    stop=false;
    int cycle = 1000000 / freq + 10;
    if (freq < 50)
    {
        set_time_scale(0);
        delay_us(cycle);
    }
    else if (freq < 500)
    {
        set_time_scale(1);
        delay_us(cycle);
    }
    else if (freq < 5000)
    {
        set_time_scale(2);
        delay_us(cycle);
    }
    else if (freq < 50000)
    {
        set_time_scale(3);
        delay_us(cycle);
    }
    else if (freq < 500000)
    {
        set_time_scale(4);
        delay_us(cycle);
    }
    else
    {
        set_time_scale(5);
        delay_us(cycle);
    }
    set_volt_scale(2);
    delay_us(100000);
    if (get_Vpp())
    {
        set_volt_scale(1);
        delay_us(100000);
    }
    if (get_Vpp())
    {
        set_volt_scale(0);
        delay_us(100000);
    }
    update(volt_scale,time_scale);
}


void save()
{
    for(int i=2;i<402;i++)
        points_buf[i]=points[i];
    points_buf[402]=volt_scale;
    points_buf[403]=time_scale;
}
void load()
{
    stop=true;
    delay_us(10000);
    send_data(&points_buf);
    // for(int i=2;i<402;i++)
    //     points[i]=points_buf[i];
    update(points_buf[402],points_buf[403]);
}
void update(volt,time)
{
    char str[20] = "";


    int n;
    if(volt==0)
        n=4;
    else if(volt==1)
        n=3;
    else 
        n=7;
    delay_us(10000);
    sprintf(str,"click b%d,0\xff\xff\xff\0", n); // n=4
    delay_us(10000);
    uart_printf(str);
    if(time==0)
        n=9;
    else if(time==1)
        n=10;
    else if(time==2)
        n=11;
    else if(time==3)
        n=12;
    else if(time==4)
        n=13;
    else 
        n=14;
    delay_us(10000);
    sprintf(str,"click b%d,0\xff\xff\xff\0", n); // n=4
    delay_us(10000);
    uart_printf(str);
}
void get_state()
{
    int16_t state[4];
    uart_clr();
    delay_us(10000);
    char str[]="click b6,0\xff\xff\xff\0";
    uart_printf(str);
    //delay_us(100);


    state[0]=uart_getc(1000000);
    state[1]=uart_getc(1000000);
    state[2]=uart_getc(1000000);
    state[3]=uart_getc(1000000);
    printf("state:%x,%x,%x,%x\n",state[0],state[1],state[2],state[3]);
    delay_us(30);
    if(state[0]==0x39)volt_scale=0;
    else if(state[0]==0x38)volt_scale=1;
    else if(state[0]==0x40)volt_scale=2;

    if(state[2]!=256)time_scale=state[2];
    if(state[3]==0x35)save();
    if(state[3]==0x36)load();
    if(state[3]==0x37)single();
    if(state[3]==0x38)auto_set();
    if(state[3]==0x40)
        {stop=!stop;update(volt_scale,time_scale);}
    set_volt_scale(volt_scale);
    set_time_scale(time_scale);

    if(state[1]==256)return;
    if(state[1]==0){trigger=65535;return;}
    if (volt_scale == 2)
        { trigger=((state[1]-128)*165/16.0+2400)/1.2207; }
    else if(volt_scale==1)
        { trigger=(state[1]+76)*9.9; }
    else
        { trigger=(state[1]+42)/0.085; }
    
}
void send_data(uint8_t* data)
{
    delay_us(10000);
    uart_printf(Uart_begin);
    delay_us(10000);
    for (int i = 2; i < 402; i++)
        uart_putc(data[i]);
    delay_us(5000);
    uart_printf(Uart_end);
    delay_us(10000);
}
int main()
{
    //----Initialize, map virtual address----
    fpga_init();
    *(pio0_virtual_base + 1) &= ~(1 << 0); // IO0 in
    *(pio0_virtual_base + 1) |= (1 << 1);  // IO1 out
    *(pio0_virtual_base + 1) |= (1 << 25); // IO25 out
    *(pio0_virtual_base + 1) |= (1 << 26); // IO26 out

    pio0 &= ~(1 << 1); // OUT1=0
    *pio0_virtual_base = pio0;
    set_volt_scale(1);
    //----code begin----
    char str[30]="";
    while (1)
    {
        freq = *(ram_1_virtual_base + 0);
        printf("%d\n", freq);
        if(freq>1000000)
            sprintf(str,"t6.txt=\"%.3fMHz\"\xff\xff\xff\0", freq/1000000.0);
        else if(freq>1000)
            sprintf(str,"t6.txt=\"%.3fkHz\"\xff\xff\xff\0", freq/1000.0);
        else 
            sprintf(str,"t6.txt=\"%dHz\"\xff\xff\xff\0", freq);
        delay_us(10000);
        uart_printf(str);
        delay_us(10000);

        // get_Vpp();
        // printf("Vpp:%d\n",Vpp);
        // float Vmv=0;
        // if(volt_scale==0)
        //     Vmv=Vpp/4096.0*10000;
        // else if(volt_scale==1)
        //     Vmv=Vpp/4096.0*10000/4.5;
        // else 
        //     Vmv=Vpp/4096.0*10000/192;
        // sprintf(str,"t8.txt=\"%fmV\"\xff\xff\xff\0", Vmv);
        // delay_us(10000);
        // uart_printf(str);
        // delay_us(10000);

        get_state();
        //auto_set();
        //scanf("%d", &trigger);
        *(ram_1_virtual_base + 2) = trigger;

        //auto_set();




        Vpp=255;
        uint8_t _min = 255;
        uint8_t _max = 0;
        uint8_t _minn = 0;
        uint8_t _maxx = 255;
        for(int k=0;k<7;k++)
        {
            read_ad();
            for (int i = 2; i < 202; i++)
            {
            _min = min(_min, points[i]);
            _max = max(_max, points[i]);
            }
            _maxx=min(_maxx,_max);
            _minn=max(_minn,_min);
        }
        Vpp=_maxx-_minn;
        if(volt_scale==0)
            Vpp=Vpp/256.0*8000;
        else if(volt_scale==1)
            Vpp=Vpp/256.0*800;
        else 
            Vpp=Vpp/256.0*16;
        printf("Vpp:%f,%d,%d\n",Vpp,_max,_min);
        sprintf(str,"t8.txt=\"%.1fmV\"\xff\xff\xff\0", Vpp);
        delay_us(7000);
        uart_printf(str);
        delay_us(7000);

        delay_us(50000);
        if(!stop)
            {//read_ad();
            send_data(&points);}
        //for (int i = 2; i < 402; i++)
            // printf("%hhd,", points[i]);
            //printf("%hu,", ADdata[i]);
        delay_us(600000);

    }

    return 0;
}
