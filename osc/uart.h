/*
 * uart.h
 *
 *  Created on: 2023年6月29日
 *      Author: Ming
 */

#ifndef UART_H_
#define UART_H_

extern uint32_t *uart_0_virtual_base;

void uart_putc(char c)
{
	unsigned short uart_status; // 状态寄存器值
	do
	{
		uart_status = *(uart_0_virtual_base + 2); // 读取状态寄存器
	} while (!(uart_status & 0x40));			  // 等待状态寄存器bit6（trdy）为1
	*(uart_0_virtual_base + 1) = c;				  // 发送一个字符
}

// 串口字符串发送函数
void uart_printf(char *str)
{
	while (*str != '\0') // 检测当前指针指向的数是否为空字符
	{
		uart_putc(*str); // 发送一个字符
		str++;			 // 字符串指针+1
	}
}
int uart_getc(void)
{
	unsigned short uart_status; // 状态寄存器值
	do
	{
		uart_status = *(uart_0_virtual_base + 2); // 读取状态寄存器
	} while (!(uart_status & 0x80));			  // 等待状态寄存器bit7（rrdy）为1
	return *(uart_0_virtual_base + 0);			  // 读取一个字符并作为函数返回值返回
}

int uart_scanf(char *p)
{
	int cnt = 0; // 接收个数计数器
	while (1)
	{
		*p = uart_getc(); // 读取一个字符的数据
		cnt++;
		if (*p == '\n') // 判断数据是否为换行
			return cnt; // 换行则停止计数,返回当前接收的字符个数
		else
			p++; // 接收指针增 1
	}
}



#endif /* UART_H_ */
