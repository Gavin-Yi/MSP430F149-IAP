#include <string.h>
#include "msp430.h"
#include "Config.h"

uchar RS485Buf[128];
uchar RS232Buf[128];
uchar Index1 = 0;
uchar OldIndex1 = 0;
uchar Index2 = 0;
uchar OldIndex2 = 0;

//***********************************************************************
//               MSP430IO口初始化
//***********************************************************************
void Port_Init()
{
  LED8SEL  = 0x00;                      //设置IO口为普通I/O模式，此句可省
  LED8DIR  = 0xFF;                      //设置IO口方向为输出
  LED8PORT = 0xFF;                      //P2口初始设置为FF
  
  DATASEL  = 0x00;                      //设置IO口为普通I/O模式，此句可省
  DATADIR  = 0xFF;                      //设置IO口方向为输出
  DATAPORT = 0xFF;                      //P4口初始设置为FF
  
  CTRSEL  =  0x00;                      //设置IO口为普通I/O模式，此句可省
  CTRDIR |=  BIT3 + BIT4;               //设置IO口方向为输出,控制口在P63,P64
  CTRPORT =  0xFF;                      //P6口初始设置为FF  
}

//*************************************************************************
//               MSP430串口0初始化  RS232
//*************************************************************************
void RS232_Init()
{
  U0CTL|=SWRST + CHAR;                //复位SWRST，8位数据模式
  U0TCTL|=SSEL1;                      //SMCLK为串口时钟
  U0BR1=baud_h;                       //BRCLK=8MHZ,Baud=BRCLK/N
  U0BR0=baud_l;                       //N=UBR+(UxMCTL)/8
  U0MCTL=0x00;                        //微调寄存器为0，波特率9600bps
  ME1|=UTXE0;                         //UART1发送使能
  ME1|=URXE0;                         //UART1接收使能
  U0CTL&=~SWRST;
  IE1|=URXIE0;
  
  P3SEL|= BIT4 + BIT5;                //设置IO口为第二功能模式，启用UART功能
  P3DIR|= BIT4;                       //设置TXD1口方向为输出
}

//*************************************************************************
//               MSP430串口1初始化  RS485
//*************************************************************************
void RS485_Init()
{
  U1CTL|=SWRST + CHAR;                //复位SWRST，8位数据模式
  U1TCTL|=SSEL1;                      //SMCLK为串口时钟
  U1BR1=baud_h;                       //BRCLK=8MHZ,Baud=BRCLK/N
  U1BR0=baud_l;                       //N=UBR+(UxMCTL)/8
  U1MCTL=0x00;                        //微调寄存器为0，波特率9600bps
  ME2|=UTXE1;                         //UART1发送使能
  ME2|=URXE1;                         //UART1接收使能
  U1CTL&=~SWRST;
  IE2|=URXIE1;
  
  P3SEL|= BIT6 + BIT7;                //设置IO口为第二功能模式，启用UART功能
  P3DIR|= BIT6;                       //设置TXD1口方向为输出
}

//*************************************************************************
//              232发送数据函数
//*************************************************************************

void Send_Byte_232(uchar data)
{
  while(!(IFG1&UTXIFG0));             //发送寄存器空的时候发送数据
  U0TXBUF=data;
}

//*************************************************************************
//              232发送字符串函数
//*************************************************************************
void Print_Str_232(uchar *s)
{
  while(*s != '\0')
  {
    Send_Byte_232(*s++);
    delay_ms(2);
  }
}

//*************************************************************************
//              485发送数据函数
//*************************************************************************

void Send_Byte_485(uchar data)
{
  RS485_CTR1;
  while(!(IFG2&UTXIFG1));             //发送寄存器空的时候发送数据
  U1TXBUF=data;
}

//*************************************************************************
//              485发送字符串函数
//*************************************************************************
void Print_Str_485(uchar *s)
{
  while(*s != '\0')
    {
        Send_Byte_485(*s++);
        delay_ms(2);
    }
  RS485_CTR0;
}

//*************************************************************************
//               处理来自RS485的接收中断
//*************************************************************************
#pragma vector=UART1RX_VECTOR
__interrupt void UART1_RX_ISR(void)
{
  RS485Buf[Index1++] = U1RXBUF;
  
  asm("POP R15;");
  asm("POP R15;");
  asm("POP R15;");
  asm("POP R15;");
}

//*************************************************************************
//               处理来自RS232的接收中断
//*************************************************************************
#pragma vector=UART0RX_VECTOR
__interrupt void UART0_RX_ISR(void)
{
  RS232Buf[Index2++] = U0RXBUF;
}

int main( void )
{
  WDT_Init();
  Clock_Init();
  Port_Init();
  RS485_Init();
  RS232_Init();
  Close_LED();
  Print_Str_485("DMA430-A Board 485 Test\r\n");
  Print_Str_232("DMA430-A Board 232 Test\r\n");
  RS485_CTR0;
  _EINT();

  while(1)
  {
    // 485 To 232
    if(Index1!=0 && OldIndex1==Index1) {
      Print_Str_232(RS485Buf);
      memset(RS485Buf,0,Index1);
      Index1=0;
    }
    OldIndex1 = Index1;
    // 232 To 485
    if(Index2!=0 && OldIndex2==Index2) {
      Print_Str_485(RS232Buf);
      memset(RS232Buf,0,Index2);
      Index2=0;
    }
    OldIndex2 = Index2;
    
    delay_ms(500);
  }
}
