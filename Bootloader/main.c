
#include "msp430.h"
#include "Config.h"
#include "string.h"
#include "stdlib.h"

unsigned char inApp = 0;
unsigned char updating = 0;
unsigned char RecBuf[512];
unsigned int index = 0;
unsigned int old_index = 0;
unsigned char count = 0;
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

//***********************************************************************
//             TIMERA初始化，设置为UP模式计数
//***********************************************************************
void TIMERA_Init(void)                                   //UP模式计数，计数周期为CCR0+1
{
  TACTL |= TASSEL0 + TACLR + ID0 + ID1 + MC0 + TAIE;     //ACLK做时钟源，8分频，增加计数模式，开中断
  TACCR0 = 12287;                                        //CCR0=12287，3s中断一次
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
//              485发送数据函数
//*************************************************************************

void Send_Byte_485(uchar data)
{
  RS485_CTR1;
  while(!(IFG2&UTXIFG1));            //发送寄存器空的时候发送数据
  U1TXBUF=data;
//  RS485_CTR0;
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

void write_Flag (unsigned char* value, unsigned int length)
{
  unsigned char *Flash_ptr;
  unsigned char count = length/4;
  unsigned char i;

  Flash_ptr = (unsigned char *)(0x9E00);
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
  *Flash_ptr = 0;                           // Dummy write to erase Flash segment
  
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
  
  for(i=0; i<count; i++){
    unsigned int temp = RecBuf[4*i];
    temp = temp << 8;
    Flash_ptr = (unsigned char *)(temp + RecBuf[4*i+1] + 0x4000);     // 先把数据存到0x9FE0,copy再转移到0x5FE0
    *Flash_ptr++ = RecBuf[4*i+2];
    *Flash_ptr   = RecBuf[4*i+3];
  }
  
  Flash_ptr = (unsigned char *)(0x9FD0);
  *Flash_ptr   = 0xA5;
  
  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void clear_Flag ()
{
  unsigned char *Flash_ptr;

  Flash_ptr = (unsigned char *)(0x9E00);
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit

  *Flash_ptr = 0;
  
  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void write_SegA (unsigned char* value, unsigned char offset)
{
  unsigned char *Flash_ptr;                          // Flash pointer
  unsigned int i;

  Flash_ptr = (unsigned char *)(APP2_FLASH + 0x200*offset);                   // Initialize Flash pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
  *Flash_ptr = 0;                           // Dummy write to erase Flash segment

  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  for (i=0; i<512; i++)
  {
    *Flash_ptr++ = *value++;                   // Write value to flash
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void copy_A2B (void)
{
  unsigned char *Flash_ptrA;                         // Segment A pointer
  unsigned char *Flash_ptrB;                         // Segment B pointer
  unsigned char i;
  unsigned char j;

  Flash_ptrA = (unsigned char *) APP2_FLASH;             // Initialize Flash segment A pointer
  Flash_ptrB = (unsigned char *) APP1_FLASH;             // Initialize Flash segment B pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
  *Flash_ptrB = 0;                          // Dummy write to erase Flash segment B
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  for (i=0; i<128; i++)
  {
    for (j=0; j<128; j++)
    {
        *Flash_ptrB++ = *Flash_ptrA++;           // Copy value segment A to segment B
    }
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void Check_Update()
{
  unsigned char *Flash_ptr;
  Flash_ptr = (unsigned char *)(0x9FD0);
  unsigned char STATUS = *(Flash_ptr);
  if(STATUS == 0xA5) {
    Print_Str_485("App Transforming...\r\n");
    copy_A2B();
    Print_Str_485("App Transform Done\r\n");
    clear_Flag();
  }
}

#pragma vector=UART1RX_VECTOR
__interrupt void UART1_RX_ISR(void)
{
  // APP1串口1接收中断向量
  if( *((unsigned char*)(0x09FD)) == 0x20 ) {
    asm("BR &0x5FE6;");
  }
  else if(updating == 1) {
    RecBuf[index++] = U1RXBUF;
  }
  else if(U1RXBUF == 0xAA) {
    updating = 1;
    Print_Str_485("Start Receiving!\r\n");
  }
}

#pragma vector=UART0RX_VECTOR
__interrupt void UART0_RX_ISR(void)
{
  // APP1串口0接收中断向量
  if( *((unsigned char*)(0x09FD)) == 0x20 ) 
    asm("BR &0x5FF2;");
  else {
    _NOP();
  }
}

#pragma vector = TIMERA1_VECTOR
__interrupt void Timer_A(void)
{
  // 读TAIV清除中断标志位
  if(TAIV) {
    if(updating == 0 && count == 1) {
      // 跳转到APP1
      inApp = 1;
      Print_Str_485("ENTER App\r\n");
      TACTL = 0;
      asm("BR 0x5FFE");
    }
    else count++;
  }
  
}

int main( void )
{
  unsigned char segment = 0;
  unsigned char half = 0;

  // 关闭看门狗
  WDT_Init();
  Clock_Init();
  Port_Init();
  RS485_Init();
  Close_LED();
  Print_Str_485("Checking...\r\n");
  Check_Update();
  RS485_CTR0;
  TIMERA_Init();
  _EINT();
  
  while(1) {
    
    // 升级文件接收
    if(index == 256 && half == 0) {
      half = 1;
      Print_Str_485("Receive Segment Successful!\r\n");
    }else if(index == 512) {
      Print_Str_485("Receive Segment Successful!\r\n");
      write_SegA(RecBuf, segment);
      memset(RecBuf, 0, index);
      index = 0;
      half = 0;
      segment++;
    }else if(index == old_index && index!= 0 && index != 256) {
      if(RecBuf[0] == 0x5f && RecBuf[1] == 0xfe) {
        write_Flag(RecBuf, index);
        memset(RecBuf, 0, index);
        index = 0;
        Print_Str_485("Receive DONE!\r\n");
        // 用看门狗软件复位
        WDTCTL = 0xFF00;
      }
      else {
        Print_Str_485("Receive Segment Successful!\r\n");
        write_SegA(RecBuf, segment);
        memset(RecBuf, 0, index);
        index = 0;
      }
    }
    old_index = index;
    
    delay_ms(500);
  }

}
