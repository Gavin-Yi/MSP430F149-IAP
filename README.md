# MSP430F149单片机-IAP升级

## 序言

系统设计的总体构想是，通过无线的方式，将需要升级的二进制文件直接下载到相应的地址空间，然后让单片机系统自动执行IAP升级。

TI公司的MSP430系列单片机是具有很高实用性价值的产品，在诸多领域得到广泛应用，主打其超低功耗的特性，16位的数据总线宽度，加上RISC指令集。IAP（In Application Programming）是用户自己的程序在运行过程中对用户Flash的部分区域进行烧写，目的是为了在产品发布后可以方便地通过预留地通信口对产品中地固件程序进行更新升级。

要实现IAP的功能，通常需要在设计固件时，编写两个项目：第一个项目程序不执行正常的功能操作。而只是通过某种通信信道（如UART）接收程序或数据（可执行的HEX文件），执行对第二部分代码的更新；第二个项目才是真正的功能代码。这两部分项目代码同时烧录在用户的Flash当中，当芯片上电时，首先运行至第一个项目代码，然后再跳转至第二个项目代码。

根据IAP的特性以及MSP430没有自带IAP的特点，需要自行编写IAP程序进行片内Flash的擦写，下面详细介绍如何进行IAP升级的设计。

比较重要的几个点包括：

 1. Bootloader的功能设计
 2. App和Bootloader的地址分配
 3. App程序的Flash写入
 4. Bootloader和App的跳转
 5. Bootloader和App中断向量的映射

## 功能设计
Bootloader是单片机上电时首先运行的程序。定时3s后芯片没有处于接收升级文件的状态的话，就会自动跳转至App程序继续运行。

Bootloader流程图：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517221206535.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)

## 地址分配
MSP430F149的Flash空间从 0x1000H - 0xFFFFH. 其中 0xFFE0H - 0xFFFFH 的32个Bytes用来存放中断向量.

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517221303204.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)

对Flash的空间如下规划:

Bootloader:
|0xF000H - 0xFFFFH|4kB  |
|--|--|

App1:
|0x2000H - 0x5FFFH|16kB  |
|--|--|

App2:
|0xF000H - 0xFFFFH|16kB  |
|--|--|

想让仿真器能够直接烧写到对应的地址,需要对配置文件进行一定的修改.

以Bootloader为例,在IAR软件中需要在 Linker 中设置使用自己的xcl配置文件,然后用文本编辑器打开xcl文件修改.
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517221513984.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)

存放 Code 和 Data 的区域是 0xF000H - 0xFFDFH, 而 0xFFE0H - 0xFFFFH 是中断向量的地址, 也是触发中断时, 硬件会前往的地址。 对应于App，其存放 Code 和 Data 的区域是 0x2000H - 0x5FDFH, 0x5FE0H - 0x5FFFH 存放App1的用户中断代码，0x5FFEH是App1的复位向量。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517221523975.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)

## Flash擦写
TI提供了FLash擦写的例程，其中核心代码如下。

```c
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

```

需要注意的是，芯片对Flash最小的擦写单位是一个 Segment. 参考MSP430的芯片手册，通常一个Segment是 512个字节，除了用来存放用户信息的 Information Memory(128 Bytes) 和 最后一个 Segment n(256 Bytes). 这也是我们选择把 0x2000H 作为 App1 起始地址的原因——便于擦写Flash代码的书写。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517221619512.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)

## App跳转
使用芯片手册中提到的BR指令，可以实现运行代码的变更。即，将目标地址放入PC寄存器。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517221634378.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)

代码如下所示：

```c
asm("BR &0x5FFE")
```
目标地址是App1的 复位(Reset) 中断向量，随后程序会运行到App1的主程序中。

## 中断向量表的偏移
在 Stm32 中，可以使用 SCB->VTOR 实现中断向量表的偏移，例如在 systeminit 函数中 VECT_TAB_OFFSET 宏定义控制偏移位置

```c
#ifdef VECT_TAB_SRAM
	SCV->VTOR = SRAM_BASE | VECT_TAB_OFFSET /* Vector Table Relocation in Internal SRAM*/
#else
	SCV->VTOR = FLASH_BASE | VECT_TAB_OFFSET /*Vector Table Relocation in Flash*/
#endif
```

但是MSP430F149这款芯片并没有类似的配置寄存器。发生中断时，芯片进入的是硬件的复位地址 0xFFE0H - 0xFFFFH，根据不同的中断进入不同的地址。所以只能在Bootloader的中断中使用类似于 asm("BR &0xXXXX") 这样的指令跳转到App中的中断服务函数，例如

Bootloader：

```c
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
```

App1:

```c
#pragma vector=UART1RX_VECTOR
__interrupt void UART1_RX_ISR(void)
{
  RS485Buf[Index1++] = U1RXBUF;
  
  asm("POP R15;");
  asm("POP R15;");
  asm("POP R15;");
  asm("POP R15;");
}
```

需要注意的有两点：

 1. 芯片从Bootloader跳转到App的复位函数进行复位后，会清除之前Bootloader保存的RAM变量(例如可能计划用来判断程序是否进入App的标志位)；也不能将该变量保存到Flash当中，不然Flash不会自动清除，当意外再次上电时，芯片在Bootloader却会认为自己在App中。所以此处，我们采用CPU的SP指针判断跳转进入中断的地址是否是来自0x20开头的代码段，从而判断是否是App的中断(我的App只是很小的代码，没有超出0x2XXXH)
 2. 另外在Bootloader反汇编出来的代码中可以看到，在执行用户代码之前，会有一系列用户入栈的操作，这在跳转到App的中断服务函数之后，没有进行出栈的操作，从而导致程序跑飞(***重要***). 所以只要在App的中断服务函数的最后执行相应数量的出栈操作，就不会使得 RETI 函数执行出错。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517222025781.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpd2VuMjYwOQ==,size_16,color_FFFFFF,t_70)
