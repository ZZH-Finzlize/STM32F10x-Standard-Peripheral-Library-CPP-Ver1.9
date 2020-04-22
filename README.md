# STM32F10x-Standard-Peripheral-Library-CPP-Ver1.9
本项目是一个基于原生[STM32F10x Standard Peripheral Library V3.5][1],使用 __C++__ 语言二次开发的 _STM32F1系列_ MCU固件库,使用的标准为 __C++11__,主要目的在于利用 __C++__ 的语言优势,提高单片机程序的 __开发效率__ ,并简化应用程序的设计

首先,在使用此软件之前,您可能需要提前了解如下概念:
> - 类与对象
>   - 构造与析构
>   - 成员访问属性
>   - 静态成员
>   - `this`指针
>   - 继承与派生
>   - 友元关系
>   - 虚函数与纯虚函数
> - 对象、指针、引用
> - 多态性
>   - 静态多态
>   - 动态多态
> - 模板
>   - 类模板
>   - 函数模板
> - 函数重载
> - 运算符重载
> - C++异常处理
> - C++内存管理
> - C++11新特性

上述内容可以在示例代码中找到对应的示例,并且,示例代码会有后续更新

固件库保留了ST官方库的全部函数,并且将其封装入类内,在此基础上,针对不同的外设,扩增了更加简洁的操作函数,各个类均拥有丰富的重载以便于最大化简化程序.

固件库实现了 __全部外设时钟__ 的 __全自动管理__ 和简洁的初始化(GPIO和NVIC除外),在实例化外设对象时,可以通过构造函数将其初始化,并且自动打开其时钟,当销毁外设对象时,自动关闭对应外设的时钟,__每一个对象都可以绑定一个外设,同一个外设可以被多个对象绑定__.其具体实现方法及原理将在[教程](#tutorial)章节中详细讲解.

如下代码,展示了一个串口输出`Hello World`程序

``` C++
#include "stm32f10x.h"//设备头文件
#define USART1_TXD Pin_9//A
#define USART1_RXD Pin_10//A
GPIO PA(GPIOA);//GPIO类实例化一个PA对象,绑定GPIOA,并启动GPIOA的时钟
USART cout(USART1,115200);//USART类实例化cout对象,传入初始化参数为波特率115200,其余参数使用默认值,绑定USART1,并启动USART1的时钟

void GPIO_Init()//GPIO和NVIC需要单独初始化的外设,其余外设均可使用构造函数初始化
{
    GPIO::InitTypeDef aaa;//初始化信息结构体,同官方库设计
    
    aaa.Speed = Speed_50MHz;
    aaa.Mode = Mode_AF_PP;//复用推挽
    aaa.Pin = USART1_TXD;//TXD引脚
    PA.Init(aaa);//引用传递

    aaa.Mode = Mode_IN_FLOATING;//浮空输入
    aaa.Pin = USART1_RXD;//RXD引脚
    PA.Init(aaa);//引用传递
}

int main()
{
    GPIO_Init();
    cout << "Hello World" << endl;//输出Hello World并换行
    while(true);
}
```

除字符串以外,所有的C++基本数据类型均可通过<<运算符由串口输出,也可以连续,混合输出不同的数据类型,例如:

``` C++
cout << 123456 << endl;//实际输出123456
cout << 'a' << endl;//实际输出a
cout << 3.1415 << endl;//实际输出3.1415
cout << 0x1b << endl;//实际输出0x1B
cout << false << endl;//实际输出false
cout << (bool)0 << endl;//实际输出false

cout << 123456 << 'a' << 3.1415 << 0x1b << false << endl;
//实际输出123456a3.14150x1Bfalse,并带有换行
```

除`<<`运算符以外,也支持传统的C语言风格输出(无需用户重定向)

``` C++
//库提供的是Printf,首字母大写
//Printf方法内部使用了缓冲区,单次输出不可以超过256字节
//Printf方法有返回值,返回值为输出的字节数量
cout.Printf("%d\r\n",123456);//实际输出123456
cout.Printf("%c\r\n",'a');//实际输出a
cout.Printf("%f\r\n",3.1415);//实际输出3.1415
cout.Printf("%#X\r\n",0x1b);//实际输出0x1B
//C语言无bool类型,因此只能以字符串输出false
cout.Printf("%s\r\n",false?"true":"false");//实际输出false

cout.Printf("%d%c%f%#X%s\r\n",123456,'a',\
3.1415,0x1b,false?"true":"false");
//实际输出123456a3.14150x1Bfalse,并带有换行
```
