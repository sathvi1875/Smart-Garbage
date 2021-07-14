//
//  emb_1.c
//  
//
//  Created by Sathvik Mula on 7/14/21.
//
// PIC18F45K50 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1L
#pragma config PLLSEL = PLL4X // PLL Selection (4x clock multiplier)
#pragma config CFGPLLEN = OFF // PLL Enable Configuration bit (PLL Disabled (firmware controlled))
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Postscaler (CPU uses system clock (no divide))
#pragma config LS48MHZ = SYS24X4 // Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)
// CONFIG1H
#pragma config FOSC = INTOSCIO // Oscillator Selection (Internal oscillator)
#pragma config PCLKEN = ON // Primary Oscillator Shutdown (Primary oscillator enabled)
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF // Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)
// CONFIG2L
#pragma config nPWRTEN = OFF // Power-up Timer Enable (Power up timer disabled)
#pragma config BOREN = SBORDIS // Brown-out Reset Enable (BOR enabled in hardware (SBOREN is ignored))
#pragma config BORV = 190 // Brown-out Reset Voltage (BOR set to 1.9V nominal)
#pragma config nLPBOR = OFF // Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)
// CONFIG2H
#pragma config WDTEN = OFF // Watchdog Timer Enable bits (WDT enabled in hardware (SWDTEN ignored))
#pragma config WDTPS = 32768 // Watchdog Timer Postscaler (1:32768)
// CONFIG3H
#pragma config CCP2MX = RC1 // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)

#pragma config PBADEN = OFF // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config T3CMX = RC0 // Timer3 Clock Input MUX bit (T3CKI function is on RC0)
#pragma config SDOMX = RB3 // SDO Output MUX bit (SDO function is on RB3)
#pragma config MCLRE = ON // Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)
// CONFIG4L
#pragma config STVREN = ON // Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
#pragma config LVP = ON // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config ICPRT = OFF // Dedicated In-Circuit Debug/Programming Port Enable (ICPORT disabled)
#pragma config XINST = OFF // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)
// CONFIG5L
#pragma config CP0 = OFF // Block 0 Code Protect (Block 0 is not code-protected)
#pragma config CP1 = OFF // Block 1 Code Protect (Block 1 is not code-protected)
#pragma config CP2 = OFF // Block 2 Code Protect (Block 2 is not code-protected)
#pragma config CP3 = OFF // Block 3 Code Protect (Block 3 is not code-protected)
// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protect (Boot block is not code-protected)
#pragma config CPD = OFF // Data EEPROM Code Protect (Data EEPROM is not code-protected)
// CONFIG6L
#pragma config WRT0 = OFF // Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
#pragma config WRT1 = OFF // Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
#pragma config WRT2 = OFF // Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
#pragma config WRT3 = OFF // Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)
// CONFIG6H
#pragma config WRTC = OFF // Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF // Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
#pragma config WRTD = OFF // Data EEPROM Write Protect (Data EEPROM is not write-protected)
// CONFIG7L
#pragma config EBTR0 = OFF // Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF // Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF // Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF // Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)
// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <xc.h>
#include <pic18f45k50.h>
#include <stdio.h>
#include "op.h"
#define _XTAL_FREQ 8000000
/*********************Definition of Ports********************************/
#define RS LATD2 /*PIN 0 of PORTB is assigned for register select Pin of LCD*/
#define EN LATD3 /*PIN 1 of PORTB is assigned for enable Pin of LCD */
#define ldata LATD /*PORTB(PB4-PB7) is assigned for LCD Data Output*/
#define LCD_Port TRISD
#define Trigger_Pulse LATD0 /* PORTD.0 pin is connected to Trig pin of HC-SR04 */
/*********************Proto-Type Declaration*****************************/
void transmit_character(char);
void MSdelay(unsigned int ); /*Generate delay in ms*/
void LCD_Init(); /*Initialize LCD*/
void LCD_Command(unsigned char ); /*Send command to LCD*/
void LCD_Char(unsigned char x); /*Send data to LCD*/
void LCD_String(const char *); /*Display data string on LCD*/
void LCD_String_xy(char, char , const char *);
void LCD_Clear(); /*Clear LCD Screen*/
void Trigger_Pulse_10us();
void main()
{
TRISCbits.TRISC6=1;
TRISCbits.TRISC7=1;
TRISA=0;
ANSELA = 0x00;
float Distance;
int Time;
char out;
float Total_distance[10];
OSCCON=0x62; /* use internal oscillator with * MHz frequency */
SPBRG = 12; /* 9600 Baud Rate */
ANSELCbits.ANSC6 = 0; /*To use UART TX, this bit must be cleared */
ANSELCbits.ANSC7 = 0; /*To use UART RX, this bit must be cleared */
TRISB = 0xff; /* define PORTB as Input port*/
TRISD = 0; /* define PORTD as Output port*/
LCD_Init();
Trigger_Pulse = 0;
T1CON = 0x10; /* enable 16-bit TMR1 Register,No pre-scale,
* use internal clock,Timer OFF */
TMR1IF = 0; /* make Timer1 Overflow Flag to '0' */
TMR1=0; /* load Timer1 with 0 */
LCD_String_xy(1,1," GARBAGE FULL ");
while(1)
{
 
Trigger_Pulse_10us(); /* transmit at least 10 us pulse to HC-SR04 */
while(PORTBbits.RB4==0); /* wait for rising edge at Echo pin of HC-SR04 */
TMR1=0; /* Load Timer1 register with 0 */
TMR1ON=1; /* turn ON Timer1*/
while(PORTBbits.RB4==1); /* wait for falling edge at Echo pin of HC-SR04*/
TMR1ON=0; /* turn OFF Timer1 */
Time = (TMR1L | (TMR1H<<8)); /* copy Time when echo is received from an object */
Distance = ((float)Time/117); /* distance = (velocity x Time)/2 */
if(Distance < 10 )
{ sprintf(Total_distance,"%.03f",Distance);
LCD_Clear();
LCD_String_xy(1,1," GARBAGE FULL ");
LCD_String_xy(2,1,Total_distance);
LCD_String(" cm ");
PORTA =0xFF;
out=”1”;
TXEN=1;
void transmit_character(out);
MSdelay(10);
}
else
{
LCD_Clear();
LCD_String_xy(1,1," GARBAGE EMPTY ");
out=”0”;
void transmit_character(out);
PORTA =0x00;
MSdelay(10);
}
}
 
}
void Trigger_Pulse_10us()
{
Trigger_Pulse = 1;
__delay_us(10);
Trigger_Pulse = 0;
}
void LCD_Init()
{
LCD_Port = 0; /*PORT as Output Port*/
MSdelay(15); /*15ms,16x2 LCD Power on delay*/
LCD_Command(0x02); /*send for initialization of LCD
for nibble (4-bit) mode */
LCD_Command(0x28); /*use 2 line and
*initialize 5*8 matrix in (4-bit mode)*/
LCD_Command(0x01); /*clear display screen*/
LCD_Command(0x0c); /*display on cursor off*/
LCD_Command(0x06); /*increment cursor (shift cursor to right)*/
}
void LCD_Command(unsigned char cmd )
{
ldata = (ldata & 0x0f) |(0xF0 & cmd); /*Send higher nibble of command first to PORT*/
RS = 0; /*Command Register is selected i.e.RS=0*/
EN = 1; /*High-to-low pulse on Enable pin to latch data*/
NOP();
EN = 0;
MSdelay(1);
ldata = (ldata & 0x0f) | (cmd<<4); /*Send lower nibble of command to PORT */
EN = 1;
NOP();
EN = 0;
MSdelay(3);
}
void LCD_Char(unsigned char dat)
{
ldata = (ldata & 0x0f) | (0xF0 & dat); /*Send higher nibble of data first to PORT*/
RS = 1; /*Data Register is selected*/
EN = 1; /*High-to-low pulse on Enable pin to latch data*/
NOP();
EN = 0;
MSdelay(1);
ldata = (ldata & 0x0f) | (dat<<4); /*Send lower nibble of data to PORT*/
EN = 1; /*High-to-low pulse on Enable pin to latch data*/
NOP();
EN = 0;
MSdelay(3);
}
void LCD_String(const char *msg)
{
while((*msg)!=0)
{
LCD_Char(*msg);
msg++;
}
}
void LCD_String_xy(char row,char pos,const char *msg)
{
char location=0;
if(row<=1)
{
location=(0x80) | ((pos) & 0x0f); /*Print message on 1st row and desired location*/
LCD_Command(location);
}
else
{
location=(0xC0) | ((pos) & 0x0f); /*Print message on 2nd row and desired location*/
LCD_Command(location);
}
LCD_String(msg);
}
void LCD_Clear()
{
LCD_Command(0x01); /*clear display screen*/
}
void MSdelay(unsigned int val)
{
unsigned int i,j;
for(i=0;i<val;i++)
for(j=0;j<165;j++); /*This count Provide delay of 1 ms for 8MHz Frequency */
}
void transmit_character(char out)
{
while(!TRMT);
TXREG=out;
}
