/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: Austin Kootz and Josh Bowen
 * Date  : 12/15/2014
 *

 *
 *
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *     IOconfig();
 *     SPIconfig();
 *     CCPconfig();
 *     LCDconfig();
 *     ADCconfig();
 *     USARTconfig();
 *     TMRconfig();
 *     INTconfig();
 *
 *   While(1)
 *     TurnSignal();
 *     BrakeLight();
 *     AccelMeter();
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   CCP1handler();
 *   CCP4handler();
 *   TMR0handler();
 *   TMR1handler();
 *   TMR2handler();
 *   RxUarthandler();

 *
 ******************************************************************************/

#include <p18cxxx.h>
#include "LCDroutines.h"
#include "adc.h"
#include "delays.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

// LED Defines
#define D6 LATBbits.LATB4
#define D2 LATBbits.LATB5
#define D3 LATBbits.LATB6
#define D4 LATBbits.LATB7

// Turn Signal Defines
#define LEFT_LED LATEbits.LATE0
#define RIGHT_LED LATEbits.LATE1
#define BRAKE LATEbits.LATE2

// Switch Defines
#define LEFT_SW LATGbits.LATG3
#define RIGHT_SW LATGbits.LATG4

#define CS LATHbits.LATH5

/******************************************************************************
 * Global variables
 ******************************************************************************/
const rom far char LCDRow1[] = {0x80,'A','C','C','E','L',' ','=',' ',0x00};
const rom far char LCDRow2[] = {0xC0,'B','O','O','T','I','I','N','G',0x00};
char accArr[3] = {'0', '0', '0'};
unsigned int left_count = 0;
unsigned int right_count = 0;
unsigned int left_off_count = 0;
unsigned int right_off_count = 0;
unsigned char PULSEWH = 0;
unsigned char PULSEWL = 0;
unsigned char CCP1SETH = 0;
unsigned char CCP1SETL = 0;
unsigned char CCP4SETH = 0;
unsigned char CCP4SETl = 0;
unsigned int TEMP = 0;
char LCDTop[] = {0x80,'T','O','P','o','f','L','C','D',0x00};
char LCDBot[] = {0x80,'B','O','T','o','f','L','C','D',0x00};
char dumpString[5] = {'D', 'u', 'm', 'p', '\n'};
char offString[4] = {'O', 'F', 'F', '\n'};
char leftString[5] = {'L', 'E', 'F', 'T', '\n'};
char rightString[6] = {'R', 'I', 'G', 'H', 'T', '\n'};
char brakeString[6] = {'B', 'R', 'A', 'K', 'E', '\n'};
char accelString[6] = {'A', 'C', 'C', 'E', 'L', '\n'};
char uart_rxbuffer[6] = {"00000"};
unsigned int rx_counter = 0;
char Test[4] = {'T', 'E', 'S', 'T'};
unsigned int BRAKE_duty = 0;

char AccelX = 0;
char AccelY = 0;
char AccelZ = 0;


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void HiPriISR(void);
void LoPriISR(void);
void CCP1handler(void);     // Interrupt handler for CCP1
void CCP4handler(void);     // Interrupt handler for CCP4
void TMR0handler(void);     // Interrupt handler for TMR0
void TMR1handler(void);     // Interrupt handler for TMR1
void TMR2handler(void);     // Interrupt handler for TMR2
void RxUartHandler(void);   // Interrupt handler for RX UART

void IOconfig(void);
void SPIconfig(void);
void CCPconfig(void);
void TMRconfig(void);
void LCDconfig(void);
void INTconfig(void);
void ADCconfig(void);
void USARTconfig(void);

void BrakeLight(void);
void AccelMeter(void);

void dumpEEPROM(void);
void sendTest(void);

void changeLCDBot(char a, char b, char c);
void vToMss(void);
void displayAccel(void);

void SPIhandler(void);
void sendAccel(void);



#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {

    Initial();                 // Initialize everything
    while(1) {
        AccelMeter();
        displayAccel();
        Delay10KTCYx(25);
        // Sit here forever
     }
}/******************************************************************************
 * subroutines
 ******************************************************************************/
void SPIhandler(){
    
    char SPIchars[20];
    char SPIrxl = 0;
    char SPIrxh = 0;
    char SPIryl = 0;
    char SPIryh = 0;
    char SPIrzl = 0;
    char SPIrzh = 0;
    char SPItx = 0b11101000;
    char SPItemp = 0;

    CS = 0;

    SPItemp = SSP1BUF;
    SSP1BUF = SPItx;

    while (SSP1STATbits.BF == 0){}

    SPItemp = SSP1BUF;
    SSP1BUF = SPItemp;

    while (SSP1STATbits.BF == 0){}

    SPIrxl = SSP1BUF;
    SSP1BUF = SPItx;

    while (SSP1STATbits.BF == 0){}

    SPIrxh = SSP1BUF;
    SSP1BUF = SPItx;

    while (SSP1STATbits.BF == 0){}

    SPIryl = SSP1BUF;
    SSP1BUF = SPItx;

    while (SSP1STATbits.BF == 0){}

    SPIryh = SSP1BUF;
    SSP1BUF = SPItx;

    while (SSP1STATbits.BF == 0){}

    SPIrzl = SSP1BUF;
    SSP1BUF = SPItx;

    while (SSP1STATbits.BF == 0){}

    SPIrzh = SSP1BUF;
    SSP1BUF = SPItx;
        
    CS = 1;
    
    AccelX = SPIrxh;
    
    if (AccelX<0){
        AccelX = 0;
    }

    AccelY = SPIryh;
    
    AccelZ = SPIrzh;
    
}
void vToMss(void)
{
    //AccelX is 0 to 127
    //0.0154 m/s^2
    unsigned int accel = AccelX * 155;
    accArr[0] = (accel/1000)%10 + '0';
    accArr[1] = (accel/100)%10 + '0';
    accArr[2] = (accel/10)%10 + '0';
    
}
void displayAccel()
{
    vToMss();
    changeLCDBot(accArr[0], accArr[1], accArr[2]);
}
void sendAccel()
{
    Write1USART(accArr[0]);
    Delay1KTCYx(4);
    Write1USART('.');
    Delay1KTCYx(4);
    Write1USART(accArr[1]);
    Delay1KTCYx(4);
    Write1USART(accArr[2]);
    Delay1KTCYx(4);
    Write1USART('\n');
}

void changeLCDBot(char a, char b, char c)
{
    char LCDRowBot[] = {0xC0, a, '.', b, c, 'm', '/', 's', '2', 0x00};
    DisplayV(LCDRowBot);
}

void sendTest(){
    Write1USART(Test[0]);
    Delay1KTCYx(4);
    Write1USART(Test[1]);
    Delay1KTCYx(4);
    Write1USART(Test[2]);
    Delay1KTCYx(4);
    Write1USART(Test[3]);
    Delay1KTCYx(4);
    Write1USART('\n');
}

void AccelMeter(){
    //save the accelerometer data to EEPROM.
    SPIhandler();
}

void dumpEEPROM(){
    //dump EEPROM over UART
}

void IOconfig(){
    // Configure the IO ports
    TRISA   = 0b11111111;
    LATA    = 0b00000000;
    
    // RB4-7 control onboard LEDs
    TRISB   = 0b00001111;
    LATB    = 0b00010000;
    
    // RC3 is SPI clock out, RC4 is SPI data in, RC5 SPI data out
    TRISC   = 0b10010011; 
    LATC    = 0b00000000;

    // RE0 is Left LED, RE1 is Right LED, RE2 is Brake
    TRISE   = 0b00000000;
    LATE    = 0b00000000;

    // RG3 is Left Switch, RG4 is Right Switch
    TRISG   = 0b00011000;
    LATG    = 0b00000000;
    
    // RH5 is SPI chip select
    TRISH   = 0b00000000;
    LATH    = 0b11111111;
    CS = 1;
}

void SPIconfig()
{
    // sample accelerometer at 50 Hz and turn on all three axis
    char SPItx2 = 0b01010111;
    // adress of con1 register in accel
    char SPItx1 = 0x20;
    // trash memory
    char SPItemp = 0;
    // input data sampled at end of output, transmit at idle to active clock
    SSP1STAT = 0b00000001;
    // enabled with clock idle high, bits 3-0 control master clock speed Fosc/4
    SSP1CON1 = 0b00110000;
    // configure Accelerometer

    CS = 0;

    SPItemp = SSP1BUF;
    SSP1BUF = SPItx1;

    while (SSP1STATbits.BF == 0){}

    SPItemp = SSP1BUF;
    SSP1BUF = SPItx2;

    while (SSP1STATbits.BF == 0){}

    SPItemp = SSP1BUF;

    CS = 1;
}

void CCPconfig(){
    // Initializing CCP1
    CCP1CON = 0b00001010;
    CCPTMRS0 = 0b00000000;
    CCPR1 = 0;

    // Initializing CCP4
    CCP4CON = 0b00001010;
    CCPTMRS1 = 0b00000000;
    CCPR2 = 0;
}

void LCDconfig(){
    // Configure the LCD pins for output
    LCD_RS_TRIS   = 0;              //TRISH1
    LCD_E_TRIS    = 0;              //TRISH2
    LCD_DATA_TRIS = 0b00001111;     // TRISJ, Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs

    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);
    DisplayC(LCDRow2);

}

void TMRconfig(){

    // Initializing TMR0
    T0CON = 0b01001000;             // 8-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;

    // Initializing TMR1
    T1CON = 0b00000000;             // 8-bit, Fosc / 8, no pre/post scale timer
    TMR1H = 0x00;                      // setting TMR1 registers
    TMR1L = 0x00;

    // Initializing TMR2
    T2CON = 0b00000010;             // 8-bit, Fosc / 4, no pre/post scale timer
    TMR2 = 0;

    // turn the timers on
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    T1CONbits.TMR1ON = 1;           // Turning on TMR1
    T2CONbits.TMR2ON = 1;           // Turning on TMR2
    
}

void INTconfig(){

    // clear all flags
    PIR1bits.TMR1IF = 0;
    PIR1bits.TMR2IF = 0;
    PIR3bits.CCP1IF = 0;
    PIR4bits.CCP4IF = 0;

    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    IPR3bits.CCP1IP = 0;            // Assign low priority to CCP1 interrupt
    IPR4bits.CCP4IP = 0;            // Assign low priority to CCP4 interrupt
    INTCON2bits.TMR0IP = 0;            // Assign low priority to TMR0 interrupt
    IPR1bits.TMR1IP = 0;            // Assign low priority to TMR1 interrupt
    IPR1bits.TMR2IP = 0;            // Assign low priority to TMR2 interrupt
    IPR1bits.RC1IP = 1;             // Assign High Priority to UART RX interrupt

    PIE3bits.CCP1IE = 0;            // Enable CCP1 interrupts (off)
    PIE4bits.CCP4IE = 0;            // Enable CCP4 interrupts (off)
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts (on)
    PIE1bits.TMR1IE = 1;            // Enable TMR1 interrupts (off)
    PIE1bits.TMR2IE = 1;            // Enable TMR2 interrupts (off)

    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

}

void ADCconfig(){

    //ADC Init (from Josh)
    ADCON2 = 0b10101111;
    ADCON1 = 0b00000000;
    ADCON0 = 0b00000001;
}

void USARTconfig(){
    //UART Init
    Open1USART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE &
              USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_LOW, 7);
     
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR1 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Initial() {

    IOconfig();
    SPIconfig();
    CCPconfig();
    LCDconfig();
    ADCconfig();
    USARTconfig();
    TMRconfig();
    INTconfig();

    // Startup LEDs
    D6 = 0; //D6 Off
    D2 = 1; //D2 On
    Delay10KTCYx(125); //500ms Delay
    D2 = 0;//D2 Off
    D3 = 1;//D3 On
    Delay10KTCYx(125); //500ms Delay
    D3 = 0;//D3 Off
    D4 = 1;//D4 On
    Delay10KTCYx(125); //500ms Delay
    D4 = 0;//D4 Off
    
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
    if( PIE1bits.RCIE && PIR1bits.RCIF)
    {
        RxUartHandler();
    }
    
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR1IF and CCP1IF are clear.
 ******************************************************************************/
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if( PIR3bits.CCP1IF ) {
            CCP1handler();
            continue;
        }
        if( PIR4bits.CCP4IF ) {
            CCP4handler();
            continue;
        }
        if( INTCONbits.TMR0IF)
        {
            TMR0handler();
            continue;
        }
        if( PIR1bits.TMR1IF ) {
            TMR1handler();
            continue;
        }
        if( PIR1bits.TMR2IF ) {
            TMR2handler();
            continue;
        }
        break;
    }
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * Change the temporary data section for all code following this #pragma
 * statement. Normally the compiler/linker uses .tmpdata section for all non
 * ISR code and a separate tempdata section for each ISR (Hi and Lo). However,
 * when your primary ISR calls other functions the called functions share the
 * .tmpdata section with the main code. This can cause issues, just like not
 * saving WREG, STATUS and BSR. There are two options:
 *
 *   1. have the ISR save the .tmpdata section. This may have large effects on
 *      latency
 *   2. Force the functions called by the ISR to have their own temp data
 *      section.
 *
 * We are using option 2. However, this means that no main code functions can
 * appear after the following pragma, unless you change the temp data section
 * back.
 *
 * The temp data section is used for complex math operations such as 
 * a = b*c + d*e so you may not need a temp data section depending on what
 * you are doing.
 *
 * Keep in mind you may have the same issue with the MATH_DATA section.
 * MATH_DATA is used for arguments, return values and temporary locations
 * for math library functions.
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#pragma tmpdata handler_temp


void TMR0handler() {

    if (LEFT_SW)
    {
        if (left_count < 2440)
        {
            left_count++;
        }
        else
        {
            LEFT_LED = ~LEFT_LED;
            RIGHT_LED = 0;
            left_count = 0;
        }
    }
    // right
    if (RIGHT_SW)
    {
        if (right_count < 2440)
        {
            right_count++;
        }
        else
        {
            RIGHT_LED = ~RIGHT_LED;
            LEFT_LED = 0;
            right_count = 0;
        }
        //blink right
    }
    else
    if (!LEFT_SW) //Switch is in off of right state
    {
        if (left_off_count < 5)
        {
            left_off_count++;
        }
        else
        {
            LEFT_LED = 0;
            left_off_count = 0;
        }
    }
    if (!RIGHT_SW) //Switch is in off or left state
    {
        if (right_off_count < 5)
        {
            right_off_count++;
        }
        else
        {
            RIGHT_LED = 0;
            right_off_count = 0;
        }
    }
    INTCONbits.TMR0IF = 0;      //Clear flag and return
}

void TMR1handler() {
    TMR1H = 0xEF;
    TMR1L = 0xFF;

    TMR2 = 0xFF - 4*AccelX; //- AccelX;

    BRAKE = 1;
    PIR1bits.TMR1IF = 0;
    PIR1bits.TMR2IF = 0;
}

void TMR2handler() {
    // Stuff goes here
    TMR2 = 0;
    PIR1bits.TMR2IF = 0;
    BRAKE = 0;
}


void CCP1handler() {
    // Stuff goes here
    PIR3bits.CCP1IF = 0;      //Clear flag and return
}

void CCP4handler() {
    // Stuff goes here
    PIR4bits.CCP4IF = 0;      //Clear flag and return
}

void RxUartHandler()
{
    uart_rxbuffer[rx_counter] = RCREG;
    ++rx_counter;
    if(RCREG == '\n')
    {
        if(!strncmp(uart_rxbuffer, dumpString, 5))
        {
            dumpEEPROM();
        }
        if(!strncmp(uart_rxbuffer, leftString, 5))
        {
            LEFT_SW = 1;
            RIGHT_SW = 0;
        }
        if(!strncmp(uart_rxbuffer, rightString, 6))
        {
            RIGHT_SW = 1;
            LEFT_SW = 0;
        }
        if(!strncmp(uart_rxbuffer, offString, 3))
        {
            LEFT_SW = 0;
            RIGHT_SW = 0;
        }
        if(!strncmp(uart_rxbuffer, accelString, 6))
        {
            sendAccel();
        }
        else
        {
            sendTest();
        }
        rx_counter = 0;
    }
    PIR1bits.RCIF = 0;      //Clear flag and return
}