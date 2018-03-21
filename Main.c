/*
 * File:   Main.c
 * Author: MiguelAng
 *
 * Created on 10 de marzo de 2018, 03:42 PM
 */


#include <xc.h>
#include "pic18f4431.h"
#include "ConfigurationBits.h"
#include "Datatypes.h"

#define pwmF TRISC2
#define pwmR TRISC1

void write_PWM(uint8_t dutyCycle);
void init_QEI(void);
void init_I2C(void);
void init_GPIO(void);
void init_PERIPH();
void enable_Interrupts(void);
void routineSelector(void);

struct
{
    uint8_t I2CADD   :1;  //I2C address received flag
    uint8_t I2CDAT   :1;  //I2C data received flag
    uint8_t          :6; //padding
}GPREG;

uint8_t i2cData;


void interrupt ISR_high(void)
{
    if(SSPIE == 1 && SSPIF == 1)
    {
        CKP = 0; //hold clock
        if(SSPOV == 1 || WCOL == 1)
        {
            i2cData = SSPBUF;
            SSPOV = 0;
            WCOL = 0;
            CKP = 1;
        }//end overrund and word collision scenario
            
        if(SSPBUF == SSPADD)
        {
           //i2cData=SSPBUF;
           GPREG.I2CADD = 1;
           BF = 0;
           CKP = 1;
        }//end address identefier scenario
        
        if(GPREG.I2CADD == 1)
        {
            i2cData = SSPBUF;
            CKP = 1;
            GPREG.I2CADD = 0;
        }
        SSPIF = 0;
    }//end I2C communication  
    
}//end interrupts handler

void main(void)
{
    init_GPIO();
    init_PERIPH();
    enable_Interrupts();
    write_PWM(50);
    while(1)
    {
        routineSelector();
    }
    
}//end main

void routineSelector(void)
{
    if(GPREG.I2CDAT == 1)
    {
        //read the reference sent from master
        //change control reference
        GPREG.I2CDAT = 0;
    }//end if
    
}//end routine selector method

void init_I2C()
{
    SSPCON =0b00110110; //I2C configuration bit-7 No collision,bit-6 No overflow
                         //bit-5 enable SP set SDA and SCL, bit 3-0 0110 I2C slave 7-bit addrs
    SSPADD = 0b10000000; //I2C address, add ranges from 001 -110
}//end I2C     


void init_GPIO()
{
    LATC2 = 0;  //Output register
    RC2 = 0;    // Input register
    LATC1 = 0;
    RC1 = 0;
    TRISD3 = 1; //SCL pin
    TRISD2 = 1; //SDA pin
    pwmF = 0; // PWM/ forward /CCP1 pin
    pwmR = 0; //PWM reverse /CCP2 pin
}//end gpio_init

void init_PERIPH()
{
    CCP1CON = 0b00001100; //PWM: duty cycle in 8 bits
    T2CON = 0b00000110; //PWM: postscaler = 1 prescaler  = 16
    CCP2CON = 0b00001100; //PWM reverse
    
}//end peripherals initialization function


void write_PWM(uint8_t dutyCycle)
{
     PR2 = 125;  //period of 1 ms counts every 8 us
     CCPR1L = dutyCycle*125/100;
     
}//end PWM write dutyCycle function


void enable_Interrupts()
{
   SSPIE = 1; //Synchronoues serial port interrupt enable
   SSPIP = 1; //Synchronous serial port priority enable
   IPEN=1;    //Interrupt priority enable
   GIE = 1;   //Global interrupt enable
}//end enable interrupts function
