/*
 * File:   Main.c
 * Author: Miguel Duran, Mario Vargas & Alejandra Plascencia
 *
 * Created on 10 de marzo de 2018, 03:42 PM
 */


#include <xc.h>
#include "pic18f4431.h"
#include "ConfigurationBits.h"
#include "Datatypes.h"

#define pwmF TRISC2
#define pwmR TRISC1
#define PWM_PERIOD 255u

#define _ZEROS_MASK 0x00
#define _ONES_MASK 0xFF

// PROTOTYPE DECLARATIONS
void write_PWMF(uint8_t dutyCycle);
void write_PWMR(uint8_t dutyCycle);
void init_QEI(void);
void init_I2C(void);
void init_GPIO(void);
void init_PERIPH();
void enable_Interrupts(void);
void routineSelector(void);
uint8_t calculatePWM(uint16_t setpoint, 
        uint8_t positionlow, uint8_t positionhigh);
void sampleTimeInit(void);

//GENERAL PURPOSE REGISTER 
struct
{
    uint8_t I2CADD   :1;  //I2C address received flag
    uint8_t I2CDAT   :1;  //I2C data received flag
    uint8_t STCTRL   :1;  //Sample time flag
    uint8_t          :5; //padding
}GPREG;




// VARIABLES FOR PID CONTROLLER
int16_t Kp = 0; //Proportional gain
int16_t Ki = 0; //Integral gain 
int16_t Kd = 0; //Derivative gain
uint8_t setpoint = 0; //Reference
uint16_t sampletime = 0; //Sample time of the system in ms
uint8_t antPosition = 0; //Previous position
int8_t error = 0;


//I2C GlOBAL VARIABLES
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
    
    if(TMR0IE == 1 && TMR0IF == 1)
    {
        GPREG.STCTRL = 1; 
        TMR0 = 262140u - (sampletime*1000u)/4u; //Timer start value
        TMR0ON = 1; //Enables timer
        TMR0IF = 0; //Clear the Timer 0 flag
    }//end sample time interrupt
    
}//end interrupts handler

void main(void)
{
    init_GPIO();
    init_PERIPH();
    enable_Interrupts();
    while(1)
    {
        routineSelector();
    }//end while
    
}//end main

void routineSelector(void)
{
    if(GPREG.I2CDAT == 1)
    {
        //read the reference sent from master
        //change control reference
        GPREG.I2CDAT = 0;
    }//end if
    if(GPREG.STCTRL == 1)
    {
        uint8_t dutyCycle;
        dutyCycle = calculatePWM(setpoint,POSCNTL,POSCNTH);
        //if( dutyCycle >= threshold) write_PWMF(dutyCycle);
        // else write_PWMR(dutyCycle);
                
    }//end control 
    
}//end routine selector method

uint8_t calculatePWM(uint16_t setpoint, uint8_t positionlow, uint8_t positionhigh)
{
    int16_t intOut,derOut,output;
    uint16_t position;
    uint8_t outputByte;
    
    position = positionhigh; //Position high byte
    position <<= 8;
    position |= positionlow; //Position low byte
        
    error = setpoint - position; //Calculate the error taking the actual position 
    intOut += sampletime * error; //sum of the error for integral control
    derOut = (position-antPosition)/sampletime; //velocity formula
    
    output = Kp*error + Ki*intOut + Kd*derOut; //Calculation of the total output
    antPosition = position; //Assigning the actual position to the previous postion 
    
    outputByte = 0; //erase when finished adjusting control
    return outputByte;
}//end PWM calculation for PID controller

void sampleTimeInit(void)
{
     /* TIMER 0 CONTROL REGISTER
     * bit 7: TMR0 disabled
     * bit 6: 16 bit counter
     * bit 5: Internal CLKO
     * bit 4: Low to high transition
     * bti 3: prescaler assigned 
     * bti 2-0: 1:8 prescaler (to get 4 micro-seconds for every change in the timer)
     */ 
    T0CON = 0b00000010; 
    
    TMR0 = 262140u - (sampletime*1000u)/4u; //Calculation for interrupt at the sample time 
    TMR0ON = 1; //Enable the timer 0 
}//End Timer 0 configuration for sample time interrupt

void init_I2C()
{
    SSPCON =0b00110110; //I2C configuration bit-7 No collision,bit-6 No overflow
                         //bit-5 enable SP set SDA and SCL, bit 3-0 0110 I2C slave 7-bit addrs
    SSPADD = 0b10000000; //I2C address, add ranges from 001 -110
}//end I2C     


void write_PWMF(uint8_t dutyCycle)
{
    uint16_t timeON;
     pwmR = 1; //deshabilitate pwmR output
     NOP();
     pwmF = 0; 
    timeON = (dutyCycle*1024u)/100u;
    CCP1CONbits.DC1B = timeON % 4u;
    timeON = (uint8_t)timeON >> 2; 
    CCPR2L = timeON;
     
}//end Forward PWM write function 

void write_PWMR(uint8_t dutyCycle)
{
    uint16_t timeON;
    pwmF = 1; //deshabilitate pwmF output
    NOP();
    pwmR = 0;     
    timeON = (dutyCycle*1024u)/100u;
    CCP2CONbits.DC2B = timeON % 4u;
    timeON = (uint8_t)timeON >> 2; 
    CCPR2L = timeON;
    
}//end Reverse PWM write function


void init_PERIPH()
{   
    CCP1CON = (_CCP1CON_DC1B1_MASK & _ZEROS_MASK) | (_CCP1CON_DC1B0_MASK &_ZEROS_MASK)
            | _CCP1CON_CCP1M_MASK; //PWM Forward, 10 bit resolution
    T2CON = (_T2CON_TOUTPS_MASK & _ZEROS_MASK) | _T2CON_TMR2ON_MASK | _T2CON_T2CKPS_MASK;
    //Timer 2: postscaler = 1 prescaler  = 16
    CCP2CON = (_CCP2CON_DC2B1_MASK & _ZEROS_MASK) | (_CCP2CON_DC2B0_MASK &_ZEROS_MASK) 
            | _CCP2CON_CCP2M_MASK;//PWM Reverse 10 bit resolution
    PR2 = PWM_PERIOD;  //define maximum frequency   
}//end peripherals initialization function

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

void enable_Interrupts()
{
   SSPIE = 1; //Synchronoues serial port interrupt enable
   SSPIP = 1; //Synchronous serial port priority enable
   TMR0IE = 1; //
   TMR0IP = 1;  //
   IPEN=1;    //Interrupt priority enable
   GIE = 1;   //Global interrupt enable
}//end enable interrupts function