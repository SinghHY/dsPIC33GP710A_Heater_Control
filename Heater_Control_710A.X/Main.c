/**********************************************************************
* Temperature Control 
* © PerkinElmer Health Sciences Canada, Inc., 2017
* This program is for Heater control U26 on old Board. 
* FileName:        Main.c
* Processor:       dsPIC33FJ256GP510A
* Compiler:        CCS v5.074 or Higher
* Version:     
************************************************************************/

#include <33FJ256GP710A.h>
#fuses XT,NOWDT,NOPROTECT
#device ADC = 12 
#use delay(clock = 100MHz, crystal = 40MHz)
#use spi(SLAVE, SPI2, BITS = 8, MODE = 1, ENABLE = PIN_G9, stream = SPI_2)


/******************************************************************************/
// ADC Parameters
// Alpha = 0.1611328125 = 660/4096 ; 660 is maximum temperature value; 5mV/ 1C change
float Alpha = 0.1611328125;
/******************************************************************************/
// PID Parameters//
float C_out = 0, M_Variable = 0, Error = 0,  Previous_Error;
float dt = 0.01,  DTM, Kp = 10, Ki = 0.5, Kd = 2, Integral = 11, Derivative = 0;
/******************************************************************************/

int8 SPI_Flag = 0, Byte_Count = 0, Rx, Tx, Cmand, ProbeID = 1,count = 0;
unsigned int8 Version = 2,SP = 0, SP_H = 0;
unsigned int Value, Duty = 0, Err_cnt = 0, ViewIntegral, ViewError, Set_Point, Old_SP;
unsigned char MV , MVH;

/******************************************************************************/
// 8 bits SPI
#INT_SPI2

void spi2_slave_isr(void)
{
  Rx = spi_xfer_in(SPI_2, 8);
  Byte_Count++; 
 
  switch(Byte_Count)
            {
            case 1:
                spi_prewrite(Version);
                if(Cmand == 1)
                    spi_prewrite(0);
                else if(Cmand == 2)
                    spi_prewrite(0);
                break;
      
            case 2: 
                Cmand = Rx;
                if(Cmand == 1)
                    spi_prewrite(MV);
                else if(Cmand == 3)
                        spi_prewrite(ProbeID);
                break;
            
            case 3: 
                 
                if(Cmand == 1)
                {SP_H = Rx;
                    spi_prewrite(MVH);}
                else if(Cmand == 3)
                   spi_prewrite(0);
                break;      
            
            
            case 4: 
                Byte_Count = 0;            
                if(Cmand == 1)
                    SP = Rx;
                else if(Cmand == 3)
                    spi_prewrite(50);
                break;
                
            default:
                Byte_Count = 0;
                break;
            }

}

#INT_TIMER1 fast
void  timer1_isr(void) 
{
    M_Variable= ((float)read_adc() * Alpha) - 6;  // 6 is the offset
    Error = Set_Point - M_Variable;
    if(Old_SP != Set_Point || Integral < 0)
            Integral = 0;

    Old_SP = Set_Point;
    if(Error < 10)
    Integral = Integral + (Error * dt);
}

void main()
{    
   output_float(PIN_G9); // SS as an input
   setup_adc_ports(sAN0, VSS_VDD);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(0);
   
   // Timer 1 for 10 ms INT when clock is 100MHz
   setup_timer1(TMR_INTERNAL | TMR_DIV_BY_64, 7812);
   // Timer 1 for 1ms INT when clock is 100MHz
   //setup_timer1(TMR_INTERNAL | TMR_DIV_BY_64, 781);
   
   setup_timer2(TMR_INTERNAL | TMR_DIV_BY_64, 500);
   setup_compare(2, COMPARE_PWM | COMPARE_TIMER2);
   set_pwm_duty(2,0);
   
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_SPI2);
   enable_interrupts(INTR_GLOBAL);
   
   DTM = 1/dt;
  
   while(1)
    {
       
      if(SP_H)
         Set_Point =  (float)SP + 256;
      else
         Set_Point = (float)SP;
      
      Value = (unsigned int16)M_Variable;
      MV  = (unsigned char)Value;
      MVH = Value >> 8; 
      

      /*if(Integral > 1000)
          Integral = 1000;*/

      
      ViewIntegral = (unsigned int)Integral;
      ViewError = (unsigned int)Error;
      C_out = (Kp * Error) + (Ki * Integral);
     
      if(C_out > 500)
          C_out = 500;
      else if(C_out < 0)
          C_out = 0;
      
      //  Version = (unsigned int8)C_out;
        Duty = (int)C_out;
        set_pwm_duty(2,Duty);

    }
}   