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
float FC_out = 0, FM_Variable = 0, FError = 0,  FPrevious_Error;
float dt = 0.01,  DTM, Kp = 10, Ki = 0.5, Kd = 2, Integral = 11, Derivative = 0;
float FKp = 10, FKi = 0.5, FKd = 2, FIntegral = 11, FDerivative = 0;
/******************************************************************************/

int8 SPI_Flag = 0, ADC_Flag = 0, Byte_Count = 0, Rx, Tx, Cmand, ProbeID = 1,count = 0;
unsigned int8 Version = 2,SP = 0, SP_H = 0, FSP = 0, FSP_H = 0;
unsigned int Value, Duty = 0, FDuty = 0, Err_cnt = 0, ViewFMV, ViewMV, Set_Point, FSet_Point, Old_SP;
unsigned char MV = 0, MVH = 0,  FMVH = 0, FMV = 10;

float Flange;
/******************************************************************************/
// 8 bits SPI
#INT_SPI2 level = 7

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
                else if(Cmand == 5)
                    spi_prewrite(FMV);                
                break;
            
            case 3: 
                 
                if(Cmand == 1)
                {SP_H = Rx;
                    spi_prewrite(MVH);}
                else if(Cmand == 3)
                   spi_prewrite(0);
                else if(Cmand == 5)
                    {FSP_H = Rx;
                    spi_prewrite(FMVH);}
                break;      
            
            
            case 4: 
                Byte_Count = 0;            
                if(Cmand == 1)
                    SP = Rx;
                else if(Cmand == 3)
                    spi_prewrite(50);
                else if(Cmand == 5)
                    FSP = Rx;
                break;
                
            default:
                Byte_Count = 0;
                break;
            }

}

#INT_TIMER1 
void  timer1_isr(void) 
{
    M_Variable= ((float)read_adc() * Alpha) - 4;  // 4 is the offset
    FM_Variable = ((float)read_adc2() * Alpha);
    ADC_Flag = 1;
    
}

void main()
{    
   output_float(PIN_G9); // SS as an input
   setup_adc_ports(sAN0, VSS_VDD);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(0);
   
   setup_adc_ports2(sAN2, VSS_VDD);
   setup_adc2(ADC_CLOCK_INTERNAL);
   set_adc_channel2(2);
   
   // Timer 1 for 10 ms INT when clock is 100MHz
   setup_timer1(TMR_INTERNAL | TMR_DIV_BY_64, 7812);

   
   setup_timer2(TMR_INTERNAL | TMR_DIV_BY_64, 500);
   setup_compare(2, COMPARE_PWM | COMPARE_TIMER2);
   set_pwm_duty(2,0);
   
   setup_compare(3, COMPARE_PWM | COMPARE_TIMER2);
   set_pwm_duty(3,0);
   
   
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
      
        if(FSP_H)
            FSet_Point =  (float)FSP + 256;
        else
            FSet_Point = (float)FSP;
      
      if(ADC_Flag)
      {
         Error = Set_Point - M_Variable;
            if(Old_SP != Set_Point || Integral < 0)
            Integral = 0;

         Old_SP = Set_Point;
         if(Error < 10)
         Integral = Integral + (Error * dt);
         ADC_Flag = 0;
      }
      
      Value = (unsigned int16)M_Variable;
      ViewMV = Value;
      MV  = (unsigned char)Value;
      MVH = Value >> 8; 
      

     Value = (unsigned int16)FM_Variable;
     ViewFMV = Value;
     FMV = (unsigned char)Value;
     FMVH = Value >> 8;

     FError = FSet_Point - FM_Variable;

      C_out = (Kp * Error) + (Ki * Integral);
      FC_out = (FKp * FError);
     
      if(C_out > 500)
          C_out = 500;
      else if(C_out < 0)
          C_out = 0;
     
      if(FC_out > 500)
          FC_out = 500;
      else if(FC_out < 0)
          FC_out = 0;
        
        Duty = (int)C_out;
        set_pwm_duty(2,Duty);
        
        FDuty = (int)FC_out;
        set_pwm_duty(3,FDuty);

    }
}   
