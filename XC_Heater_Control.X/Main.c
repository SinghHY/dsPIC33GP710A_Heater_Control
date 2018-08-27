/**********************************************************************
* Corona Control 
* © PerkinElmer Health Sciences Canada, Inc., 2017
* This program is Corona Voltage control. 
* FileName:        Main.c
* Processor:       dsPIC33FJ256GP510
* Compiler:        XC16 v1.32 or Higher
* Version:     
************************************************************************/
#include "p33Fxxxx.h"
#include <spi.h>
#include <outcompare.h>
#include <timer.h>

_FGS(GSS_OFF & GWRP_OFF); //code protect off,write protect disabled
_FOSCSEL(FNOSC_PRIPLL & IESO_OFF);
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF & POSCMD_HS);
_FWDT(FWDTEN_OFF);//watch dog disabled

unsigned char version = 20;   //new version for corona//
int txdData1, rxdData1, SPIFlag = 0, CommandCounter1 = 0;

/* **********************   SPI Interrupt *************************/
void __attribute__((__interrupt__,no_auto_psv)) _SPI2Interrupt(void) 
{    
    IFS2bits.SPI2IF = 0;
    SPI2STATbits.SPIROV = 0;  // Clear SPI1 receive overflow flag if set //
    rxdData1 = SPI2BUF;
    
    SPIFlag =1;
    if (!SPI2STATbits.SPITBF)
                SPI2BUF = txdData1;

}

unsigned short ADC_Read(unsigned short ch)
  {
    unsigned short ADC_out;
    AD1CHS0 = ch;//select AN0 //
    AD1CON1bits.SAMP = 1; // start sampling  ADC1
    while (!AD1CON1bits.DONE);
    AD1CON1bits.DONE = 0;
    ADC_out = ADC1BUF0;
    ADC_out >>= 2; //adjust adc from 10 bit to 8 bit value//
    return ADC_out;
  }


void main(void)
{

    int Instruction1; // 1,2,3


    unsigned char TempSetpointLow=0;
    unsigned char TempSetpointHi=0;
    unsigned char VoltageMonitorLow1=0;
    unsigned char VoltageMonitorHi1=0;
    unsigned char HVSetpointLow1=0;
    unsigned char HVSetpointHi1=0;
    unsigned char APCIRealTime =0;
    unsigned char ProbeId=0x0;
    unsigned char version = 19;   //new version for corona//

    SPI2CON1bits.SSEN = 1; // slave select pin enabled//
    SPI2CON1 = 0x8080;  // enable slave, mode8, cke=1, ckp = 0, smp = 0
    SPI2STAT = 0x8000;  // enables the spi
 
    TRISG = 0x00;
    TRISG = 0x00;
    
    //init PWM for HV1 and HV2 , fc 90KHZ //
    CloseOC1();
    ConfigIntOC1(OC_INT_OFF );
    OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 298, 300);

    CloseOC2();
    ConfigIntOC2(OC_INT_OFF );
    OpenOC2(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 298, 300);


    //init timer2, 0x175 for 90khz//
    ConfigIntTimer2(T2_INT_OFF);
    WriteTimer2(0);
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_1 ,  300);
    T2CON = 0x8000;
    SetPulseOC1(0x0 , 2 );//2
    SetPulseOC2(0x0 , 2 );//2

    //init PWM for Temp1 and Temp2 , fc 60 HZ //
    CloseOC3();
    ConfigIntOC3(OC_INT_OFF );
    OpenOC3(OC_IDLE_CON & OC_TIMER3_SRC & OC_CONTINUE_PULSE, 0xffff, 0xfffd);
    SetPulseOC3(0x0, 0xfffd);

    //init timer3, 0xffff for 60Hz//
    ConfigIntTimer3(T3_INT_OFF);
    WriteTimer3(0);
    OpenTimer3(T3_ON & T3_GATE_OFF & T3_PS_1_1 ,  0xffff);
    T3CON = 0x8010;


    //init ADC channels for AN0 and AN2 //
    AD1CON1 = 0x00e0;
    AD1CON2 = 0x0000;
    AD1CON3 = 0x1f02;
    AD1PCFGL = 0xfffa; //AN0 and AN2 ADC enabled 
    AD1PCFGH = 0xffff; //needed for probe ID reading //
    AD1CSSH = 0x0000;     
    AD1CSSL = 0x0000;  //ano and an2 scan          
    AD1CON1bits.ADON =1; // ADC1 on


    /* Configure SPI2 interrupt */
    ConfigIntSPI2(SPI_INT_EN &  SPI_INT_PRI_6);
  while(1)
  {

    if ( PORTEbits.RE1 == 0 )   //reading POL2 corona HV//
            PORTGbits.RG15 = 0;
     else
            PORTGbits.RG15 = 1;

 //Now its the time to read APCI and ESI inputs//
 
    if (( PORTEbits.RE6 == 1 ) || ( PORTEbits.RE7 == 1 ))   //reading APCI-1 and APCI-2//
            APCIRealTime = 1 ; 
  
     else
            APCIRealTime = 0 ;

  
         //Time to check SPI2 for any new data//
         //time to check spi flag//

    if(SPIFlag == 1)
    {   SPIFlag = 0;
        
    switch(CommandCounter1) 
        {
          case 4: 
                   CommandCounter1 = 0;
                   txdData1 = version;
                   if(Instruction1 == 1)
                       TempSetpointLow = rxdData1;
                   else if((Instruction1 == 2) ^ (Instruction1 == 12))
                            HVSetpointLow1 = rxdData1;
                   break;
          case 1:      
                   Instruction1 = rxdData1;
                   if ((Instruction1 == 1) ^ (Instruction1 == 11))
                        txdData1=0;
                   else if ((Instruction1 == 2) ^ (Instruction1 == 12))
                            txdData1=VoltageMonitorLow1;
                   else if (Instruction1 == 3)
                            txdData1=ProbeId;
                   break;
          case 2:   
                   if ((Instruction1 == 1)^(Instruction1 == 11))	
                        txdData1=0;
                   else if ((Instruction1 == 2)^(Instruction1 == 12))
                         txdData1=0;     
                   else if (Instruction1 == 3)
                         txdData1=0;
                   break;
          case 3:
                   txdData1=0;
                    if (Instruction1 == 1)
						  TempSetpointHi = rxdData1;
                    else if  (Instruction1 == 2)
                              HVSetpointHi1 = rxdData1;
                    break;
          default:
                    CommandCounter1 = 0;
                    break;
        }  
    }

                    
  }//for while

}
//for main