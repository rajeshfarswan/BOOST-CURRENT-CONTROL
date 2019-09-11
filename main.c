//**********************************************************//
//        AC LAB KIT BOOST CURRENT CONTROLLED DC-DC         //
//**********************************************************//

//include files
#include "p30f6010A.h"
#include "main.h"     // Include file containing processor registors definitions and function definitions
#include "asmMATH.h"  // Include definition of functions 


//
//user variables 
int Vcount = 50; //
int Vtick = 0;

int VDC1 = 0;
int VDC2 = 0;
int VDC = 0;
int VDC_max = 920; //180V //max dc link voltage protection limit

int PID_Isample = 0; //current PID
int PID_I_count =  4; //25Khz //4

int Ipv = 0;

int Irtrip = 130; //6A //maximum fault current setting

int IPreError = 0;

// PI gain variable
int Ipgain = 36;
int Iigain = 9;
//

int Ipre_err = 0;

int Ipi_out = 0;

int Iref = 21; //21==1A //current ref setting

int PWM_max =  14000; //14208
int PWM_min = -14000;

int SAT = 1776;

//

int main(void)
{

init(); // call processor initilisation code

PWMenable = 0; //reset PWM control register
PWM_InterruptEnable = 0;

ADC_ON = 1; //enable adc
FAULT_ENABLE = 1; //0x000f; //reset fault register

PWM1 = 0; //reset PWM registers
PWM2 = 0;
PWM3 = 0;
//precharging init ends


T1ON = 1; //enable timers
PWMenable = 1; //enable PWM
PWM_InterruptEnable = 1;

    while(1)
    {

//DC link voltage protection
if(Vtick >= Vcount)
	    {

	VDC = (VDC1+VDC2)>>1; //avg dc -link voltage

	if(VDC >= VDC_max) fault_Flag = 1; //set fault if vdc high

	Vtick = 0;
		     }

if(PID_Isample >= PID_I_count) //Current PI
             {

    IPreError = Ipre_err;
    Ipi_out = asmPIcontroller(Iref,Ipv,Ipgain,Iigain); //Current PI controller
    Ipre_err = IPreError;

    if(Ipi_out >=  PWM_max) Ipi_out =  PWM_max;
    if(Ipi_out <=  PWM_min) Ipi_out =  PWM_min;

    Ipi_out = Ipi_out>>3;
    Ipi_out = Ipi_out + SAT;

    asm("disi #0x3FFF");
    PWM1 =Ipi_out; //set duty cycle 1
    asm("disi #0x0000");
    
    PID_Isample = 0;
                     } //current sample end


                                  } //while end

                                      } //main end

//

////////////////////////////////////////////////////////////////////////////////

//T1 interrupt for program tracking
void _ISRFAST __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
  {     
     Vtick++; //dc link function counter
     PID_Isample++; //PI function counter
      
     T1us_Flag = 0;
   } //T1 interupt end

///////////////////////////////////////////////////////////////////////

//PWM interrupt for MPPT
void _ISRFAST __attribute__((interrupt, no_auto_psv)) _PWMInterrupt(void)
  {
 /*read DC link voltage from ADC*/  
	asm("MOV #0x0303, W0");       //select adc channel 3 for VDC1
	asm("MOV W0, ADCHS");    
	asm("BSET ADCON1,#1");        //start converstion
	asm("LOOP1:");
	asm("BTSS ADCON1,#0");        //wait for converstion
	asm("BRA LOOP1");
	asm("BTG ADCON1,#0"); 
	asm("MOV ADCBUF0, W0");   
	asm("MOV W0, _VDC1");

    asm("MOV #0x0202, W0");       //select adc channel 2 VDC2
	asm("MOV W0, ADCHS");    
	asm("BSET ADCON1,#1");        //start converstion
	asm("LOOP2:");
	asm("BTSS ADCON1,#0");        //wait for converstion
	asm("BRA LOOP2");
	asm("BTG ADCON1,#0"); 
	asm("MOV ADCBUF0, W0");   
	asm("MOV W0, _VDC2");

/*read input inductor current from ADC*/
   
	asm("MOV #0x0808, W0"); //0x0b0b //read channel 11 //input inductor current Ir
	asm("MOV W0, ADCHS");    
	asm("BSET ADCON1,#1");        //start converstion
	asm("LOOPb:");
	asm("BTSS ADCON1,#0");        //wait for converstion
	asm("BRA LOOPb");
	asm("BTG ADCON1,#0"); 
	asm("MOV ADCBUF0, W0");
    asm("SUB W5,W0,W0");
    asm("CLR W6");                //check greater than zero
    asm("CPSGT W0, W6");
    asm("CLR W0");
    asm("MOV _Irtrip, W6");       //read current max trip limit
    asm("CPSLT W0, W6");
    asm("BSET _IFS2+1,#4");       //set fault if high
	asm("MOV W0, _Ipv");          //copy inductor current to Ipv

PWM_Flag = 0;

   } //PWM interupt end

///////////////////////////////////////////////////////////////////////

//fault interrupt
void _ISRFAST __attribute__((interrupt, no_auto_psv)) _FLTBInterrupt(void)
   {
     
     PWMenable = 0; //disable pwm if fault set
     SET = 0;       //all switches off
     
ClrWdt();

fault_Flag = 0;
   }//fault end

///////////////////////////////////////////////////////////////////////


