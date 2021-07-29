//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "IQmathLib.h"
#include "resonant_converter.h"
#include <math.h>

//
// Typedefs
//
typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
} EPWM_INFO;

//
// Prototype statements
//
__interrupt void cpu_timer0_isr(void);
__interrupt void main_isr(void);
__interrupt void epwm3_isr(void);

void InitEPwmTimer(void);
void update_frequency_and_duty_ratio(EPWM_INFO*);
void InitEPwm3();
void pwm_trip();
void pwm_trip_clear();


#define MATH_TYPE  IQ_MATH

//
// Maximum Dead Band Defines
//
#define EPWM3_DB   0x000A
#define ISR_FREQUENCY 100   //isr frequency in khz


#define PWM1_TIMER_TBPRD   300

#define PI 3.14159265358979
//
// Defines that keep track of which way the Dead Band is moving
//
#define DB_UP   1
#define DB_DOWN 0

//
// Defines default frequency and duty ratio
//
#define EPWM3_DEFAULT_TBPRD  300
#define EPWM3_DEFAULT_CMPA   150

#define   GLOBAL_Q       16

//

//
// Globals
//
Uint16 EPwm3_DB_Direction;
Uint16 frequency_in_clk_cycles;
Uint16 duty_ratio_in_clk_cycles;
Uint16 frequency = 300;
Uint16 duty = 150;
float32 T = 0.001/ISR_FREQUENCY;    // Sampling period (sec), see parameter.h

_iq j_max = _IQ(0.8);
_iq r1_inverse_lookup[100] = {_IQ(0.8944),_IQ(0.9025),_IQ(0.9107),_IQ(0.9190),_IQ(0.9275),_IQ(0.9361),_IQ(0.9448),_IQ(0.9537),_IQ(0.9627),_IQ(0.9719),_IQ(0.9812),_IQ(0.9907),_IQ(1.0003),_IQ(1.0101),_IQ(1.0201),_IQ(1.0303),_IQ(1.0406),_IQ(1.0511),_IQ(1.0618),_IQ(1.0727),_IQ(1.0838),_IQ(1.0951),_IQ(1.1066),_IQ(1.1182),_IQ(1.1301),_IQ(1.1423),_IQ(1.1546),_IQ(1.1671),_IQ(1.1799),_IQ(1.1930),_IQ(1.2062),_IQ(1.2197),_IQ(1.2335),_IQ(1.2475),_IQ(1.2617),_IQ(1.2763),_IQ(1.2911),_IQ(1.3061),_IQ(1.3214),_IQ(1.3371),_IQ(1.3529),_IQ(1.3691),_IQ(1.3856),_IQ(1.4023),_IQ(1.4193),_IQ(1.4366),_IQ(1.4542),_IQ(1.4721),_IQ(1.4903),_IQ(1.5087),_IQ(1.5274),_IQ(1.5464),_IQ(1.5656),_IQ(1.5851),_IQ(1.6048),_IQ(1.6247),_IQ(1.6447),_IQ(1.6650),_IQ(1.6854),_IQ(1.7058),_IQ(1.7264),_IQ(1.7469),_IQ(1.7674),_IQ(1.7878),_IQ(1.8081),_IQ(1.8280),_IQ(1.8477),_IQ(1.8668),_IQ(1.8854),_IQ(1.9033),_IQ(1.9204),_IQ(1.9364),_IQ(1.9511),_IQ(1.9645),_IQ(1.9761),_IQ(1.9858),_IQ(1.9933),_IQ(1.9981),_IQ(2.0000),_IQ(1.9985),_IQ(1.9932),_IQ(1.9836),_IQ(1.9692),_IQ(1.9494),_IQ(1.9236),_IQ(1.8913),_IQ(1.8518),_IQ(1.8044),_IQ(1.7484),_IQ(1.6833),_IQ(1.6083),_IQ(1.5229),_IQ(1.4266),_IQ(1.3187),_IQ(1.1988),_IQ(1.0662),_IQ(0.9203),_IQ(0.7593),_IQ(0.5800),_IQ(0.3714)};
_iq r2_inverse_lookup[100] = {_IQ(0.8944),_IQ(0.8865),_IQ(0.8787),_IQ(0.8710),_IQ(0.8634),_IQ(0.8559),_IQ(0.8486),_IQ(0.8413),_IQ(0.8342),_IQ(0.8272),_IQ(0.8202),_IQ(0.8134),_IQ(0.8067),_IQ(0.8000),_IQ(0.7935),_IQ(0.7870),_IQ(0.7807),_IQ(0.7744),_IQ(0.7682),_IQ(0.7621),_IQ(0.7560),_IQ(0.7501),_IQ(0.7442),_IQ(0.7384),_IQ(0.7327),_IQ(0.7270),_IQ(0.7214),_IQ(0.7159),_IQ(0.7105),_IQ(0.7051),_IQ(0.6998),_IQ(0.6945),_IQ(0.6893),_IQ(0.6842),_IQ(0.6791),_IQ(0.6741),_IQ(0.6691),_IQ(0.6642),_IQ(0.6593),_IQ(0.6545),_IQ(0.6497),_IQ(0.6450),_IQ(0.6403),_IQ(0.6357),_IQ(0.6311),_IQ(0.6265),_IQ(0.6220),_IQ(0.6176),_IQ(0.6131),_IQ(0.6087),_IQ(0.6043),_IQ(0.6000),_IQ(0.5957),_IQ(0.5914),_IQ(0.5871),_IQ(0.5829),_IQ(0.5787),_IQ(0.5745),_IQ(0.5703),_IQ(0.5662),_IQ(0.5620),_IQ(0.5579),_IQ(0.5538),_IQ(0.5497),_IQ(0.5455),_IQ(0.5414),_IQ(0.5373),_IQ(0.5331),_IQ(0.5290),_IQ(0.5248),_IQ(0.5206),_IQ(0.5164),_IQ(0.5122),_IQ(0.5079),_IQ(0.5035),_IQ(0.4991),_IQ(0.4946),_IQ(0.4901),_IQ(0.4854),_IQ(0.4807),_IQ(0.4758),_IQ(0.4708),_IQ(0.4656),_IQ(0.4602),_IQ(0.4546),_IQ(0.4487),_IQ(0.4425),_IQ(0.4359),_IQ(0.4288),_IQ(0.4212),_IQ(0.4129),_IQ(0.4038),_IQ(0.3935),_IQ(0.3819),_IQ(0.3684),_IQ(0.3524),_IQ(0.3326),_IQ(0.3070),_IQ(0.2714),_IQ(0.2140)};
/*_iq r1_inverse_lookup[100] = {_IQ(0.8944),_IQ(0.9025),_IQ(0.9107),_IQ(0.9190),_IQ(0.9275),_IQ(0.9361),_IQ(0.9448),_IQ(0.9537),_IQ(0.9627),_IQ(0.9719),_IQ(0.9812),_IQ(0.9907),_IQ(1.0003),_IQ(1.0101),_IQ(1.0201),_IQ(1.0303),_IQ(1.0406),_IQ(1.0511),_IQ(1.0618),_IQ(1.0727),_IQ(1.0838),_IQ(1.0951),_IQ(1.1066),_IQ(1.1182),_IQ(1.1301),_IQ(1.1423),_IQ(1.1546),_IQ(1.1671),_IQ(1.1799),_IQ(1.1930),_IQ(1.2062),_IQ(1.2197),_IQ(1.2335),_IQ(1.2475),_IQ(1.2617),_IQ(1.2763),_IQ(1.2911),_IQ(1.3061),_IQ(1.3214),_IQ(1.3371),_IQ(1.3529),_IQ(1.3691),_IQ(1.3856),_IQ(1.4023),_IQ(1.4193),_IQ(1.4366),_IQ(1.4542),_IQ(1.4721),_IQ(1.4903),_IQ(1.5087),_IQ(1.5274),_IQ(1.5464),_IQ(1.5656),_IQ(1.5851),_IQ(1.6048),_IQ(1.6247),_IQ(1.6447),_IQ(1.6650),_IQ(1.6854),_IQ(1.7058),_IQ(1.7264),_IQ(1.7469),_IQ(1.7674),_IQ(1.7878),_IQ(1.8081),_IQ(1.8280),_IQ(1.8477),_IQ(1.8668),_IQ(1.8854),_IQ(1.9033),_IQ(1.9204),_IQ(1.9364),_IQ(1.9511),_IQ(1.9645),_IQ(1.9761),_IQ(1.9858),_IQ(1.9933),_IQ(1.9981),_IQ(2.0000),_IQ(1.9985),_IQ(1.9932),_IQ(1.9836),_IQ(1.9692),_IQ(1.9494),_IQ(1.9236),_IQ(1.8913),_IQ(1.8518),_IQ(1.8044),_IQ(1.7484),_IQ(1.6833),_IQ(1.6083),_IQ(1.5229),_IQ(1.4266),_IQ(1.3187),_IQ(1.1988),_IQ(1.0662),_IQ(0.9203),_IQ(0.7593),_IQ(0.5800),_IQ(0.3714)};
_iq r2_inverse_lookup[100] = {_IQ(0.8944),_IQ(0.8865),_IQ(0.8787),_IQ(0.8710),_IQ(0.8634),_IQ(0.8559),_IQ(0.8486),_IQ(0.8413),_IQ(0.8342),_IQ(0.8272),_IQ(0.8202),_IQ(0.8134),_IQ(0.8067),_IQ(0.8000),_IQ(0.7935),_IQ(0.7870),_IQ(0.7807),_IQ(0.7744),_IQ(0.7682),_IQ(0.7621),_IQ(0.7560),_IQ(0.7501),_IQ(0.7442),_IQ(0.7384),_IQ(0.7327),_IQ(0.7270),_IQ(0.7214),_IQ(0.7159),_IQ(0.7105),_IQ(0.7051),_IQ(0.6998),_IQ(0.6945),_IQ(0.6893),_IQ(0.6842),_IQ(0.6791),_IQ(0.6741),_IQ(0.6691),_IQ(0.6642),_IQ(0.6593),_IQ(0.6545),_IQ(0.6497),_IQ(0.6450),_IQ(0.6403),_IQ(0.6357),_IQ(0.6311),_IQ(0.6265),_IQ(0.6220),_IQ(0.6176),_IQ(0.6131),_IQ(0.6087),_IQ(0.6043),_IQ(0.6000),_IQ(0.5957),_IQ(0.5914),_IQ(0.5871),_IQ(0.5829),_IQ(0.5787),_IQ(0.5745),_IQ(0.5703),_IQ(0.5662),_IQ(0.5620),_IQ(0.5579),_IQ(0.5538),_IQ(0.5497),_IQ(0.5455),_IQ(0.5414),_IQ(0.5373),_IQ(0.5331),_IQ(0.5290),_IQ(0.5248),_IQ(0.5206),_IQ(0.5164),_IQ(0.5122),_IQ(0.5079),_IQ(0.5035),_IQ(0.4991),_IQ(0.4946),_IQ(0.4901),_IQ(0.4854),_IQ(0.4807),_IQ(0.4758),_IQ(0.4708),_IQ(0.4656),_IQ(0.4602),_IQ(0.4546),_IQ(0.4487),_IQ(0.4425),_IQ(0.4359),_IQ(0.4288),_IQ(0.4212),_IQ(0.4129),_IQ(0.4038),_IQ(0.3935),_IQ(0.3819),_IQ(0.3684),_IQ(0.3524),_IQ(0.3326),_IQ(0.3070),_IQ(0.2714),_IQ(0.2140)};*/

_Bool MODE = 0;
_Bool start = 0;
_Bool stop = 1;
_Bool pwm_trip_flag = 0;  //high when pwm is tripped

EPWM_INFO epwm3_info;

PI_CONTROLLER pi_i_output = {0, 0,0,_IQ(0.0006),_IQ(0.0006), _IQ(0.3),_IQ(-2.0), _IQ(0.0),_IQ(0.0),_IQ(0.0),_IQ(0.0), _IQ(1.0)};

Uint32 IsrTicker = 0;

// Global variables used in this system

_iq output_current = 0;

_iq m_o = _IQ(0);
_iq frequency_to_clk_factor = _IQ(318);
_iq frequency_to_clk_factor_div_pi = _IQ(101);
_iq output_current_reference = _IQ(2);
Uint16 m_o_index = 0;

_iq delta_v = _IQ(0);
_iq r1_inverse = _IQ(1);
_iq r2_inverse = _IQ(1);
_iq count = _IQ(0);
_iq theta = _IQ(0);
_iq delta = _IQ(0);
_iq total_angle = _IQ(0);

_Bool theta_state = 0;

_iq theta_limit = _IQ(0.45*PI);
_iq iq_pi = _IQ(PI);

_iq temp0 = _IQ(0);
_iq temp1 = _IQ(0);
_iq temp2 = _IQ(0);
_iq temp3 = _IQ(0);

// Default ADC initialization
int ChSel[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int TrigSel[16] = { 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9 };
int ACQPS[16] = { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6 };

//
// Main
//
void main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2803x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2803x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //
    // For this case just init GPIO pins for ePWM3
    // These functions are in the DSP2803x_EPwm.c file
    //
    InitEPwm3Gpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2803x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2803x_DefaultIsr.c.
    // This function is found in DSP2803x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    IER = 0x0000;
    IFR = 0x0000;

    EALLOW;
    // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.EPWM1_INT = &main_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;
    EDIS;
    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // Not required for this example
    //
    InitCpuTimers();   // initialize the Cpu Timers for start and stop signal
    //
    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    // 60MHz CPU Freq, 50 millisecond Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 60, 60*1000000);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in DSP2803x_CpuTimers.h), the
    // below settings must also be updated.
    //

    //
    // Use write-only instruction to set TSS bit = 0
    //
    CpuTimer0Regs.TCR.all = 0x4001;


    InitEPwmTimer();    // For this example, only initialize the ePWM Timer

    //
    // Configure GPIO34 as a GPIO output pin
    //
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31= 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO30= 0;
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
    EDIS;

    InitEPwm3();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //
    // Step 5. User specific code, enable interrupts
    // Initialize counters:
    //

    // Initialize ADC for DMC Kit Rev 1.1    0 --> A0, 7 --> A7, 8 --> B0, 15 --> B7
    // Reading results: 0V --> 0x00, 3.3V --> 0xFFF
    // EOC triggered every 4th conversion by default --

    //potential improvements - (1 simultaneous measurement during start up and 1 independent measurement during current control) - will save 7 clock cycles for sampling

    ChSel[0] = 7;     //  ChSelect: ADC A7 -> Output Voltage
    ChSel[1] = 15;    //  ChSelect: ADC B7 -> Input Voltage
    ChSel[2] = 12;    //  ChSelect: ADC B4 -> Load Current

    ADC_MACRO_INIT(ChSel, TrigSel, ACQPS);

    // Initialize the PI module for load current
    pi_i_output.Kp = _IQ(0.0006);
    pi_i_output.Ki = _IQ(T * 61);
    pi_i_output.Umax = _IQ(0.3);
    pi_i_output.Umin = _IQ(-2.0);

    EALLOW;

    // Enable EOC interrupt(after the 3rd conversion)
    AdcRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcRegs.INTSEL1N2.bit.INT1CONT = 1;
    AdcRegs.INTSEL1N2.bit.INT1SEL = 3;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 3;

    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    //
    IER |= M_INT3;

    // Enable CPU INT1 for timer0
    IER |= M_INT1;

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 3
    //
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    //
    // Enable CPU 0 Timer interrupt in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;


    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM
    EDIS;
    //
    // Step 6. IDLE loop. Just sit and loop forever (optional):
    //
    for (;;)
    {
        __asm("          NOP");
    }
}

//
// Main Isr - Main Isr containing all controls
//
__interrupt void
main_isr()
{
    //GpioDataRegs.GPADAT.bit.GPIO30 = 1;
    IsrTicker++;

    m_o = _IQdiv(_IQ12toIQ(AdcResult.ADCRESULT0 - 10) , _IQ12toIQ(AdcResult.ADCRESULT1 + 6)); //ADCRESULT0 - output voltage, ADCRESULT1 - input voltage*/
    m_o_index = (Uint16) _IQmpy(m_o,100);
     if(AdcResult.ADCRESULT0 > 725) // 725 corresponds to a output voltage of 200
     {
         pwm_trip();
     }

    //open loop control set frequency to 1.1 times natural frequency respectively, check for any abnormal behavior
    if (MODE == 0)
    {
        pwm_trip_clear();
        frequency = _IQint(_IQdiv(frequency_to_clk_factor, _IQ(1.1))); // value of 1 signifies natural frequency of tank
        duty = frequency / 2;
    }
    else if (MODE == 1)
    {
        if (stop == 1)
        {
            //set values for next start cycle and stop pwm
            frequency = 300;  // directly calculated and put here, only depends upon the value of j_max
            duty = 150;
            theta_state = 0;
            count = 0;

            pwm_trip();
        }
        if (start == 1)
        {
/*            start = 0;
            frequency = 58;  // directly calculated and put here, only depends upon the value of j_max
            duty = 18;
            pwm_trip_clear();   //restart pwm
        }
        else if (m_o_index < 90)
        {
            //do start-up routine
          temp1 = _IQmpy(m_o,m_o);
            temp2 = _IQ(1) - temp1;
            temp3 = _IQdiv(temp0, temp2);
            temp4 =  _IQ(1) + temp3;

            delta_v = _IQsqrt(temp4) - _IQ(1);
            delta_v = _IQ(0.4);
            r1 = _IQ(1) - m_o + delta_v;
            r2 = _IQ(1) + m_o + delta_v;

            r1_inverse = r1_inverse_lookup[m_o_index];
            r2_inverse = r2_inverse_lookup[m_o_index];

            temp0 = j_max;
            temp1 = _IQmpy(temp0,r1_inverse);
            temp2 = _IQmpy(temp0,r2_inverse);

            theta = _IQasin(temp1);
            delta = _IQasin(temp2);

            if(theta > theta_limit)
            {
                theta_state = 1;
            }
            if (theta_state == 1)
            {
                theta = _IQ(PI) - theta;
                count = count + _IQ(0.001);
            }

            total_angle = theta + delta;
            temp3 = _IQmpy(frequency_to_clk_factor_div_pi, total_angle);

            frequency = _IQint(temp3);
            duty = frequency / 2;
        }
        else
        {*/
            // current ADC has a gain of 33.8 and offset of 2077, 1180 corresponds to a current of 35 Amps
            output_current = _IQ12toIQ(AdcResult.ADCRESULT2 - 2077);

            if(output_current > _IQ(1180))
            {
                pwm_trip();
            }

            pi_i_output.Ref = output_current_reference;
            pi_i_output.Fbk = output_current;

            PI_MACRO(pi_i_output)

            frequency =  _IQint(_IQdiv(frequency_to_clk_factor , (_IQ(1) - pi_i_output.Out)));
            duty = frequency >> 1;
            frequency = duty << 1;
        }

    }
    else
    {
        MODE = 0;
    }

    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge interrupt to receive more interrupts from PIE group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    //GpioDataRegs.GPADAT.bit.GPIO30 = 0;
}

//
// epwm3_isr - EPwm3 ISR
// 
__interrupt void
epwm3_isr()
{
    update_frequency_and_duty_ratio(&epwm3_info);

//
// Clear INT flag for this timer
//
    EPwm3Regs.ETCLR.bit.INT = 1;

//
// Acknowledge this interrupt to receive more interrupts from group 3
//
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void
cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    //
    //Toggle GPIO34 once per 500 milliseconds
    //
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    stop = !stop;
    start = !stop;
    //
    // Acknowledge this interrupt to receive
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// InitEPwmTimer -
//
void
InitEPwmTimer()
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;

    //
    // Disable Sync
    //
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through

    //
    // Initially disable Free/Soft Bits
    //
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0;

    EPwm1Regs.TBPRD = PWM1_TIMER_TBPRD;              // Set up PWM1 Period
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;   // Count up mode
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;        // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                   // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;              // Generate INT on 1st event
    EPwm1Regs.TBCTR = 0x0000;                        // Clear timer counter

    //
    // CompareA event at half of period
    //
    EPwm1Regs.CMPA.half.CMPA = PWM1_TIMER_TBPRD/2;

    //
    // Action-qualifiers, Set on CMPA, Clear on PRD
    //
    EPwm1Regs.AQCTLA.all = 0x0024;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Start all the timers synced
    EDIS;
}


//
// InitEPwm3 - EPwm3
//
void
InitEPwm3()
{
//
// Setup TBCLK
//
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm3Regs.TBPRD = EPWM3_DEFAULT_TBPRD;         // Set Default Timer Period
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1; //Slow so we can observe on the scope

//
// Setup shadow register load on ZERO
//
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

//
// Setup compare
//
    EPwm3Regs.CMPA.half.CMPA = EPWM3_DEFAULT_CMPA;

//
// Set actions
//
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on CAU
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;              // Clear PWM1A on CAD

    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;           // Clear PWM1B on CAU
    EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;             // Set PWM1B on CAD

//
// Active high complementary PWMs - Setup the deadband
//
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = EPWM3_DB;
    EPwm3Regs.DBFED = EPWM3_DB;
    EPwm3_DB_Direction = DB_UP;

//
// Interrupt where we will change the frequency and duty ratio
//
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event

    epwm3_info.EPwmRegHandle = &EPwm3Regs; //Set the pointer to the ePWM module
}

void
update_frequency_and_duty_ratio(EPWM_INFO *epwm_info)
{
    epwm_info->EPwmRegHandle->TBPRD = frequency;
    if (duty > frequency / 2)
    {
        epwm_info->EPwmRegHandle->CMPA.half.CMPA = frequency / 2;
    }
    else
    {
        epwm_info->EPwmRegHandle->CMPA.half.CMPA = duty;
    }
}

void pwm_trip()
{
    EALLOW;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EDIS;
    pwm_trip_flag = 1;
}

void pwm_trip_clear()
{
    EALLOW;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;
    pwm_trip_flag = 0;
}

//
// End of File
//

