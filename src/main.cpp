/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //        ==== ABOVE SINCE 1994 ====
   //
   //   Ab0VE TECH - Wideband O2 sensor Controller
   // based on http://www.waltech.com/open-source-designs/wbo2_report/
 */

/*
 * ramp disabled
 * set for vref of 1.8v
 * vref is monitored on adc1
 */

//Includes here:

#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "pid.h"
#include "stdint.h"
#include "initilize_hardware.h"
#include "dataout.h"


////////calibration settings://///////////
//offset for lamda value, uses to calibrate (lambda*100) = 100  at stoic.  Checking against NB sensor
//POSOFFSET is added, NEGOFFSET is subtracted.
#define POSOFFSET 0
#define NEGOFFSET 10

/*Lambda*100 to DAC (0-255)
 *
 *	formula for output voltage:
 *	Vout = [(L-Lmin)/(Lmax-Lmin)]*5
 *
 * example to provide lambda range from 0.64 to 1.36:
 *	 LAMBDA_MIN	64UL
 *	 LAMBDA_MAX 136UL
 * Set two values below:
 */
#define LAMBDA_MIN  64UL// set to desired value for 0v output (Lambda*100)
#define LAMBDA_MAX 136UL//set to desired value for 5v output	(Lambda*100
////////end calibration settings/////////////


//////constants for filtering:
#define MAX_BOXES  20
#define TOEXIT  20 //counts to exit spike
//#define NIT_POINTS_TO_USE = 5
#define SPIKEHEIGHT  6

//#define BIGSPIKE  20
//#define POINTSTOSKIP  2

#define CLOSE_ENOUGH  2 //1 is must be equal
#define CLOSE_COUNT  5
/////end constants for filtering

#define DAC_FACTOR 2550000UL/(LAMBDA_MAX-LAMBDA_MIN)  //don't change: just a calculation pre-compile

//#define MAXPUMP_I 196 // maximum current value for pump (leanest).
#define MAXPUMP_I 329 // maximum current value for pump (leanest).
//see initilize_hardware.h for PWMPUMP_FREQ, if timer OCR1A = PWMPUMP_FREQ, DAC is maxed (5v)
 #define MINPUMP_I 0 // minimum current value for pump.

#define ZERO_CURRENT  116
//#define ZERO_CURRENT  97 //for lower ref v
// 0 current is at 116// now 97 (r4=8.2k ZERO_CURRENT

//#define TARGET_NERNST 832 // lambda=1 value for nurnst, target for pump pid
#define TARGET_NERNST 821 // lambda=1 value for nurnst, target for pump pid
#define TARGET_TEMP 237
#define P_temp    350 //was 450
#define I_temp    100 //5
#define D_temp    0   //1

//#define P_pump    15
//#define I_pump    74
//#define D_pump    2

#define P_pump    9
#define I_pump    10
#define D_pump    0

//#define F_CPU 4000000UL
#define BAUD 9600UL
#define UBRRVAL (F_CPU/(BAUD*16)-1)

//test
//for manual ip ramp:
uint32_t IpCount = 0;
uint16_t ipramp = 0;

//in pid.h, changed scalevalue to 1 since parampeters are now integers: K * 128

uint8_t heat_power; //global value fed to timer0 for heater pwm
uint16_t ADC_data;  //read the ADC into this

volatile uint16_t nurnst = 0; // value read from adc2 for the nernst cell w/o DC
volatile uint16_t pump = 0;   //measured voltage at pump
uint16_t DC_val = 0;          //value read from adc2 for the nernst cell w/DC

//int16_t IpumpVolts=0;//difference from above proportional to current flowing to pump.
uint16_t measured_temperature;  //measured temperature value. also known as Ri
uint32_t zero_to_5_WB;          //value applied to timer2 to make DAC output

uint8_t ramp_flag=0;            // flag gets set once the startup temperature ramp is done
uint8_t its_off;                //flag to keep track of the heater pwm state
volatile uint8_t ADC_flag;      //keeps track in the ADC interrupt
volatile uint8_t charspot=0;    //keeps track of the position of the numbers going into the string
volatile uint8_t cycle_counter = 0; //counts main lops for sampling
//volatile uint8_t nurnst_spike = 0;//flag for nurnst spike detected
//volatile int8_t points_to_use = 3;
//volatile uint8_t timeinspike = 0;//how many samples it stayed in the spike filter mode.

//variables used in filtering:
volatile uint8_t nurnst_spike = 0;
volatile uint8_t points_to_use = 3;
volatile uint8_t timeinspike = 0;
volatile uint8_t close_count = 0;
volatile uint16_t nurnst_data[MAX_BOXES];
volatile uint16_t Ip_data[MAX_BOXES];
volatile uint8_t spike_dir = 3;
//volatile uint8_t out_point_counter = POINTSTOSKIP ;

////////////////////////////////////////////////////////////////////////
//for PID:
uint8_t pidCounter; //True when PID control loop should run one time
struct PID_DATA pidData_temp; //termperature PID structure of vars
struct PID_DATA pidData_pump; //pump PID structure of vars
////////////////////////////////////////////////////////////////////////

/////function prototypes/////
uint16_t readadc(void);
void do_things(void);
void two(void);
void three(void);
void four(void);
void six_1(void);
void six_2(void);
void six_3(void);
void seven(void);
void eight(void);
void PID_heater(void);
void PID_pump(uint16_t nurnst_val);
uint16_t data_extrap(uint8_t points, volatile uint16_t *box_array );
void data_array_update(uint16_t newval, volatile uint16_t *box_array);

////Interrupt Service Routines
ISR(ADC_vect)
{
        ADC_data = readadc();
        if (ADC_flag == 7)//
        {
                four();
        }
        else if (ADC_flag == 2)
        {
                six_1();
        }
        else if (ADC_flag == 8)
        {
                six_2();
        }
        else if (ADC_flag == 9)
        {
                six_3();
        }
        else if (ADC_flag == 3) //nurnst
        {
                seven();
        }
        else if (ADC_flag == 4) //nurnst+DC and calculations
        {
                eight();
        }
        else
        {
                ADC_data = readadc(); //make sure adc is read to clear
        }
}

ISR(TIMER0_OVF_vect)
{
        if (its_off==1)//pulse is off
        {
                PORTD |= _BV(5);//turn pin on
                its_off = 0;
                uint8_t newtimerval= (255-heat_power);
                if (newtimerval < 128)
                {
                        do_things();
                }
                TCNT0 = newtimerval;
        }
        else//pulse is on
        {
                PORTD &=~_BV(5);//turn pin off
                its_off = 1;
                uint8_t newtimerval= (heat_power);
                TCNT0 = newtimerval;//heat_power setting into TCNTO
                if (newtimerval < 128)
                {
                        do_things();
                }
                TCNT0 = (newtimerval);//heat_power setting into TCNTO
        }
}

//////////////////////////////vvvvvvvvvvvvvvv MAIN  vvvvvvvvvvvvvvvvvvvvv///////////////////////////
int main()
{
//set up all the pins as inputs and outputs
/*
 *  PC5 //outputs for R2R DAC/not used
 *  PC4
 *  PC3
 *  PB5
 *  PB4
 *  PB0
 *
 *  PD4  LED
 *
 * Nernst DC connection: PB2
 * nch mosfet for heater: PD5
 *
 * PB1 = pump power OC1A timer out
 * PB3 = output voltage OC2 timer output
 *   //ADC:
 * nernst V:  adc2
 * pump			adc0
 *
 */
        DDRC |= _BV(5)| _BV(4) | _BV(3);//six bit dac
        DDRB |= _BV(5)| _BV(4) | _BV(0);//six bit dac
        DDRD |= _BV(4);//LED
        DDRD |= ( _BV(5));// mosfet for heater

////setup uart:////
        cli();//  disable interrupts until things are set up
        //init uart
        /* set baud rate */
        UBRRH = (UBRRVAL >> 8);
        UBRRL = (UBRRVAL & 0xff);
        /* set frame format: 8 bit, no parity, 1 bit */
        UCSRC |=  (1 << URSEL | 1 << UCSZ1 | 1 << UCSZ0);
        UCSRB |= (1 << RXEN | 1 << TXEN | 1 << RXCIE);//enable receive, transmit, and receive complete interrupt
//disable uart input, avoid Rx buffer overrun:
        UCSRB &= ~(1 << RXEN);
        UCSRB &= ~(1 << RXCIE);

        setup_timer1();// pump control current dac on OC1A
        setup_timer2();//output 0-5v on OC2

        pid_Init(P_temp, I_temp, D_temp, &pidData_temp);//set up PID structure for temperature
        pid_Init(P_pump, I_pump, D_pump, &pidData_pump);//set up PID structure for nernst
        sei();//enable interrupts
        adc_init();
// ramp up heat:
        heat_power = 130;//initial time
        timer0init();
        PORTD |= _BV(4);//LED on
        uart_putst("ramp temp\n");
        while (heat_power<200)
        {
                heat_power++;
                _delay_ms(150);
                uart_put16dec(heat_power);
                uart_putch(',');
                uart_putch(' ');
        }
        uart_putch('\n');
        ramp_flag=1;
///////////////////////
        while(1)
        {
//most stuff handled in timer0 interrupt
//Suggested: use a state machine, just read ADC data in the interrupt, and poll a flag to see if it is done?
        }
        return 0;
}
//// end of main

void do_things(void)//do first adc
{
        if (ramp_flag == 1)// startup temp ramp is finished
        {
                cycle_counter++;
                if (cycle_counter >3)
                {
                        cycle_counter = 0;//reset
                        _delay_us(50);//maybe let things settle?
                        ADC_flag = 7;//sets to run function after conversion
                        ADMUX =(192 + 1);//V refrence plus mux
                        //use 192 for internal 2.5v ref//use 64 for avcc as vref
                        ADCSRA |= _BV(ADSC);// starts  conversion
                }
        }
}

void four(void)//record aux adc 1, mux for pump current
{
        charspot = put_in_string(ADC_data,'\0',charspot);//puts data in big string and sends back new char spot
        ADC_flag = 2;
        ADMUX =(192 + 0);//V refrence plus mux (pump)
        ADCSRA |= _BV(ADSC);// starts  conversion
}

void six_1(void)//measures pump current 1/3 sample
{
        pump = ADC_data;
        ADC_flag = 8;
        ADCSRA |= (1<<ADSC);// starts  next conversion
}

void six_2(void)//measures pump current 2/3
{
        pump = pump + ADC_data;
        ADC_flag = 9;
        ADCSRA |= (1<<ADSC);// starts  next conversion
}

void six_3(void)//measures pump current 3/3
{
        pump = pump + ADC_data;
        pump = pump/3;
        charspot = put_in_string(pump,'\0',charspot);//puts data in big string and sends back new char spot
        ADC_flag = 3;
        ADMUX =(192 + 2);//V refrence plus mux channel//use 192 for internal 2.5v ref//use 64 for avcc as vref
        ADCSRA |= (1<<ADSC);// starts  next conversion
}

void seven(void)//measure nurnst
{
        nurnst = ADC_data;
        charspot = put_in_string(nurnst,'\0',charspot);//puts data in big string
        ADC_flag = 4;
        ADMUX =(192 + 2);//V refrence plus mux channel//use 192 for internal 2.5v ref//use 64 for avcc as vref
        DDRB |= _BV(2);// dc for temperature measurment
        _delay_us(20);
        PORTB |= _BV(2);//DC on
        ADCSRA |= (1<<ADSC);// starts  conversion
}

void eight(void)
{
        PORTB &=~_BV(2);//back to lo
        _delay_us(20);
        DDRB &=~_BV(2);//hiZ
        DC_val = ADC_data;
//	charspot = put_in_string(DC_val,'d',charspot);//puts data in big string
        ADC_flag = 0;
        ////do calculations and PIDs
        measured_temperature = (DC_val - nurnst);
        if (measured_temperature <= 255)//make into (8-bit - 10 bit value) and prevent negatives
        {
                measured_temperature = (255 - measured_temperature);
        }
        else
        {
                measured_temperature = 0;
        }
        if ( (measured_temperature> (TARGET_TEMP-5))&&(measured_temperature<(TARGET_TEMP+5)) )
        {
                PORTD &=~_BV(4);//LED off
        }
        else
        {
                PORTD |= _BV(4);//LED on
        }
        charspot = put_in_string(measured_temperature,'\0',charspot);//puts data in big string

/////////////Filtering:////////////////////////////////
        uint16_t nurnst_calc = 0;
        uint16_t Ip_calc = 0;
        int16_t nurnst_spike_height =0;
        /*
           ////// snap rich/lean spike check:
           // out_point_counter: counts the points to skip after a big spike, up to POINTSTOSKIP
           //if is  POINTSTOSKIP or more, then not activly skipping points.
           if (out_point_counter < POINTSTOSKIP)// big spiking active
           {									//compare to last recorded value.
           nurnst_spike_height = nurnst_data[MAX_BOXES-POINTSTOSKIP]- nurnst;
           }
           else//not big spiking:
           {
           //compare to last value
           nurnst_spike_height = nurnst_data[MAX_BOXES-1]- nurnst;  //
           }
           uint16_t spike_height = abs(nurnst_spike_height);
           if ((spike_height>BIGSPIKE) && (spike_height < BIGSPIKE+100) && (out_point_counter >= POINTSTOSKIP))
           {	//new big spike occoured
           out_point_counter = 0;
           }
           if ((out_point_counter < POINTSTOSKIP) && (nurnst_spike_height>BIGSPIKE))//in point skipping mode, and another spike
           {
           out_point_counter =  out_point_counter +1;

           if (nurnst_spike_height<0) //pos spike
           {
            pump = Ip_data[MAX_BOXES-2] -2; //use previous value plus small trend
            nurnst = nurnst/3;
           }
           else //neg spike
           {
            pump = Ip_data[MAX_BOXES-2] +2;
            nurnst = nurnst/3;
           }
           }
           if (nurnst_spike_height<BIGSPIKE){out_point_counter = POINTSTOSKIP;}
           /////end of rich/lean spike check
         */

        nurnst_spike_height = nurnst_data[MAX_BOXES-1]- nurnst;//check spike height (again, for little spikes)
        data_array_update(nurnst,nurnst_data);
        data_array_update(pump,Ip_data);
        //////nurnst was not spiking, check if it is .
        uint16_t spike_height = 0;
        if (nurnst_spike == 0)
        {
                points_to_use = 1;
                spike_height = abs(nurnst_spike_height);
                if (spike_height > SPIKEHEIGHT) //nurnst spiked SPIKEHEIGHT or more in one cycle
                {
                        nurnst_spike = 1;//set spiking flag
                        //set spike direction reg.
                        if (nurnst_spike_height <0) { spike_dir = 0;} //positive spike
                        else{ spike_dir = 1;}
                        if (spike_height < MAX_BOXES-1) { points_to_use = spike_height;}
                        else{ points_to_use = MAX_BOXES-1;}
                }
        }
        //////nurnst is in a spike
        else
        {
                timeinspike = timeinspike+1;
                nurnst_calc = data_extrap(points_to_use,nurnst_data);
                if ((abs(nurnst_calc-nurnst)<CLOSE_ENOUGH)||(timeinspike>TOEXIT))//conditions to count points to get out of spike mode
                {
                        close_count = close_count +1;
                }
                if (close_count > CLOSE_COUNT)//not spiking any more
                {
                        close_count = 0;
                        nurnst_spike = 0;
                        timeinspike = 0;
                }
                spike_height = abs(nurnst_spike_height);
                ///// check spike height again:
                if (spike_height > SPIKEHEIGHT)
                {
                        //see if has changed direction:
                        if (  ((nurnst_spike_height <0) && (spike_dir == 0)) || ((nurnst_spike_height >=0) && (spike_dir == 1))  )
                        {
                                if (points_to_use < MAX_BOXES) { points_to_use = points_to_use +1;}//increment poins to use in average
                                timeinspike = 0;
                                nurnst_spike = 1;
                        }
                }
        }

        nurnst_calc = data_extrap(points_to_use,nurnst_data); //update if points to use changed
        Ip_calc = data_extrap(points_to_use,Ip_data); //update if points to use changed

/////////////end Filtering section////////////////////////////////
        charspot = put_in_string(points_to_use,'\0',charspot);//puts data in big string
        charspot = put_in_string(nurnst_spike,'\0',charspot);//puts data in big string
        charspot = put_in_string(OCR1A,'\0',charspot);//puts data in big string
        charspot = put_in_string(nurnst_calc,'\0',charspot);//puts data in big string and sends back new char spot
        charspot = put_in_string(Ip_calc,'\0',charspot);//puts data in big string and sends back new char spot
        PID_pump(nurnst_calc);//run PID on pump and update pump pwm.

        //calculate lambda output from Look Up Table:
        struct two_col {
                uint16_t x;
                uint16_t y;
        };
        struct two_col lambda_curve[]={ //table columns: pump, lambda(x=pump current ADC value, y=lambda)
                {1,0},
                {480,68},
                {570,80},
                {610,85},
                {650,90},
                {690,100},
                {700,110},
                {750,143},
                {795,170},
                {810,242},
                {845,20200},
                {1024,26000},
        };
        uint8_t n = 12;//number of rows in table
        uint32_t lambda=0;
        //out of range check:
        if (Ip_calc<lambda_curve[0].x)//smaller than the lowest value in LOT
        {lambda = lambda_curve[0].y;}
        else if (Ip_calc>lambda_curve[n-1].x)//larger than the highest value in the LOT
        {lambda = lambda_curve[n-1].y;}
        //lookup in table, interpolate
        for( uint8_t i = 0; i < n-1; i++ )//loop through table and find value
        {
                if ( (lambda_curve[i].x <= Ip_calc )&& (lambda_curve[i+1].x >= Ip_calc) )
                {
                        uint16_t diffx = Ip_calc - lambda_curve[i].x; //difference between the pump value and the x value in the LOT
                        uint16_t diffn = lambda_curve[i+1].x - lambda_curve[i].x;//spacing between the values in the LOT
                        uint16_t diffy = lambda_curve[i+1].y - lambda_curve[i].y;//spacing of y values in table
                        lambda = lambda_curve[i].y + ((diffy * diffx )/diffn); //output value is interpolated.
                }
        }
        lambda = lambda + POSOFFSET;
        lambda = lambda - NEGOFFSET;

        charspot = put_in_string(lambda,'\0',charspot);//puts data in big string
        if((lambda>=LAMBDA_MIN) && (lambda<=LAMBDA_MAX))
        {
                uint32_t dacval = (lambda-LAMBDA_MIN)*DAC_FACTOR;
                zero_to_5_WB = dacval/10000UL;
        }
        else if (lambda>LAMBDA_MAX) {zero_to_5_WB = 255;}
        else {zero_to_5_WB = 0;}
        OCR2 = zero_to_5_WB;// set DAC output
//	charspot = put_in_string(zero_to_5_WB,'\0',charspot);//puts data in big string timer counts passed

        PID_heater();//run the pid on the temperature and update timer 0
//	charspot = put_in_string(heat_power,'h',charspot);//puts data in big string
        charspot=spitout(charspot);//send it all out the uart
}

uint16_t readadc(void)
{
        uint8_t adcDataL = ADCL;
        uint8_t adcDataH = ADCH;
        uint16_t adcData = 0;
        adcData = adcData | adcDataH;
        adcData = adcData << 8;//left shift 8 spots
        adcData = adcData | adcDataL;
        return adcData;
}

void PID_heater(void)
{
        int32_t calculated = pid_Controller(TARGET_TEMP, measured_temperature, &pidData_temp); // for temp PWM
        if ((calculated) > 255 )
        {
                heat_power = 255;
        }
        else if ((calculated) < 0 )
        {
                heat_power = 0;
        }
        else
        {
                heat_power =(calculated);
        }
}

void PID_pump(uint16_t nurnst_val)
{
        int32_t pumpV =  ( pid_Controller(TARGET_NERNST, nurnst, &pidData_pump) );// PID
        pumpV = pumpV + ZERO_CURRENT;// is zero current.
//apply calculated to pump dac timer
        if ((pumpV) > MAXPUMP_I)
        {
                OCR1A = MAXPUMP_I;
        }
        else if ((pumpV) < MINPUMP_I)
        {
                OCR1A = MINPUMP_I;
        }
        else
        {
                OCR1A =(pumpV);
        }
}

uint16_t data_extrap(uint8_t points, volatile uint16_t *box_array )
{
        uint16_t SUMx = 0;
        int16_t SUMy =0;
        int16_t SUMxy = 0;
        int16_t SUMxx  = 0;
        uint16_t average;
        //uint16_t slope;
        if (points >2)
        {
                for (uint8_t i = (MAX_BOXES-points); i<MAX_BOXES; i++)
                {
                        uint8_t x = i - (MAX_BOXES-points);
                        SUMx = SUMx+x;
                        SUMy = SUMy+box_array[i];
                        SUMxy = SUMxy+(x*box_array[i]);
                        SUMxx = SUMxx+(x*x);
                }
                average = SUMy/points; // average
                //slope = ((SUMx*SUMy)- points*SUMxy) / ( (SUMx*SUMx) - points*SUMxx);
        }
        else //just giving the last point back
        {
                average = box_array[MAX_BOXES-1];
                //slope = 0;
        }
        return(average);
        //return(average+slope);
}

void data_array_update(uint16_t newval, volatile uint16_t *box_array) //update boxcars: data_nurnst_update(nurnst);
{
        for (uint8_t i = 0; i<MAX_BOXES-1; i++)
                box_array[i]=box_array[i+1];
        box_array[MAX_BOXES-1] = newval; //put in latest value
}
