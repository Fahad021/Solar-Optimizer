

#include <avr/io.h>
#include <avr/delay.h> 
#include <avr/interrupt.h>
#include <math.h>
#include <avr/eeprom.h>


          


#ifndef F_CPU
#define F_CPU 8000000UL // or whatever may be your frequency
#endif

#define firmware_version 300 //4  //with AV button & start bit condition  1// fault automatic OFF problem 2//AV button Cancel & Read eeprom before turn off 3// integrated version without AV button 

#define TRUE 1
#define FALSE 0

#define LOW_BAT_BLINK_START_LEVEL 735  // 11.5  : 11.5*10/32 = 3.59: 3.59*1024/5 = 735
#define CHARGING_LIMIT_ENABLE HVR= 730; HVD= 750;  // 16mV 
#define CHARGING_LIMIT_DISABLE HVR= 819; HVD= 860;
uint16_t HVR; //= 819;	// 13.67v 		  // 12.8V : 12.8*10/32 = 4   : 4*1024/5    = 819           
uint16_t HVD; //= 850;  // 14.2v                        // 13.6V : 13.6*10/32 = 4.25: 4.25*1024/5 = 870                 
#define LVR 708 //724//=12.16V //753  //724   // 12.10v                     // 11.9  : 11.9*10/32 = 3.72: 3.72*1024/5 = 761
//#define LVD 653 //662//=11.14V //692  //662 //682   // 11.40v                         // 10.9  : 10.9*10/32 = 3.41: 3.41*1024/5 = 697
uint16_t LVD = 653;
uint16_t EEMEM LVD_VALUE; 
#define SYSTEM_HIGH 744 //736  // 12.3v +
#define SYSTEM_MEDIUM 728 //718 // 12v +
  // #define SYSTEM_LOW   
#define PUSH_BUTTON_THRESHOLD 200      // .
#define SHORT_CIRCUIT_THRESHOLD 500 //280 //380    // 1.25V equivalent to 60V output
#define RELEASE_THRESHOLD 450          
#define SHORT_CIRCUIT_ON_TIME	10    // short circuit on time
#define SHORT_CIRCUIT_OFF_TIME   1000  // short circuit off time
#define SHORT_CIRCUIT_BLANKING_TIME 5
#define PROCESS_PULSE // PORTB|=(1<<PORTB3); PORTB&= ~(1<<PORTB3);
#define PROCESS_PULSE_ON // PORTB|=(1<<PORTB3);
#define PROCESS_PULSE_OFF // PORTB&= ~(1<<PORTB3); 
#define MID_LED_ON_LEVEL  765
#define MID_LED_OFF_LEVEL 860
  // #define DIODE_DROP 32  // .7v 
 //  #define CHARG_OFFSET 3 // 50mv
#define DIODE_DROP 42  // .7v 
#define CHARG_OFFSET 20 // 50mv

#define TEST_EXIT_TIME   600  // 10 min 
#define INIT_LOCK_NUM 5 //5000 //5
 
#define START_PWM   TCCR1A |=(1<<COM1A1)|(1<<COM1A0)|(1<<WGM10); TCCR1B |= (1<<WGM12)|(1<<CS11)|(1<<CS10); //TCNT1 = 0;
#define START_CHARGE_PUMP   TCCR0A |= (1<<COM0B1)|(COM0B0)|(WGM02)|(WGM01)|(WGM00); TCCR0B|=(1<<CS01);//  GTCCR |= (1<<PWM1B)|(1<<COM1B1)|(1<<COM1B0); TCCR1 |= (1<<CS12);
#define STOP_CHARGE_PUMP TCCR0A &= ~(1<<COM0B1)|(1<<COM0B0);
// #define STOP_PWM   TCCR1 &= ~(1<<CS12); GTCCR &= ~((1<<PWM1B)|(1<<COM1B1)|(1<<COM1B0));  TCNT1 = 0;
#define STOP_PWM TCCR1A &= ~(1<<COM1A1)|(1<<COM1A0);
  
  
  
#define OPTIMIZER_ON  PORTB &= ~(1<<PORTB0);
#define OPTIMIZER_OFF PORTB |=  (1<<PORTB0);
  
#define displayON // PORTB |= (1<<PORTB1); 
#define displayOFF// PORTB &= ~(1<<PORTB1);
  
#define CHARGER_ON  PORTA &= ~(1<<PORTA6);
#define CHARGER_OFF PORTA |= (1<<PORTA6);
#define SET_OPTIMIZER_PIN  //xx DDRA |= (1<<PORTA2);
//#define OPTIMIZER_ON   //xx PORTA &= ~(1<<PORTA2);
//#define OPTIMIZER_OFF  //xx PORTA |=  (1<<PORTA2);
 
 #define SET_2LED_INDICATORS_AS_OUTPUT // DDRA |= (1<<PORTA2); PORTA |=  (1<<PORTA5);// DDRA |= (1<<PORTA5);//DDRA |= (1<<PORTA4);DDRA |= (1<<PORTA3);DDRA |= (1<<PORTA4);
 #define SET_2LED_INDICATORS_AS_INPUT  // DDRA &= ~(1<<PORTA2); PORTA &= ~(1<<PORTA5);// DDRA &= ~(1<<PORTA5);PORTA &= ~(1<<PORTA5);DDRA &= ~(1<<PORTA4);PORTA &= ~(1<<PORTA4);DDRA &= ~(1<<PORTA3);PORTA &= ~(1<<PORTA3);DDRA &= ~(1<<PORTA4);PORTA &= ~(1<<PORTA4);  

//xx #define SET_TEST_LED_PIN      DDRB |= (1<<PORTB1);  PORTB &= ~(1<<PORTB1); pc.test_led_state = FALSE; 
//xx #define TEST_LED_ON   PORTB |=  (1<<PORTB1);
//xx #define TEST_LED_OFF  PORTB &= ~(1<<PORTB1);



#define INPUT_0       INPUT_1// PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);    //0000
#define D0			   PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);
#define INPUT_1        PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);     //0001
#define D1			   PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);
#define INPUT_2       INPUT_1// PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA |=  (1<<PORTA4);  PORTB &=  ~(1<<PORTB1);    //0010
#define D2						PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA |=  (1<<PORTA4);  PORTB &=  ~(1<<PORTB1);
#define INPUT_3       INPUT_1// PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA |=  (1<<PORTA4);  PORTB |=  (1<<PORTB1);     //0011
#define D3						 PORTA &=  ~(1<<PORTA2); PORTA &=  ~(1<<PORTA3); PORTA |=  (1<<PORTA4);  PORTB |=  (1<<PORTB1);
#define INPUT_4       INPUT_1// PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);    //0100
#define D4						PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);
#define INPUT_5       INPUT_1//PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);     //0101
#define D5						PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);
#define INPUT_6      INPUT_1// PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA |=  (1<<PORTA4);  PORTB &=  ~(1<<PORTB1);    //0110
#define D6						PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA |=  (1<<PORTA4);  PORTB &=  ~(1<<PORTB1);
#define INPUT_7      INPUT_1// PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA |=  (1<<PORTA4);  PORTB |=  (1<<PORTB1);     //0111
#define D7						PORTA &=  ~(1<<PORTA2); PORTA |=  (1<<PORTA3);  PORTA |=  (1<<PORTA4);  PORTB |=  (1<<PORTB1);
#define INPUT_8      INPUT_1// PORTA |=  (1<<PORTA2);  PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);    //1000
#define D8						PORTA |=  (1<<PORTA2);  PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);
#define INPUT_9       INPUT_1//PORTA |=  (1<<PORTA2);  PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);     //1001
#define D9						PORTA |=  (1<<PORTA2);  PORTA &=  ~(1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);
#define D10						PORTA&=  ~(1<<PORTA2);  PORTA |=  (1<<PORTA3); PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);
#define D11
#define D12
#define D13
#define D14
#define D15
#define D16 

#define INPUT_CAL     INPUT_1//PORTA |=  (1<<PORTA2);  PORTA &=  ~(1<<PORTA3); PORTA |=  (1<<PORTA4);  PORTB &=  ~(1<<PORTB1);    //1010

#define INPUT_POWER   INPUT_1//PORTA |=  (1<<PORTA2);  PORTA &=  ~(1<<PORTA3); PORTA |=  (1<<PORTA4);  PORTB |=  (1<<PORTB1);     //1011

#define INPUT_SLEEP   INPUT_1//PORTA |=  (1<<PORTA2);  PORTA |=  (1<<PORTA3);  PORTA &=  ~(1<<PORTA4); PORTB &=  ~(1<<PORTB1);    //1100

#define INPUT_ZOOM    INPUT_1//PORTA |=  (1<<PORTA2);  PORTA |=  (1<<PORTA3);  PORTA &=  ~(1<<PORTA4); PORTB |=  (1<<PORTB1);     //1101

#define INPUT_MUTE    INPUT_1//PORTA |=  (1<<PORTA2);  PORTA |=  (1<<PORTA3);  PORTA |=  (1<<PORTA4);  PORTB &=  ~(1<<PORTB1);    //1110

#define INPUT_FEEDBACK PORTA |=  (1<<PORTA2);  PORTA |=  (1<<PORTA3);  PORTA |=  (1<<PORTA4);  PORTB |=  (1<<PORTB1);     //1111 

#define RESET_DIGIT digit_value[0] = 0; digit_value[1] = 0; digit_value[2] = 0; digit_value[3] = 0; pc.digit_counter = 0;

#define SETUP_IR    DDRB  &= ~(1<<PORTB2);  PORTB |=  (1<<PORTB2);   
#define ENABLE_IR   GIMSK |= (1<<PCIE1);  PCMSK1 |= (1<<PCINT10); sei(); // interrupt on any logical change
#define DISABLE_IR  GIMSK &= ~(1<<PCIE1); PCMSK1 &= ~(1<<PCINT10);
#define RESET_IR    ir.interrupt_counter = 0; ir.interrupted = FALSE; ir.blanking_started = FALSE;  ir.decimal_value = 0; 

#define ENABLE_IR_TIMER    TCNT1 = 0;// OCR1A = 256; TCCR1B |= (1<<WGM12)|(1<<CS11)|(1<<CS10); // CLK/64 = 8000000/64 = 125000Hz == 8us per step 
#define ENABLE_IR_TIMER_COMPAIR_MATCH_INTERRUPT   TIMSK1 |= (1<<OCIE1A); sei();
#define DISABLE_IR_TIMER_COMPAIR_MATCH_INTERRUPT  TIMSK1 &= ~(1<<OCIE1A);
//-------------------------------------------END OF GLOBAL VARIABLES--------------------------------------------------------
#define DEFAULT_RELEASE_VALUE 10
#define GRACE_PERIOD_DEMO	12
#define TIME_BUDGET_DEMO	22
#define GRACE_PERIOD_REAL  1296000 //30 //300 // 1296000 //900 //7200 //43200 // 300//1296000   // in seconds 15 days
#define TIME_BUDGET_REAL  3888000 //45 //900 //3888000 //2700 //21600 //86400 // 900 //3888000	// in seconds 45 days
#define HIGH_LED_OFF PORTA &= ~ (1<<PORTA2);
#define HIGH_LED_ON PORTA |= (1<<PORTA2);
#define LOW_LED_OFF PORTA &= ~ (1<<PORTA4);
#define LOW_LED_TOGGLE PORTA = PORTA^0x8;  //PORTD = PORTD ^ 0x80
#define LOW_LED_ON PORTA |= (1<<PORTA4);
#define MID_LED_ON PORTA |= (1<<PORTA3);
#define MID_LED_OFF PORTA &= ~ (1<<PORTA3);
#define SOLAR_LED_ON PORTB |= (1<<PORTB1); 
#define SOLAR_LED_OFF PORTB &= ~ (1<<PORTB1);
        
#define IR_BLANKING_TIME 200			// in 2ms
#define EEPROM_UPDATE_INTERVAL  3600 //10 //60 //3600 // 10    // in seconds 1 hour 
#define MINIMUM_TIME_BALANCE 2       // in seconds
#define BLINK_FREQUENCY  200          // one in every  BLINK_FREQUENCY times 2 ms
#define CODE_LOCK_DISPLAY_TIME_DEMO 10
#define CODE_LOCK_DISPLAY_TIME_REAL 30


  



//______________________________________________________________________________________________________
typedef struct
{
  
  
  // battery measurement variables
  uint16_t battery_voltage;
  uint16_t battery_voltage_sum;
  
  uint16_t feedback_voltage;
  uint16_t feedback_voltage_sum;
      
} process;

// instance of struct process
process p;

      
// function declaration
//void Reg_init(void);




   
   

  
   
   
// Code Lock*********************************************************************************************************

//-------------------------------------------GLOBAL VARIABLES--------------------------------------------------------

typedef struct  
{	
	
	volatile unsigned char interrupted;
	uint16_t interrupt_counter;
	uint32_t second;
	
    
} real_time_clock;

// instance of struct time
real_time_clock clock;


typedef struct  
{	

	uint8_t  entry_mode; 
	uint8_t digit_counter;
	
	uint32_t entered_code;
	uint8_t optimizer_state;
	uint8_t code_lock_state; 
	
	uint8_t optimizer_state_button;
	uint32_t time_balance;
	uint32_t released;
	uint8_t initialized;

	unsigned char feedback_blinking;
	unsigned char blink_state;

	uint16_t blink_counter;
	uint16_t blink_timer;
	
	
	uint32_t expected_renew_code;
	
	
	uint32_t customer_id;
	
	uint8_t customer_id_digit_0;
	uint8_t customer_id_digit_1;
	uint8_t customer_id_digit_2;
	uint8_t customer_id_digit_3;
	
	uint8_t operator_id_digit_0;
	uint8_t operator_id_digit_1;
	
	uint8_t month_digit_0;
	uint8_t month_digit_1;
	
	uint8_t product_digit_0;
	uint8_t product_digit_1;
	uint8_t product_digit_2;
	uint8_t product_digit_3;
	uint8_t product_digit_4;
	uint8_t product_digit_5;
	
	uint32_t product;
	uint32_t product_minus_last;
	uint32_t op_month_id;
	
	uint32_t renew_code_cal; // 32 bit 

	uint32_t release_code;
	uint32_t release_product;
	uint32_t result;
	uint8_t expire_blink;
	uint8_t code_lock_expire; 
	
	
} process_control;

// instance of struct time
process_control pc;




typedef struct  
{	
	
	volatile uint16_t interrupt_counter;
	volatile unsigned char interrupted;
	volatile unsigned char transmission_started;
	uint16_t transmission_started_counter;
	
	volatile unsigned char blanking_started;
	uint16_t blanking_counter;
	
	int decimal_value;
	

} infrared;

// instance of struct time
infrared ir;


typedef struct
{
	uint8_t lvd_detect;
	uint8_t lvr_detect;
	uint8_t hvd_detect;
	uint8_t hvr_detect;
	uint8_t helth_state; 
	uint8_t hvd_blink; 
	uint8_t hvd_blink_timer;
	uint8_t hvd_blink_counter; 
} battery;

battery bat;

typedef struct
{
	uint8_t on;
	uint8_t byforce_on;
	uint8_t byforce_off;
	uint8_t off; 
	uint8_t sc_detect;
	uint32_t sc_counter; 
}optimizer;

optimizer opt; 


uint16_t adc_read(uint8_t ch);
uint16_t battery_voltage_read(void);
uint16_t feedback_voltage_read(void);
void scan_system(void);

void begin_pwm(uint16_t battery);
void stop_pwm(void);
void process_key(void);
void process_code(void);
void clock_interrupt_handler(void);	
//void init(void);
void calculate(void);
uint16_t solar_read(void);


 
uint8_t PWM=FALSE; 
uint8_t Others_task=TRUE;
uint8_t Others_task_counter=0;
uint8_t Others_task_counter_limit=50;
uint8_t ir_data_found_timer=0;
//uint32_t decimale_v; 
uint16_t battery_display_on_counter;
uint8_t battery_display;
	
volatile uint8_t bat_state_led;
uint8_t low_led_state=0;
uint16_t BLINK_NUMBER;//      10          // total no of blinking
uint16_t Code_lock_led_time;
volatile uint32_t GRACE_PERIOD;         
volatile uint32_t TIME_BUDGET;
unsigned char LVD_LEVEL;
unsigned char PRE_LVD_LEVEL;
uint8_t bit_value[10];
volatile uint16_t timer_value[10];
volatile uint16_t new_timer_value[5]; 
volatile uint8_t start_bit_counter[2];
volatile uint8_t start_pulse;
volatile uint8_t valid_signal;
uint8_t digit_value[4];


uint32_t EEMEM EXPECTED_RENEW_CODE;
uint32_t EEMEM CODE_LOCK_STATE;
uint32_t EEMEM TIME_BALANCE;
uint32_t EEMEM RELEASED;
uint32_t EEMEM RELEASE_CODE_1_ENTERED;
uint32_t EEMEM INITIALIZED;
uint32_t EEMEM CUSTOMER_ID;
uint16_t EEMEM RN_CODE[60];
uint32_t EEMEM RLC;
uint32_t EEMEM INS_MONTH;
uint16_t EEMEM INIT_NUM;
uint16_t EEMEM TEMP[5];
uint16_t init_num; 

uint16_t EEMEM information[5]; 



uint8_t installment_month;
uint8_t renew_success;
uint8_t program_mode;//=FALSE; 
uint8_t program_mode_in_out_counter;
uint32_t auto_exit_time_pmode;
uint8_t expire_warning_blink;
//uint8_t xxx=999999;

uint8_t  expire_warning;					
//uint16_t ew_time;
//uint8_t ew_time_ON;
//uint16_t ew_time_OFF;
uint16_t expire_warning_counter; 
uint8_t test_enable; 

//int decimale_v=0; 
uint8_t CODE_LOCK_DISPLAY=FALSE;
uint8_t lvd_led_blink_delay;
uint8_t LOW_LED_BLINK;
uint16_t byforce_off_release_delay;
static int count; // count variable to average 16 readings (load and charger currents
uint16_t solar_v;
uint16_t bat_v;
uint16_t fb_v; 
uint8_t button_sense;
uint8_t feedback_state; 

uint32_t time_balance_percent;
uint8_t sleep_pressed=FALSE;

// start of main function
int main (void)
{
	
LOW_LED_BLINK=25;
bat.helth_state=1;
bat.lvr_detect=TRUE;
bat.lvd_detect=FALSE;

bat_state_led=4;


  
  DDRA &= ~(1<<PORTA0);  // push button & battery voltage read
 // PORTA |= (1<<PORTA0);
  
  DDRA &= ~(1<<PORTA1); // feedback read 
 
  DDRA |= (1<<PORTA2);  // System LED
  PORTA &= ~ (1<<PORTA2);
  
  DDRA |= (1<<PORTA3);	// Battery HIGH LED
  PORTA &= ~ (1<<PORTA3);
  
  DDRA |= (1<<PORTA4);  // Battery LOW LED
  PORTA &= ~ (1<<PORTA4);
  
  
  DDRA &= ~ (1<<PORTA5);	// Solar sense
 // PORTA &= ~ (1<<PORTA5);  
 //
 
  DDRB |= (1<<PORTB1);  // Solar LED
  PORTB &= ~ (1<<PORTB1); 
  
  DDRB &= ~ (1<<PORTB3); // phush button sense 
  PORTB|= (1<<PORTB3);
 
  
  DDRA |= (1<<PORTA6);  // PWM charger 
 
  DDRA|= (1<<PORTA7);  // charge pump 
 
  
  DDRB |= (1<<PORTB0);  // optimizer control 
  
  
  ADCSRA |=(1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); // ADC clock system clock/8 = 128 kHz for 1MHz clock

  // init ADMUX: 
  ADMUX |= (1<<MUX1)|(1<<MUX0); // left adjust ADC result & pin2 is selected for ADC input
  
  INPUT_FEEDBACK
  _delay_ms(200);
  //xINPUT_0
  D0
 
  
 // DDRB|=(1<<PORTB3);    // system pulse
  
  SETUP_IR
  ENABLE_IR
  RESET_IR
  ENABLE_IR_TIMER
  ENABLE_IR_TIMER_COMPAIR_MATCH_INTERRUPT 
  
  OPTIMIZER_ON
 pc.optimizer_state=1;
  CHARGING_LIMIT_DISABLE
  
  stop_pwm();
  CHARGER_ON
  _delay_ms(2000);
  
  	
  // init DIDR0: digital input buffer disable register
//  DIDR0 |= (1<<ADC3D);  // Pin2 digital input buffer disabled

  // init ADCSRA


  

 
	
	
	// variables
	ir.transmission_started_counter = 0;
	ir.blanking_counter = 0;
	pc.blink_counter = 0;
    pc.blink_timer  = 0;
	
	test_enable=1;
	
	battery_display=TRUE;
	battery_display_on_counter=0;
	START_PWM
    //STOP_PWM
    PWM=TRUE;
    START_CHARGE_PUMP
    OCR0B=1;
    opt.byforce_off=FALSE; // normally on



	GRACE_PERIOD=GRACE_PERIOD_REAL;           // in seconds 15 days
    TIME_BUDGET=TIME_BUDGET_REAL;			// in seconds 45 days 
	program_mode=FALSE;
	
	
	pc.optimizer_state_button=1;
	pc.expire_blink=TRUE;
	BLINK_NUMBER=10;
	//ew_time=0;
	//expire_warning_ON=FALSE;
	expire_warning=TRUE;
	feedback_state=0;
	expire_warning_counter=0;
	
	//ew_time_ON=250;
	//ew_time_OFF=2500;
	Code_lock_led_time=CODE_LOCK_DISPLAY_TIME_REAL;
	pc.code_lock_expire=FALSE;
	
	
	
	cli();
	
	pc.initialized =  eeprom_read_dword(&INITIALIZED);
	if (pc.initialized==1)
	{
		test_enable=0;
	//	pc.renew_code_cal=eeprom_read_dword(&RN_CODE);
	
		
		pc.release_code=eeprom_read_dword(&RLC);
		
	}
	else if (pc.initialized>1)
	{
		pc.time_balance=0;
		eeprom_write_dword(&TIME_BALANCE, pc.time_balance);
		pc.initialized=0;
		eeprom_write_dword(&INITIALIZED,pc.initialized);
		pc.code_lock_state=0;
		eeprom_write_dword(&CODE_LOCK_STATE,pc.code_lock_state);
		init_num=0;
		eeprom_write_dword(&INIT_NUM,init_num=0);
		
		pc.released=DEFAULT_RELEASE_VALUE; // initial release 
		eeprom_write_dword(&RELEASED,pc.released);
		
		eeprom_write_dword(&information[0],firmware_version);
		eeprom_write_dword(&information[1],EEPROM_UPDATE_INTERVAL);
		eeprom_write_word(&LVD_VALUE,LVD);
				
	}
//	pc.expected_renew_code    =  eeprom_read_dword(&EXPECTED_RENEW_CODE);
	pc.code_lock_state        =  eeprom_read_dword(&CODE_LOCK_STATE);
	pc.time_balance           =  eeprom_read_dword(&TIME_BALANCE);
	pc.released				  =  eeprom_read_dword(&RELEASED);
	init_num				  =	eeprom_read_word(&INIT_NUM);
	LVD						= eeprom_read_word(&LVD_VALUE); 
	if (LVD<500)
	{
		LVD=563;
	}
	
//	pc.release_code_1_entered =  eeprom_read_dword(&RELEASE_CODE_1_ENTERED);
	pc.customer_id            =  eeprom_read_dword(1);
	
	sei();
	
	if (pc.time_balance>EEPROM_UPDATE_INTERVAL)
	{
		pc.code_lock_state=1;
	}
	
	if (pc.released==DEFAULT_RELEASE_VALUE)
	{
		pc.code_lock_state=1;  
	}
	
	//pc.code_lock_state=1; //for without ir gs 
	
	/*
	
	if(pc.released == 1)
	{
		OPTIMIZER_ON
		pc.code_lock_state = 1;
	}
	
	else
	{
		if((pc.customer_id < 0)||(pc.customer_id > 100000))
		{
			pc.customer_id = 0;
			eeprom_write_dword(1, pc.customer_id);
		}
	

	
	
		if((pc.released < 0)||(pc.released > 1))
		{
			pc.released = 0;
			eeprom_write_dword(&RELEASED, pc.released);
		}
	
		if((pc.initialized < 0)||(pc.initialized > 1))
		{
			pc.initialized = 0;
			eeprom_write_dword(&INITIALIZED, pc.initialized);
		}
	
	
		if((pc.code_lock_state < 0)||(pc.code_lock_state > 1))
		{
			pc.code_lock_state = 0;
			eeprom_write_dword(&OPTIMIZER_STATE, pc.code_lock_state); 
		}
	
		if((pc.time_balance < 0)||(pc.time_balance >  TIME_BUDGET))
		{
			pc.time_balance = GRACE_PERIOD;
			eeprom_write_dword(&TIME_BALANCE, pc.time_balance); 
		}
		
		
	
		if(pc.code_lock_state == 1)
		{
			OPTIMIZER_ON
		
		}
		else if(pc.code_lock_state == 0)
		{
			if(pc.released == 0)
			{
				OPTIMIZER_OFF
			}
		
		}
	
	
	
	}
	
	
	
    sei();
	*/
	
	



//****************************************







	
   for (;;)  // Enters infinite loop
   {
	   
		if(clock.interrupted == TRUE) 
		{
			clock.interrupted = FALSE;
			//clock_interrupt_handler();
			scan_system();			
		}	
   }
   


}  // end of main function





// Read ADC function begins
uint16_t adc_read(uint8_t ch)
{
	
	// set ADC channel
	ch &= 0b00001111;
	ADMUX &= 0xF0;
	ADMUX |= ch;
	
	
	
	//start single conversion 
    ADCSRA |= (1<<ADSC);
	
	//Wait for conversion to complete
	//ADSC become '0' again till then, run loop continuously
	 while(ADCSRA & (1<<ADSC));
	 
	 return	(ADC);
	
} // end of adc read routine





uint16_t battery_voltage_read(void)
{
	uint16_t batterySample = 0;
	uint16_t battery = 0;
	
	adc_read(1);  // ignore first reading
	
	// Start of adc reading
	
	for(int i = 0; i<16; i++)
  {
	 batterySample = adc_read(1);
	 _delay_us(100);
	 battery += batterySample;
	 
  } // End of adc reading.
  
   battery = battery >> 4;
  // End of battery condition detection
  return battery;
  
}  // End of battery Read.




uint16_t feedback_voltage_read(void)
{
	uint16_t feedback_Sample = 0;
	uint16_t feedback = 0;
	
	adc_read(0);  // ignore first reading
	
	// Start of adc reading
	
	for(int i = 0; i<16; i++)
	{
	 feedback_Sample = adc_read(0);
	 _delay_us(100);
	 feedback += feedback_Sample;
	 
	} // End of adc 
  
   feedback = feedback >> 4;
    
  
  // End of battery condition detection
  
   return feedback;
	
}




uint16_t solar_read(void)
{
	uint16_t solar_Sample = 0;
	uint16_t solar = 0;
	
	adc_read(5);  // ignore first reading
	
	// Start of adc reading
	
	for(int i = 0; i<16; i++)
	{
	 solar_Sample = adc_read(5);
	 _delay_us(100);
	 solar += solar_Sample;
	 
	} // End of adc 
  
   solar = solar >> 4;
    
  
  // End of battery condition detection
  
   return solar;
	
}






void scan_system(void)
{

count++; 	
button_sense=(PINB & (1<<PINB3));	
	
	
	
    fb_v = feedback_voltage_read();
	
if ((fb_v<SHORT_CIRCUIT_THRESHOLD)&&(pc.optimizer_state==1))  //bat.lvd_detect==FALSE
	{
		opt.sc_counter++;
		if (opt.sc_counter>100)
		{
		opt.sc_detect=TRUE;
		OPTIMIZER_OFF
		pc.optimizer_state=0;
		opt.byforce_off=TRUE;
		bat.helth_state=0;
		//LOW_LED_BLINK=1;
		
		LOW_LED_ON
		_delay_ms(20);
		LOW_LED_OFF
		_delay_ms(20);
		}
		
	}
	
if (fb_v>SHORT_CIRCUIT_THRESHOLD)
 {
	 opt.sc_detect=FALSE;
	 opt.sc_counter=0;
 }
 
 
 if (opt.byforce_off==TRUE)
 {
	  byforce_off_release_delay++;
	 
	 if (byforce_off_release_delay>4000)
	 {
		 opt.byforce_off=FALSE;
		 byforce_off_release_delay=0;
		 
		
	 }		 
 }
 
	
if (button_sense==0)
{
	pc.expire_blink=FALSE;
	battery_display=TRUE;
	battery_display_on_counter=0;
	ir.transmission_started = FALSE;
	ir.transmission_started_counter = 0;
	RESET_DIGIT	
	//INPUT_0	
	CODE_LOCK_DISPLAY=FALSE;
	//pc.expire_blink=TRUE;
	
	ir.transmission_started = TRUE;

	bat.hvd_blink=FALSE;	
	bat.hvd_blink_counter=0;
	
}	

	
	// Average 16 samples 
	if(count > 15)
	{
		count = 0;
		
	bat_v= battery_voltage_read();
	
	bat_v=(bat_v-5);
	
	
	
	solar_v = solar_read();
	
	
	
	if (bat_v>LVR)
	{
		bat.lvr_detect=TRUE;
		bat.lvd_detect=FALSE;
	}
	
	if (bat_v<LVD)
	{
		bat.lvd_detect=TRUE;
		bat.lvr_detect=FALSE;
	}
	
	if (bat_v>HVD)
	{
		
	}
	/*
	if (bat_v<HVR)
	{
		
	}
	*/
	

	
 //Action on output
 
 if ((bat.lvd_detect==FALSE)&&(bat.lvr_detect==TRUE)&&(opt.byforce_off==FALSE)&&(bat.helth_state==0))
 {
	 // code lock condition to on output
//	static int8_t lvr_delay;
//	 lvr_delay++;
	
//	if (lvr_delay>50)
//	 {
//	 lvr_delay=0;
  //  pc.code_lock_state=1;
	bat_state_led=4;

	// OPTIMIZER_ON 
	bat.helth_state=1;
	
//	 }
	
 }
 
 if ((bat.helth_state==1)&&(pc.code_lock_state==1)&&(pc.optimizer_state==0))
 {
	 pc.optimizer_state=1;
	 
	  OPTIMIZER_ON
	 // _delay_ms(5000);
	 _delay_ms(300);
 }
// if (((bat.helth_state==0)||(pc.code_lock_state==0))&&(pc.optimizer_state==1))
if ((bat.helth_state==0)||(pc.code_lock_state==0))
 {
	 pc.optimizer_state=0;
	 OPTIMIZER_OFF
 }
 
 /*
 // AV Button operation 
 if (pc.optimizer_state==1)
  {
   if (pc.optimizer_state_button==1)
     {
	  OPTIMIZER_ON
     }
	else if (pc.optimizer_state_button==0)
	{
		OPTIMIZER_OFF
	}
			
  } 
  */
 
 
 



 
 
 if ((bat.lvd_detect==TRUE)&&(bat.helth_state==1))
 {
	 //OPTIMIZER_OFF
	 //pc.code_lock_state=0;
	 bat.helth_state=0;
 }	 	 
	
// Action on charger
	if(bat_v < HVR)    // Reconnect 
		{
			stop_pwm();
			CHARGER_ON	
		}
	else if((bat_v >= HVR)&&(bat_v < HVD))
		{
			// start and operate pwm as per bat level
			//begin_pwm(bat_v);
		//	CHARGER_ON		
		}
	else if(bat_v >= HVD)  // disconnect 
		{
			stop_pwm();
			CHARGER_OFF
			if (pc.expire_blink==FALSE)
			{
				bat.hvd_blink=TRUE;
			}
			
		}

	if ((bat.hvd_blink==TRUE)&&(pc.expire_blink==FALSE)&&(CODE_LOCK_DISPLAY==FALSE))
	{
		battery_display=FALSE;
		bat.hvd_blink_timer++;
		if (bat.hvd_blink_counter<5)
		{
			
			if (bat.hvd_blink_timer>50)
				{
				//LED OFF
				HIGH_LED_OFF
				bat.hvd_blink_timer=0;
				bat.hvd_blink_counter++;
			
				}
			else if (bat.hvd_blink_timer>25)
				{
				//LED ON 
				HIGH_LED_ON
			
				}
		}
		
		
		
		else 
		{
		bat.hvd_blink=FALSE;	
		bat.hvd_blink_counter=0;
		D0	
		}
		
	}
	
	
	
	/*
	if (pc.code_lock_state==1)
		{
			if (LVD_LEVEL == FALSE)
				{
				if (pc.code_lock_state_button==1)
					{
					OPTIMIZER_ON
					}									
							
				}
		}	
		*/	
		
		
		
	if ((CODE_LOCK_DISPLAY == FALSE)&&(opt.sc_detect==FALSE)&&(battery_display==TRUE))
					
		{
			battery_display_on_counter++;
			
			if ((solar_v)>(bat_v+CHARG_OFFSET+DIODE_DROP))
											
				//		if (solar_v>440)
						{
							SOLAR_LED_ON
			            }
			          //if (solar_v<440)
			else if ((solar_v)<(bat_v+CHARG_OFFSET+DIODE_DROP))
						{
							SOLAR_LED_OFF
						}
				
		  if (bat.lvd_detect==FALSE)
		     {		
				 
				 // for HIGH **********************************				
				if (bat_v >=(SYSTEM_HIGH))
					{
					 //	BATTERY HIGH
					 /*
					HIGH_LED_ON
					MID_LED_OFF
					LOW_LED_OFF
					*/
					bat_state_led=3;
					}
					
				if ((bat_v < (SYSTEM_HIGH-7))&&(bat_state_led==3))
					{
						
					 //	BATTERY HIGH
					//HIGH_LED_OFF
					bat_state_led=6;
					//MID_LED_OFF
					//LOW_LED_OFF
					}
					
				// for mid***********************************	
				if ((bat_v >=(SYSTEM_MEDIUM))&&((bat_state_led==6)||(bat_state_led==4)))
					{
					 //	BATTERY HIGH
					//HIGH_LED_OFF
					//MID_LED_ON
					//LOW_LED_OFF
					bat_state_led=2;
					}
					
				if (bat_v < (SYSTEM_MEDIUM-10))
					{
					 //	BATTERY HIGH
					//HIGH_LED_OFF
					//MID_LED_OFF
					//LOW_LED_ON
					bat_state_led=4;
					}
					
					
				if ((bat_v < SYSTEM_MEDIUM)&&(bat_state_led==4))
					{
					 //	BATTERY HIGH
					//HIGH_LED_OFF
					//MID_LED_OFF
					//LOW_LED_ON
					HIGH_LED_OFF
					MID_LED_OFF
					LOW_LED_ON
					}
					
					
				
					
					
				if (bat_state_led==3)
				{
					HIGH_LED_ON
					MID_LED_OFF
					LOW_LED_OFF
				}	
				
				if (bat_state_led==2)
				{
					HIGH_LED_OFF
					MID_LED_ON
					LOW_LED_OFF
				}
				
							
					
				/*
				if ((bat_v>(SYSTEM_MEDIUM))&&(bat_v<(SYSTEM_HIGH)))
					{
					 //	BATTERY MEDIUM
					// HIGH_LED_OFF
					MID_LED_ON
					//LOW_LED_OFF
					
					}
				
				if (bat_v<SYSTEM_MEDIUM)
					{
					
					LOW_LED_ON
					//HIGH_LED_OFF
					//MID_LED_OFF
					}
					*/
					
			 }
			 
		 else if (bat.lvd_detect==TRUE)	
		 {
			 
			 LOW_LED_BLINK=25;
			 
						 
		 }			 					
					
		}	
		
		
	if ((battery_display==TRUE)&&(battery_display_on_counter>1000))
	{
		battery_display=FALSE;
		battery_display_on_counter=0;
		//xINPUT_0
		 D0
	}
			
		
		// low led blink control 

	if ((pc.code_lock_expire==FALSE)||(pc.expire_blink==FALSE))
	{
      if ((bat.lvd_detect==TRUE)||(opt.sc_detect==TRUE))
	  {
		  lvd_led_blink_delay++;
          if (lvd_led_blink_delay>LOW_LED_BLINK)
			 {
				 lvd_led_blink_delay=0;
				  if (low_led_state==0)
					{
				    low_led_state=1;
				    LOW_LED_ON
			        }
			     else if (low_led_state==1)
			       {
				   low_led_state=0;
				   LOW_LED_OFF
			       }
			 }
	  }	
	  
	  CHARGING_LIMIT_DISABLE
	 }	 
	 if (pc.code_lock_expire==TRUE)
	 {
		 CHARGING_LIMIT_ENABLE
	 }		  
	  
	  // led on error clear
	  
	  if ((CODE_LOCK_DISPLAY == FALSE)&&(low_led_state==1)&&(bat.lvd_detect==FALSE))
	  {
		  //xINPUT_0
			D0
		  low_led_state=0;
	  }		  
	  		 
			 				
						
						
					
					
					//pc.code_lock_state_button
						
						
		
		// measure feedback voltage
		   //p.feedback_voltage = p.feedback_voltage_sum >> 4;
		   //p.feedback_voltage_sum = 0;
		
		/*
		if( s.optimizer_already_on == TRUE)
		{
			 p.feedback_voltage = feedback_voltage_read();
		}
		*/
		
		 
		 
	
			
			
			

		
		
	}
	
	
	
	
}


//___________________________________________________________________________________________________________________________

void begin_pwm(uint16_t battery)
{
	
	// calculate duty cycle:
	/*
	
	
	uint16_t x = battery - HVR;
	
	uint16_t range = HVD - HVR; // 870 - 819 = 51
	
	if(x > range)
	{
		x = range - 1;
	}
	else if(x < 0)
	{
		x = 0;
	}
	
	
	uint16_t duty_value = (uint16_t)(198 - x*195/range);
	OCR1A = (uint8_t)duty_value;
	

	//DDRA |= (1<<PORTA6);
	
	
	if (PWM==FALSE)
	{
		PWM=TRUE;
		START_PWM
	}
	//DDRA |= (1<<PORTA6);
	*/
		
}

//_____________________________________________________________________________________________________________________________________

void stop_pwm(void)
{
	
	if (PWM==TRUE)
	{
		STOP_PWM
		//PWM=FALSE;
	}
	
		
	
}



//---------------------------------------------------FUNCTION: clock_interrupt_handler----------------------------------------------------------    
 void clock_interrupt_handler(void)
 {
	 clock.interrupt_counter++; // incremented every 2 ms
	 
	 //********************SECOND GENERATOR*********************************
			if(clock.interrupt_counter > 500)
			{
				clock.interrupt_counter = 0;
				
				
				
				// auto exit from program mode 
				
				if ((program_mode==TRUE))
				{
					if (auto_exit_time_pmode<TEST_EXIT_TIME)
					{
						auto_exit_time_pmode++;
					}
					else
					{
						program_mode=FALSE;
						//ew_time_ON=250;
						//ew_time_OFF=2500;
						auto_exit_time_pmode=0;
						//xx SET_LED_INDICATORS_AS_OUTPUT
						pc.feedback_blinking=TRUE;
						ir.transmission_started=TRUE;  // only for led control automatic turn off after 60 seconds 
						GRACE_PERIOD=GRACE_PERIOD_REAL;
						TIME_BUDGET=TIME_BUDGET_REAL;
					//	program_mode=TRUE;
						cli();
						pc.released=eeprom_read_dword(&RELEASED);
						pc.initialized=eeprom_read_dword(&INITIALIZED);
						pc.code_lock_state=eeprom_read_dword(&CODE_LOCK_STATE);
						pc.time_balance=eeprom_read_dword(&TIME_BALANCE);
						sei();
					
						
						
						program_mode_in_out_counter++;
						Code_lock_led_time=CODE_LOCK_DISPLAY_TIME_REAL;
					}
				 }					
					
			
				
				
				
				
				//turn off led control after 1 (5) minutes of any ir interrupt
				if(ir.transmission_started == TRUE)
				{
					ir.transmission_started_counter++;
					
					if(ir.transmission_started_counter > Code_lock_led_time) //300
					{
						ir.transmission_started = FALSE;
						ir.transmission_started_counter = 0;
						RESET_DIGIT	
						//xINPUT_0
						 D0

						CODE_LOCK_DISPLAY=FALSE;
						pc.expire_blink=TRUE;
					}
					
				}
		
				
				// expire blinking 
				if ((pc.code_lock_state==0)&&(pc.initialized==1))
				{
					pc.code_lock_expire=TRUE;
					if (pc.expire_blink==TRUE)
					{
					BLINK_NUMBER=10;
					battery_display=FALSE;
					pc.feedback_blinking=TRUE;
				   
					}
				}
				
				
				
				
									
				/*	
				
				if((pc.code_lock_state == 1)&&(pc.released == 0)) // if optimizer on and system is not released.
				{
					
					pc.time_balance--;
					
					if(pc.time_balance < MINIMUM_TIME_BALANCE)
					{
						//OPTIMIZER_OFF
						pc.code_lock_state = 0;
						if (program_mode==FALSE)
						{
						cli();
						eeprom_write_dword(&CODE_LOCK_STATE, pc.code_lock_state); 
						sei();
						}
						 
					}
					*/
					
				if((pc.code_lock_state == 1)&&(pc.released == 0)) // if optimizer on and system is not released.
				{
					
					pc.time_balance--;
					
					if(pc.time_balance < MINIMUM_TIME_BALANCE)
					{
						if (program_mode==FALSE)
						{
							
						cli();
						pc.time_balance=eeprom_read_dword(&TIME_BALANCE); // startup filtered
						pc.optimizer_state=eeprom_read_dword(&CODE_LOCK_STATE); // depend on startup time balance 
						pc.initialized=eeprom_read_dword(&INITIALIZED);  // startup filtered 
						pc.released=eeprom_read_dword(&RELEASED);		// startup filtered 
						pc.release_code=eeprom_read_dword(&RLC);		// startup filtered 
						init_num =	eeprom_read_word(&INIT_NUM);  //startup filtered
						sei();
						
						if (pc.time_balance>MINIMUM_TIME_BALANCE)
						{
							pc.time_balance--;							
						}
						
						}						
						
						if(pc.time_balance <= (EEPROM_UPDATE_INTERVAL))
						{
							pc.time_balance=0;
							pc.code_lock_state =0;
							if (program_mode==FALSE)
								{
								cli();
								eeprom_write_dword(&CODE_LOCK_STATE, pc.code_lock_state); 
								eeprom_write_dword(&TIME_BALANCE, pc.time_balance);
								sei();
								}
								//OPTIMIZER_OFF
						}
						
						
						 
					}
					
					
					
					static int update_counter;
					update_counter++;
					
					if(update_counter > EEPROM_UPDATE_INTERVAL)
					{   
						update_counter = 0;
						if (program_mode==FALSE)
						{
						if (ir.transmission_started==FALSE)
						{
							
// 							//INPUT_8
// 							HIGH_LED_ON MID_LED_OFF LOW_LED_OFF SOLAR_LED_OFF
// 							_delay_ms(250);
// 							HIGH_LED_OFF MID_LED_ON LOW_LED_OFF SOLAR_LED_OFF
// 							//INPUT_4
// 							_delay_ms(250);
// 							HIGH_LED_OFF MID_LED_OFF LOW_LED_ON SOLAR_LED_OFF
// 							//INPUT_2
// 							_delay_ms(250);
// 							HIGH_LED_OFF MID_LED_OFF LOW_LED_OFF SOLAR_LED_ON
// 							//INPUT_1
// 							_delay_ms(250);
							
							//xINPUT_0
							/* D0*/
							
								MID_LED_ON
								_delay_ms(500);
								MID_LED_OFF
							
						}
						
						cli();
						eeprom_write_dword(&TIME_BALANCE, pc.time_balance);
						sei();
						
						}	
					}											
					
					
					
					/*
					
					static int update_counter;
					update_counter++;
					
					if(update_counter > EEPROM_UPDATE_INTERVAL)
					{   
						update_counter = 0;
						if (program_mode==FALSE)
						{
						cli();
						eeprom_write_dword(&TIME_BALANCE, pc.time_balance);
						sei();
						}
						  
					}
					*/
				    
				}
				
			
				                
			} // end of second generator

//********************BLINK GENERATOR*********************************	
			
			if((pc.feedback_blinking == TRUE)&&(battery_display== FALSE))
			{
			//	SET_LED_INDICATORS_AS_OUTPUT
				
			    pc.blink_timer++;
				if(pc.blink_timer > BLINK_FREQUENCY)
				{
					pc.blink_timer = 0;
					pc.blink_counter++;
					
					if(pc.blink_state == TRUE)
				    {
					   //xINPUT_0
						D0
					   pc.blink_state = FALSE;
				    }
				    else if(pc.blink_state == FALSE)
				    {
					   INPUT_FEEDBACK
					   pc.blink_state = TRUE;
					
					   if(pc.blink_counter > BLINK_NUMBER)
					   {
						  pc.blink_counter = 0;
						  pc.blink_timer  = 0;
						  pc.feedback_blinking = FALSE;
						  //xINPUT_0
						  D0
						  pc.blink_state = FALSE;
						  
						
					
					   }
				    }
				}					
				
			//	SET_LED_INDICATORS_AS_INPUT	
			 }
			 
//***************************WARNING FLASH*******************
// expire warning blink 
				if (pc.initialized==1)
				{
					
					
					 
					
					if ((pc.time_balance<((TIME_BUDGET/100)*33))&&(pc.feedback_blinking==FALSE)&&(pc.code_lock_state==1)&&(pc.expire_blink==TRUE)&&(battery_display==FALSE))
					{
						if ((expire_warning_counter<10)&&(feedback_state==0))
						{
							INPUT_FEEDBACK
							feedback_state=1;
							
						}
						if ((expire_warning_counter>100)&&(feedback_state==1))
						{
							//xINPUT_0
							D0
							feedback_state=0;
							
						}
						if (expire_warning_counter>2500)
						{
							expire_warning_counter=0;
						}
						else
						{
							expire_warning_counter++;
						}
						/*
						if (expire_warning_ON==FALSE)
						{
							
							if (ew_time<ew_time_ON)
							{
								//expire_ON=TRUE;
							
							INPUT_FEEDBACK
							ew_time++;
							}
							else
							{
							expire_warning_ON=TRUE;
							ew_time=0;
							}
							
						}
						
						
						if (expire_warning_ON==TRUE)
						{
							if (ew_time<ew_time_OFF)
							{
								//expire_ON=TRUE;
							INPUT_0
							ew_time++;
							
							}
							else
							{
							expire_warning_ON=FALSE;
							ew_time=0;
							}
							
						}
						
						
						
						
						*/
						
					}
				}
				
				
				
			
			
//**************************Re START IR************************
/*
		
			if(ir.blanking_started == TRUE)
			{
				//PROCESS_PULSE_ON
				    ir.blanking_counter++; // incremented every 2 ms
					
					if(ir.blanking_counter > IR_BLANKING_TIME)
				    {
					ir.blanking_started = FALSE;
					ir.blanking_counter = 0;
					INPUT_0
					process_key();
					ENABLE_IR
					RESET_IR
				//PROCESS_PULSE_OFF
				//INPUT_POWER
					
				   }
							
			}
			*/
			
			//**************************************************
			
			
		
 } // end of clock interrupt handler
 
 //---------------------------------------------END OF FUNCTION: clock_interrupt_handler--------------------------------------------------------

 

 
  //--------------------------------------------FUNCTION: process_key---------------------------------------------------------------------------
  
  


void process_key(void)
{
	
	//*******************BIT VALUE GENERATOR FROM TIMER VALUE***************************************
	
	for(uint8_t i = 0; i<8; i++)
	{
		if(timer_value[i] < 100)
		{
			bit_value[i] = 0;
		}
	    else
		{
			bit_value[i] = 1;
		} 
	}
						
	//*******************************DECIMAL VAlUE GENERATOR**************************************************
	ir.decimal_value = bit_value[7]*128 + bit_value[6]*64 + bit_value[5]*32 + bit_value[4]*16 + bit_value[3]*8 + bit_value[2]*4 +  bit_value[1]*2 + bit_value[0]*1;
//	eeprom_write_dword(&TEMP[0],new_timer_value[0]);
//	if (new_timer_value[0]>200)
//	{
		//pc.feedback_blinking=TRUE;
		//eeprom_write_dword(&TEMP[0],new_timer_value[0]);
		//eeprom_write_dword(&TEMP[1],new_timer_value[1]);
	    //eeprom_write_dword(&TEMP[2],new_timer_value[2]);
	//	ir.decimal_value = bit_value[7]*128 + bit_value[6]*64 + bit_value[5]*32 + bit_value[4]*16 + bit_value[3]*8 + bit_value[2]*4 +  bit_value[1]*2 + bit_value[0]*1;
//	}
	
	//eeprom_write_dword(&TEMP[0],new_timer_value[0]);
	//eeprom_write_dword(&TEMP[1],new_timer_value[1]);
	//eeprom_write_dword(&TEMP[2],new_timer_value[2]);
	//new_timer_value[0]
	
	
	// reset led control disable timer
	if ((ir.decimal_value>5)&&(ir.decimal_value<32))
	{
	
		ir.transmission_started_counter=0;
		//SET_2LED_INDICATORS_AS_OUTPUT
		battery_display=FALSE;
		CODE_LOCK_DISPLAY=TRUE;
		INPUT_FEEDBACK
		_delay_ms(100);
		//xINPUT_0
		 D0
		_delay_ms(100);
		INPUT_FEEDBACK
		_delay_ms(100);
		//xINPUT_0
		 D0
		//_delay_ms(100);
		if (pc.released==DEFAULT_RELEASE_VALUE)
		{
			pc.released=0;
			cli();
			eeprom_write_dword(&RELEASED,pc.released);
			pc.code_lock_state=0;
			sei();
		}

	}
	
	//******************************KEY MAPPING AND KEY ACTION GENERATOR FROM DECIMAL VALUE
	if(ir.decimal_value == 14) // power Key
	{
		pc.entry_mode = 1;  //  action: general code entry mode selection
		pc.expire_blink=FALSE;
		pc.feedback_blinking=FALSE;
		pc.blink_counter = 0;
		pc.blink_timer  = 0;
		
		
		//cli();
		//time_balance_percent=(pc.time_balance/(TIME_BUDGET/100));
		//eeprom_write_dword(&TEMP,time_balance_percent);
		//sei();
				
		RESET_DIGIT
		INPUT_POWER
	}
	
	if(ir.decimal_value == 15) // recall Key
	{
		//pc.feedback_blinking=TRUE;
		
		//time_balance_percent=((pc.time_balance/TIME_BUDGET_REAL)*100);
		time_balance_percent=(pc.time_balance/(TIME_BUDGET/100));
		 
		
		 if (time_balance_percent>80)
		{
			HIGH_LED_ON MID_LED_ON LOW_LED_ON SOLAR_LED_ON
			//D10
		}
		else if (time_balance_percent>65)
		{
			HIGH_LED_OFF MID_LED_ON LOW_LED_ON SOLAR_LED_ON
			//D7
		}
		else if (time_balance_percent>50)
		{
			HIGH_LED_OFF MID_LED_OFF LOW_LED_ON SOLAR_LED_ON
			//D5
		}
		else if (time_balance_percent>35)
		{
			HIGH_LED_OFF MID_LED_OFF LOW_LED_OFF SOLAR_LED_ON
			//D1
		}
	}
	
	if(ir.decimal_value == 13) // AV key
	
	
	{ 
	MID_LED_ON
	_delay_ms(500);
	MID_LED_OFF
     /*
		
		pc.entry_mode = 0;
		RESET_DIGIT			//  action: turn off led control
		
		//pc.expire_blink=TRUE;
		if (pc.optimizer_state==1)
		{
		//	pc.code_lock_state=0;
		 if (pc.optimizer_state_button==1)
		 {
			OPTIMIZER_OFF
			pc.optimizer_state_button=0;
		//	pc.feedback_blinking=TRUE;
		 }
		 else
		 {
			 OPTIMIZER_ON
			 pc.optimizer_state_button=1;
		//	 pc.feedback_blinking=TRUE;
		 }			
		}
		else
		{
			pc.optimizer_state_button=0;
			pc.expire_blink=TRUE;
		} 
		*/
	//pc.feedback_blinking=TRUE; 
		
	}
	else if(ir.decimal_value == 27) // zoom key
	{
		if ((pc.customer_id>1000)&&(pc.customer_id<9999)&&(sleep_pressed==TRUE))
		{
		
		pc.entry_mode = 3;  // action : operator id + month info entry mode selection
		RESET_DIGIT
		INPUT_ZOOM
		}
	sleep_pressed=FALSE;
		
	}
	else if(ir.decimal_value == 31) //sleep  key
	{
		pc.entry_mode = 2;  // action: customer id entry mode selection
		sleep_pressed=TRUE;
		RESET_DIGIT
		INPUT_SLEEP
		if (pc.initialized==0)
		{
			pc.customer_id=0;
			pc.op_month_id=0;
		}
		
	}
	else if(ir.decimal_value == 9) //mute key
	{
		pc.entry_mode = 4;  // action: release code entry mode selection
		RESET_DIGIT
		//INPUT_MUTE
	CODE_LOCK_DISPLAY=FALSE;
	battery_display=TRUE;
	battery_display_on_counter=0;
	ir.transmission_started = FALSE;
	ir.transmission_started_counter = 0;
	pc.expire_blink=TRUE;
	}
	else if((pc.entry_mode > 0)&&(pc.digit_counter < 4)) // Accepts 0 to 9 key on condition: 1. valid mode 2. Entered digit not more than 4
	{
		                                                 
		switch(ir.decimal_value)  // action: digit(0 to 9) value storage.
		{
		    case 25: // 0 
				digit_value[pc.digit_counter++]= 0;
				INPUT_0
				//D0
				
				break;
			case 16: // 1
				digit_value[pc.digit_counter++]= 1;
				INPUT_1
				break;
			case 17: // 2
				digit_value[pc.digit_counter++]= 2;
				INPUT_2
				break;
			case 18: // 3
				digit_value[pc.digit_counter++]= 3;
				INPUT_3
				break;
			case 19: // 4
				digit_value[pc.digit_counter++]= 4;
				INPUT_4
				break;
			case 20: // 5
				digit_value[pc.digit_counter++]= 5;
				INPUT_5
				break;
			case 21: // 6
				digit_value[pc.digit_counter++]= 6;
				INPUT_6
				break;
			case 22: // 7
				digit_value[pc.digit_counter++]= 7;
				INPUT_7
				break;
			case 23: // 8
				digit_value[pc.digit_counter++]= 8;
				INPUT_8
				break;
			case 24: // 9
				digit_value[pc.digit_counter++]= 9;
				INPUT_9
				break;
			
			default:
				break;
			
		
		} // end of switch for digit(0 to 9) value storage.
		
	}
	else if(ir.decimal_value == 6) // cal key
	{
		   // action:1. update LED for cal key press  2. process entered digit if at least 4 digit is entered 3. Reset digit storage after processing  
		if(pc.digit_counter > 3)
		{
			process_code();
			if(pc.feedback_blinking ==  FALSE)
			{
			  INPUT_CAL	
			}       
		    RESET_DIGIT  
			
		}
		else
		{
			 RESET_DIGIT
			 INPUT_CAL	  
		}
		         
	}
	
	

} // end of process key
//-------------------------------------------------------END OF FUNCTION: process_key------------------------------------------------------------- 







//-----------------------------------------------------FUNCTION: process_code---------------------------------------------------------------------
void process_code(void)
{
	
	//****************************** CALCULATE ENTERED CODE FROM DIGIT STORAGE ARRAY*************************************
	pc.entered_code = digit_value[0]*1000 + digit_value[1]*100 + digit_value[2]*10 + digit_value[3];
	
	//******************************Process entered code as per entry mode************************************************
	switch(pc.entry_mode)
	{
		
		case 1: // general code entry mode // power button
				
			if (pc.entered_code>0)
			{
				cli();
				installment_month = eeprom_read_word(&INS_MONTH);
				sei();
				
						if (pc.entered_code==110)
							{
							LVD	= 653;
							eeprom_write_word(&LVD_VALUE,LVD);
							pc.feedback_blinking=TRUE;
			
							}
						if (pc.entered_code==100)
							{
							LVD= 590;
							eeprom_write_word(&LVD_VALUE,LVD);
							pc.feedback_blinking=TRUE; 
							}
							
				
				// for program mode test mode 
				if ((pc.entered_code==1111)&&(program_mode_in_out_counter<2)&&(test_enable==1))
				{
					if (program_mode==FALSE)
					{
						pc.feedback_blinking=TRUE;
						program_mode=TRUE;
						//ew_time=0;
						//expire_warning_ON=FALSE;
	
						//ew_time_ON=100;
						//ew_time_OFF=200;
						GRACE_PERIOD=GRACE_PERIOD_DEMO;
						TIME_BUDGET=TIME_BUDGET_DEMO;
					
						
						program_mode_in_out_counter++;
						Code_lock_led_time=CODE_LOCK_DISPLAY_TIME_DEMO;
						pc.code_lock_state=1;
						pc.released=1;
						
					}
					else   // if on test mode 
					{
						program_mode=FALSE;
						auto_exit_time_pmode=0;
						pc.feedback_blinking=TRUE;
						//program_mode=TRUE;
						GRACE_PERIOD=GRACE_PERIOD_REAL;
						TIME_BUDGET=TIME_BUDGET_REAL;
						//ew_time_ON=250;
						//ew_time_OFF=2500;
						cli();
						pc.released=eeprom_read_dword(&RELEASED);
						pc.initialized=eeprom_read_dword(&INITIALIZED);
						pc.code_lock_state=eeprom_read_dword(&CODE_LOCK_STATE);
						pc.time_balance=eeprom_read_dword(&TIME_BALANCE);
						sei();
					
						
						program_mode_in_out_counter++;  
						Code_lock_led_time=CODE_LOCK_DISPLAY_TIME_REAL;
					}
					
					   
				
				} // end of test mode 
				
				
				
				if (pc.entered_code==pc.release_code)
				{
					pc.released=1;
					pc.initialized = 0;
					init_num=0;					
					pc.code_lock_state = 1;
					pc.release_code=0;
					pc.customer_id=0;
					cli();
					if (program_mode==FALSE)
					{
						eeprom_write_dword(&CODE_LOCK_STATE, pc.code_lock_state);
						eeprom_write_dword(&RELEASED, pc.released);
						eeprom_write_dword(&INITIALIZED, pc.initialized); 
						eeprom_write_dword(&RLC,pc.release_code);
						eeprom_write_word(&INS_MONTH,0);
						eeprom_write_word(&INIT_NUM,0);
						eeprom_write_dword(&CUSTOMER_ID,pc.customer_id); 
						
					}
					
					 
			        
					pc.feedback_blinking = TRUE;
					renew_success=TRUE;
				//	pc.entered_code=pc.renew_code_cal=0;
				for( int i=0; i<(installment_month+1); i++)
				{
					eeprom_write_word(&RN_CODE[i],0);
				}					
				
				sei();	
				
				 
				} // end of release system 
				
				
				for( int i=0; i<(installment_month+1); i++)
				{	cli();
					pc.renew_code_cal=eeprom_read_word(&RN_CODE[i]);
					if (pc.entered_code==pc.renew_code_cal)
					{
						pc.renew_code_cal=0;
						eeprom_write_word(&RN_CODE[i],pc.renew_code_cal);
						pc.feedback_blinking=TRUE;
						renew_success=TRUE;
						
						if (pc.initialized==0)
						{
							pc.initialized=1;
							if (program_mode==FALSE)
							{
								eeprom_write_dword(&INITIALIZED, pc.initialized);
							}
							
						}
						
					sei();
					}
				}

						
				
				if (renew_success==TRUE)
				{
				pc.code_lock_expire=FALSE;
				
				renew_success=FALSE;
				pc.time_balance = TIME_BUDGET;
				pc.code_lock_state = 1;
				cli();
				if (program_mode==FALSE)
				{
					eeprom_write_dword(&CODE_LOCK_STATE, pc.code_lock_state);
					eeprom_write_dword(&TIME_BALANCE, pc.time_balance);
					test_enable=0; 
				}
				
				sei();
				
				}
				 
			}
				
			break;
		case 2: // customer id // sleep button
			    
				pc.customer_id = pc.entered_code;
				
				pc.customer_id_digit_0 = digit_value[0];
				pc.customer_id_digit_1 = digit_value[1];
				pc.customer_id_digit_2 = digit_value[2];
				pc.customer_id_digit_3 = digit_value[3];
			
			break;
		case 3: // op+ month  // zoom button
			    
			   pc.op_month_id =  pc.entered_code;
			   
			   installment_month=(pc.op_month_id % 100); // number of code generate 
			   
			   pc.operator_id_digit_0 = digit_value[0];
			   pc.operator_id_digit_1 = digit_value[1];
			   pc.month_digit_0       = digit_value[2];
			   pc.month_digit_1       = digit_value[3];
			   
			   
			 
			   // calculate expected code, store the expected code in EEPROM.
			  
			   pc.product =  pc.customer_id* pc.op_month_id;
			   
			   calculate();
	  
		if((pc.initialized == 0)&&(init_num<INIT_LOCK_NUM))
			 {
			 init_num++; 	   
			   
			   
			 pc.renew_code_cal  = pc.result; // pc.product_digit_5*1000 + pc.product_digit_4*100 + pc.product_digit_3*10 + pc.product_digit_2;
		//	 pc.renew_code[0]= pc.renew_code_cal;
			pc.code_lock_state = 1;
		//	pc.initialized = 1;
			pc.released = 0;
			pc.time_balance = GRACE_PERIOD;
			 cli();
			 eeprom_write_dword(&RN_CODE[0],pc.renew_code_cal);
			 eeprom_write_dword(&CUSTOMER_ID,pc.customer_id);
			 eeprom_write_word(&INS_MONTH,installment_month);
			
			 if (program_mode==FALSE)
			 {
				eeprom_write_dword(&RELEASED,pc.released);
				eeprom_write_dword(&TIME_BALANCE, pc.time_balance); 
				eeprom_write_word(&INIT_NUM,init_num); 
			 }
			 
			 sei();
			 
			pc.feedback_blinking = TRUE;

			
			
			//volatile uint32_t xx=1234;
		//	volatile uint32_t i;
			 
			// pc.customer_id
			
			// renew code ^2 
			 	for(int i = 1; i<installment_month; i++)
				 {
					 
				
						pc.product =  pc.renew_code_cal*pc.renew_code_cal;   // 4 digit renew code x 4 digit renew code
						calculate();	                               // get for digit data & replace unexpected 0 
						pc.renew_code_cal=pc.result;
				
					//	pc.renew_code[i]=pc.renew_code_cal;
						cli();
						eeprom_write_word(&RN_CODE[i],pc.renew_code_cal); // write to eeprom 
						sei();
						
						pc.product=(pc.renew_code_cal*pc.customer_id); // to generate release code (last code x customer ID) 
						calculate();								// get for digit data & replace unexpected 0
						pc.release_code=pc.result;					// copy 4 digit release code s
						
			
						 
					 
					}	
					
					 cli();
					 eeprom_write_dword(&RLC,pc.release_code);
					 sei();	
					 
				}					 			 
					 	
					 
					 
			   
			break;
		case 4: // release code 1 // mute
	
			   
			break;
			
		case 5:
		//LVD	= 653;

		break;
		
		default:
			  
			break;
		
	} // end of switch
	
	pc.entry_mode = 0;
	
	
	
}  // end of process code



//----------------------------------------- END OF FUNCTION: process_code-----------------------------------------------------------------

	
	


 //-----------------------------------------INTERRUPT SERVICE ROUTINE: IR-----------------------------------------------------------------------------------------
ISR(PCINT1_vect)
{
			cli();
			char sregbuffer = SREG;
			
			
			ir.interrupted = TRUE;
			ir.interrupt_counter++;
			
	
			//ir.transmission_started = TRUE;
			//SET_LED_INDICATORS_AS_OUTPUT
			//INPUT_FEEDBACK
			
			if (ir.interrupt_counter==1)
			{
				TCNT1=0; 
				start_pulse=1;
			}
			if (ir.interrupt_counter==2)
			{
				start_pulse=2;
				
			}
			
			if (ir.interrupt_counter==3)
			{
				
				start_pulse=3;
					

			}
			
			
			if(ir.interrupt_counter >21)
			{
				switch(ir.interrupt_counter)
				{
		
					case 22: // reset timer
						TCNT1 = 0;
						break;
					case 23: // calculate timer for bit 0
						timer_value[0] = TCNT1;
						break;
					case 24: // reset timer
						TCNT1 = 0;
						break;
					case 25: // calculate timer for bit 1
						timer_value[1] = TCNT1;
						break;
					case 26:// reset timer
						TCNT1 = 0;
						break;
					case 27:// calculate timer for bit 2
						timer_value[2] = TCNT1;
						break;
					case 28: // reset timer
						TCNT1 = 0;
						break;
					case 29:// calculate timer for bit 3
						timer_value[3] = TCNT1;
						break;
					case 30: // reset timer
						TCNT1 = 0;
						break;
					case 31: // calculate timer for bit 4
						timer_value[4] = TCNT1;
						break;
					case 32: // reset timer
						TCNT1 = 0;
						break;
					case 33: // calculate timer for bit 5
						timer_value[5] = TCNT1;
						break;
					case 34: // reset timer
						TCNT1 = 0;
						break;
					case 35: // calculate timer for bit 6
						timer_value[6] = TCNT1;
						break;
					case 36: // reset timer
						TCNT1 = 0;
						break;
					case 37: // calculate timer for bit 7
						DISABLE_IR
						timer_value[7] = TCNT1;
						
						
						//*****************
						//INPUT_0
						
						//OPTIMIZER_OFF
				      // _delay_us(100);
				     //  OPTIMIZER_ON
						
						//process_key();
						//battery_display=FALSE;
						//MID_LED_ON
						//eeprom_write_dword(&TEMP[0],new_timer_value[0]);
					//	eeprom_write_dword(&TEMP[2],start_bit_counter[0]);
					//	eeprom_write_dword(&TEMP[3],start_bit_counter[1]);
						//_delay_ms(30);
						_delay_ms(15);
					//	MID_LED_OFF
					
					if ((start_bit_counter[0]>3)&&(start_bit_counter[0]<5)&&(start_bit_counter[1]>1)&&(start_bit_counter[1]<3))
					{
						process_key();
						ir.blanking_started = TRUE; 
						ir.transmission_started = TRUE;
					}
					start_bit_counter[0]=0;
					start_bit_counter[1]=0;
					start_pulse=0;
					
						
						
					   ENABLE_IR
					   RESET_IR
						
						
						
						//****************     
					    break;
		
				} // end of switch
			}
	
	
	SREG = sregbuffer;
	sei();
	
}
//-----------------------------------------END OF INTERRUPT SERVICE ROUTINE: IR-----------------------------------------------------------------------------------



//------------------------------------------INTERRUPT SERVICE ROUTINE: IR TIMER------------------------------------------------------------------------------------------


ISR (TIM1_COMPA_vect)
{
  cli();
  char sregtemp = SREG;
  clock.interrupted = TRUE;
  clock_interrupt_handler();
  if (start_pulse==1)
  {
	  start_bit_counter[0]++;
  }
   if (start_pulse==2)
  {
	  start_bit_counter[1]++;
  }
  
  SREG = sregtemp;
  sei();
// clock_interrupt_handler();
}

//-----------------------------------------END OF INTERRUPT SERVICE ROUTINE: IR TIMER--------------------------------------------------------------------------------------


//------------------------------------------FUNCTION: init_eeprom---------------------------------------------------------------------------------
void init(void)
{
	
}

//------------------------------------------END OF FUNCTION: init_eeprom--------------------------------------------------------


void calculate(void)
{
					pc.product_digit_0 = (pc.product % 10 );
					pc.product_minus_last = (pc.product / 10);
			   
					pc.product_digit_1 = (pc.product_minus_last % 10 );
					pc.product_minus_last = (pc.product_minus_last / 10);
			   
					pc.product_digit_2 = (pc.product_minus_last % 10 );
					pc.product_minus_last = (pc.product_minus_last / 10);
			   
					pc.product_digit_3 = (pc.product_minus_last % 10 );
					pc.product_minus_last = (pc.product_minus_last / 10);
			   
					pc.product_digit_4 = (pc.product_minus_last % 10 );
					pc.product_minus_last = (pc.product_minus_last / 10);
			   
					pc.product_digit_5 = (pc.product_minus_last % 10 );
					pc.product_minus_last = (pc.product_minus_last / 10);
			   
			  
			  
					if(pc.product_digit_2 == 0)
					{
						pc.product_digit_2 = 1; 
					}
			  
					if(pc.product_digit_3 == 0)
					{
						pc.product_digit_3 = 1; 
					}
			  
					if(pc.product_digit_5 == 0)
					{
						 pc.product_digit_5 = 1; 
					}
			   
			        pc.result  =  pc.product_digit_5*1000 + pc.product_digit_4*100 + pc.product_digit_3*10 + pc.product_digit_2; 
				
}


