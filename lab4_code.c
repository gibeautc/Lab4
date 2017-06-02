// Chad Gibeaut
// 10/28/15

//Hardware Setup
// LED bar graph is multiplexed, PORTA outputs for the segments
// PORT B is used for SPI and binary output for decoder for LED digits
// SPI port is used to send data to the bar graph, and read data in from Encoders
// Since each SPI device is only a one way data stream, these can be done at the same time
#include <avr/io.h>
//#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define bar_clk 2  //on port F
//#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>//added
#include "lcd_functions.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"
#include "uart_functions.h"
//#include "si4734.h"

//volatile uint8_t ext_count=0; not being used at this time
#define SEL0 4
#define SEL1 5
#define SEL2 6
#define PWM 7
#define FM_TUNE_FREQ 0x20
#define FM_PWR_UP    0x01
#define PWR_DOWN     0x11
#define FM_TUNE_STATUS_IN_TACK 0x01
#define GPO_IEN_STCIEN  0x0001
#define GPO_IEN  0x0001
#define SET_PROPERTY 0x12
#define FM_RSQ_STATUS 0x23
#define FM_RSQ_STATUS_IN_INTACK 0x01

#define SI4734_ADDRESS  0x22
//*******************Global*****************************************
//holds data to be sent to the segments. logic zero turns segment on

//volatile uint16_t current_fm_freq=9990; //krkt default
uint8_t si4734_wr_buf[9];//not sure we ever use 9 on either of these
uint8_t si4734_rd_buf[9];
uint8_t si4734_tune_status_buf[8];
volatile uint8_t STC_interrupt;
uint16_t current_fm_freq;
uint8_t radio=0;
int8_t remote_temp=100;
volatile uint16_t lm73_temp;
extern uint8_t lm73_wr_buf[];
extern uint8_t lm73_rd_buf[];
uint8_t line1[17]=" ALARM           ";
uint8_t line2[17]="   Alarm  off   ";
uint8_t temp_array[17]=" O:   I:         ";
//outside temp in 3-4  inside in 9-10
uint8_t lcd_count=0;
uint8_t volume=150;
uint8_t segment_data[5]; 
int8_t min=30;            //Startup time
int8_t hour=8;           //Startup time
int8_t alarm=0;           //Alarm starts off
int8_t alarm_min=31;      //start up time for alram
int8_t alarm_hour=8;      //This will get changed when alarm is put in EEPROM
int8_t snooze=0;
int16_t countL=0;
int16_t countR=0;
uint8_t mode=0;            //Used for tracking mode 
uint8_t inactive=0;
uint8_t count=0;
volatile uint16_t count_t=0;
uint8_t count_2=0;
uint8_t dec_state=0;        //current state of decoders
uint8_t dec_state_last=0;  //previous state of decoders
uint8_t inc=1;//This is the increment amount set by buttons 
//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={192,249,164,176,153,146,131,248,128,152,255,4} ;
uint8_t up=1;
uint8_t local_temp=0;
uint8_t radio_rs=0;
uint16_t tune_time=20000;
int16_t convert_temp(uint16_t temp)
{
 temp=(temp>>7);
 temp=temp*1.8;
 temp=temp+32;
 return temp;
}

void radio_reset(){
PORTE &=~(1<<7);
DDRE |=(1<<7);//set as output
PORTE|=(1<<2);//set reset
_delay_us(200);//wait
PORTE &=~(1<<2);//release reset
_delay_us(30);
DDRE &=~(1<<7);//back to input
}


void fm_tune_freq(){

si4734_wr_buf[0]=0x20;//check on buffer
si4734_wr_buf[1]=0x00;
si4734_wr_buf[2]=(uint8_t)(current_fm_freq>>8);//Add current
si4734_wr_buf[3]=(uint8_t)(current_fm_freq);
si4734_wr_buf[4]=0x00;
STC_interrupt=FALSE;
twi_start_wr(SI4734_ADDRESS,si4734_wr_buf,5);//******Add address
while(!STC_interrupt){}//Spin till complete
}

void set_property(uint16_t property,uint16_t property_value){

si4734_wr_buf[0]=SET_PROPERTY;
si4734_wr_buf[1]=0x00;
si4734_wr_buf[2]=(uint8_t)(property>>8);
si4734_wr_buf[3]=(uint8_t)(property);
si4734_wr_buf[4]=(uint8_t)(property_value>>8);
si4734_wr_buf[5]=(uint8_t)(property_value);
twi_start_wr(SI4734_ADDRESS,si4734_wr_buf,6);
_delay_ms(10);

}//end set property



void fm_pwr_up(){
//radio_reset();

current_fm_freq=9990;
si4734_wr_buf[0]=FM_PWR_UP;    //***********ADD
si4734_wr_buf[1]=0x50;
si4734_wr_buf[2]=0x05;
twi_start_wr(SI4734_ADDRESS, si4734_wr_buf,3);
_delay_ms(120);
set_property(GPO_IEN,GPO_IEN_STCIEN);//Check all this shit
}//end fm power up



void radio_pwr_dwn(){
si4734_wr_buf[0]=0x11;
twi_start_wr(SI4734_ADDRESS,si4734_wr_buf,1);
_delay_us(310);

}//end power down

void fm_rsq_status(){
si4734_wr_buf[0]=FM_RSQ_STATUS;
si4734_wr_buf[1]=FM_RSQ_STATUS_IN_INTACK;//******ADD
twi_start_wr(SI4734_ADDRESS,si4734_wr_buf,2);
while(twi_busy()){}//spin still done
_delay_us(300);//Blind wait, maybe change to look at interupt
twi_start_rd(SI4734_ADDRESS, si4734_tune_status_buf,8);
while(twi_busy()){}//Spin till done
}//End request status





//index 10 is used as the 'blank' digit

//                                   segment_sum
//determine number of digits
//bound count by 1023
//break count up into its digits and store as needed
//digits are stored index 0-3   4 can be used later for colon
void segsum(int16_t sum) {
 uint8_t dig;
 for(dig=0;dig<4;dig++){segment_data[dig]=10;}
 segment_data[0]=sum/1000;sum=sum-segment_data[0]*1000;
 segment_data[1]=sum/100;sum=sum-segment_data[1]*100;
 segment_data[2]=sum/10;sum=sum-segment_data[2]*10;
 segment_data[3]=sum;
}//segment_sum


void spiRW(uint8_t bar){
  SPDR=bar;
  while(bit_is_clear(SPSR,SPIF)){}
  PORTF&=(0<<bar_clk);//reads decoder into registor
  PORTF|=(1<<bar_clk);//returns to normal mode
  SPDR=bar;//Sends out byte passed into funciton to bar graph
  while(bit_is_clear(SPSR,SPIF)){}  //wait for byte to send
  dec_state=SPDR;                  //reads byte in from decoders
  PORTF&=(0<<bar_clk);            
  if(dec_state_last<128){dec_state_last=dec_state;}//takes care of setting this the first time
 
//Following nested if statements check current state vs last state to determine direction of rotation
  if(bit_is_clear(dec_state_last,0) && bit_is_set(dec_state_last,1))
  {
   if(bit_is_set(dec_state,0) && bit_is_set(dec_state,1)){countL--;}    //Left decoder backward
   if(bit_is_clear(dec_state,0) && bit_is_clear(dec_state,1)){countL++;} //Left decoder forward
  }
  if(bit_is_clear(dec_state_last,2) && bit_is_set(dec_state_last,3))
  {
   if(bit_is_set(dec_state,2) && bit_is_set(dec_state,3)){countR--;}  //right decoder backward
   if(bit_is_clear(dec_state,2) && bit_is_clear(dec_state,3)){countR++;}  //rigth decoder forward
  }

  dec_state_last=dec_state; //resets last state
  PORTF|=(1<<bar_clk);
}

//Sets up SPI port 
void spi_init(void){
//DDRB is already setup in main 
 DDRF=(1<<bar_clk)|(1<<3);
 
 SPCR = (1<<SPE) | (1<<MSTR);   //master mode
 SPSR = (1<<SPI2X);             //sets speed
}//spi_init

//debounce_switch function for the push buttons
uint8_t debounce_switch(uint8_t pin)
{
 static uint16_t state[8]={0,0,0,0,0,0,0,0};
 state[pin] = (state[pin]<<1)|(! bit_is_clear(PINA,pin))| 0xE000;
 if (state[pin] == 0xF000){return 1;}
 return 0;
}

//checks the switches and changes the mode/increment amount
void check_sw()
{
PORTA=0xFF;//outputs all high to charge curcuit
//asm("nop");
//asm("nop");
DDRA=0x00;//Set port to inputs
PORTA=0xFF; //turns on PU resistors
//Tri state is on Y7
PORTB=(1<<SEL0)|(1<<SEL1)|(1<<SEL2);
asm("nop");
asm("nop");
//line2[11] and 12 for alarm  oXX
if(debounce_switch(0)==1){mode =1;countL=0;countR=0;}//Set time 
if(debounce_switch(1)==1){mode =2;countL=0;countR=0;}//Set alarm time
if(debounce_switch(2)==1){
alarm ^=1<<7;
if(bit_is_clear(alarm,7)){line2[11]='f';line2[12]='f';snooze=0;}
if(bit_is_set(alarm,7)){line2[11]='n';line2[12]=' ';}
countL=0;countR=0;}//Alarm on/off 
if(debounce_switch(3)==1){if(mode==0){
  mode=3;radio_reset();fm_pwr_up();
 while(twi_busy()){} 
 fm_tune_freq();}else 
  if(mode==3){mode=0;radio_pwr_dwn();}   
  countL=0;
  countR=0;}//Radio on/off  LOST 13Hz adding 6 if statements
if(debounce_switch(4)==1){countL=0;countR=0;}//  12/24 
if(debounce_switch(5)==1){countL=0;countR=0;snooze++;}// Snooze
if(debounce_switch(6)==1){countL=0;countR=0;}// Unused 
if(debounce_switch(7)==1){mode=0;countL=0;countR=0;}//Back to mode 0 normal time
}

void update_LED(){
PORTC ^=1<<1;//used for external time checking
DDRA=0xFF;//PORT A outputs
uint8_t dig;
for(dig=1;dig<6;dig++)//loops through the digits
{
PORTA=0xFF;
switch(dig){
case 1:PORTB=(0<<SEL0)|(0<<SEL1)|(1<<SEL2);break;//Y4 SELECTED
case 2:PORTB=(1<<SEL0)|(1<<SEL1)|(0<<SEL2);break;//Y3 SELECTED
case 3:PORTB=(1<<SEL0)|(0<<SEL1)|(0<<SEL2);break;//Y1 SELECTED
case 4:PORTB=(0<<SEL0)|(0<<SEL1)|(0<<SEL2);break;//Y0 SELECTED
case 5:PORTB=(0<<SEL0)|(1<<SEL1)|(0<<SEL2);break;//Collon selected
}
if(dig==4){PORTA=dec_to_7seg[segment_data[dig-1]]-alarm;}
else{PORTA=dec_to_7seg[segment_data[dig-1]];}
asm("nop");
_delay_ms(.01);//0.02
 }//end dig
}//end update_LED


//sets up the timer 0 for RTC
void init_tcnt0(){
  
  ASSR  |=(1<<AS0);     //run off external 32khz osc (TOSC)
  TIMSK=0x00; //reset to all 0
  TIMSK |=(1<<OCIE0);
  //need to set the interupt for the compare match  
  TCCR0=0x00;//reset to zero
  TCCR0 |=  (1<<WGM01) | (0<<CS02)|(0<<CS01)|(1<<CS00);  //CTC mode,no prescaler
  OCR0 =40;          //250 was calculated
}//timeer0 end

//Timer 1 is for alarm tone
void init_tcnt1(){
 //Compare mode, OC1A,B,C disconected
 //CTC Mode 1<<WGM12   1024 prescale
 //with prescale its running on 7812.5 Hz
 TCCR1B=(1<<WGM12)|(1<<CS12)|(0<<CS10);
 OCR1A=0x10;  //should produce tone of 1116 Hz
 TIMSK|=(1<<OCIE1A);
}//timer1 end

//Sets up timer 2 for PWM dimming control
void init_tcnt2(){
 TCCR2=(0<<FOC2)|(1<<WGM20)|(1<<COM21)|(1<<COM20)|(0<<WGM21)|(0<<CS22)|(0<<CS21)|(1<<CS20);
 OCR2=100;//Starting value  about the lowest i would go
 //no output compare,PWM phase correct, Set OC2 on match up count, clean on compare down count, 64 prescale
 //this should give a freq of about 490hz 
}//init_tcnt2 end


//timer3 will be used for volume control  should be smoothed to be 1-4 volts
//20% simulation shows 1.1V   OCR value of 51
//80% shows 4V                OCR value of 204
void init_tcnt3()//PORTE 3
{//COM3A1=1  COM3A0=0   should set OC3A when down counting, clear when up counting
 //phase correct   mode 11   WGM33,31,30 =1
 TCCR3A=(1<<COM3A1)|(1<<WGM32)|(1<<WGM30);
 TCCR3B|=(0<<WGM33)|(1<<CS31)|(1<<CS30);
 //TCCR3C|=(0<<FOC3A);
 OCR3A=55;
}//End timer3 init


void startup_test()
{
 DDRA= 0xFF;//port A all output  (segments of display)
 PORTA=0xFF;//all segments off
 uint8_t num;
 uint8_t loop_count;
 uint8_t dig;
 for(num=0;num<10;num++)
 {
  if(num==0){spiRW(255);}
  else{spiRW(num);};
  for(loop_count=0;loop_count<100;loop_count++)
  {
   for(dig=1;dig<5;dig++)
   {
   if(dig==1){PORTB=(0<<SEL0)|(0<<SEL1)|(1<<SEL2);}//Y4 SELECTED
   if(dig==2){PORTB=(1<<SEL0)|(1<<SEL1)|(0<<SEL2);}//Y3 SELECTED
   if(dig==3){PORTB=(1<<SEL0)|(0<<SEL1)|(0<<SEL2);}//Y1 SELECTED
   if(dig==4){PORTB=(0<<SEL0)|(0<<SEL1)|(0<<SEL2);}//Y0 SELECTED
    PORTA=dec_to_7seg[num];
    _delay_ms(1);
   }//end dig
  }//end loop count
 } //End num loop
 PORTA=0xFF;
}//End startup_test

void ADC_init(){
ADMUX|=(0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
//selects ref voltage,left justify also needs to select chan
ADCSRA|=(1<<ADEN)|(1<<ADSC)|(0<<ADFR);//Enable, take measurment, and single mode

}//end ADC_init

//Radio "ready" interupt
ISR(INT7_vect){STC_interrupt=TRUE;}


//Alarm tone toggle PORTC pin2
ISR(TIMER1_COMPA_vect){PORTC ^=(1<<2);}

ISR(TIMER0_COMP_vect){	
//check_sw(); //checks switches	
//spiRW(mode);//Updates mode on bar graph and gets new encoder value
count_t++;
if (count_t>391){count_t=0;}

if(count_t%10==0){
}

switch(count_t)
{
case 35:uart_putc(100);
         break;
case 45:remote_temp=uart_getc();
        if(remote_temp%10==remote_temp){temp_array[3]=remote_temp+'0';break;}
        temp_array[3]=(remote_temp/10)+'0';
        temp_array[4]=(remote_temp%10)+'0';
        break;
case 391://PORTC ^=1<<0;
         count++;
         if(count==120){min++;count=0;}
         //count_t=0;
         if(segment_data[4]==11 && mode!=1){segment_data[4]=10;}
         else{segment_data[4]=11;}
         break;
      
case 65:lm73_temp=lm73_rd_buf[0];break;
case 75:lm73_temp=(lm73_temp<<8);break;
case 85:lm73_temp |=lm73_rd_buf[1];break;
case 95:lm73_temp =convert_temp(lm73_temp);
       
case 105:local_temp=lm73_temp; //8 bit only now
         if(local_temp%10==local_temp){temp_array[8]=local_temp+'0';break;}
         temp_array[9]=(local_temp/10)+'0';
         temp_array[10]=(local_temp%10)+'0';
         
         //PUT TEMP in LCD string here
         break;
default: //check_sw();
         //spiRW(mode);
         break;
}//switch
}//IRS





//*****************************************************************
int main()
{



//Setup******************************************************************
//DDRC=(1<<0)|(1<<1);// Sets bit 6 and 7 to output, used for checking timing
DDRE=(1<<3)|(1<<2)|(1<<4);//3-volume control 2-Radio enable, 
//7 also gets toggled and 4 is Enable for the 3.3V reg
//PORTC=(1<<0)|(1<<1);//Sets both bits high to start wit
PORTE|=(1<<4);//turn on 3.3V reg
segment_data[4]=10;
init_tcnt0();      //RTC  **KEEP AS FIRST INIT, it resets TIMSK
init_tcnt1();      //Alarm tone 
init_tcnt2();      //PWM for display dimming
init_tcnt3();      //Volume control
segsum(count);//sets up the segment data using initial count
DDRB=0b11110111;//set port bits 4-7 B as outputs  and sets up SPI port
spi_init();     //sets up SPI 
lcd_init();
cursor_off();
//startup_test(); //Runs through all digits to ensure they are working
//spiRW(mode);  //initial mode displayed on bar graph (all off)
ADC_init();
init_twi();
uart_init();
lm73_wr_buf[0]=0;
twi_start_wr(0x90,lm73_wr_buf,1);

EICRB|=(1<<ISC70)|(1<<ISC71); //enable external interupt 7 rising edge
EIMSK|=(1<<INT7);
sei();
//radio_reset();
//fm_pwr_up();
//fm_tune_freq();

//startup_test();
//sei();         //enable global interrupts
//End setup**************************************************************
while(1){
//spiRW(mode);
count_2++;
check_sw();
//if(count_2==0xFF){count_2=0;}

if(count_2%200==0){twi_start_rd(0x90,lm73_rd_buf,2);}

if(count_2%250==0){
if(lcd_count==0){cursor_home();}
if(lcd_count<17){char2lcd(line1[lcd_count]);}
else{char2lcd(line2[lcd_count-16]);}
lcd_count++;
update_LED();
if(lcd_count==17){home_line2();}
if(lcd_count==32){lcd_count=0;}
}
//temp conversion to F
//lm73_temp=convert_temp(lm73_temp);

//volume
if(volume<50){volume=50;}
if(volume>204){volume=204;}
OCR3A=volume;

//receive strength

if(count_2%50==0){
while(twi_busy()){}
fm_rsq_status();
while(twi_busy()){}
radio_rs=si4734_tune_status_buf[4];
}
if(radio_rs<=8){radio_rs=0;}else
if(radio_rs<=16){radio_rs=1;}else
if(radio_rs<=24){radio_rs=3;}else
if(radio_rs<=32){radio_rs=7;}else
if(radio_rs<=40){radio_rs=15;}else
if(radio_rs<=48){radio_rs=31;}else
if(radio_rs<=56){radio_rs=63;}else
if(radio_rs<=64){radio_rs=127;}else
if(radio_rs>64){radio_rs=128;}
spiRW(radio_rs);
update_LED();
//spiRW(radio);
//Dimming
if(bit_is_clear(ADCSRA,ADSC))
{
OCR2=ADCH;
ADCSRA|=(1<<ADSC);
}
if(bit_is_set(alarm,7) && min==alarm_min+snooze && hour==alarm_hour)
{
 //Do alarm stuff    this will only sound alarm for 1 min
 if(bit_is_set(PORTE,4)){PORTE&=~(1<<4);}
 DDRC |=(1<<2);
 TCCR1B|=(1<<CS12);
 //line1[1]='A';line1[2]='L';line1[3]='A';line1[4]='R';line1[5]='M';
}
if(bit_is_clear(alarm,7)||min!=alarm_min+snooze)
{//line1[1]=' ';line1[2]=' ';line1[3]=' ';line1[4]=' ';line1[5]=' ';
 if(mode==3){PORTE|=(1<<4);}
 DDRC &=~(1<<2);PORTC &=~(1<<2);
 TCCR1B &= ~(1<<CS12);
}//end else
if(min==60){hour++;min=0;}
if(hour==24){hour=0;}
switch(mode){
case 0:segsum((hour*100)+min);
       update_LED();
       volume+=(countL*6);
       countL=0;
       uint8_t i;
       for(i=0;i<17;i++){line1[i]=temp_array[i];}
       break;
case 1:segsum((hour*100)+min);update_LED();min+=countR;hour+=countL;countR=0;countL=0;count=0;
      if(min<0){min=59;hour--;}
      if(hour<0){hour=24;}
      break;
case 2:segsum((alarm_hour*100)+alarm_min);update_LED();alarm_min+=countR;alarm_hour+=countL;countR=0;countL=0;
      if(alarm_min<0){alarm_min=59;alarm_hour--;}
      if(alarm_hour<0){hour=24;}
      if(alarm_min==60){hour++;min=0;}
      if(alarm_hour==25){alarm_hour=0;}
      break;
case 3:if(tune_time!=0){tune_time--;
       segment_data[4]=10;
       segsum(current_fm_freq/10);}
       else{segsum((hour*100)+min);}
       if(countR!=0)
       {tune_time=20000;
        current_fm_freq+=countR*20;
        countR=0;
        fm_tune_freq();
       }
       volume+=(countL*6);countL=0;
       update_LED();

      break;
      default: update_LED();
}//switch(mode)
}//while
return 0;
}//main
