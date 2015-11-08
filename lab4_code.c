// lab4_code.c 
// Chad Gibeaut
// 10/28/15
//Code is a compilation of functions from multiple labs
// Adopted from skelleton code for lab3

//Hardware Setup
// LED bar graph is multiplexed, PORTA outputs for the segments
// PORT B is used for SPI and binary output for decoder for LED digits
// SPI port is used to send data to the bar graph, and read data in from Encoders
// Since each SPI device is only a one way data stream, these can be done at the same time
#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define bar_clk 2  //on port F
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>//added

//volatile uint8_t ext_count=0; not being used at this time
#define SEL0 4
#define SEL1 5
#define SEL2 6
#define PWM 7
//*******************Global*****************************************
//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 
int8_t min=49;            //Startup time
int8_t hour=17;           //Startup time
int16_t countL=0;
int16_t countR=0;
uint8_t mode=0;            //Used for tracking mode 
uint8_t inactive=0;
uint8_t count=0;
volatile uint16_t count_t=0;
uint8_t dec_state=0;        //current state of decoders
uint8_t dec_state_last=0;  //previous state of decoders
uint8_t inc=1;//This is the increment amount set by buttons 
//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={192,249,164,176,153,146,131,248,128,152,255,4} ;
uint8_t up=1;
//index 10 is used as the 'blank' digit

//                                   segment_sum
//determine number of digits
//bound count by 1023
//break count up into its digits and store as needed
//digits are stored index 0-3   4 can be used later for colon
void segsum(int16_t sum) {
//if(sum>1023){sum=sum-1024;count=count-1024;}
//if(sum<0){sum=sum+1024;count=count+1024;}

 //uint8_t digits=1; //how many digits will be used
 //if(sum>9){digits=2;}
 //if(sum>99){digits=3;}
 //if(sum>999){digits=4;}
 uint8_t dig;
 for(dig=0;dig<4;dig++){segment_data[dig]=10;}
 segment_data[0]=sum/1000;sum=sum-segment_data[0]*1000;
 segment_data[1]=sum/100;sum=sum-segment_data[1]*100;
 segment_data[2]=sum/10;sum=sum-segment_data[2]*10;
 segment_data[3]=sum;
}//segment_sum


void spiRW(uint8_t bar){
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
 DDRF=(1<<bar_clk);
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
if(debounce_switch(0)==1){mode =1;countL=0;countR=0;}//toggle the 0 bit 
if(debounce_switch(1)==1){mode =2;countL=0;countR=0;}//toggle the 1 bit
if(debounce_switch(2)==1){mode =3;countL=0;countR=0;}//toggle the 0 bit 
if(debounce_switch(3)==1){mode =4;countL=0;countR=0;}//toggle the 1 bit  LOST 13Hz adding 6 if statements
if(debounce_switch(4)==1){mode =5;countL=0;countR=0;}//toggle the 0 bit 
if(debounce_switch(5)==1){mode =6;countL=0;countR=0;}//toggle the 1 bit
if(debounce_switch(6)==1){mode =7;countL=0;countR=0;}//toggle the 0 bit 
if(debounce_switch(7)==1){mode =0;countL=0;countR=0;}//toggle the 1 bit
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
PORTA=dec_to_7seg[segment_data[dig-1]];
asm("nop");
_delay_ms(.02);//0.02
 }//end dig
}

//sets up the timer
void init_tcnt0(){
  ASSR  |=(1<<AS0);     //run off external 32khz osc (TOSC)
  TIMSK=0x00; //reset to all 0
  TIMSK |=(1<<OCIE0);
  //need to set the interupt for the compare match  
  TCCR0=0x00;//reset to zero
  TCCR0 |=  (1<<WGM01) | (0<<CS02)|(0<<CS01)|(1<<CS00);  //CTC mode,no prescaler
  OCR0 =25;          //250 was calculated
}

//  startup_test runs once at the beginning of main, this is just used
//  to show all digits and ensure wiring is correct at start up
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


ISR(TIMER0_COMP_vect){	
check_sw(); //checks switches	
spiRW(mode);//Updates mode on bar graph and gets new encoder value
count_t++;
if(count_t==635)//640 calc 631 shows 500ms but i think its off a bit
{
PORTC ^=1<<0;
count++;
if(count==120){min++;count=0;}
count_t=0;
if(segment_data[4]==11 && mode==0){segment_data[4]=10;}
else{segment_data[4]=11;}
}
}//IRS

//*****************************************************************
int main()
{
DDRC=(1<<0)|(1<<1);// Sets bit 6 and 7 to output, used for checking timing
PORTC=(1<<0)|(1<<1);//Sets both bits high to start wit
segment_data[4]=10;
init_tcnt0();               //initalize timer counter zero
segsum(count);//sets up the segment data using initial count
DDRB=0b11110111;//set port bits 4-7 B as outputs  and sets up SPI port
spi_init();     //sets up SPI 
startup_test(); //Runs through all digits to ensure they are working
spiRW(mode);  //initial mode displayed on bar graph (all off)
sei();         //enable global interrupts
while(1){
if(min<0){min=59;hour--;}
if(hour<0){hour=24;}
if(min==60){hour++;min=0;}
if(hour==25){hour=0;}
switch(mode){
case 0:segsum((hour*100)+min);update_LED();break;
case 1:segsum((hour*100)+min);update_LED(); min+=countR;hour+=countL;countR=0;countL=0;count=0;break;
}
}//while
return 0;
}//main
