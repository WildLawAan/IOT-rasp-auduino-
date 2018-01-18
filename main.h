#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <stdlib.h>

#define IS(input) (input<6)?(PINC & (1<<input)):(PINB & (1<<(input-4)))
#define SET(output) (output<2)?(PORTB |= (1<<output)):(PORTD |= (1<<output))
#define CLR(output) (output<2)?(PORTB &= ~(1<<output)):(PORTD &= ~(1<<output))

#define OUT1 0x01  
#define OUT2 0x02  
#define OUT3 0x04  
#define OUT4 0x08  
#define OUT5 0x10  
#define OUT6 0x20  
#define OUT7 0x40  
#define OUT8 0x80  

#define IN1	(INPUT&0x01)
#define IN2	(INPUT&0x02)
#define IN3	(INPUT&0x04)
#define IN4	(INPUT&0x08)
#define IN5	(INPUT&0x10)
#define IN6	(INPUT&0x20)
#define IN7	(INPUT&0x40)

#define REMOCON1 (REMOCON&0x01)
#define REMOCON2 (REMOCON&0x02)
#define REMOCON3 (REMOCON&0x04)
#define REMOCON4 (REMOCON&0x08)
#define REMOCON5 (REMOCON&0x10)

#define PWM_PERIOD 200     // 0.01us X 200 = 2ms

/// General function declaration
void sensor(void);	//interrupt func
unsigned char RcvCal(unsigned char data);	//interrupt remocon func 
void RcvRemoconData(unsigned char raw_nbl_data);	//interrupt remocon func

void motor1(signed int velocity);
void motor2(signed int velocity);
void motor3(signed int velocity);
void motor4(signed int velocity);

void servo1(unsigned char position);//0~256
void servo2(unsigned char position);//0~256
void servo3(unsigned char position);//0~256
void servo4(unsigned char position);//0~256
void servo5(unsigned char position);//0~256
void servo6(unsigned char position);//0~256
void servo7(unsigned char position);//0~256
void servo8(unsigned char position);//0~256

void delay_ms(unsigned int count);
void Delay_us(unsigned int count);
void on(unsigned char output_port);
void off(unsigned char output_port);

volatile unsigned char g_bPwmDuty[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile unsigned char g_bPortServo = 0;
volatile unsigned char g_bServoStart = 0;
volatile unsigned int  g_iServoPosition[8];
volatile unsigned char g_cMotor=0;
volatile unsigned int g_iServoTracking[8];
volatile unsigned int g_iServoVelocity[8];
volatile unsigned char g_bPortLevel=0;
volatile unsigned char g_bPortFlag=0;
volatile unsigned char RemoconPort = 6;
volatile unsigned char PreRemoconLevelFlag = 0;
volatile unsigned char RemoconHighCnt = 0;
volatile unsigned char RemoconLowCnt = 0;
volatile unsigned char REMOCON = 0;  // the remocon key data : [/][/][/][M][R][L][B][F]
volatile unsigned char RemoconUse = 0; //remocon use flag
volatile unsigned char T = 1;
volatile unsigned char INPUT=0;
volatile unsigned char REMOCON_USE=0;
volatile unsigned int g_iGlobalDelay=0;
INTERRUPT(SIG_OVERFLOW1)
{
	static unsigned int temp=0;
	if(g_bPortServo & (1<<g_cMotor))
	{
		g_iServoTracking[g_cMotor]=g_iServoPosition[g_cMotor];
		if(g_bPortLevel)
		{

			SET(g_cMotor);
			temp=0xffff-g_iServoTracking[g_cMotor];
	        TCNT1=temp;
			g_bPortLevel=0;
		}
		else
		{
			CLR(g_cMotor);
			temp=0x0af0-g_iServoTracking[g_cMotor];
			temp=0xffff-temp;
	            TCNT1=temp;
			g_bPortLevel=1;
			g_cMotor++;
			if(g_cMotor>7)	g_cMotor=0;
		}
	}
	else
	{
		TCNT1= 0xf50f;
		g_cMotor++;
		if(g_cMotor>7)	g_cMotor=0;
	}

}

INTERRUPT(SIG_OUTPUT_COMPARE2)
{
    sensor();
	g_iGlobalDelay++;
    if((g_bPortServo & 0x01)==0)
   {
	    if((g_bPortFlag & 0x01) && T<=g_bPwmDuty[0])
	        PORTB |= 0x01;         
	    else
	        PORTB &= ~0x01;        
    }
    if((g_bPortServo & 0x02)==0)
    {
	    if((g_bPortFlag & 0x02) && T<=g_bPwmDuty[1])
	        PORTB |= 0x02;          
	    else
	        PORTB &= ~0x02;        
    }
    if((g_bPortServo & 0x04)==0)
    {
	    if((g_bPortFlag & 0x04) && T<=g_bPwmDuty[2])
	        PORTD |= 0x04;        
	    else
	        PORTD &= ~0x04;        
    }
    if((g_bPortServo & 0x08)==0)
    {
	    if((g_bPortFlag & 0x08) && T<=g_bPwmDuty[3])
	        PORTD |= 0x08;         
	    else
	        PORTD &= ~0x08;        
    }
    if((g_bPortServo & 0x10)==0)
    {
	    if((g_bPortFlag & 0x10) && T<=g_bPwmDuty[4])
	        PORTD |= 0x10;         
	    else
	        PORTD &= ~0x10;       
    }
    if((g_bPortServo & 0x20)==0)
    {
	    if((g_bPortFlag & 0x20) && T<=g_bPwmDuty[5])
	        PORTD |= 0x20;        
	    else
	        PORTD &= ~0x20;       
    }
    if((g_bPortServo & 0x40)==0)
    {
	    if((g_bPortFlag & 0x40) && T<=g_bPwmDuty[6])
	        PORTD |= 0x40;        
	    else
	        PORTD &= ~0x40;      
    }
    if((g_bPortServo & 0x80)==0)
    {
	    if((g_bPortFlag & 0x80) && T<=g_bPwmDuty[7])
	        PORTD |= 0x80;        
	    else
	        PORTD &= ~0x80;     
    }

	if(PreRemoconLevelFlag)
	{
		if(IS(RemoconPort))
		{
			RemoconHighCnt++;
		}
		else
		{
			RemoconLowCnt = 1;
			RcvRemoconData(RemoconHighCnt);
			PreRemoconLevelFlag = 0;    
		}
	}
	else
	{
		if(IS(RemoconPort))
		{
			RemoconHighCnt = 1;         
			PreRemoconLevelFlag = 1;    
		}
		else
		{
			if(RemoconLowCnt++ > 20)
			{
				RemoconLowCnt = 1;
				REMOCON = 0;

			}
		}
	}
    if(T == PWM_PERIOD)  T = 1;
    else    T++;
}
unsigned char RcvCal(unsigned char data)
{
	if(data<70)
	{
		if(data<20)	return 1;
		else if(data<30)	return 2;
		else if(data<40)	return 3;
		else if(data<50)	return 4;
		else if(data<60)	return 5;
		else if(data<70)	return 6;
	}
	else if(data<130)
	{
		if(data<80)	return 7;
		else if(data<90)	return 8;
		else if(data<100)	return 9;
		else if(data<110)	return 10;
		else if(data<120)	return 11;
		else if(data<130)	return 12;

	}
	else if(data<190)
	{
		if(data<140)	return 13;
		else if(data<150)	return 14;
		else if(data<160)	return 15;
		else if(data<170)	return 16;
		else if(data<180)	return 17;
		else if(data<190)	return 18;
	}
    return 0;

}
void servo1(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x01;
	g_bPortFlag &= ~ (0x01);
	g_iServoVelocity[0]=0x40;
	g_iServoPosition[0]=(unsigned int)((position<<3)+650); 
	if((g_bServoStart&0x01)==0) g_iServoTracking[0]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x01;
}
void servo2(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x02;
	g_bPortFlag &= ~ (0x02);
	g_iServoVelocity[1]=0x40;
	g_iServoPosition[1]=(unsigned int)((position<<3)+650);
	if((g_bServoStart&0x02)==0) g_iServoTracking[1]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x02;
}
void servo3(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x04;
	g_bPortFlag &= ~ (0x04);
	g_iServoVelocity[2]=0x40;
	g_iServoPosition[2]=(unsigned int)((position<<3)+650); 
	if((g_bServoStart&0x04)==0) g_iServoTracking[2]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x04;
}
void servo4(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x08;
	g_bPortFlag &= ~ (0x08);
	g_iServoVelocity[3]=0x40;
	g_iServoPosition[3]=(unsigned int)((position<<3)+650); 
	if((g_bServoStart&0x08)==0) g_iServoTracking[3]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x08;
}
void servo5(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x10;
	g_bPortFlag &= ~ (0x10);
	g_iServoVelocity[4]=0x40;
	g_iServoPosition[4]=(unsigned int)((position<<3)+650); 
	if((g_bServoStart&0x10)==0) g_iServoTracking[4]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x10;
}
void servo6(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x20;
	g_bPortFlag &= ~ (0x20);
	g_iServoVelocity[5]=0x40;
	g_iServoPosition[5]=(unsigned int)((position<<3)+650); 
	if((g_bServoStart&0x20)==0) g_iServoTracking[5]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x20;
}
void servo7(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x40;
	g_bPortFlag &= ~ (0x40);
	g_iServoVelocity[6]=0x40;
	g_iServoPosition[6]=(unsigned int)((position<<3)+650);
	if((g_bServoStart&0x40)==0) g_iServoTracking[6]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x40;
}
void servo8(unsigned char position)
{
	if(position>205)		position=205;
	g_bPortServo |= 0x80;
	g_bPortFlag &= ~ (0x80);
	g_iServoVelocity[7]=0x40;
	g_iServoPosition[7]=(unsigned int)((position<<3)+650); 
	if((g_bServoStart&0x80)==0) g_iServoTracking[7]=(unsigned int)((position<<3)+650); 
	g_bServoStart|=0x80;
}

void RcvRemoconData(unsigned char raw_nbl_data)
{
unsigned char cal_data;
    static unsigned char step = 0;
    static unsigned char buf[2];
    unsigned char nbl_data;

    if(raw_nbl_data < 185)
    {
		cal_data=raw_nbl_data+5;
		nbl_data=RcvCal(cal_data);
    }
    else
        nbl_data = 0;

    switch(step)
    {
    case 0:
        if(nbl_data == 1)       
            step++;
        break;
    case 1:
        buf[0] = nbl_data - 2;  
        step++;
        break;
    case 2:
        buf[1] = nbl_data - 2;  
        REMOCON = ((buf[0]&0x0F)|((buf[1]<<4)&0xF0));
        step = 0;
        break;
    }
}

void motor1(signed int velocity)
{
    g_bPortFlag|=0x03;
    g_bPortServo&=~(0x03);
	
    if(velocity==0)
    {
	   g_bPwmDuty[0] = 200;
	   g_bPwmDuty[1] = 200;
    }
    else if(velocity<0)
    {
	   g_bPwmDuty[0] = 0;
	   g_bPwmDuty[1] = abs(velocity);
    }
    else
    {
	   g_bPwmDuty[0] = velocity;
	   g_bPwmDuty[1] = 0;
    }
}
void motor2(signed int velocity)
{
    g_bPortFlag|=0x0c;
    g_bPortServo&=~(0x0c);
	
    if(velocity==0)
    {
	   g_bPwmDuty[2] = 200;
	   g_bPwmDuty[3] = 200;
    }
    else if(velocity<0)
    {
	   g_bPwmDuty[2] = 0;
	   g_bPwmDuty[3] = abs(velocity);
    }
    else
    {
	   g_bPwmDuty[2] = velocity;
	   g_bPwmDuty[3] = 0;
    }
}
void motor3(signed int velocity)
{
    g_bPortFlag|=0x30;
    g_bPortServo&=~(0x30);
	
    if(velocity==0)
    {
	   g_bPwmDuty[4] = 200;
	   g_bPwmDuty[5] = 200;
    }
    else if(velocity<0)
    {
	   g_bPwmDuty[4] = 0;
	   g_bPwmDuty[5] = abs(velocity);
    }
    else
    {
	   g_bPwmDuty[4] = velocity;
	   g_bPwmDuty[5] = 0;
    }
}
void motor4(signed int velocity)
{
    g_bPortFlag|=0xc0;
    g_bPortServo&=~(0xc0);
	
    if(velocity==0)
    {
	   g_bPwmDuty[6] = 200;
	   g_bPwmDuty[7] = 200;
    }
    else if(velocity<0)
    {
	   g_bPwmDuty[6] = 0;
	   g_bPwmDuty[7] = abs(velocity);
    }
    else
    {
	   g_bPwmDuty[6] = velocity;
	   g_bPwmDuty[7] = 0;
    }
}

void on(unsigned char output_port)
{
unsigned char i;
	for(i=0;i<8;i++)
	{
		if(output_port&(1<<i))
		{
			g_bPortFlag|=(1<<i);
			g_bPortServo&=~(1<<i);
			g_bPwmDuty[i]=200;
		}
	}
}

void off(unsigned char output_port)
{
unsigned char i;
	for(i=0;i<8;i++)
	{
		if(output_port&(1<<i))
		{
			g_bPortFlag&=~(1<<i);
			g_bPortServo&=~(1<<i);
			g_bPwmDuty[i]=0;
		}
	}
	
}

void delay_ms(unsigned int count)
{
unsigned int i;

	for(i=0;i<count;i++)
	{
		Delay_us(840);
	}
}

void Delay_us(unsigned int count)
{
unsigned int i;
	for(i=0;i<count;i++);
}

void sensor(void)
{
    INPUT=~(PINC&0x3f);
    if(PINB&0x04)   INPUT&=~(0x40);
    else    INPUT|=0x40;
    INPUT&=0x7f;
}

void start(void) 
{
	DDRB |= 0x03;            
	DDRD |= 0xFC;            
	TIMSK |= 0x80;
	TCCR2 = 0x0a; 			
	OCR2 = 100;             
	TCCR1A = 0x00;
	TCCR1B = 0x02;			
	TIMSK|=0x04;
	PORTB |= 0x04;
	PORTC |= 0x3F;
	sei();                  
	delay_ms(500);
}

