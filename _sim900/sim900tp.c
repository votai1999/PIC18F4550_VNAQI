#include <18F4431.h>

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale
#FUSES HS                       //High speed Osc (> 4mhz FOR PCM/PCH) (>10mhz for PCD)
#FUSES FCMEN                    //Fail-safe clock monitor enabled
#FUSES IESO                     //Internal External SWITCH Over mode enabled
#FUSES NOPUT                    //No Power Up Timer
#FUSES NOBROWNOUT               //No brownout reset
#FUSES BORRES                
#FUSES NOWINEN                  //WDT Timer Window Disabled
#FUSES NOPWMPIN                 //PWM outputs drive active state upon Reset
#FUSES LPOL_HIGH                //Low-Side Transistors Polarity is Active-High (PWM 0,2,4 and 6)

//PWM module low side output pins have active high output polar
#FUSES HPOL_HIGH                //High-Side Transistors Polarity is Active-High (PWM 1,3,5 and 7)

//PWM module high side output pins have active high output polarity
#FUSES T1LOWPOWER               //Timer1 low power operation when in sleep
#FUSES FLTAC1                   //FLTA input is multiplexed with RC1
#FUSES SSP_RC                   //SCK/SCL=RC5, SDA/SDI=RC4, SDO=RC7
#FUSES PWM4B5                   //PWM4 output is multiplexed on RB5
#FUSES EXCLKC3                  //TMR0/T5CKI external clock input is muliplexed with RC3
#FUSES MCLR                     //Master Clear pin enabled
#FUSES STVREN                   //Stack full/underflow will cause reset
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used FOR I/O
#FUSES NODEBUG                  //No Debug mode FOR ICD
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOCPB                    //No Boot Block code protection
#FUSES NOCPD                    //No EE protection
#FUSES NOWRT                    //Program memory not write protected
#FUSES NOWRTC                   //configuration not registers write protected
#FUSES NOWRTB                   //Boot block not write protected
#FUSES NOWRTD                   //Data EEPROM not write protected
#FUSES NOEBTR                   //Memory not protected from table reads
#FUSES NOEBTRB                  //Boot block not protected from table reads
#device *=16 HIGH_INTS=TRUE
#use delay(clock=20000000)
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,stream=PORT1SIM)
#use rs232(baud=9600,parity=N,xmit=PIN_C5,rcv=PIN_C4,bits=8,stream=PORT2CBSA) // thiet lap UART mem C5 tx, c4 rx
#include <string.h>
#INCLUDE <stdlib.h>
#define buffer_size 160 //Buffer size you can adjust this size

//cac ham trong chuong trinh
Long hienthims(INT1 i);

//cac bien
unsigned char ma_led_7seg[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};    //mã led Anode chung
char c, mag[30], result[20], buffer[buffer_size], buff[10]; // SAVE Response 
char rsensor; // gia tri cam bien
int xbuff=0x00, xbuff2, count =0,i, answer = 0,kt=0,x; //
//Chuong trinh ngat UART
#INT_RDA

void RDA_isr()
{

	c=fgetc(PORT1SIM);
	buffer[xbuff]=c;
	if (c =='*')
	{
		kt =1;
		xbuff2 =0;
		for ( i =0; i<10;i++)
		{
			buff[i] = 0x00;
		}
	}
	else if 
	(c=='#'){
		kt =0;

	}
	if (kt ==1)
	{
		buff[xbuff2] = c;
		xbuff2 ++;
	}
	xbuff++;


}

// Chuong trinh ngat timer 0
#INT_RTCC

void  ngat_timer0(VOID)
{
	set_timer0(1);//dem tu 1 tro di
	Count++;

	IF(Count==20)
	{
		//dinh thoi 20*0.0512*(2^16)=67100ms-hon 1 phut,muon nhieu hon thi thay 20 thanh so khac
		
		Count=0;
	}
}

void erase_buffer(VOID)
{
	INT i;
	FOR(i=0;i<buffer_size;i++){ buffer[i]=0x00;}
	xbuff=0x00;
	RETURN;
}

INT SendATcommand(char *command, char *expected_answer, LONG mstimeout)
{
	INT answer=0;
	//CHAR*ATcommand="this is a test";
	delay_ms(500);
	xbuff=0;
	erase_buffer();
	delay_ms(100);//Delay to be sure no passed commands interfere
	fprintf(PORT1SIM,command);
	fputc(13,PORT1SIM);
	fputc(10,PORT1SIM);
	delay_ms(100);

	hienthims(0);

	DO
	{
		IF(strstr(buffer,result)!=NULL)
		{
			answer=1;
		}

		ELSE answer=0;
	}

	WHILE((answer==0)&&(hienthims(1)<mstimeout));

	RETURN answer;
}

void timer0_config()
{
	//Cau Hinh Timer 0//
	//setup_timer_0(RTCC_INTERNAL|RTCC_DIV_64|RTCC_8_bit);//T_tm0=T_osc*4*64
	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_256);//T_tm0=T_osc*4*256,chuyen qua 8bit tran thi them RTCC_8_bit
	//setup_timer_0(RTCC_EXT_L_TO_H|RTCC_DIV_256|RTCC_8_bit);//Tang gia tri timer0 len 1 khi co 256 xung len tai chan PIN_A4(T0CKI)
	//setup_timer_0(RTCC_EXT_H_TO_L|RTCC_DIV_64|RTCC_8_bit);//Tang gia tri timer0 len 1 khi co 64 suon xuong tai chan PIN_A4(T0CKI)
	/******************Thiet lap ngat timer*********************/
	enable_interrupts(INT_TIMER0);//Cho phep ngat tran timer0 hoat dong
	//disable_interrupts(INT_TIMER0);//Khong cho phep ngat tran timer0 hoat dong(mac dinh)
}

Long hienthims(INT1 i)
{
	//Ham nay dung de hien thi thoi gian tu luc set_timer(1)
	//neu ham la hienthims(0)thi bat dau bo dem lai tu dau
	//neu ham la hienthims(1)thi xuat thoi gian milisec
	IF(i==0)
	{
		count=0;
		set_timer0(1);//dung lenh set_timer0(1)de dem lai tu dau
		RETURN 0;
	}

	ELSE
	{
		LONG val=get_timer0()*0.0512+3355.44*count;//T_ms=chuky thach anh*4*256=0.0512 ms(timer dem len 1 don vi mat 0.0512 ms)
		//O Pic 16F bo dem mac dinh 8 bit nen dem duoc 0.0512*256 thi set co tran,PIc 18F dem duoc 16Bit=0.0512*(2^16)duoc hon 3000ms thi tran
		RETURN val;
	}
}

int docsensor(LONG mstimeout)
{
	hienthims(0);
	rsensor = 0;
	WHILE(!kbhit(PORT2CBSA)&&(hienthims(1)<mstimeout))
	{
		
		;
	}

	while ((rsensor == 0)&&(hienthims(1)<mstimeout))
	{
		rsensor = fgetc(PORT2CBSA);
		delay_us(10);
	}
	if (rsensor !=0) return rsensor;
	else RETURN 99;
}

void get_http()
{
	answer=0;
	mag="AT+HTTPINIT";
	result="OK";
	answer=sendATcommand(mag,result,5000);

	WHILE(answer==1)

	{
		mag="AT+HTTPPARA=\"CID\",1";
		result="OK";
		answer=sendATcommand(mag,result,5000);

		IF(answer==1)
		{
			rsensor=docsensor(5000);
			fprintf(PORT1SIM,"AT+HTTPPARA=\"URL\",\"http://height-increase.info/getdata.php?nhietdo=27&doam=%u",rsensor);        
			mag="\"";
			result="OK";
			answer=sendATcommand(mag,result,5000);

			mag="AT+HTTPACTION=0";
			result="+HTTPACTION:0";
			
			answer=sendATcommand(mag,result,3000);
			delay_ms(1000);
			
			mag="AT+HTTPREAD";
			result="OK";
			answer=sendATcommand(mag,result,3000);
			delay_ms(1000);
			fprintf(PORT1SIM,"-----%s-----",buff);
			char *toInt = buff+1 ;// xoa 1 ky tu dau tien cua chuoi buff
			x = atol(toInt); //chuyen chuoi so sang so
			if (x!=0){
				output_d(ma_led_7seg[x]);}
			fprintf(PORT1SIM,"-x:---%u-----",x);
			fputc(13,PORT1SIM);
			IF(answer==1)
			{
				
			}
			
		}
	}

	mag="AT+HTTPTERM";
	result="OK";
	answer=sendATcommand(mag,result,5000);
}

void main()
{
	/*Cai dat ngat va timer*/
	enable_interrupts(INT_RDA);//cho phep ngat uart
	enable_interrupts(GLOBAL);//cho phep ngat toan cuc
	timer0_config();
	hienthims(0);
	output_low(PIN_C0); // Bat led 7 doan thu 1 tren kit
	output_d(ma_led_7seg[0]); // hien thi so 0 tren led 7 doan
	delay_ms(5000);//Cho Module sim khoi dong,tam 5000ms
	mag="AT";
	result="OK";
	sendATcommand(mag,result,5000);

	mag="AT+SAPBR=3,1,\"Contype\",\"GPRS\"";
	result="OK";
	sendATcommand(mag,result,5000);

	mag="AT+SAPBR=1,1";
	result="OK";
	sendATcommand(mag,result,5000);

	WHILE(sendATcommand(mag,result,5000)!=1)
	{
		fPrintf(PORT1SIM,"AT+SAPBR=0,1");
		fputc(13,PORT1SIM);
		delay_ms(3000);
	}


	WHILE(1)
	{
		
		get_http();
		// DELAY_ms(3000);
	}

	fprintf(PORT1SIM,"KET THUC CT");
}

