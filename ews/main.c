// yeu cau nguon cap cho sim808 la 12 V_v5 of sim 2h neu nguoi duoi 10.6V
//V6 testWDT neu chip bi treo quá 2 phút (128s)
// LUU Y CHON LAI CAM BIEN DO DO SAU TUONG UNG
// THAY DOI SO VOI V3 CHO SDA B2->A3.
#include <18F4550.h>
#device PASS_STRINGS = IN_RAM    
#DEVICE ADC=10                 
#fuses HSPLL,MCLR,WDT,NOPROTECT,NOLVP,NODEBUG,USBDIV,PLL5,CPUDIV1,VREGEN                 
#use delay(clock=48000000)      
//#use rs232(baud=9600,parity=N,xmit=PIN_D0,rcv=PIN_D1,bits=8,stream=LORA) // thiet lap UART mem
#use rs232(baud=9600,parity=N,xmit=PIN_D4,rcv=PIN_D5,bits=8,stream=COM2) // thiet lap UART mem chuan rs485 bo moi nhat giao tiep voi RS485
#use rs232(baud=9600,parity=N,xmit=PIN_C0,rcv=PIN_D2,bits=8,stream=UART2)
//#use rs232(baud=9600,parity=N,xmit=PIN_C1,rcv=PIN_C2,bits=8,stream=UART2)
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,stream=PORT1SIM)
#INCLUDE <math.h>
#include ".\_inc\types.h"
//#include ".\_inc\DS1307.c"

#define NoMes "AT+CMGF=0"

#define atw "AT&W"
#define atcs "AT+CSCS=\"GSM\""
#define atcm "AT+CMGF=1"
#define atcn "AT+CNMI=2,2,0,0,0"
//#define thanh "AT+CNMI=0,0,0,0,0"
#define atas "AT+CSAS" //bien phat hien co trom de bao coi
#define kiemtratn "AT+CMGR=1"
#define xoatn "AT+CMGDA=\"DEL ALL\""
//URL WEB
//#define URL "AT+HTTPPARA=\"URL\",\"http:\/\/sim900v1.tk/getdata.php?"
//---------LENH AT---------
#define BATGPRS "AT+SAPBR=1,1"
#define TATGPRS "AT+SAPBR=0,1"
#define HTTPINIT "AT+HTTPINIT"
#define HTTPPARA "AT+HTTPPARA=\"CID\",1"
#define HTTPACTION "AT+HTTPACTION=0"
#define RESULTHTTLACTION "+HTTPACTION: 0,200"
#define HTTPREAD "AT+HTTPREAD"
#define HTTPTERM "AT+HTTPTERM"
#define OK "OK"
#define SETAPN "AT+SAPBR=3,1,\"Contype\",\"GPRS\""
#define AT "AT"
#define GETTIME "AT+CCLK?"
#define CCLK "CCLK"
//---------LENH AT---------
#include <stdlib.h> // for atoi32             


#include "mmcsd.c"
//FAT library.                                                    
#include <fat.c>
char *sdt5;
char *tachsdt1[2];
char sdt6[10];
#define COMMAND_SIZE 10                                                 
#define NUM_COMMANDS 11


float power1,power2,power3;
//unsigned int power;
unsigned int16 rcount = 0, rcount2 = 0, rcount3=0,rcount4=0, rcount5=0,rcount6=0;
unsigned int16 rcount1;
unsigned int16 Pulse=0,pre_Pulse=0, rain=0;
signed long pre_rain=0, delta=0;
float timeupdate=0;
//***********kt doc mua va adc************
int cambien_deep=0;
//uint8 sensor_count; 
BYTE sec; 
BYTE min; 
BYTE hrs; 
BYTE day; 
BYTE month; 
BYTE yr; 
BYTE dow; 
BYTE ds_H;
float h1,h2,h3;
int tuantu=0;
unsigned chatluong = 0;
char bufferTP[70];  
char buff_date[15]; 
char buff_time[15];
char gfilename[15];
char gfileopen[15];
void AppendFile(char *fileName, char *appendString);
void MakeFile(char *fileName);
//float pre_power = 0,delta_power=0;
unsigned int16 Temp, RH,countcmra=0;
short Time_out ;
unsigned int8 T_byte1, T_byte2, RH_byte1, RH_byte2, CheckSum ;
int new_sms=0,baodong=0,relay3=0;
////////////////////////////////
//                            //
// LAY BEN CODE SIM 900       //
////////////////////////////////
int biencamera=0;
/* Danh cho sim 900*/
char URLeeprom[100],phoneeeprom[50];
unsigned int dodaiURL =0,dodaiphone=0;
#define buffer_size 100 //Buffer size you can adjust this size
#define buff2_size 100
char relay11[1], relay22[1],reset1[1];
//#DEVICE HIGH_INTS=TRUE

//cac bien
//unsigned char ma_led_7seg[]={0xc0,0xf9,0xD3,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};    //m? led Anode chung
char c,c1,c2,c3,c4, mag[30], result[20], buffer[buffer_size], buff[buff2_size], *toInt, *ptr,sdt1[10],sdt[10],*sdt2,thutin[9]; // SAVE Response 
unsigned int rsensor; // gia tri cam bien
int xbuff=0x00, xbuff2, count =0,i, answer = 0,kt=0,gg, demgprs=0; //
float tachdata[11];
char *tachsdt[5];
int ss_sdt0,ss_sdt1,ss_sdt2,ss_sdt3,ss_sdt4,s=0,idmach = tachdata[0], setcapnhat = tachdata[1],setbit=0, setbit1=0, bit_bat=0, bit_binhthuong=0, bit_kt=0, bit_sl=0, relay1, relay2,reset,read_http=0;
//float ofset1=tachdata[7]=32.75, ofset2=tachdata[8]=0, ofset3=tachdata[9]=0, ofset,ofset1_n, ofset2_n, ofset3_n;
float muccanhbao1=tachdata[7]=0, muccanhbao2=tachdata[8]=0, muccanhbao3=tachdata[9]=0;
int1 kiemtraread=0, truyenlandau = 0,kt2=0,no_off_sim=0;
char querystring[100];
//Chuong trinh ngat UART
#define RTC_SDA  PIN_A3    
#define RTC_SCL  PIN_B3
#use i2c(master, sda=RTC_SDA, scl=RTC_SCL,fast=450000)
int DEC_2_BCD (int to_convert);
void Set_Time_Date(BYTE day, BYTE mth, BYTE year, BYTE hr, BYTE min, BYTE sec);
void Update_Current_Date_Time();
#INT_RDA
void RDA_isr()
{
   c=fgetc(PORT1SIM);
   buffer[xbuff]=c;
   if (c =='+')      
   c1=c; 
   if(c=='C')
   c2=c; 
   if(c=='M') 
   c3=c; 
   //if(c=='T') 
  // c4=c; 
   if(c1=='+'&&c2=='C'&&c3=='M');//&&c4=='T') 
   {new_sms=1;
    //kt2=0;
   }    
   if (c =='*')
   {//kt2=1; 
      kt =1;
      xbuff2 =0;
      for ( i =0; i<buff2_size;i++)
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

//*****************CT RAIN********//
void eeprom_write_string(unsigned int8 addr,unsigned char*str)
{
   while(*str)
   {
      write_eeprom(addr,*str);
      addr++;
      str++;
   }
}

void eeprom_read_string(unsigned int8 addr, unsigned char* str,unsigned int8 len)
{
   unsigned int8 i,j,c;
   j = 0;
   for(i=0;i<len;i++)
   {
      c = read_eeprom(addr+i);
      if ((c > 32) && (c < 127))
      {
         str[j]= c;
         j++;
      }
   }
   str[j]=0;
}


#INT_TIMER1                                 // Timer1 interrupt ISR

void timer1_isr()
{
   set_timer1(5536); //1/(48M/16)x60000
   rcount++;
   rcount2++;
   
   if(rcount%30==0)
   {
      delta=abs(Pulse-pre_Pulse);
      pre_Pulse=Pulse;
      if(delta==1)
      {  rain++;
         pre_rain = rain;
      }
      else 
      {
         rain = pre_rain;
      }
   // eeprom_write_string(0x40,rain);
  /////////?????????????????????????????????????????????????????????????????????????????????????????????????????
   }
   if(rcount==3000)//15000)//50 la 1 giay/3000 la 1 phut, 15000 la 5 phut
   {rcount=0; 
      rcount1++;
      rcount3++;
      rcount4++;
      rcount6++;
      countcmra++;
      //  rain=Pulse;     //tinh so lan dong cua cam bien (trong 1 sampling time) dung 1s câp nhât 1 lân
      //  Pulse =0;
      //  delta=abs(rain-pre_rain);// tính chênh lêch giua cac lân do dê xuat canh bao len server ma ko can phai dung 5 phut/60phut
      //  pre_Pulse=Pulse;
   // fprintf(UART2,"rain= %lu, kt=%u, Bat=%u,binhthuong=%u \n", rain,bit_kt,bit_bat,bit_binhthuong);////////////////
      //  pre_rain=rain;
   }

   // rain = 0;
   // clear_interrupt(INT_TIMER1);
   // disable_interrupts(INT_TIMER1);
}
#INT_EXT                                    // External interrupt ISR

void ext_isr()
{

   // if(!input(PIN_B2))
   //{
   delay_us(100);
   //  fprintf(UART2,"bat dau doc cam bien\n",); 
   //}
   // else
   Pulse++;
 //  output_bit (PIN_B7, !input(PIN_B7));

}

//*****************HET CT RAIN*********//


void erase_buffer()
{
   int i;
   for(i=0;i<buffer_size;i++){ buffer[i]=0x00;}
   xbuff=0x00;

   return;
}
//-------------- sendATcommand--------------//
// command: LenhAT--------------------------//
// expected_answer: phan hoi ---------------//
// mstimeout: thoi gian timeout: 3000 la 1 phut, 50 la 1 giay //
//-------------- sendATcommand--------------//
int1 sendATcommand(char *command, char *expected_answer,unsigned int16 mstimeout)
{


   int1 answerAT=0;
   //CHAR*ATcommand="this is a test";
   //delay_ms(500);
   new_sms=0;
   xbuff=0;
   erase_buffer();
   delay_ms(1000);//Delay to be sure no passed commands interfere
   fprintf(PORT1SIM,command);
   fputc(13,PORT1SIM);
   fputc(10,PORT1SIM);
   delay_ms(1000); //delay doi phan hoi @@

   // hienthims(0);
   rcount2 = 0;
   do
   {
      if(strstr(buffer,expected_answer)!=NULL)
      {
         answerAT=1;
      }

      else answerAT=0;
   }

   while((answerAT==0)&&(rcount2<mstimeout));
   //   fprintf(UART2,"%lu - %lu\n",rcount2, mstimeout);
   fprintf(UART2,"%s",buffer);
   return answerAT;
}



#define O "O"
void _httpinit()
{
   //bat gprs lai neu tu tat
   //  mag="AT+SAPBR=1,1";
   // result="O"; // OK: bat thanh cong // ERROR LA BAT THAT BAI
 /* if(setcapnhat<10)
  {answer= sendATcommand(BATGPRS,O,350);
  }*/
 
   /*
  if (answer==0)
  {answer= sendATcommand(BATGPRS,O,350);
  }*/
   //********
   kiemtraread=0;
   answer=0;
   //  mag="AT+HTTPINIT";
   //  result="OK";
   answer=sendATcommand(HTTPINIT,OK,250);

   if(answer==1)

   {
      //     mag="AT+HTTPPARA=\"CID\",1";
      //   result="OK";
      answer=sendATcommand(HTTPPARA,OK,250);
   }

}

void _httpread()
{

   mag="\"";
   //  result="OK";
   answer=sendATcommand(mag,OK,250);
   delay_ms(2000);
   //   mag="AT+HTTPACTION=0";
   //      result="+HTTPACTION: 0,200";

   answer=sendATcommand(HTTPACTION,RESULTHTTLACTION,10000);// tang tu 3000 len 10000


   //  mag="AT+HTTPREAD";
   //    result="OK";
   answer=sendATcommand(HTTPREAD,OK,3000);
   delay_ms(2000);
   kiemtraread=1;

   delay_ms(100);






}
void sendURL()
{
   fprintf(PORT1SIM,"AT+HTTPPARA=\"URL\",\"%s?",URLeeprom);


}
void get_http()
{

   sendURL();
   // fprintf(PORT1SIM,"nhietdo=%lu&doam=%lu&ngay=%u&thang=%u&gio=%u&phut=%u&nguon=%.1f&cb1=%u&cb2=%lu&cb3=33",Temp,RH,day,month,hrs,min,power1,rsensor,rain);
   fprintf(PORT1SIM,"%s",querystring);

}
void guiloixong()
{

   sendURL();
   fprintf(PORT1SIM,"auto=0");
   
   
}

void sendhttp(int chonhamgui) // chonhamgui: 0: guidulieu binh thuong 1: gui xong loi tu the nho
{
   _httpinit();

   if (answer ==1)
   {
      if (chonhamgui == 0)
      {
         get_http();
      }
      else 
      {
         guiloixong();
      }
      _httpread();
     //  fprintf(UART2,"%c : %c :%c  ",buffer[13],buffer[14],buffer[15]);
     read_http=(buffer[13]-48);
   //fprintf(UART2,"%u ",read_http);
     
   }


   answer=sendATcommand(HTTPTERM,OK,250);
//fprintf(UART2,"%u  ",answer);
}


/*Het sim 900 */





//////////////////////
// CT DOC NHIET DO DO AM //


void start_signal(){                     
   output_low(PIN_A5);
   delay_ms(25);
   output_high(PIN_A5);
   delay_us(30);
}
short check_response(){
   delay_us(40);
   if(!input(PIN_A5)){                     // Read and test if connection pin is low
      delay_us(80);
      if(input(PIN_A5)){                    // Read and test if connection pin is high
         delay_us(50);
         return 1;}
   }
}
unsigned int8 Read_Data(){
   unsigned int8 i, k, _data = 0;     // k is used to count 1 bit reading duration
   if(Time_out)
   break;
   for(i = 0; i < 8; i++){
      k = 0;
      while(!input(PIN_A5)){                          // Wait until pin goes high
         k++;
         if (k > 100) {Time_out = 1; break;}
         delay_us(1);}
      delay_us(30);
      if(!input(PIN_A5))
      bit_clear(_data, (7 - i));               // Clear bit (7 - i)
      else{
         bit_set(_data, (7 - i));                 // Set bit (7 - i)
         while(input(PIN_A5)){                         // Wait until pin goes low
            k++;
            if (k > 100) {Time_out = 1; break;}
            delay_us(1);}
      }
   }
   return _data;
}

//keT THUC CT DOC NHIET DO DO AM//


////////////////////////////////
///                          ///
/// Function Implementations ///
///                          ///
////////////////////////////////



/*
Summary: Deletes a file.
Param: The full path of the file to delete.
Returns: None.
*/


/*
Summary: Creates a file.
Param: The full path of the file to create.
Returns: None.
Example Usage: \> make "Log.txt"
*/
void MakeFile(char *fileName)
{
   // fprintf(UART2,"\r\nMaking file '%s': ", fileName);
   if(mk_file(fileName) != GOODEC)
   {
      //     fprintf(UART2,"Error creating file");
      return;
   }
   // fprintf(UART2,"OK");                                
}

/*
Summary: Append a string to a file.
Param: The full path of the file to append to.
Param: A pointer to a string to append to the file.
Returns: None.
Example Usage: \> append "Log.txt" "This will be appended to the end of Log.txt"
Note: A "\r\n" will be appended after the appendString.
*/
void AppendFile(char *fileName, char *appendString)
{
   FILE stream;
   //    fprintf(UART2,"\r\nAppending '%s' to '%s': ", appendString, fileName);
   if(fatopen(fileName, "a", &stream) != GOODEC)
   {                   
      //  fprintf(UART2,"Error opening file '%s'",fileName);
      return;
   }

   fatputs(appendString, &stream);
   fatputs("\r\n", &stream);

   if(fatclose(&stream) != GOODEC)
   {
      //     fprintf(UART2,"Error closing file");
      return;
   }
   //  fprintf(UART2,"OK");
}

/*
Summary: Change the working directory.
Param: The new working directory to switch to.
Returns: None.                              
Example Usage: \> cd ftp/     -> /ftp/
\ftp\> cd files/  -> /ftp/files/
\ftp\files> cd..  -> /ftp/
\ftp\> cd ..      -> /
\> cd /ftp/files/ -> /ftp/files/



#define CAT_FROM_START  FALSE
#define CAT_FROM_END    TRUE
/*
Summary: Prints either all of or the last 80 characters in a file.
Param: The full path of the file to print off.
Param: If true, this function will print off the last 80 characters in the file.
If false, this funciton will print off the entire file.
Returns: None.
Example Usage: /> cat "Logs.txt" (this will display the entire file)
Example Usage: /> tail "Logs.txt" (this will display the last 80 characters in the file)
*/

void PrintFile(char *fileName, int1 startFromEnd)
{
   FILE stream;

   if(fatopen(fileName, "r", &stream) != GOODEC)
   {
      printf("\r\nError opening file '%s'",fileName);  
      return;
   }

   fprintf(UART2,"\r\n");

   if(startFromEnd)                                   
   fatseek(&stream, 80, SEEK_END);

   fatprintf(&stream); 

   // fatclose(&stream);
   if(fatclose(&stream) != GOODEC)
   {
      printf("Error closing file '%s'",fileName);
      return;
   }
}


signed int tachdl(FILE* stream)
{
   signed int ch; // character read in

   // keep on printf any characters read in as long as we don't run into an end of file or a media error
   do
   {
      ch = fatgetc(stream);
      if (ch==60) //60 la ma thap phan cua <
      {
         //  fprintf(UART2,"chuoihttp?");
         //----
         answer=0;
         //   mag="AT+HTTPINIT";
         //    result="OK";
         _httpinit();

         if(answer==1)

         {
            
            
            //rsensor=docsensor(5000);
            // fprintf(PORT1SIM,"AT+HTTPPARA=\"URL\",\"http://height-increase.info/getdata.php?nhietdo=27&doam=666");  
            //  fprintf(PORT1SIM,"AT+HTTPPARA=\"URL\",\"http://tamphuc.16mb.com/getdata.php?auto=0&nguon=88&cb1=77&cb2=66&cb3=55&"); 
            //  fprintf(PORT1SIM,"AT+HTTPPARA=\"URL\",\"http://bkv.netplus.vn/getdata.aspx?auto=0&cb3=2");
            //  fprintf(PORT1SIM,URL);
           // fprintf(UART2,"Lay Data Tu the nho\n");
            sendURL();
            fprintf(PORT1SIM,"auto=0&");
            //fprintf(PORT1SIM,"AT+HTTPPARA=\"URL\",\"http://sim900v1.tk/getdata.php?auto=0&cb3=2");
         }
         //--------
      }
      else  if (ch==62) // 62 la ma thap phan cua >
      {
         
         _httpread();
         answer=sendATcommand(HTTPTERM,OK,250);
      }
      /* //********
t: nhietdo
h: do am
x: cam bien1 cb1
y: cb2
n: nguon
g: gio
p: phut
ngay: d
thang: m

//*******/
      /*
   else if(ch==116) //  la ma cua t
   {
      fprintf(PORT1SIM,"&nhietdo=");
   }
   else if(ch==104) // h
   {
      fprintf(PORT1SIM,"&doam=");
   }
   else if(ch==120) // x
   {
      fprintf(PORT1SIM,"&cb1=");
   }
   else if(ch==121) // y
   {
      fprintf(PORT1SIM,"&cb2=");
   }
   else if(ch==110) //n
   {
      fprintf(PORT1SIM,"&nguon=");
   }
   else if(ch==103) //g
   {
      fprintf(PORT1SIM,"&gio=");
   }
   else if(ch==112) //p
   {
      fprintf(PORT1SIM,"&phut=");
   }
   else if(ch==100) //d
   {
      fprintf(PORT1SIM,"&ngay=");
   }
   else if(ch==109) //m
   {
      fprintf(PORT1SIM,"&thang=");
   }
   
   */
      else
      {
         fprintf(PORT1SIM,"%c", ch);
      }
   } while(ch != EOF);

   return ch;
}

void Printdl(char *fileName)
{
   FILE stream;

   if(fatopen(fileName, "r", &stream) != GOODEC)
   {
      sendhttp(1); // khong mo duoc file thi dung ham gui loi xong
      //     printf("\r\nLoi mo file  '%s'",fileName);  
      return;
   }

   // printf("\r\n");



   tachdl(&stream); 

   fatclose(&stream);
   if(fatclose(&stream) != GOODEC)
   {
      //  printf("CloseFileError '%s'",fileName);
      return;
   }
}

/* 
//Interupt cu dung de chinh thoi gian
#INT_RDA
void serial_isr()
{
char c;

c=getc();
putchar(c);                                                              
Buffer_time[CharRec] = c;
if (c==13)                    
{
ComRec=1; 
ProcesaTime();
}
else
CharRec = (CharRec+1) % sizeof(Buffer_time);
} 
//Interupt cu dung de chinh thoi gian 
*/

void ghitxt()
{


   sprintf(bufferTP,"<%s>",querystring);
   if(kiemtraread == 1)
   {
      sprintf(gfilename,"/log%02d-%02d.txt",day,month);
   }
   else
   {
      sprintf(gfilename,"/loi%02d-%02d.txt",day,month);
   } 

   //strcpy(gfilename,"/logtesttest.txt");
   MakeFile(gfilename); //tao file moi neu chua co
   AppendFile(gfilename,bufferTP);//Ghi du lieu vao file log.txt


}

int1 checkthesd()
{
   //kiem tra the nho n lan, loi: return 0, thanh cong return 1.
   unsigned int solan =0;
   int1 i;   // pointer to the buffer
   // delay_ms(1000); // cho on dinh de kiem tra the nho

   //check the nho 
   do
   {
      i = fat_init();

      if (i)                                                          
      {            
         fprintf(UART2,"No SD\n");  
      //   output_high(PIN_A1);
         
         
         //  return; 
      }                                                                               
      else              
      {                                                                   
         fprintf(UART2,"SD OK");   
       //  output_low(PIN_A1);
         return 1;
      } 
      solan++;
      delay_ms(1000); // 
   }while (solan<5); //i =1: loi the nho, i = 0 : co the nho

   return 0;

   //check the nho **********
}

void batsim900()
{

   delay_ms(5000);//Cho Module sim khoi dong,tam 15000ms

   delay_ms(10000);




   fprintf(UART2,"Sim808 On\n"); 
   fprintf(PORT1SIM,"ATE0"); //ATE1: bat ATE0:tat phan hoi module sim
   fputc(13,PORT1SIM);
   fputc(10,PORT1SIM);

   sendATcommand(AT,OK,50);

}



void batgprs()
{
   sendATcommand(SETAPN,OK,250);

   //Ham bat gprs v2 ********
   do 
   {
      demgprs++;
      answer = 0;
      answer = sendATcommand(BATGPRS,OK,200);

      if(demgprs >10) // thu lai 10 lan khong duoc thi bo qua
      {
         break;
      }
      if (answer == 0)
      {


         sendATcommand(TATGPRS,OK,150);
        // output_high(PIN_A1);
         delay_ms(500);
        // output_low(PIN_A1);
         delay_ms(500);
         

      }
   }

   while(answer == 0);

}

void docadc()
{ set_adc_channel(0);
   power1 = read_adc();
   power1=(((power1*5)/20000)*119);// (x*20000)/120000.............5
                                    //adc  ........................1024  => x=(adc*5*120000)/1024*20000 

  // delta_power = abs(power1 - pre_power);
  // pre_power = power1;

}

void docadc1()// doc tin hieu analog cam bien
{ set_adc_channel(0);
power2 = read_adc();
  // dosau_new = measurement() / 100; 
 // Result = ((power1-216.0)*100.0)/(1024.0-216.0);  
 // Result = (power1*10.0)/(1024.0); 
}

void docadc2()// doc nguon matrroi
{ set_adc_channel(2);
power3 = read_adc();
   power3=(((power3*5)/20000)*119);
   }

void docdht22()
{
   //-----doc nhiet do - do am--------/
   //  delay_ms(1000); 
   Time_out = 0;
   start_signal();
   if(check_response()){                    // If there is response from sensor
      RH_byte1 = Read_Data();                 // read RH byte1
      RH_byte2 = Read_Data();                 // read RH byte2
      T_byte1 = Read_Data();                  // read T byte1
      T_byte2 = Read_Data();                  // read T byte2
      CheckSum = Read_Data();                 // read checksum
      
      if(Time_out){                           // If reading takes long time
         
         //    fprintf(UART2,"\r\n\nTime out1!\r\n\n"); 
      }
      else
      {
         if(CheckSum == ((RH_byte1 + RH_byte2 + T_byte1 + T_byte2) & 0xFF))
         {
            RH = RH_byte1;
            RH = (RH << 8) | RH_byte2;// lay nhiet do
            
            Temp = T_byte1;
            Temp = (Temp << 8) | T_byte2;// lay do am
            
            if (Temp > 0x8000)
            {
               
               Temp = Temp & 0x7FFF; 
            }
            
            
            //    printf( "%4lu",Temp);   
            //  printf( "%4lu",RH);
            
            
            
         }
         else {
            
            // printf("\r\n\nChecksum Error!\r\n\n");
         }
      }
   }
   else {

      //   printf("\r\n\nNo response\r\n\n");
      //    printf("\r\n\nfrom the sensor\r\n\n");
   }

}


void caidatngatvatimer() 
{

   //  Cai dat ngat va timer

   enable_interrupts(INT_RDA); //cho phep ngat uart
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_4);   // Timer1 configuration
   //enable_interrupts(INT_EXT2_H2L);                 // Enable external interrupt
   ext_int_edge(H_TO_L);
   enable_interrupts(INT_EXT);//kich hoat ngat ngoai

   enable_interrupts(INT_TIMER1);

   setup_adc(ADC_CLOCK_DIV_8);            // Set ADC conversion time to 8Tosc
   setup_adc_ports(AN0_TO_AN2);                  // Configure AN0 as analog input SETUP_ADC_PORTS(AN0_TO_AN1);
  // set_adc_channel(0);          
   //**
   enable_interrupts(GLOBAL);//cho phep ngat toan cuc

}






#define ATCCLK "AT+CCLK?"
#define CCLK "CCLK"
#define CNTPCID "AT+CNTPCID=1"
#define NTPSERVER "AT+CNTP=\"pool.ntp.org\",28" //00: GMT+0, 28:GMT+7
#define CNTP "AT+CNTP"
#define CNTPRESULT "+CNTP: 1"

void sync_ntp(int1 i) // i=0: luon cap nhat gio theo ntp, i = 1: tham chieu voi ds1307
{
   int nam, thang, ngay, gio, phut, giay;
   int1 ktx = 0;
   //sendATcommand(ATCCLK,CCLK,1000);
 //  sendATcommand(BATGPRS,O,350);// da bo phan nay ra khoi tiet kiem nang luong
   if(setcapnhat<10)
  {answer= sendATcommand(BATGPRS,O,350);
  }
 
   sendATcommand(CNTPCID,OK,1000);

   sendATcommand(NTPSERVER,OK,1000);


   ktx = sendATcommand(CNTP,CNTPRESULT,1000);
   if (ktx == 1)
   {
      sendATcommand(ATCCLK,CCLK,1000);
      //int i =0;
      //for (i=0;i<27;i++)
      //{
      //fprintf(UART2,"ky tu %u:%c\n",i,buffer[i]);
      //}
      nam = (buffer[10] -48)*10+ (buffer[11] - 48);
      thang = (buffer[13] -48)*10+ (buffer[14] - 48);
      ngay = (buffer[16] -48)*10+ (buffer[17] - 48);
      gio = (buffer[19] -48)*10+ (buffer[20] - 48);
      phut =(buffer[22] -48)*10+ (buffer[23] - 48);
      giay = (buffer[25] -48)*10+ (buffer[26] - 48);
      //fprintf(UART2,"%u/%u/%u - %u:%u:%u\n",ngay,thang,nam,gio,phut,giay);
    //  ds1307_get_date(day,month,yr,dow);  
    //  ds1307_get_time(hrs,min,sec); // lay thoi gian ds1307 de kiem tra co lech mui gio khong ?
       Update_Current_Date_Time(); 
         if ((i == 0)||(ds_H-hrs>2)||(hrs-ds_H>2))
      {
  // fprintf(UART2,"da cap nhat gio1");
         //ds1307_set_date_time(ngay,thang,nam,2,gio,phut,giay);
          Set_Time_Date(ngay,thang,nam,gio,phut,giay);
      }
      if ((gio - hrs == 7)||(hrs - gio == 17)||(gio - hrs == 14)||(hrs - gio == 10)) 
      {
         rcount5++;
         
      }
      else
      {  //fprintf(UART2,"da cap nhat gio2");
       //  ds1307_set_date_time(ngay,thang,nam,2,gio,phut,giay);
         Set_Time_Date(ngay,thang,nam,gio,phut,giay);//13,30,20); //set time and date on the RTC module

      }
   }
}
#define SETMUIGIO "AT+CCLK=\"17/04/01,12:12:12+07\""

void caimuigio()
{
   //fprintf(UART2,SETMUIGIO);
   sendATcommand(SETMUIGIO,OK,250);

}
/*
void testEEProm()
{
   unsigned char chuoia[20],chuoib[20];
   sprintf(chuoia,"Data EEPROM");
   eeprom_write_string(0x04,chuoia);
   eeprom_read_string(0x04,chuoib,11);
   fprintf(UART2,"chuoi b:%s\n",chuoib);
}*/
#define CSQ "AT+CSQ" 
unsigned int kiemtrasong()
{
   int1 kty = 0;
   unsigned int chatluong = 0;
   kty = sendATcommand(CSQ,OK,250);
  // Pulse=0;
   if (kty == 1)
   {
      //for (i=0;i<6;i++)
      //{
      //fprintf(UART2,"song:%i: %c:",i,buffer[i]);
      //}
      chatluong = (buffer[8] - 48)*10 + buffer[9] - 48;
   //   fprintf(UART2,"8:%c 9: %c : Chatluong:%u",buffer[8],buffer[9]chatluong);
      return chatluong;
   }

}

signed int tachdl2(FILE* stream)
{
   char docurl[100]="";
   char chuoitam[100] = "";
   unsigned int dodaichuoi;
   signed int ch; // character read in

   // keep on printf any characters read in as long as we don't run into an end of file or a media error
   do
   {
      ch = fatgetc(stream);
      if ((ch > 32) &&  (ch < 127))// 32 - 126 la nhung ky tu in duoc// 32 - 126 la nhung ky tu in duoc
      {
         sprintf(chuoitam,"%s%c",docurl,ch);
         sprintf(docurl,"%s",chuoitam);
         //     fprintf(UART2,"%c\n", ch);
         // fprintf(UART2,"URL:--%s--\n",docurl);
      }
   } while(ch != EOF);
   dodaichuoi = strlen(docurl);
   // fprintf(UART2,"Ghi %s - do dai:%u vao bo nho\n",docurl,dodaichuoi);
   eeprom_write_string(0x04,docurl);
   write_eeprom(0xA0,dodaichuoi);
}

void PrintURL(char *fileName)
{
   FILE stream;

   if(fatopen(fileName, "r", &stream) != GOODEC)
   {
      fprintf(UART2,"\r\nKhong ton tai '%s'- lay URL cu",fileName);  
      return;
   }

   //printf("\r\n");

   //if(startFromEnd)                                   
   //fatseek(&stream, 80, SEEK_END);

   //fatprintf(&stream); 
   tachdl2(&stream); 
   // fatclose(&stream);



   fatclose(&stream);


   if(fatclose(&stream) != GOODEC)
   {
      //fprintf(UART2,"Error closing file '%s'",fileName);
      return;
   }
}
signed int tachdl3(FILE* stream)
{
   char docurl[100]="";
   char chuoitam[100] = "";
   unsigned int dodaichuoi;
   signed int ch; // character read in

   // keep on printf any characters read in as long as we don't run into an end of file or a media error
   do
   {
      ch = fatgetc(stream);
      if ((ch > 32) &&  (ch < 127))// 32 - 126 la nhung ky tu in duoc// 32 - 126 la nhung ky tu in duoc
      {
         sprintf(chuoitam,"%s%c",docurl,ch);
         sprintf(docurl,"%s",chuoitam);
         //     fprintf(UART2,"%c\n", ch);
         // fprintf(UART2,"URL:--%s--\n",docurl);
      }
   } while(ch != EOF);
   dodaichuoi = strlen(docurl);
   // fprintf(UART2,"Ghi %s - do dai:%u vao bo nho\n",docurl,dodaichuoi);
   eeprom_write_string(0x74,docurl);
   write_eeprom(0xB0,dodaichuoi);
}

void PrintURL1(char *fileName)
{
   FILE stream;

   if(fatopen(fileName, "r", &stream) != GOODEC)
   {
      fprintf(UART2,"\r\nKhong ton tai '%s'- lay phone cu",fileName);  
      return;
   }

   //printf("\r\n");

   //if(startFromEnd)                                   
   //fatseek(&stream, 80, SEEK_END);

   //fatprintf(&stream); 
   tachdl3(&stream); 
   // fatclose(&stream);



   fatclose(&stream);


   if(fatclose(&stream) != GOODEC)
   {
      //fprintf(UART2,"Error closing file '%s'",fileName);
      return;
   }
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char doc;
char docduoc[20];

int16 MSB_B3, SevenBit_B3, MSB_B2, SevenBit_B2, ByteExponent;
int32 bit_Mantis;
//float ketquadoduoc;
float thuaso1, thuaso2, thuaso3;
float sodau;
signed int16 somu;

int read_rs485(unsigned char* str,unsigned int len)
{
   int kt485 = 0; // kiem tra = 0 : loi do timeout , kiem tra = 1 : OK, kiem tra = 2: loi do break
   
   rcount2=0;
   unsigned int i;
   //= char doc;
   while((!kbhit(COM2))&&(rcount2<100))
   {
      // delay_us(10);
   }
   
   
   if (rcount2<100)
   {
      for(i=0;i<len;i++)
      {
        if (rcount2>100) 
        {
        kt485 = 2;
        break;
        }
         
         str[i]=fgetc(COM2) ;
         
         kt485 = 1;
      }
      
   //   fprintf(UART2,"Doc xong!");        
      str[len]=0;
      
       
   }
   
  

//fprintf(UART2,"kt:%u\n",kt);
return kt485;

}



void send_ham1()
{  char a[4] = {0x01,0x30,0x34,0x00};
   int ll;
for(ll=0;ll<4;ll++)
{fprintf(COM2,"%c",a[ll]);

}

}

void send_ham2()
{  char a[5] = {0x01,0x1E,0x50,0x9C,0x29};
   int ll;
for(ll=0;ll<5;ll++)
{fprintf(COM2,"%c",a[ll]);
}
}
void send_ham3()
{
   //01 1e 51 5c e8
   char a[5] = {0x01,0x1E,0x51,0x5C,0xE8};
   int ll;
   for(ll=0;ll<5;ll++)
{  fprintf(COM2,"%c",a[ll]);

}
}

void send_ham4()
{
   //01 45 d3 c1 
     char a[4] = {0x01,0x45,0xd3,0xC1};
   int ll;
for(ll=0;ll<4;ll++)
{fprintf(COM2,"%c",a[ll]);


}

}
void send_ham5()
{ 
   //01 49 01 50 d6 
   char a[5] = {0x01,0x49,0x01,0x50,0xD6};
   int ll;
for(ll=0;ll<5;ll++)
{fprintf(COM2,"%c",a[ll]);
}

}
///////////////////////////////////////////////////////////////////////////////////


void send_hamHPT604()
{ char a[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
int ll;
for(ll=0;ll<8;ll++)
{fprintf(COM2,"%c",a[ll]);
}
}
/////////////////////////GLT500//////////////////////////////////////////
void send_hamGLT500()

{ char a[8] = {0x01,0x03,0x00,0x02,0x00,0x03,0xA4,0x0B};
int ll;
for(ll=0;ll<8;ll++)
{fprintf(COM2,"%c",a[ll]);
}
}

void send_hamL703()
{ char a[8] = {0x01,0x03,0x00,0x04,0x00,0x01,0xC5,0xCB};
int ll;
for(ll=0;ll<8;ll++)
{fprintf(COM2,"%c",a[ll]);
}

}





int  BCD_2_DEC(int to_convert)
{
   return (to_convert >> 4) * 10 + (to_convert & 0x0F); 
}
int DEC_2_BCD (int to_convert)
{
   return ((to_convert / 10) << 4) + (to_convert % 10);
}

void Set_Time_Date(BYTE day, BYTE mth, BYTE year, BYTE hr, BYTE min, BYTE sec)
{
   i2c_start();       
   i2c_write(0xD0); 
   i2c_write(0);  
   i2c_write(DEC_2_BCD(sec)); //update sec
   i2c_write(DEC_2_BCD(min)); //update min
   i2c_write(DEC_2_BCD(hr)); //update hour
   i2c_write(1); //ignore updating day
   i2c_write(DEC_2_BCD(day)); //update date
   i2c_write(DEC_2_BCD(mth)); //update month
   i2c_write(DEC_2_BCD(year)); //update year
   i2c_stop();
}

void Update_Current_Date_Time()
{
   //START to Read
   i2c_start();       
   i2c_write(0xD0); 
   i2c_write(0);    
   i2c_stop(); 
   
  //READ
   i2c_start();
   i2c_write(0xD1);                              // Initialize data read
   sec = BCD_2_DEC(i2c_read(1));    
   min = BCD_2_DEC(i2c_read(1));   // Read sec from register 
   hrs = BCD_2_DEC(i2c_read(1));  
   i2c_read(1);
   day = BCD_2_DEC(i2c_read(1));  
   month = BCD_2_DEC(i2c_read(1));  
   yr = BCD_2_DEC(i2c_read(1));  
   i2c_stop(); 
    
  //END Reading  
    i2c_start();
    i2c_write(0xD1);                              // Initialize data read
    i2c_read(1);    
    i2c_stop(); 

}





float _ketquars4851()
{
int kiemtra;
   float ketqua = 0;
   send_ham5();
  kiemtra = read_rs485(docduoc,9);
  
  if (kiemtra == 1) 
  {
   MSB_B3 = docduoc[2] >> 7;
   if (MSB_B3 == 1)
   {
      sodau = -1;
   }
   else
   {
      sodau = 1;
      
   }
   SevenBit_B3 = docduoc[2] & 0x7F;
   MSB_B2 = docduoc[3] >> 7;
   SevenBit_B2 = docduoc[3] & 0x7F;
   ByteExponent = ((SevenBit_B3) <<1) + MSB_B2;
   bit_Mantis = SevenBit_B2*0x10000 + docduoc[4]*0x100 + docduoc[5];
   thuaso2 = 1+ bit_Mantis/8388608.00;
   somu = ByteExponent - 0x7F;
   thuaso3 = pow(2,somu);
   ketqua = sodau * thuaso2 * thuaso3 ;
  }
  else 
  {
  ketqua = 0;
  }
   return ketqua;
}





float _ketquars485()
{
repead:
int kiemtra=0;
   float ketqua = 0, ketqua1=0,ketqua2=0;
  // tuantu++;
   if(cambien_deep==1)
   {
  send_hamL703();
   }
   if(cambien_deep==2)
   {
 send_hamHPT604();
   }
   if(cambien_deep==3)
   {
   send_hamGLT500();
   }
//!    if(cambien_deep==4)
//!   {
//!      send_ham1();
//!      delay_ms(1500);    
//!      send_ham2();
//!      delay_ms(1500);     
//!      send_ham3();
//!      delay_ms(1500);
//!      send_ham4();
//!      delay_ms(1200);
//!      h1=_ketquars4851();
//!    goto xittt;
//!   }
   if(cambien_deep==0)
   {
   h1=0;
   goto xittt;
   }
  kiemtra = read_rs485(docduoc,7);
  
  if (kiemtra == 1) 
  {kiemtra=0;

    ketqua1 =((docduoc[3] >> 4) * 4096) + ((docduoc[3] & 0x0F)*256);
  // ketqua1 =((docduoc[3] >> 4) * 4096) + (docduoc[4] & 0x0F);
   ketqua2= ((docduoc[4] >> 4) * 16) + (docduoc[4] & 0x0F);
  // ketqua2= ((docduoc[4] >> 4) * 16) + (docduoc[4] & 0x0F);
  ketqua=ketqua1+ketqua2;
  if(ketqua>1200)
  {ketqua=0;
  }
  }
  else 
  {
  ketqua=0;
  }
  h1=ketqua;
  fprintf(UART2,"ketqua=%2.1f\n",h1);
  xittt:
   return ketqua;
}






#include <stdio.h> 
#ZERO_RAM 

#define HX711_DO  PIN_B4
#define HX711_CLK PIN_B5
int32 measurement(void); 
int32 measurement(void)
{
   unsigned int32 Count; 
   unsigned int8 i,A_1,A_2,A_3; 
   output_bit( HX711_DO, 1); 
   output_bit( HX711_CLK, 0); 
   Count=0; 
   while(input(HX711_DO)); 
   for (i=0;i<24;i++){// gain 128 
   output_bit( HX711_CLK, 1); 
   Count=Count<<1; 
   output_bit( HX711_CLK, 0); 
   if(input(HX711_DO)) Count++; 
} 
   output_bit( HX711_CLK, 1); 
   Count=Count^0x800000; 
   
   output_bit( HX711_CLK, 0); 
//************************ 
   A_1=make8(Count, 0); 
  A_2=make8(Count, 1); 
   A_3=make8(Count, 2); 
 A_2=(A_2 & 0b11111000); 
   Count= make16( A_3, A_2); 
   
   return(Count); 

} 

void SIM808_SendMessenger(char *PhoneNumber,char *TextToSend)
{
 
    fprintf(PORT1SIM,"AT+CMGS=\"%s\"\n\r",PhoneNumber);
     fprintf(UART2,"%s",buffer);
    delay_ms(1000);
    fprintf(PORT1SIM,TextToSend);
     fprintf(UART2,"%s",buffer);
    delay_ms(5);
    fputc(0x1A,PORT1SIM);// Ctr+Z
    // fputc(0x1A,PORT1SIM);// Ctr+Z
    //return 1;
    delay_ms(100);
}


void main(void)
{   port_b_pullups(TRUE);
   output_low(PIN_B1);
   output_low(PIN_B2);
   output_low(PIN_B4);
   output_low(PIN_B5);
   output_low(PIN_A2);
   output_low(PIN_D3);
   //-----------------------------------  
   output_low(PIN_D6);// 2 CHAN GIAO TIEP VOI ATMEGA DIEU KHIEN BAO DONG
   output_low(PIN_D7);
   //-------------------------------
   output_low(PIN_A4);
   delay_ms(5000);
   output_high(PIN_A4);
   float dosau_new1 = 0;
   caidatngatvatimer();
   int1 kiemtraSD = 0; 
   kiemtraSD =  checkthesd();
   // testsdhc();
   if (kiemtraSD ==1)
   {
      PrintURL("/url.txt");
   }
   dodaiURL=read_eeprom(0xA0);
   eeprom_read_string(0x04,URLeeprom,dodaiURL);
   // fprintf(UART2,"URLeeprom: %u\n",dodaiURL);
  fprintf(UART2,"URLeeprom: *%s*\n",URLeeprom);
     if (kiemtraSD ==1)
   {
      PrintURL1("/phone.txt");
   } 
    dodaiphone=read_eeprom(0xB0);
   eeprom_read_string(0x74,phoneeeprom,dodaiphone);
   // fprintf(UART2,"URLeeprom: %u\n",dodaiURL);
   fprintf(UART2,"phone_eeprom: *%s*\n",phoneeeprom);  
     //tach cac so dien thoai ra//
      ptr = strtok(phoneeeprom, ";");
         gg=0;
         while(ptr!=0) 
         {
            tachsdt[gg] = (ptr);
            ptr = strtok(0, ";");
            gg++;
         }
batsim900();
sendATcommand(atw,OK,200);
delay_ms(1000);
sendATcommand(atcs,OK,200);
delay_ms(1000);
sendATcommand(atcm,OK,200);
delay_ms(1000); // cau hinh nhan tin nhan dang text(mac dinh la PDU)
sendATcommand(atcn,OK,200);
delay_ms(1000); // cau hinh khi sim co tin nhan moi
sendATcommand(xoatn,OK,200);
delay_ms(1000);
   batgprs();
 
   rcount1 =1;
   setcapnhat =1; //setcapnhat =30 //60 : mac dinh 30 // 60 phut truyen du lieu 1 lan, neu module sim khong phan hoi
  

   ds_H=hrs;
   sync_ntp(0);// ko tham chieu
   docadc();
   new_sms=0;
   thutin="";
   sdt="";
   Update_Current_Date_Time(); 
   setup_wdt(WDT_128S);
   _ketquars485();///////////////////////
   dosau_new1=h1;
   h2=h1;
   delay_ms(500);
 if(dosau_new1==0)
 { _ketquars485();///////////////////////
   dosau_new1=h1;
 }
   Pulse=0;
   rain =0;
   pre_Pulse=0;
   pre_rain=0;
   while(1)
   {
      restart_wdt();
     if(new_sms==1)
     {
//!         new_sms=0;               
//!         sendATcommand(kiemtratn,OK,200);  
//!         sdt="0";
//!         strncpy(sdt1,buffer+11,9);
//!         strcat(sdt,sdt1);
//!         ss_sdt0=strcmp(sdt,tachsdt[0]);
//!         ss_sdt1=strcmp(sdt,tachsdt[1]); 
//!         ss_sdt2=strcmp(sdt,tachsdt[2]);
//!         ss_sdt3=strcmp(sdt,tachsdt[3]);
//!         ss_sdt4=strcmp(sdt,tachsdt[4]);

new_sms=0;
         sdt="0";     
         sendATcommand(kiemtratn,OK,200);              
         sdt5=buffer;
        //strncpy(sdt1,buffer+11,9);
        
           ptr = strtok(sdt5, "4");
         //  ptr=ptr+2;
         gg=0;
         while(ptr!=0) 
         {
            tachsdt1[gg] = (ptr);
            ptr = strtok(0, ",");
            gg++;
         }
    //   strncat(sdt,tachsdt1[1],
         strncpy(sdt6,tachsdt1[1],9);
         strcat(sdt,sdt6);
         ss_sdt0=strcmp(sdt,tachsdt[0]);
         ss_sdt1=strcmp(sdt,tachsdt[1]); 
         ss_sdt2=strcmp(sdt,tachsdt[2]);
         ss_sdt3=strcmp(sdt,tachsdt[3]);
         ss_sdt4=strcmp(sdt,tachsdt[4]);
        //fprintf(UART2,"so dien thoai nhan tin=%s_%u_%u_%u_%u_%u\n",sdt,ss_sdt0,ss_sdt1,ss_sdt2,ss_sdt3,ss_sdt4);
       // fprintf(UART2,"sdt=%s_\n",sdt);
         if(ss_sdt0==0||ss_sdt1==0||ss_sdt2==0||ss_sdt3==0||ss_sdt4==0)
         {
         toInt = buff+1; // xoa ky tu dau tien cua chuoi buff                    
         delay_ms(300);
         ptr = strtok(toInt, ";");
         gg=0;
         while(ptr!=0) 
         {
            tachdata[gg] = atof(ptr);
            ptr = strtok(0, ";");
            gg++;
         }
          relay1 = tachdata[1];
          relay2=tachdata[2];
          baodong=tachdata[3];
          reset=tachdata[4];
          countcmra=0;
          relay3=0;
          sprintf(thutin,"0;%u;%u;%u;%u",relay1,relay2,baodong,reset);
          fprintf(UART2,"thu tin= %s\n",thutin);

         if(setcapnhat>=5)
         {     
          batgprs();
         }
          bit_binhthuong=1;
          }

         new_sms=0;
          }
         else
        {thutin="";}
          if(reset==1)
          {while(1);
          }
           if(relay1==1)
           {output_high(PIN_B2);        
           }
          else 
          {
            output_low(PIN_B2);
          }
          
           if(relay2==1)// CAMERA
          {output_high(PIN_B1);  
             biencamera=1;
          
          }
          else 
          {output_low(PIN_B1);
          }
          
          if(baodong>=1)// CAP NHAT GIO
          {baodong=0;
          sendATcommand(BATGPRS,O,350);
          delay_ms(1000);
          sync_ntp(0);
          }
     
    if((min%setcapnhat==timeupdate)&&(setbit==1)&& (setbit1==0)&&(rcount4!=0))
  //  if((rcount4==5)&&(setbit==1))
     { bit_bat++;// tang nhieu qua
       setbit1=1;
     if((power1>10.6)||((bit_bat>=2)&&(power1<=10.6)))
     {bit_bat=0;
      rcount4=0;   
      batgprs();      
      goto hello;  
      }
     }
      kiemtraread=0;    
     Update_Current_Date_Time();
     ds_H=hrs;

    if(rcount3>=2)
    {     fprintf(UART2,"r123=%lu",rain); 
    
    rcount3=0;
         _ketquars485();///////////////////////
         dosau_new1=h1;
         h2=h1;
         delay_ms(500);
      if(dosau_new1==0)
         { _ketquars485();///////////////////////
           dosau_new1=h1;
         }
        if((dosau_new1>=muccanhbao1)||(dosau_new1>=muccanhbao2)||(dosau_new1>=muccanhbao3)&&(kt2==1))
        {      
         if(setcapnhat>=5)
         {     
          batgprs();
         }
          bit_binhthuong=1;
        } 
    }  
    
      if(countcmra>=28 && biencamera>=1)// kiem tra 30 phut tat camera
   {
  fprintf(UART2,"da tu tat Camera");
   output_low(PIN_B1);
   biencamera=0;
   countcmra=0;
   relay2=0;
  
   
   }
    
    
    
     if ((min % setcapnhat == 0)&&(rcount1!=0)&& (power1>10.6))
      { bit_binhthuong=1;
       rcount1=0;
      }
     if ((min % setcapnhat == 0)&&(rcount1!=0)&& (power1<=10.6))
    { bit_binhthuong=0;
      bit_kt++;
      rcount1=0;
    }      
 if ((bit_binhthuong==1)||(bit_kt>=2))
      {  setbit=1;// de ban dau sau khi truyen lan dau thi moi chay vao chuong trinh bat sim, tranh truong hop moi khoi dong cung vao bat sim
         setbit1=0;// giúp khong vao bat sim 2 lan trong 1 lan truyen
     // fprintf(UART2,"da vao chuong trinh truyen  bit_binhthuong= %u bit_kt=%u\n",bit_binhthuong,bit_kt);
      bit_binhthuong=0;
      bit_kt=0;
      kiemtraread=0;
      chatluong = kiemtrasong();
      docadc();  
      docdht22(); 
       Temp=Temp/10;
       RH=RH/10;
       if(setcapnhat==1)
       {Pulse=0;
   rain =0;
   pre_Pulse=0;
   pre_rain=0;
       }
       sprintf(querystring,"ngay=%u&thang=%u&gio=%u&phut=%u&nguon=%.1f&nhietdo=%lu&doam=%lu&Dosau=%2.1f&mua=%lu&songdidong=%u&barier=%u&den=%u&camera=%u&loa=%u&dienthoai=%s&noidungtinnhan=%s",day,month,hrs,min,power1,Temp,RH,dosau_new1,rain,chatluong,relay1,relay3,relay2,baodong,sdt1,thutin);
       sendhttp(0);
       new_sms=0;
      while((read_http <= 0)||(read_http == 208))
      { fprintf(UART2,"truyenlai \n");
       sendATcommand(TATGPRS,OK,150);
       delay_ms(300);
       sendATcommand(SETAPN,OK,250);
       sendATcommand(BATGPRS,O,350);
       sendhttp(0);
      }       
         rcount1 =0;
         rcount=0;
         truyenlandau =1; //1: khong gui du lieu ke tu vong lap tiep theo       
      }
if((hrs == 0)&&(min ==1))
      {  rain = 0;
      setcapnhat=1;
      }
       if(kiemtraread == 1)
         {
         toInt = buff+1; // xoa ky tu dau tien cua chuoi buff
         delay_ms(700);      
         ptr = strtok(toInt, ";");        
         gg=0;
         while(ptr!=0) 
         {
            tachdata[gg] = atof(ptr);
            ptr = strtok(0, ";");
            gg++;
         }
            setcapnhat = tachdata[1];
            muccanhbao1=tachdata[2];
            muccanhbao2=tachdata[3];
            muccanhbao3=tachdata[4];
            cambien_deep=tachdata[6];
            thutin="";
            sdt1="";
            rcount3=0;
  if(power1<=10.6)
     {setcapnhat=60;
     }

 if(setcapnhat>=5)
  { 
   kt2=1;// lora se gui lai du lieu sau 1 lan truyen    
   sendATcommand(TATGPRS,OK,150);
   delay_ms(300);
   new_sms=0;
   timeupdate=setcapnhat-1;
   
  }

hello:
 _ketquars485();///////////////////////
 dosau_new1=h1;
 h2=h1;
 delay_ms(500);
 if(dosau_new1==0)
 { _ketquars485();///////////////////////
   dosau_new1=h1;
 }



}  
 

}

}



