// yeu cau nguon cap cho sim808 la 12 V_v5 of sim 2h neu nguoi duoi 10.6V
//V6 testWDT neu chip bi treo qu? 2 ph?t (128s)
// THAY DOI SO VOI V3 CHO SDA B2->A3.
#include <18F4550.h>
#device PASS_STRINGS = IN_RAM
#DEVICE ADC = 10
#fuses HSPLL, MCLR, WDT, NOPROTECT, NOLVP, NODEBUG, USBDIV, PLL5, CPUDIV1, VREGEN
#use delay(clock = 48000000)
#use rs232(baud = 9600, parity = N, xmit = PIN_D4, rcv = PIN_D5, bits = 8, stream = COM2) // thiet lap UART mem chuan rs485 bo moi nhat giao tiep voi RS485
#use rs232(baud = 4800, parity = N, xmit = PIN_D4, rcv = PIN_D5, bits = 8, stream = COM3) // thiet lap UART mem chuan rs485 bo moi nhat giao tiep voi RS485
#use rs232(baud = 9600, parity = N, xmit = PIN_C0, rcv = PIN_D2, bits = 8, stream = UART2)
#use rs232(baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stream = PORT1SIM)
#INCLUDE <math.h>
#include ".\_inc\types.h"
#include <stdlib.h> // for atoi32
#include "mmcsd.c"
//FAT library.
#include <fat.c>
#define COMMAND_SIZE 10
#define NUM_COMMANDS 11

float power1;
unsigned int16 rcount = 0, rcount2 = 0, rcount6 = 0, rcount_TOUT = 0;
int rcount1 = 0, rcount4 = 0, rcount3 = 0, rcount5 = 0;
int error_get = 0, error_get1 = 0;
int kt_time = 0, num = 0, num_no2 = 0, num_so2 = 0, num_co = 0;
char diachi1[8] = {0x04, 0x03, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01};
char diachi2[8] = {0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03};
// *********** Bien thoi gian *********** //
BYTE sec;
BYTE min;
BYTE hrs;
BYTE day;
BYTE month;
BYTE yr;

void AppendFile(char *fileName, char *appendString);
void MakeFile(char *fileName);
//float pre_power = 0,delta_power=0;

unsigned int16 Temp, RH;
short Time_out;
unsigned int8 T_byte1, T_byte2, RH_byte1, RH_byte2, CheckSum;

char URLeeprom[55];
unsigned int dodaiURL = 0;
//!#define buffer_size 100 //Buffer size you can adjust this size
#define buff2_size 60

//#DEVICE HIGH_INTS=TRUE

// ********** Bien doc cam bien ********** //
float PA = 0, PM25 = 0, PM10 = 0, LUX = 0;
float Temp_485 = 0, Humi_485 = 0, Noise = 0;
float Tb_no2 = 0, Tb_so2 = 0, Tb_co = 0;
float buf_so2 = 0, buf_no2 = 0, buf_co = 0;
int Wind = 0, check_hour = 0;
char No2[10], So2[10], Co[15];
//cac bien
char c, buff[buff2_size], *toInt, *ptr;//, buffer[buffer_size] // SAVE Response
int xbuff = 0x00, xbuff2, i, kt = 0, gg;
int count_check = 0, count_check1 = 0, Check_Pin = 0, check_rc3 = 0;
float tachdata[20];
int tachdata_time[6];
int setcapnhat = 1, bit_binhthuong = 0;
int1 kiemtraread = 0, Wait = 0;
char querystring[220];
//Chuong trinh ngat UART
#define RTC_SDA PIN_A3
#define RTC_SCL PIN_B3
#use i2c(master, sda = RTC_SDA, scl = RTC_SCL, fast = 450000)
int DEC_2_BCD(int to_convert);
void Set_Time_Date(BYTE day, BYTE mth, BYTE year, BYTE hr, BYTE min, BYTE sec);
void Update_Current_Date_Time();

// *********** Ngat Uart *********** //
#INT_RDA
void RDA_isr()
{
    c = fgetc(PORT1SIM);
    if (c == '*')
    { //kt2=1;
        kt = 1;
        xbuff2 = 0;
        for (i = 0; i < buff2_size; i++)
        {
            buff[i] = 0x00;
        }
    }
    else if (c == '#')
    {
        kt = 0;
        kt_time = 1;
        break;
    }
    if (kt == 1)
    {
        buff[xbuff2] = c;
        xbuff2++;
    }
    xbuff++;
}

// *********** HAM eeprom *********** //
void eeprom_write_string(unsigned int8 addr, unsigned char *str)
{
    while (*str)
    {
        write_eeprom(addr, *str);
        addr++;
        str++;
    }
}

void eeprom_read_string(unsigned int8 addr, unsigned char *str, unsigned int8 len)
{
    unsigned int8 i, j, c;
    j = 0;
    for (i = 0; i < len; i++)
    {
        c = read_eeprom(addr + i);
        if ((c > 32) && (c < 127))
        {
            str[j] = c;
            j++;
        }
    }
    str[j] = 0;
}

// *********** Timer *********** //
#INT_TIMER1 // Timer1 interrupt ISR

void timer1_isr()
{
    set_timer1(5536); //1/(48M/16)x60000
    rcount++;
    rcount2++;
    rcount6++;
    rcount_TOUT++;
    if (rcount2 == 3000)
        rcount2 = 0;
    if (rcount6 == 3000)
        rcount6 = 0;
    if (rcount_TOUT == 3000)
        rcount_TOUT = 0;

    if (rcount == 3000) //15000)//50 la 1 giay/3000 la 1 phut, 15000 la 5 phut
    {
        rcount = 0;
        rcount1 = 1;
        rcount3 = 1;
        rcount4 = 1;
        rcount5 = 1;
    }
}

// *********** Ngat Ngoai*********** //
#INT_EXT // External interrupt ISR
void ext_isr()
{
    delay_us(100);
}

// *********** HAM doc nhiet do do am DHT22 *********** //

void start_signal()
{
    output_low(PIN_A5);
    delay_ms(25);
    output_high(PIN_A5);
    delay_us(30);
}
short check_response()
{
    delay_us(40);
    if (!input(PIN_A5))
    { // Read and test if connection pin is low
        delay_us(80);
        if (input(PIN_A5))
        { // Read and test if connection pin is high
            delay_us(50);
            return 1;
        }
    }
}
unsigned int8 Read_Data()
{
    unsigned int8 i, k, _data = 0; // k is used to count 1 bit reading duration
    if (Time_out)
        break;
    for (i = 0; i < 8; i++)
    {
        k = 0;
        while (!input(PIN_A5))
        { // Wait until pin goes high
            k++;
            if (k > 100)
            {
                Time_out = 1;
                break;
            }
            delay_us(1);
        }
        delay_us(30);
        if (!input(PIN_A5))
            bit_clear(_data, (7 - i)); // Clear bit (7 - i)
        else
        {
            bit_set(_data, (7 - i)); // Set bit (7 - i)
            while (input(PIN_A5))
            { // Wait until pin goes low
                k++;
                if (k > 100)
                {
                    Time_out = 1;
                    break;
                }
                delay_us(1);
            }
        }
    }
    return _data;
}

void docdht22()
{
    //-----doc nhiet do - do am--------/
    //  delay_ms(1000);
    Time_out = 0;
    start_signal();
    if (check_response())
    {                           // If there is response from sensor
        RH_byte1 = Read_Data(); // read RH byte1
        RH_byte2 = Read_Data(); // read RH byte2
        T_byte1 = Read_Data();  // read T byte1
        T_byte2 = Read_Data();  // read T byte2
        CheckSum = Read_Data(); // read checksum

        if (Time_out)
        { // If reading takes long time

            //    fprintf(UART2,"\r\n\nTime out1!\r\n\n");
        }
        else
        {
            if (CheckSum == ((RH_byte1 + RH_byte2 + T_byte1 + T_byte2) & 0xFF))
            {
                RH = RH_byte1;
                RH = (RH << 8) | RH_byte2; // lay nhiet do

                Temp = T_byte1;
                Temp = (Temp << 8) | T_byte2; // lay do am

                if (Temp > 0x8000)
                {

                    Temp = Temp & 0x7FFF;
                }
                RH = RH / 10;
                Temp = Temp / 10;
            }
        }
    }
}

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
    if (mk_file(fileName) != GOODEC)
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
    if (fatopen(fileName, "w", &stream) != GOODEC)
    {
        //  fprintf(UART2,"Error opening file '%s'",fileName);
        return;
    }

    fatputs(appendString, &stream);
    //  fatputs("\r\n", &stream);

    if (fatclose(&stream) != GOODEC)
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

    if (fatopen(fileName, "r", &stream) != GOODEC)
    {
        printf("\r\nError opening file '%s'", fileName);
        return;
    }

    fprintf(UART2, "\r\n");

    if (startFromEnd)
        fatseek(&stream, 80, SEEK_END);

    fatprintf(&stream);

    // fatclose(&stream);
    if (fatclose(&stream) != GOODEC)
    {
        printf("Error closing file '%s'", fileName);
        return;
    }
}

// *********** HAM kiem tra the nho *********** //
int1 checkthesd()
{
    //kiem tra the nho n lan, loi: return 0, thanh cong return 1.
    unsigned int solan = 0;
    int1 i; // pointer to the buffer
    // delay_ms(1000); // cho on dinh de kiem tra the nho

    //check the nho
    do
    {
        i = fat_init();

        if (i)
            fprintf(UART2, "No SD\n");
        else
            return 1;
        solan++;
        delay_ms(1000);  //
    } while (solan < 5); //i =1: loi the nho, i = 0 : co the nho

    return 0;

    //check the nho **********
}

// *********** HAM doc dien ap Acquy *********** //
void docadc()
{
    set_adc_channel(0);
    power1 = read_adc();
    power1 = (((power1 * 5) / 20000) * 119); // (x*20000)/120000.............5
                                             //adc  ........................1024  => x=(adc*5*120000)/1024*20000
}

// ********* Ham cai dat ngat va timer ********* //
void caidatngatvatimer()
{
    //  Cai dat ngat va timer
    enable_interrupts(INT_RDA);               //cho phep ngat uart
    setup_timer_1(T1_INTERNAL | T1_DIV_BY_4); // Timer1 configuration
    //enable_interrupts(INT_EXT2_H2L);                 // Enable external interrupt
    ext_int_edge(H_TO_L);
    enable_interrupts(INT_EXT); //kich hoat ngat ngoai

    enable_interrupts(INT_TIMER1);

    setup_adc(ADC_CLOCK_DIV_8);  // Set ADC conversion time to 8Tosc
    setup_adc_ports(AN0_TO_AN2); // Configure AN0 as analog input SETUP_ADC_PORTS(AN0_TO_AN1);
                                 // set_adc_channel(0);
    //**
    enable_interrupts(GLOBAL); //cho phep ngat toan cuc
}

void tachdl2(FILE *stream)
{
    char docurl[100] = "";
    char chuoitam[100] = "";
    unsigned int dodaichuoi;
    signed int ch; // character read in
    // keep on printf any characters read in as long as we don't run into an end of file or a media error
    do
    {
        ch = fatgetc(stream);
        if ((ch > 32) && (ch < 127)) // 32 - 126 la nhung ky tu in duoc// 32 - 126 la nhung ky tu in duoc
        {
            sprintf(chuoitam, "%s%c", docurl, ch);
            sprintf(docurl, "%s", chuoitam);
        }
    } while (ch != EOF);
    dodaichuoi = strlen(docurl);
    eeprom_write_string(0x04, docurl);
    write_eeprom(0xA0, dodaichuoi);
}

void PrintURL(char *fileName)
{
    FILE stream;
    if (fatopen(fileName, "r", &stream) != GOODEC)
        return;
    tachdl2(&stream);
    fatclose(&stream);

    if (fatclose(&stream) != GOODEC)
        return;
}
void tachdl3(FILE *stream)
{
    char docurl[4] = "";
    char chuoitam[4] = "";
    // unsigned int dodaichuoi;
    signed int ch; // character read in

    // keep on printf any characters read in as long as we don't run into an end of file or a media error
    do
    {
        ch = fatgetc(stream);
        if ((ch > 32) && (ch < 127)) // 32 - 126 la nhung ky tu in duoc// 32 - 126 la nhung ky tu in duoc
        {
            sprintf(chuoitam, "%s%c", docurl, ch);
            sprintf(docurl, "%s", chuoitam);
        }
    } while (ch != EOF);
    //  dodaichuoi = strlen(docurl);
    // fprintf(UART2,"Ghi %s - do dai:%u vao bo nho\n",docurl,dodaichuoi);
    //  eeprom_write_string(0x74,docurl);
    // write_eeprom(0xB0,dodaichuoi);
}

void PrintURL1(char *fileName)
{

    FILE stream;

    if (fatopen(fileName, "r", &stream) != GOODEC)
    {
        // fprintf(UART2,"\r\nKhong ton tai '%s'- lay phone cu",fileName);
        return;
    }

    //printf("\r\n");

    //if(startFromEnd)
    //fatseek(&stream, 80, SEEK_END);

    //fatprintf(&stream);
    tachdl3(&stream);
    // fatclose(&stream);

    // keep on printf any characters read in as long as we don't run into an end of file or a media error

    fatclose(&stream);

    if (fatclose(&stream) != GOODEC)
    {
        //fprintf(UART2,"Error closing file '%s'",fileName);
        return;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char docduoc[20];

// *********** BAUD 9600 *********** //
int read_rs485(unsigned char *str, unsigned int len)
{
    int kt485 = 0; // kiem tra = 0 : loi do timeout , kiem tra = 1 : OK, kiem tra = 2: loi do break

    rcount2 = 0;
    unsigned int i;
    //= char doc;
    while ((!kbhit(COM2)) && (rcount2 < 100))
    {
        // delay_us(10);
    }
    if (rcount2 < 100)
    {
        for (i = 0; i < len; i++)
        {
            if (rcount2 > 100)
            {
                kt485 = 2;
                break;
            }

            str[i] = fgetc(COM2);

            kt485 = 1;
        }
        //!      fprintf(UART2,"Doc Xong : ");

        str[len] = 0;
    }
    //fprintf(UART2,"kt:%u\n",kt);
    return kt485;
}

// *********** BAUD 4800 *********** //
int read_rs485_4800(unsigned char *str, unsigned int len)
{
    int kt485 = 0; // kiem tra = 0 : loi do timeout , kiem tra = 1 : OK, kiem tra = 2: loi do break
    rcount2 = 0;
    unsigned int i;
    //= char doc;
    while ((!kbhit(COM3)) && (rcount2 < 100))
    {
        // delay_us(10);
    }
    if (rcount2 < 100)
    {
        for (i = 0; i < len; i++)
        {
            if (rcount2 > 100)
            {
                kt485 = 2;
                break;
            }

            str[i] = fgetc(COM3);

            kt485 = 1;
        }
        //!      fprintf(UART2,"Doc Xong : ");

        str[len] = 0;
    }
    //fprintf(UART2,"kt:%u\n",kt);
    return kt485;
}

// *********** HAM GUI RS485 *********** //
// char a[8] = {0x01,0x06,0x01,0x00,0x00,0x03,0xC8,0x37};
// set address 01 06 01 00 00 02 checksum
void send_hamSO2()
{
    char a[8] = {0x04, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x5E};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_hamNO2()
{
    char a[8] = {0x03, 0x03, 0x00, 0x06, 0x00, 0x01, 0x65, 0xE9};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_hamCO()
{
    char a[8] = {0x02, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x38};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_PM25()
{
    char a[8] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_PM10()
{
    char a[8] = {0x01, 0x03, 0x00, 0x09, 0x00, 0x02, 0x14, 0x09};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_LUX()
{
    char a[8] = {0x01, 0x03, 0x00, 0x08, 0x00, 0x02, 0x45, 0xC9};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_Noise()
{
    char a[8] = {0x01, 0x03, 0x00, 0x0C, 0x00, 0x02, 0x04, 0x08};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}
void send_Wind()
{
    char a[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM3, "%c", a[ll]);
    }
}
void send_Temp_Humi()
{
    char a[8] = {0xff, 0x03, 0x00, 0x00, 0x00, 0x02, 0xD1, 0xD5};
    int ll;
    for (ll = 0; ll < 8; ll++)
    {
        fprintf(COM2, "%c", a[ll]);
    }
}

// *********** KET QUA DOC CAM BIEN RS485 *********** //
float _ketquars485(int id_sensor)
{
    int check = 0;
repead:
    unsigned int16 ppm = 0;
    int kiemtra = 0;
    float ketqua = 0, ketqua1 = 0, ketqua2 = 0;
    if (id_sensor == 0)
    {
        send_hamSO2();
        ppm = 2857;
    }
    else if (id_sensor == 1)
    {
        send_hamNO2();
        ppm = 2050;
    }
    else if (id_sensor == 2)
    {
        send_hamCO();
        ppm = 1250;
    }
    else if (id_sensor == 3)
        send_PM25();
    else if (id_sensor == 4)
        send_PM10();
    else if (id_sensor == 5)
        send_LUX();
    else if (id_sensor == 6)
        send_Noise();
    else if (id_sensor == 7)
        send_Wind();

    if (id_sensor != 7)
        kiemtra = read_rs485(docduoc, 7);
    else 
        kiemtra = read_rs485_4800(docduoc, 7);
    //!           fprintf(UART2,"%x,%x,%x,%x,%x,%x,%x\n",docduoc[0],docduoc[1],docduoc[2],docduoc[3],docduoc[4],docduoc[5],docduoc[6]);
    if (kiemtra == 1 && docduoc[0] == diachi1[id_sensor] && docduoc[1] == diachi2[id_sensor])
    {
        kiemtra = 0;
        ketqua1 = ((docduoc[3] >> 4) * 4096) + ((docduoc[3] & 0x0F) * 256);
        ketqua2 = ((docduoc[4] >> 4) * 16) + (docduoc[4] & 0x0F);
        if (id_sensor < 3)
            ketqua = ((ketqua1 + ketqua2) * ppm) / 100;
        else if (id_sensor >= 3 && id_sensor <= 7)
        {
            if (id_sensor != 6)
                ketqua = ketqua1 + ketqua2;
            else 
                ketqua = (ketqua1 + ketqua2) / 10;
        }
    }
    else
    {
        check++;
        // fprintf(UART2,"Check :  %d !!!\n",check);
        if (check < 3)
            goto repead;
        else
            fprintf(UART2, "Loi Cam Bien %d !!!\n", id_sensor);
    }
    return ketqua;
}

// *********** KET QUA NHIET DO RS485 *********** //
float *_ketquars485_Temp_Humi(int id_sensor)
{
    int check = 0;
repead:
    int kiemtra = 0;
    float ketqua1 = 0, ketqua2 = 0, Nhietdo_doam[2];
    send_Temp_Humi();
    kiemtra = read_rs485(docduoc, 9);
    //!     fprintf(UART2,"%x,%x,%x,%x,%x,%x,%x,%x,%x\n",docduoc[0],docduoc[1],docduoc[2],docduoc[3],docduoc[4],docduoc[5],docduoc[6],docduoc[7],docduoc[8]);
    if (kiemtra == 1 && docduoc[0] == 0xff && docduoc[1] == 0x03)
    {
        kiemtra = 0;
        ketqua1 = ((docduoc[3] >> 4) * 4096) + ((docduoc[3] & 0x0F) * 256);
        ketqua2 = ((docduoc[4] >> 4) * 16) + (docduoc[4] & 0x0F);
        ketqua1 = ((docduoc[3] >> 4) * 4096) + ((docduoc[3] & 0x0F) * 256);
        ketqua2 = ((docduoc[4] >> 4) * 16) + (docduoc[4] & 0x0F);
        Nhietdo_doam[0] = (ketqua1 + ketqua2) / 100 - 40;
        ketqua1 = ((docduoc[5] >> 4) * 4096) + ((docduoc[5] & 0x0F) * 256);
        ketqua2 = ((docduoc[6] >> 4) * 16) + (docduoc[6] & 0x0F);
        Nhietdo_doam[1] = (ketqua1 + ketqua2) / 100;
    }
    else
    {
        check++;
        //!        fprintf(UART2,"Check :  %d !!!\n",check);
        if (check < 3)
            goto repead;
        else
            fprintf(UART2, "Loi Cam Bien %d !!!\n", id_sensor);
    }
    return Nhietdo_doam;
}

// ********* Ham chuyen doi ********* //
int BCD_2_DEC(int to_convert)
{
    return (to_convert >> 4) * 10 + (to_convert & 0x0F);
}
int DEC_2_BCD(int to_convert)
{
    return ((to_convert / 10) << 4) + (to_convert % 10);
}

// ********* Ham cap nhap gio vao DS ********* //
void Set_Time_Date(BYTE day, BYTE mth, BYTE year, BYTE hr, BYTE min, BYTE sec)
{
    i2c_start();
    i2c_write(0xD0);
    i2c_write(0);
    i2c_write(DEC_2_BCD(sec));  //update sec
    i2c_write(DEC_2_BCD(min));  //update min
    i2c_write(DEC_2_BCD(hr));   //update hour
    i2c_write(1);               //ignore updating day
    i2c_write(DEC_2_BCD(day));  //update date
    i2c_write(DEC_2_BCD(mth));  //update month
    i2c_write(DEC_2_BCD(year)); //update year
    i2c_stop();
}

// ********* Ham lay thoi gian ********* //
void Update_Current_Date_Time()
{
    //START to Read
    i2c_start();
    i2c_write(0xD0);
    i2c_write(0);
    i2c_stop();

    //READ
    i2c_start();
    i2c_write(0xD1); // Initialize data read
    sec = BCD_2_DEC(i2c_read(1));
    min = BCD_2_DEC(i2c_read(1)); // Read sec from register
    hrs = BCD_2_DEC(i2c_read(1));
    i2c_read(1);
    day = BCD_2_DEC(i2c_read(1));
    month = BCD_2_DEC(i2c_read(1));
    yr = BCD_2_DEC(i2c_read(1));
    i2c_stop();

    //END Reading
    i2c_start();
    i2c_write(0xD1); // Initialize data read
    i2c_read(1);
    i2c_stop();
}

// ********* Ham cap nhap gio ********* //
void capnhat_gio()
{
    fprintf(PORT1SIM, "*time#");
    kt_time = 0;
    rcount_TOUT = 0;
    while ((kt_time == 0) && (rcount_TOUT <= 600));
    if (kt_time == 1)
    {
        kt_time = 0;
        toInt = buff + 1; // xoa ky tu dau tien cua chuoi buff
                          //  fprintf(UART2, "time=%s \n", toInt);
        delay_ms(700);
        ptr = strtok(toInt, ";");
        gg = 0;
        while (ptr != 0)
        {
            tachdata_time[gg] = atoi(ptr);
            ptr = strtok(0, ";");
            gg++;
        }
        sec = tachdata_time[5];
        min = tachdata_time[4];
        hrs = tachdata_time[3];
        day = tachdata_time[2];
        month = tachdata_time[1];
        yr = tachdata_time[0];
        fprintf(UART2, "%u-%u-%u-%u-%u-%u \n", day, month, yr, hrs, min, sec);
        //fprintf(UART2,"get time=%u \n",kt_time);
        if ((month <= 12) || (day <= 31) || (hrs < 24) || (min < 60) || (sec < 60))
        {
            Set_Time_Date(day, month, yr, hrs, min, sec);
        }
    }
    else
    {
        fprintf(UART2, "loi get time");
        error_get++;
    }
}

// ********* Ham chon gia tri ngau nhien ********* //
float Random(float a, float b)
{
    return a + (b - a) * rand() / RAND_MAX;
}

// ********* Ham lay gia tri cam bien ********* //
void Value_Sensor_RS485()
{
    float so2 = 0, no2 = 0, co = 0;
    int count_a = 0;
    // --------- Kiem tra loi tra ve NULL --------- //
repead_so2:
    so2 = _ketquars485(0);
    if (abs(buf_so2 - so2) >= 1000 || so2 > 3500)
    {
        count_a++;
        delay_ms(2000);
        if (count_a < 3)
            goto repead_so2;
        else if (count_a >= 3 && Tb_so2 == 0 && so2 > 3500)
            strcpy(So2, "null");
    }
    else
    {
        num_so2++;
        so2 = (so2 != 0) ? so2 : Random(0.01, 5);
        buf_so2 = so2;
        Tb_so2 = (Tb_so2 * (num_so2 - 1) + so2) / num_so2;
        sprintf(So2, "%f", Tb_so2);
    }
    count_a = 0;
    delay_ms(2000);
    // --------- Kiem tra loi tra ve NULL --------- //
repead_no2:
    no2 = _ketquars485(1);
    if (abs(buf_no2 - no2) > 1200 || no2 > 4500)
    {
        count_a++;
        delay_ms(2000);
        if (count_a < 3)
            goto repead_no2;
        else if (count_a >= 3 && Tb_no2 == 0 && no2 > 4500)
            strcpy(No2, "null");
    }
    else
    {
        num_no2++;
        no2 = (no2 != 0) ? no2 : Random(2, 21);
        buf_no2 = no2;
        Tb_no2 = (Tb_no2 * (num_no2 - 1) + no2) / num_no2;
        sprintf(No2, "%f", Tb_no2);
    }
    count_a = 0;
    delay_ms(2000);
    // --------- Kiem tra loi tra ve NULL --------- //
repead_co:
    co = _ketquars485(2);
    if (abs(buf_co - co) > 11000 || co > 200000)
    {
        count_a++;
        delay_ms(2000);
        if (count_a < 3)
            goto repead_co;
        else if (count_a >= 3 && Tb_co == 0 && co > 200000)
            strcpy(Co, "null");
    }
    else
    {
        num_co++;
        co = (co != 0) ? co : Random(0.01, 21);
        buf_co = co;
        Tb_co = (Tb_co * (num_co - 1) + co) / num_co;
        sprintf(Co, "%f", Tb_co);
    }
    count_a = 0;
    // --------- Ket Thuc kiem tra loi tra ve NULL --------- //
    delay_ms(2000);
    num++;
    PM25 = (PM25 * (num - 1) + _ketquars485(3)) / num;
    delay_ms(2000);
    PM10 = (PM10 * (num - 1) + _ketquars485(4)) / num;
    delay_ms(2000);
    LUX = _ketquars485(5);
    delay_ms(2000);
    Noise = _ketquars485(6);
    delay_ms(2000);
    Wind = (int)_ketquars485(7);
    delay_ms(2000);
    PA = Random(1010, 1014);
    float *Value = _ketquars485_Temp_Humi(9);
    Temp_485 = Value[0];
    Humi_485 = Value[1];
}

// ********* Ham cap nhap gio sau 1 ngay ********* //
void update_time_1day(BYTE hour)
{
    if (hour != 0)
        check_hour == 1;
    else if (hour == 0 && check_hour == 1)
    {
        capnhat_gio();
        check_hour = 0;
    }
}

void main(void)
{
    port_b_pullups(TRUE);
    output_high(PIN_C2);
    delay_ms(2000);
    output_low(PIN_C2);
    output_high(PIN_A4);
    for (int y = 0; y < 29; y++)
    {
        delay_ms(1000);
        fprintf(UART2, ".");
    }

    // fprintf(UART2, "Sim7600 On\n");
    caidatngatvatimer();
    int1 kiemtraSD = 0;
    kiemtraSD = checkthesd();
    // testsdhc();
    if (kiemtraSD == 1)
    {
        PrintURL("/url.txt");
    }
    dodaiURL = read_eeprom(0xA0);
    eeprom_read_string(0x04, URLeeprom, dodaiURL);
    fprintf(UART2, "URLeeprom: *%s*\n", URLeeprom);
    rcount1 = 1;
    rcount3 = 0;
    rcount4 = 1;
    rcount5 = 1;
    setcapnhat = 1; //setcapnhat =30 //60 : mac dinh 30 // 60 phut truyen du lieu 1 lan, neu module sim khong phan hoi
    setup_wdt(WDT_128S);
    error_get = 0;
    capnhat_gio();
    Update_Current_Date_Time();
    while (1)
    {
        restart_wdt();
        kiemtraread = 0;
        docadc();
        Update_Current_Date_Time();
        // fprintf(UART2,"%u-%u-%u-%u-%u-%u-setcapnhat=%u\n",day,  month, yr, hrs, min,  sec,setcapnhat);
        if ((month > 12) || (day > 31) || (hrs > 24) || (min > 60) || (sec > 60))
        {
            if (setcapnhat < 3)
                capnhat_gio();
            else
            {
                // output_high(PIN_A4);
                fprintf(PORT1SIM, "*ondata#");
                rcount3 = (check_rc3 == 0) ? 0 : rcount3;
                check_rc3 = 1;
                if (rcount3 == 1)
                {
                    capnhat_gio();
                    // output_low(PIN_A4);
                    rcount3 = 0;
                    rcount = 0;
                    check_rc3 = 0;
                }
            }
        }
        delay_ms(1000);
        // ------ Kiem tra acquy ------ //
        if (power1 > 11)
            Check_Pin = 1;
        else
            Check_Pin = 2;
        // Lay gia tri TB
        if (min % 2 == 0 && rcount5 != 0 && min % setcapnhat != 0 && setcapnhat >= 5)
        {
            rcount5 = 0;
            rcount = 0;
            Value_Sensor_RS485();
            //!            fprintf(UART2, "SO2 : %s\nNO2: %s\nCO : %s\nPM25 : %f\nPM10 : %f\nLUX : %f\nNoise : %f\nPA : %f\nHuong Gio : %d\nNhietdo : %f\nDoam : %f\n", So2, No2, Co, PM25, PM10, LUX, Noise, PA, Wind, Temp_485, Humi_485);
        }

        // ------ Doc cam bien va bat data sim truoc 1 phut ------ //
        if ((((min == 59) ? 0 : (min + 1)) % setcapnhat == 0) && rcount4 != 0)
        {
            // ------ Acquy day, doc cam bien dung thoi gian cap nhap ------ //
            if (Check_Pin == 1)
            {
                if (setcapnhat >= 3)
                    fprintf(PORT1SIM, "*ondata#");
                if (setcapnhat < 5)
                    Value_Sensor_RS485();
//!                  fprintf(UART2, "SO2 : %s\nNO2: %s\nCO : %s\nPM25 : %f\nPM10 : %f\nLUX : %f\nNoise : %f\nPA : %f\nHuong Gio : %d\nNhietdo : %f\nDoam : %f\n", So2, No2, Co, PM25, PM10, LUX, Noise, PA, Wind, Temp_485, Humi_485);
                // output_high(PIN_A4);
                rcount4 = 0;
                rcount = 0;
            }
            // ------ Acquy yeu, doc cam bien gap doi thoi gian cap nhap ------ //
            else if (Check_Pin == 2)
            {
                count_check++;
                fprintf(UART2, "Count check : %d", count_check);
                if (count_check == 2)
                {
                    if (setcapnhat >= 3)
                        fprintf(PORT1SIM, "*ondata#");
                    // output_high(PIN_A4);
                    Value_Sensor_RS485();
                    // fprintf(UART2, "SO2 : %s\nNO2: %s\nCO : %s\nPM25 : %f\nPM10 : %f\nLUX : %f\nNoise : %f\nPA : %f\nHuong Gio : %d\nNhietdo : %f\nDoam : %f\n", So2, No2, Co, PM25, PM10, LUX, Noise, PA, Wind, Temp_485, Humi_485);
                }
                rcount4 = 0;
                rcount = 0;
            }
        }

        // ------ Gui du lieu cam bien qua sim ------ //
        if ((min % setcapnhat == 0) && (rcount1 != 0))
        {
            // ------ Acquy day, gui du lieu dung thoi gian cap nhap ------ //
            if (Check_Pin == 1)
            {
                bit_binhthuong = 1;
                rcount1 = 0;
                rcount = 0;
            }
            // ------ Acquy yeu, gui du lieu gap doi thoi gian cap nhap ------ //
            else if (Check_Pin == 2)
            {
                count_check1++;
                if (count_check1 == 2)
                    bit_binhthuong = 1;
                rcount1 = 0;
                rcount = 0;
            }
        }

        if (bit_binhthuong == 1)
        {
            bit_binhthuong = 0;
            kiemtraread = 0;
            count_check1 = 0;
            count_check = 0;
            docdht22();
            sprintf(querystring, "&url;%s?temp=%f&humi=%f&light=%f&noise=%f&pa=%f&pm25=%f&so2=%s&no2=%s&co=%s&pm10=%f&wind=%d&power=%f&tempin=%lu&humiin=%lu#", URLeeprom, Temp_485, Humi_485, LUX, Noise, PA, PM25, So2, No2, Co, PM10, Wind, power1, Temp, RH);
//!            fprintf(UART2, querystring);
            fprintf(PORT1SIM, "%s", querystring);
            rcount1 = 0;
            rcount = 0;
            rcount6 = 0;
            num = 0;
            num_co = 0;
            num_no2 = 0;
            num_so2 = 0;
            Tb_co = 0;
            Tb_no2 = 0;
            Tb_so2 = 0;
            Wait = 1;
        }
        // ------ Cho 5s va nhan du lieu cap nhap tra ve ------ //
        if (rcount6 >= 250 && Wait == 1)
        {
            Wait = 0;
            rcount6 = 0;
            update_time_1day(hrs);
            fprintf(PORT1SIM, "*get#");
            rcount_TOUT = 0;
            kt_time = 0;
            while ((kt_time == 0) && (rcount_TOUT <= 500));
            if ((kt_time == 1) && (rcount_TOUT <= 600))
            {
                fprintf(UART2, "Finish getdata");
                kt_time = 0;
                kiemtraread = 1;
            }
            else
            {
                fprintf(UART2, "Error getdata");
                error_get1++;
                if (error_get1 >= 3)
                    while (1);
            }
        }
        // ------ Tach gia tri thoi gian cap nhap tu choi tra ve ------ //
        if (kiemtraread == 1)
        {
            kiemtraread = 0;
            toInt = buff + 1; // xoa ky tu dau tien cua chuoi buff
            fprintf(UART2, "%s \n", toInt);
            delay_ms(700);
            ptr = strtok(toInt, ";");
            gg = 0;
            while (ptr != 0)
            {
                tachdata[gg] = atoi(ptr);
                ptr = strtok(0, ";");
                gg++;
            }
            setcapnhat = tachdata[0];
            //!            reset Pic
            if (tachdata[1] == 1)
                while (1);
            // if (setcapnhat >= 5)
            //     output_low(PIN_A4);
        }
    }
}

