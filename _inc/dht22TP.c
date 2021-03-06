/**
  **********************************************************************************************
  * @file       dht22TP.c
  * @author     TamPhuc
  * @version    V1.0.0
  * @date       
  * @TomTat      Chuong trinh do nhie do, do am su dung DHT22
  *
  **********************************************************************************************
  * Chu Y      :
  *                      
  **********************************************************************************************
  */



#byte PORTA=0x05


  
#bit   DHT_DATA  = PORTA.2            //chan A.2  
#define  SET_PIN      SET_TRIS_A 
#define DHT_ER      0 
#define DHT_OK      1 
#define DHT_ND      2 
#define DHT_ND1     3
#define DHT_DA      0
#define DHT_DA1      1

////////////////////
//
//    tra ve nhiet do float doctemp()//
///////////////////////

////////////////////
//
//    tra ve do am float dochuni()//
///////////////////////

/***********************************************************************************************
Chuc nang   :      Gui tin hieu do va doc gia tri tra ve cua DHT22
Tham so     :      select: Chon lua lay gia tri do am hay nhiet do
Tra ve      :      buffer: tra ve gia tri nhiet do, do am
                   DHT_ER: giao tiep voi DHT11 bi loi
                   DHT_OK: giao tiep tot voi DHT11
***********************************************************************************************/
unsigned char DHT_GetTemHumi (unsigned char select) 
{ 
    unsigned char buffer[5]={0,0,0,0,0}; 
    unsigned char ii,i,checksum; 
    
    SET_PIN(0x00);  // set la cong ra 
    DHT_DATA=1;
    delay_us(60);
    DHT_DATA = 0;
    delay_ms(18);
    DHT_DATA = 1; // it nhat 18ms 
    SET_PIN(0xff); set la cong vao
    delay_us(40); 
    if(DHT_DATA==1)return DHT_ER ; 
    else while(DHT_DATA==0);    //Doi DaTa len 1 
    delay_us(60);       //60
    if(DHT_DATA==0)return DHT_ER; 
    else while((DHT_DATA==1));    //Doi Data ve 0  
    //delay_us(80);
    for(i=0;i<5;i++) 
    { 
        for(ii=0;ii<8;ii++) 
        {    
        while(DHT_DATA==0);//Doi Data len 1 !DHT_DATA
        delay_us(50); //50
        if(DHT_DATA==1) 
            { 
            buffer[i]|=(1<<(7-ii)); 
            while(DHT_DATA==1);//Doi Data xuong 0 
            } 
        } 
    } 
    //Tinh toan check sum 
    checksum=buffer[0]+buffer[1]+buffer[2]+buffer[3]; 
    //Kiem tra check sum 
    if((checksum)!=buffer[4])return DHT_ER; 
    //Lay du lieu 
      if (select==DHT_ND) 
      {    
            return(buffer[2]); 
      } 
      else if(select==DHT_ND1) 
      { 
            return(buffer[3]); 
      } 
      else if(select==DHT_DA) 
      { 
            return(buffer[0]); 
      } 
      else if(select==DHT_DA1) 
      { 
            return(buffer[1]); 
      } 
    return DHT_OK; 
    
} 

float doctemp()
{
float ND, ND1;
 ND= DHT_GetTemHumi (DHT_ND);
      delay_ms(400);
      ND1= DHT_GetTemHumi (DHT_ND1);
      delay_ms(400);
      ND=((ND*256)+ND1)/10;
      return ND;
 }
float dochuni()
{
float DA, DA1;
  DA= DHT_GetTemHumi  (DHT_DA);
      delay_ms(400);
      DA1= DHT_GetTemHumi  (DHT_DA1); 
      delay_ms(400);
      DA=((DA*256) + DA1)/10;
return DA;

}

//------------------------------------------------------// 
