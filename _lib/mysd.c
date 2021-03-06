
//Slightly modified SD initialization 

MMCSD_err mmcsd_init2() 
{ 
    
   //fprintf(SERIAL2,"\r\n---SD-SPI initialization....\r\n"); 
    
    
    
   unsigned int8 i,r1,ra,rb; 

   g_CRC_enabled = TRUE; 
   g_mmcsdBufferAddress = 0; 

   output_drive(MMCSD_PIN_SCL); 
   output_drive(MMCSD_PIN_SDO); 
   output_drive(MMCSD_PIN_SELECT); 
   output_float(MMCSD_PIN_SDI); 
    
   //delay_ms(1000); 
   //spi_xfer(mmcsd_spi, 0x55); 
   //delay_ms(1000); 
   //spi_xfer(mmcsd_spi, 0x55); 
  
   output_high(MMCSD_PIN_SELECT); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   output_low(MMCSD_PIN_SELECT); 
    


   mmcsd_deselect(); 
   delay_ms(20); 
      
   /* begin initialization */ 
   //fprintf(SERIAL2,"\r\nIdle in\r\n"); 
   i = 0; 
   do 
   { 
      mmcsd_select(); 
      r1=mmcsd_go_idle_state(); 
      mmcsd_deselect(); 
      i++; 
      if(i >= 250) 
      { 
         mmcsd_deselect(); 
         return r1; 
      } 
   } while(!bit_test(r1, 0)); 
   spi_xfer(mmcsd_spi, 0xFF); 
   //fprintf(SERIAL2,"Idle out\r\n"); 
   // following initialization is for MMC card only 
   i = 0; 
   do 
   { 
      delay_ms(20); 
      mmcsd_select(); 
      //r1=mmcsd_send_op_cond();//for MMC card 
      ra=mmcsd_app_cmd(); 
      mmcsd_deselect(); 
      //spi_xfer(mmcsd_spi, 0xFF); 
      delay_us(100); 
      mmcsd_select(); 
      rb=mmcsd_sd_send_op_cond(); 
      //spi_xfer(mmcsd_spi, 0xFF); 
      mmcsd_deselect(); 
      i++; 
      if(i >=250) 
      { 
         mmcsd_deselect(); 
         return r1; 
      } 
   } while((ra | rb) & MMCSD_IDLE); 
    
   /* figure out if we have an SD or MMC */ 
  // mmcsd_select(); 
  // r1=mmcsd_app_cmd(); 
  // r1=mmcsd_sd_send_op_cond(); 
  // mmcsd_deselect(); 

   /* an mmc will return an 0x04 here */ 
   /*if(r1 == 0x04) 
      g_card_type = MMC; 
   else 
      g_card_type = SD; 

   // set block length to 512 bytes 
   printf("Card Type: %X\r\n",r1);*/ 
   mmcsd_select(); 
   r1 = mmcsd_set_blocklen(MMCSD_MAX_BLOCK_SIZE); 
   if(r1 != MMCSD_GOODEC) 
   { 
      mmcsd_deselect(); 
      //fprintf(SERIAL2,"Block error EN: %X\r\n",r1); 
      return r1; 
   } 
    
   mmcsd_deselect(); 
   //fprintf(SERIAL2,"Return from block EN: %X\r\n",r1); 

   /* turn CRCs off to speed up reading/writing */ 
   mmcsd_select(); 
   r1 = mmcsd_crc_on_off(0);//sharry: changed from 0 to 1 
   if(r1 != MMCSD_GOODEC) 
   { 
      mmcsd_deselect(); 
      //return r1;//for proteus simulation only 
     //fprintf(SERIAL2,"CRC error on/off: %X\r\n",r1); 
   } 
    
   mmcsd_deselect(); 
   //fprintf(SERIAL2,"Return from CRC OFF: %X\r\n",r1); 
   //r1=0; 
   r1 = mmcsd_load_buffer(); 
    
   //fprintf(SERIAL2,"\r\n---SD successfully initialized...\r\n"); 
   //delay_ms(2000); 

   return r1; 
} 



//SDHC Initialization 
//#use spi(FORCE_HW, MASTER, DI = MMCSD_PIN_SDI, DO = MMCSD_PIN_SDO, CLK = MMCSD_PIN_SCL, BITS = 8, MSB_FIRST, MODE = 0, stream = mmcsd_spi, CLOCK_HIGH = 1, CLOCK_LOW = 1) 

unsigned int8 sd_cmd1[9] = { 
   0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x95, 0xFF, 0xFF 
}; 
unsigned int8 sd_cmd2[13] = { 
   0xFF, 0x48, 0x00, 0x00, 0x01, 0xAA, 0x87, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 
}; 
unsigned int8 sd_cmd3[8] = { 
   0xFF, 0x77, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF 
}; 
unsigned int8 sd_cmd4[9] = { 
   0xFF, 0x69, 0x40, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF 
}; 
unsigned int8 sd_cmd5[10] = { 
   0xFF, 0x7A, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF 
}; 

unsigned int8 send_spi(unsigned int8 spi_msg) { 

   static unsigned int8 spi_rcv; 
   spi_rcv = spi_xfer(mmcsd_spi, spi_msg); 

   return spi_rcv; 

} 

unsigned int8 Send_Command(unsigned int8 cmd[], unsigned int8 size) { 
   static unsigned int8 p; 
   static unsigned int8 resp; 
   delay_us(20); 
   //mmcsd_select(); 
   for (p = 0; p < size; p++) { 
      resp = send_spi(cmd[p]); 
   } 
   //mmcsd_deselect(); 
   return resp; 
} 

void Initialize(void) { 

   output_drive(MMCSD_PIN_SCL); 
   output_drive(MMCSD_PIN_SDO); 
   output_drive(MMCSD_PIN_SELECT); 
   output_float(MMCSD_PIN_SDI); 
   output_high(MMCSD_PIN_SELECT); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   spi_xfer(mmcsd_spi, 0xFF); 
   output_low(MMCSD_PIN_SELECT); 
   mmcsd_deselect(); 
   delay_us(50); 
} 

int1 Initialize_SDHC(void) { 

   int1 done = 0; 
   static unsigned int8 rcv; 
   static unsigned int16 count = 0; 
   //Send Dummys 

   Initialize(); 


   mmcsd_select(); 
   Send_Command(sd_cmd1, 9); 
   mmcsd_deselect(); 

   mmcsd_select(); 
   Send_Command(sd_cmd2, 13); 
   mmcsd_deselect(); 
   while (!done) { 

      mmcsd_select(); 
      rcv = Send_Command(sd_cmd3, 8); 
      if (rcv == 0x00) { 
         done = 1; 
      } else { 
         mmcsd_deselect(); 
      } 

      mmcsd_select(); 
      rcv = Send_Command(sd_cmd4, 9); 
      if (rcv == 0x00) { 
         done = 1; 
      } else { 
         mmcsd_deselect(); 
      } 

      count++; 
      if (count == 5000) { 
         return 0; 
      } 
   } 

   mmcsd_select(); 
   if (Send_Command(sd_cmd5, 10) == 0xC0) { 


      send_spi(0xFF); 
      send_spi(0xFF); 
      send_spi(0xFF); 
      send_spi(0xFF); 

      return 1; 
   } else { 
      return 0; 
      mmcsd_deselect(); 
   } 
} 

/********************* 
Function: InitializeSD() 
Return 1 if sucess 
Return 0 if unsucessful 
**********************/ 
int1 InitializeSD(void) { 

   if (Initialize_SDHC()) { 
      return 1; 
   } else if (!mmcsd_init2()) { 
      return 2; 
   } else { 
      return 0; 
   } 
} 
