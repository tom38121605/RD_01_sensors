#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_spi.h"
#include "icm42605.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
 
void icm42605_cs_high()
{
   nrf_gpio_pin_set(ICM42605_SPI_SS_PIN);
}

void icm42605_cs_low()
{
   nrf_gpio_pin_clear(ICM42605_SPI_SS_PIN);
}

uint8_t icm42605_spi_rw(uint8_t reg)
{
   uint8_t rx_data = 0;
   nrf_gpio_pin_set(ICM42605_SPI_SCK_PIN);
   for(uint8_t i =0;i<8;i++)
	 {
       if(reg & 0x80)
          nrf_gpio_pin_set(ICM42605_SPI_MOSI_PIN);
       else
          nrf_gpio_pin_clear(ICM42605_SPI_MOSI_PIN); 
       nrf_gpio_pin_clear(ICM42605_SPI_SCK_PIN);
       reg <<= 1;
       rx_data <<= 1;
       if(nrf_gpio_pin_read(ICM42605_SPI_MISO_PIN))
          rx_data |= 0x01;
       nrf_gpio_pin_set(ICM42605_SPI_SCK_PIN);    
   }
   return rx_data;   
}
 
uint8_t icm42605_read_reg(uint8_t reg)
{
   uint8_t temp;
   icm42605_cs_low();
   icm42605_spi_rw((reg | 0x80));
   temp = icm42605_spi_rw(0);
   icm42605_cs_high();
   return temp;
}

uint8_t icm42605_write_reg(uint8_t reg,uint8_t value)
{
   icm42605_cs_low();
   icm42605_spi_rw((reg & 0x7f));
   icm42605_spi_rw(value);
   icm42605_cs_high();
   return 0;
}

uint8_t icm42605_read_regs(uint8_t reg,uint8_t *buf,uint16_t len)
{
   uint16_t i = 0;
   icm42605_cs_low();
   icm42605_spi_rw((reg | 0x80));
   while(i < len){
        buf[i] = icm42605_spi_rw(0);
        i++;
   }
   icm42605_cs_high();
   return 0;
}


//icm42605 初始化
uint8_t icm42605_init(void)
{   
    uint8_t reg_val;
    icm42605_write_reg(reg_bank_sel,0x00);  //Set to bank 0    //76H = 00H
    icm42605_write_reg(device_config_reg, bit_soft_reset_chip_config);  // 11h = 01h ， chip soft reset       //--same
    nrf_delay_ms(100);
			
    icm42605_write_reg(reg_bank_sel,0x00);     //Set to bank 0    //76H = 00H
    reg_val = icm42605_read_reg(who_am_i);   //who_am_i          //ival = 75h
    if(reg_val==0x42)
    {
        icm42605_write_reg(reg_bank_sel,0x01);       //76H = 01H , bank1
        icm42605_write_reg(intf_config4,0x02);         //7ah.1 = 1,   4 wire spi mode     //--default
			
        icm42605_write_reg(reg_bank_sel,0x00);       //76H = 00H    
        icm42605_write_reg(fifo_config_reg,0x40);     //16h.7-6 = 01,  Stream-to-FIFO Mode

        //-----------watermark----------------------
        reg_val = icm42605_read_reg(int_source0_reg);     // ival =65h  
        icm42605_write_reg(int_source0_reg,0x00);           // 65h =00 , 清INT1的关联

        icm42605_write_reg(fifo_config2_reg,0x00);          // 60h = 00h, watermark
        icm42605_write_reg(fifo_config3_reg,0x02);          // 61h = 02h, watermark

        icm42605_write_reg(int_source0_reg, reg_val);          // 65h = ival
        //-----------watermark--------end--------------

        icm42605_write_reg(fifo_config1_reg,0x63);      // 5fh.6-5 =11， 5fh.1-0 =11，或环形fifo并使能门槛中断，使能 accel and gyro to the FIFO
			
        icm42605_write_reg(reg_bank_sel,0x00);           //76H = 00H
        icm42605_write_reg(int_config_reg,0x36);          //14h = 36H ，设置INT1,INT2为脉冲模式，push pull , 高电平有效  //--INT1相同，INT2不同
			

        icm42605_write_reg(reg_bank_sel, 0x00);         //76H = 00H
        reg_val = (icm42605_read_reg(int_source0_reg)|bit_int_fifo_ths_int1_en);    // ival = 65h | 04h
        icm42605_write_reg(int_source0_reg, reg_val);                                        // 65h.2 =1 ， FIFO threshold 关联到 INT1   //--chg
			

        icm42605_write_reg(reg_bank_sel, 0x00);      //76H = 00H
        reg_val = ((icm42605_read_reg(accel_config0_reg)&0x1F)|(bit_accel_ui_fs_sel_8g));      // 8g
        icm42605_write_reg(accel_config0_reg, reg_val);                                                      // 50h.7-5 = 1<<5,  acc的fs为8g  //--chg
				
        icm42605_write_reg(reg_bank_sel, 0x00);   //76H = 00H
        reg_val = ((icm42605_read_reg(accel_config0_reg)&0xF0)|bit_accel_odr_50hz);            // 50h.3-0 = 9 , acc的ODR为50hz  
        icm42605_write_reg(accel_config0_reg, reg_val); 
				
        icm42605_write_reg(reg_bank_sel, 0x00);    //76H = 00H
        reg_val = ((icm42605_read_reg(gyro_config0_reg)&0x1F)|(bit_gyro_ui_fs_sel_1000dps));   // 4fH.7-5 = 1<<5,  gyro的fs为1000dps  //--chg
        icm42605_write_reg(gyro_config0_reg,reg_val);
				
        icm42605_write_reg(reg_bank_sel, 0x00);      //76H = 00H   
        reg_val = ((icm42605_read_reg(gyro_config0_reg)&0xF0)|bit_gyro_odr_50hz);                // 4fh.3-0 = 9  , gyro的ODR为50hz    //--chg
        icm42605_write_reg(gyro_config0_reg, reg_val); 
				


        icm42605_write_reg(reg_bank_sel, 0x00);     //76H = 00H   
        reg_val = icm42605_read_reg(pwr_mgmt0_reg)|(bit_accel_mode_ln);      // Accel on in LNM       //--same
        icm42605_write_reg(pwr_mgmt0_reg, reg_val);                                     //4eh.1-0  = 3,  acc 设置为低噪声模式
        nrf_delay_us(400);  
				
        icm42605_write_reg(reg_bank_sel, 0x00);    //76H = 00H   
        reg_val = icm42605_read_reg(pwr_mgmt0_reg)|(bit_gyro_mode_ln);        // Gyro on in LNM     //--same
        icm42605_write_reg(pwr_mgmt0_reg, reg_val);                                      //4eh.3-2  = 11  , gyro 设置为低噪声模式 


        nrf_delay_us(400); 
        printf("ok");

        return 1;
   }
  else 
     return 0;
}


uint8_t  fifocount_l, fifocount_h;
uint16_t fifocount;
void icm42605_read_fifo(Sample_data_type_t *data,uint16_t len)
{
	 uint8_t reg_val;
   uint8_t tempbuff[512]={0};
	 reg_val = icm42605_read_reg(int_source0_reg);      
   icm42605_write_reg(int_source0_reg,0x00); 
	 
	 fifocount_h = icm42605_read_reg(fifo_byte_count_h_res); // Read the FIFO size
   fifocount_l = icm42605_read_reg(fifo_byte_count_l_res);
   fifocount = (fifocount_h<<8)|fifocount_l;
	 
   icm42605_read_regs(fifo_data_port,tempbuff,len);
	 if(fifocount>=fifo_packet_size) // If we have a complete packet in the FIFO
   {
		 for(uint8_t i=0;i<32;i++)
		 {
				if((tempbuff[i*16]&fifo_accel_en)&&(tempbuff[i*16]&fifo_gyro_en))
				{
							data->Sample_accdata[0+i*3] = ((int16_t)((tempbuff[1+i*16] << 8) | tempbuff[2+i*16]))*acc_ssl;
							data->Sample_accdata[1+i*3] = ((int16_t)((tempbuff[3+i*16] << 8) | tempbuff[4+i*16]))*acc_ssl;
							data->Sample_accdata[2+i*3] = ((int16_t)((tempbuff[5+i*16] << 8) | tempbuff[6+i*16]))*acc_ssl;
							data->Sample_gyrdata[0+i*3] = ((int16_t)((tempbuff[7+i*16] << 8) | tempbuff[8+i*16]))/gyr_ssl;
							data->Sample_gyrdata[1+i*3] = ((int16_t)((tempbuff[9+i*16] << 8) | tempbuff[10+i*16]))/gyr_ssl;
							data->Sample_gyrdata[2+i*3] = ((int16_t)((tempbuff[11+i*16]<< 8) | tempbuff[12+i*16]))/gyr_ssl;
				}
		 }
   }
	 icm42605_write_reg(int_source0_reg, reg_val); 
}

void icm42605_stop(void)
{
    uint8_t reg_val; 
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = icm42605_read_reg(pwr_mgmt0_reg)&(~bit_accel_mode_ln); // Accel off
    icm42605_write_reg(pwr_mgmt0_reg, reg_val);
    nrf_delay_us(400);  
	
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = icm42605_read_reg(pwr_mgmt0_reg)|(bit_gyro_mode_ln); // Gyro on in LNM
    icm42605_write_reg(pwr_mgmt0_reg, reg_val);  
    nrf_delay_us(400); 
	  nrf_delay_ms(400); 

}


