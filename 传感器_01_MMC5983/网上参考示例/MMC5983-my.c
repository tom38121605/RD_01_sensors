#include "MMC5983.h"
#include <string.h>
#include "nrf_delay.h"

static void write_onebuff_5983(uint8_t address , uint8_t data)
{
    uint8_t tx_data[2] = {0,0};
    tx_data[0] = (address|SPI_W);
    tx_data[1] = data;
    MMC5983_CS_LOW();
    spi2_read_write(2 , tx_data , 0 , NULL);
    MMC5983_CS_HIGH();
}

static uint8_t read_onebuff_5983(uint8_t address)
{
    uint8_t tx_data = 0;
    uint8_t id_value = 0;
    tx_data = (address|SPI_R);
    MMC5983_CS_LOW();
    spi2_read_write(1 , &tx_data , 1 , &id_value);
    MMC5983_CS_HIGH();
    return id_value;
}


uint8_t read_mmc5983_id(void)
{
    uint8_t id_value = 0;
    id_value = read_onebuff_5983(MMC5983_ADDR);
    return id_value;
}

void set_Internal_Control_0(void)
{
    uint8_t reg_data = 0x20;
    write_onebuff_5983(INTERNAL_CTRL0 , reg_data);    //09H.5=1,   automatic set/reset.
}

//set resolution 
//这里的resolution 要配合control2设置
void set_Internal_Control_1(void)
{
    uint8_t reg_data = RS_4MS_200Hz;  //1
    write_onebuff_5983(INTERNAL_CTRL1 , reg_data);    //0AH.1-0 = 01,   4MS_200Hz
}

void set_Internal_Control_2(void)
{
    uint8_t reg_data = (CM_FREQ_200Hz|CMM_EN_CONTINUS|PRD_SET_1TIMES);
    write_onebuff_5983(INTERNAL_CTRL2 , reg_data);   //0BH.2-0=110, 0BH.3=1, 0BH.6-4=000, 200Hz, 连续， 1TIMES
}

void set_Internal_Control_3(void)
{

}

void mmc5983_init(void)
{
    if(read_mmc5983_id() != MMC5983_ID)
    {
        return;
    }
    set_Internal_Control_0();
    set_Internal_Control_1();
    set_Internal_Control_2();

}

uint8_t mmc5983_status_read(void)
{
    uint8_t id_value = 0;
    id_value = read_onebuff_5983(MMC_STATUS);
    return id_value;
}


void mmc5983_data_read(mmc_5983_data* mag)
{
    uint8_t status = mmc5983_status_read();
    uint8_t low_bit = 0;
    (mag->x) = read_onebuff_5983(XOUT0);
    (mag->x) <<= 8;
    (mag->x) |= read_onebuff_5983(XOUT1);
    (mag->x) <<= 2;
    (mag->y) = read_onebuff_5983(YOUT0);
    (mag->y) <<= 8;
    (mag->y) |= read_onebuff_5983(YOUT1);
    (mag->y) <<= 2;
    (mag->z) = read_onebuff_5983(ZOUT0);
    (mag->z) <<= 8;
    (mag->z) |= read_onebuff_5983(ZOUT1);
    (mag->z) <<= 2;
    low_bit = read_onebuff_5983(XYZOUT2);
    (mag->x) |= ((low_bit >> 6)&0x03);
    (mag->y) |= ((low_bit >> 4)&0x03);
    (mag->z) |= ((low_bit >> 2)&0x03);
    (mag->x) = (((mag->x)<<14)/16384);
    (mag->y) = (((mag->y)<<14)/16384);
    (mag->z) = (((mag->z)<<14)/16384);   
    
}


uint16_t mmc5983_read_temp(void)
{
    uint8_t id_value=0;
    id_value = read_onebuff_5983(TOUT);
    return (id_value*100);
}










