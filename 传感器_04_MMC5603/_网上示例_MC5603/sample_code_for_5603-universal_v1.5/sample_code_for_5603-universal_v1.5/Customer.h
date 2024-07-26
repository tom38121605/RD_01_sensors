
/**
 * @brief Delay function.
 */
void Delay_Ms(int cnt);

/**
 * @brief Delay function.
 */
void Delay_Us(int cnt);

/**
 * @brief I2C write register.
 */
int I2C_Write_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char cmd);

/**
 * @brief I2C read register.
 */
int I2C_Read_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char *data);

/**
 * @brief I2C multi read.
 */
int I2C_MultiRead_Reg(unsigned char i2c_add, unsigned char reg_add, int num, unsigned char *data);
