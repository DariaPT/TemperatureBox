#include "bme280.h"

#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
// Registers
#define BMP280_REG_CALIB00              ((uint8_t)0x88) // Calibration data calib00
#define BMP280_REG_CALIB25              ((uint8_t)0xA1) // Calibration data calib25
#define BMP280_REG_ID                   ((uint8_t)0xD0) // Chip ID
#define BMP280_REG_RESET                ((uint8_t)0xE0) // Software reset control register
#define BMP280_REG_STATUS               ((uint8_t)0xF3) // Device status register
#define BMP280_REG_CTRL_MEAS            ((uint8_t)0xF4) // Pressure and temperature measure control register
#define BMP280_REG_CONFIG               ((uint8_t)0xF5) // Configuration register
#define BMP280_REG_PRESS_MSB            ((uint8_t)0xF7) // Pressure readings MSB
#define BMP280_REG_PRESS_LSB            ((uint8_t)0xF8) // Pressure readings LSB
#define BMP280_REG_PRESS_XLSB           ((uint8_t)0xF9) // Pressure readings XLSB
#define BMP280_REG_TEMP_MSB             ((uint8_t)0xFA) // Temperature data MSB
#define BMP280_REG_TEMP_LSB             ((uint8_t)0xFB) // Temperature data LSB
#define BMP280_REG_TEMP_XLSB            ((uint8_t)0xFC) // Temperature data XLSB

// Register masks
#define BMP280_STATUS_MSK               ((uint8_t)0x09) // unused bits in 'status'
#define BMP280_OSRS_T_MSK               ((uint8_t)0xE0) // 'osrs_t' in 'control'
#define BMP280_OSRS_P_MSK               ((uint8_t)0x1C) // 'osrs_p' in 'control'
#define BMP280_MODE_MSK                 ((uint8_t)0x03) // 'mode' in 'control'
#define BMP280_STBY_MSK                 ((uint8_t)0xE0) // 't_sb' in 'config'
#define BMP280_FILTER_MSK               ((uint8_t)0x1C) // 'filter' in 'config'

#define BMP280_SOFT_RESET_KEY           ((uint8_t)0xB6)

//GPIO and I2C Peripheral (I2C1 Configuration)
#define RCC_APB1Periph_I2Cx       RCC_APB1Periph_I2C1 //Bus where the peripheral is connected
#define RCC_AHB1Periph_GPIO_SCL   RCC_AHB1Periph_GPIOB  //Bus for GPIO Port of SCL
#define RCC_AHB1Periph_GPIO_SDA   RCC_AHB1Periph_GPIOB  //Bus for GPIO Port of SDA
#define GPIO_AF_I2Cx              GPIO_AF_I2C1    //Alternate function for GPIO pins
#define GPIO_SCL                  GPIOB
#define GPIO_SDA                  GPIOB
#define GPIO_Pin_SCL              GPIO_Pin_6
#define GPIO_Pin_SDA              GPIO_Pin_7
#define GPIO_PinSource_SCL        GPIO_PinSource6
#define GPIO_PinSource_SDA        GPIO_PinSource7


#define I2C_SPEED		100000

#define BMP_I2C_ADDRESS 0xEC

#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.


static struct BMP280_cal_param_t
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} calibrationData;

static void I2C1_Init(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure I2C_EE pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x38;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);

}

static void i2c1_read_buf(uint8_t *buf, uint16_t bytesToRead, uint8_t readAddr)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	 /* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, BMP_I2C_ADDRESS, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, readAddr);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);

	 /* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM regAddr for read */
	I2C_Send7bitAddress(I2C1, BMP_I2C_ADDRESS, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while(bytesToRead)
	{
		if(bytesToRead == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(I2C1, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

	/* Test on EV7 and clear it */
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the EEPROM */
			*buf = I2C_ReceiveData(I2C1);

			/* Point to the next location where the byte read will be saved */
			buf++;

			/* Decrement the read bytes counter */
			bytesToRead--;
		}
	}
}


static void bmp280_write_byte(unsigned char regAddr, unsigned char data)
{
  I2C_GenerateSTART(I2C1,ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1, BMP_I2C_ADDRESS, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1,regAddr);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_SendData(I2C1,data);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(I2C1,ENABLE);

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}


static int32_t bmp280_compensate_T_int32(int32_t adcTemperature)
{
	adcTemperature >>= 4;
	int32_t var1, var2, T;

	var1 = ((((adcTemperature>>3) - ((int32_t)calibrationData.dig_T1<<1))) * ((int32_t)calibrationData.dig_T2)) >> 11;
	var2 = (((((adcTemperature>>4) - ((int32_t)calibrationData.dig_T1)) * ((adcTemperature>>4) - ((int32_t)calibrationData.dig_T1))) >> 12) *
		((int32_t)calibrationData.dig_T3)) >> 14;
	int32_t tFine = var1 + var2;
	T = (tFine * 5 + 128) >> 8;

	return T;
}

static void bmp280_read_calibration_data(void)
{
  uint8_t buf[30] = {0};
  i2c1_read_buf(buf, 24, BMP280_REG_CALIB00);

  calibrationData.dig_T1 = (buf[0] | (buf[1] << 8));
  calibrationData.dig_T2 = (buf[2] | (buf[3] << 8));
  calibrationData.dig_T3 = (buf[4] | (buf[5] << 8));
  calibrationData.dig_P1 = (buf[6] | (buf[7] << 8));
  calibrationData.dig_P2 = (buf[8] | (buf[9] << 8));
  calibrationData.dig_P3 = (buf[10] | (buf[11] << 8));
  calibrationData.dig_P4 = (buf[12] | (buf[13] << 8));
  calibrationData.dig_P5 = (buf[14] | (buf[15] << 8));
  calibrationData.dig_P6 = (buf[16] | (buf[17] << 8));
  calibrationData.dig_P7 = (buf[18] | (buf[19] << 8));
  calibrationData.dig_P8 = (buf[20] | (buf[21] << 8));
  calibrationData.dig_P9 = (buf[22] | (buf[23] << 8));
}

static bool bmp_20_reset(void)
{
	bmp280_write_byte(0x0E, 0xB6);

	return 0;
}

void bmp280_init()
{
    I2C1_Init();

    bmp_20_reset();

    bmp280_read_calibration_data();

    bmp280_write_byte(0xF2, 0x01); // oversampling1
    uint8_t f2RegisterValue;
    i2c1_read_buf(&f2RegisterValue, 1, 0xF2);
    bmp280_write_byte(0xF2, f2RegisterValue); // ?????                         			                      //
    bmp280_write_byte(0xF4, ((0x03 << 5) | (0x02 << 2) | 0x03));       // mode normal
    bmp280_write_byte(0xF5, ((0x03 << 5) | (0x04 << 2)));	// 250 ms
}

static int32_t bme280_get_raw_temp()
{
	uint8_t tempLsb = 0;
	uint8_t tempMsb = 0;
	uint8_t tempXlsb = 0;

	i2c1_read_buf(&tempLsb, 1, 0xFA);
	i2c1_read_buf(&tempMsb, 1, 0xFB);
	i2c1_read_buf(&tempXlsb, 1, 0xFC);

	uint32_t rawTemp = ((uint32_t)tempLsb << 16) | ((uint32_t)tempMsb << 8) | (uint32_t)tempXlsb ;

	return bmp280_compensate_T_int32(rawTemp);
}

double bmp280_get_float_temp()
{
	return (double)bme280_get_raw_temp() / 100;
}

int8_t bme280_get_temp_in_celcius()
{
	return bme280_get_raw_temp() / 100;
}

int32_t get_remaped_value_for_zero_min_border(
		int8_t curTempInCelc,
		int8_t maxTempValue,
		int32_t maxRemapValue)
{
	float coef = (float)maxRemapValue / maxTempValue;

	return curTempInCelc * coef;
}

