/* NXP Microelectronics FXLS8974CF 3-axis accelerometer driver
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */    

#ifndef ZEPHYR_DRIVERS_SENSOR_FXLS8974CF_FXLS8974CF_H_
#define ZEPHYR_DRIVERS_SENSOR_FXLS8974CF_FXLS8974CF_H_

#include <zephyr/drivers/i2c.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

/**
 * struct iis2dh_device_config - iis2dh hw configuration
 * @spi: SPI bus spec.
 * @i2c: I2C bus spec.
 * @pm: Power mode (FXLS8974CF_powermode).
 * @int_gpio: GPIO spec for sensor pin interrupt.
 */

#define fxls8974cf_REG_STATUS           0x14
#define FXLS8974CF_REG_ACCEL_X_LSB		0x04
#define FXLS8974CF_REG_ACCEL_Y_LSB		0x06
#define FXLS8974CF_REG_ACCEL_Z_LSB		0x08
#define FXLS8974CF_REG_ACCEL_X_MSB		0x05
#define FXLS8974CF_REG_ACCEL_Y_MSB		0x07
#define FXLS8974CF_REG_ACCEL_Z_MSB		0x09

#define FXLS8974CF_SENS_CONFIG1_ACTIVE_MASK ((uint8_t)0x01)

#define FXLS8974CF_STATUS_DRDY_MASK		BIT_MASK(4)

#define FXLS8974CF_BUF_SZ			7
#define FXLS8974CF_READ_CMD       0x80
#define NUM_AXES  0x03
#define ACCEL_RAW_BYTE (NUM_AXES * 2)
#define NUM_ORIENTATIONS (6)
#define FIFO_SIZE 0x1
#define ACCEL_SAMPLE_SIZE 0x6
#define FXLS8974CF_MAX_FIFO_SIZE      32
#define FXLS8974CF_FIFO_STATUS_REG   0x0b

#define FXLS8974CF_SENS_CONFIG1    0x15
#define FXLS8974CF_FS_SHIFT			1
#define FXLS8974CF_FS_MASK			(BIT_MASK(2) << FXLS8974CF_FS_SHIFT)

#define FXLS8974CF_FS_MASK1    0x0e



#if defined(CONFIG_FXLS8974CF_ACCEL_RANGE_2G) ||\
	defined(CONFIG_FXLS8974CF_ACCEL_RANGE_RUNTIME)
	#define FXLS8974CF_FS_IDX		0
#elif defined(CONFIG_FXLS8974CF_ACCEL_RANGE_4G)
	#define FXLS8974CF_FS_IDX		1
#elif defined(CONFIG_FXLS8974CF_ACCEL_RANGE_8G)
	#define FXLS8974CF_FS_IDX		2
#elif defined(CONFIG_FXLS8974CF_ACCEL_RANGE_16G)
	#define FXLS8974CF_FS_IDX		3
#endif


#define FXLS8974CF_FS_SELECT(fs)		((fs) << FXLS8974CF_FS_SHIFT)

#define FXLS8974CF_FS_BITS			(FXLS8974CF_FS_SELECT(FXLS8974CF_FS_IDX))

#define FXLS8974CF_HR_BIT		0x00

union fxls8974cf_sample {
	uint8_t raw[FXLS8974CF_BUF_SZ];
	struct {
		uint8_t status;
		int16_t xyz[3];
	} __packed;
};

typedef int (*fxls8974cf_reg_access_fn)(const struct device *dev, uint8_t cmd,
				     uint8_t reg_addr, uint8_t *data, size_t length);

struct fxls8974cf_device_config {

 	struct i2c_dt_spec i2c;

    fxls8974cf_reg_access_fn reg_access;
};

int fxls8974cf_i2c_init(const struct device *dev);

/* sensor data */
struct fxls8974cf_data {

   unsigned int selected_range;

    const struct device *dev;

    const struct fxls8974cf_transfer_function *hw_tf;

    union fxls8974cf_sample sample;
	/* current scaling factor, in micro m/s^2 / lsb */
	uint32_t scale;

    int16_t bufx[FXLS8974CF_MAX_FIFO_SIZE];
	int16_t bufy[FXLS8974CF_MAX_FIFO_SIZE];
	int16_t bufz[FXLS8974CF_MAX_FIFO_SIZE];


	int16_t acc[3];
	uint32_t gain;
	
};

struct fxls8974cf_transfer_function {
	int (*read_data)(const struct device *dev, uint8_t reg_addr,
			 uint8_t *value, uint8_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr,
			  uint8_t *value, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t *value);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr,
			 uint8_t value);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr,
			  uint8_t mask, uint8_t value);
};

typedef struct
{
    uint8_t accel[ACCEL_RAW_BYTE * FIFO_SIZE]; /*!< The accel data. */
}fxls8974cf_accel_raw_t;

typedef enum fxls8974cf_odr
{
	FXLS8974CF_WAKE_ODR_3200HZ   = 0x00,
	FXLS8974CF_WAKE_ODR_1600HZ   = 0x10,
	FXLS8974CF_WAKE_ODR_800HZ    = 0x20,
	FXLS8974CF_WAKE_ODR_400HZ    = 0x30,
	FXLS8974CF_WAKE_ODR_200HZ    = 0x40,
	FXLS8974CF_WAKE_ODR_100HZ    = 0x50,
	FXLS8974CF_WAKE_ODR_50HZ     = 0x60,
	FXLS8974CF_WAKE_ODR_25HZ     = 0x70,
	FXLS8974CF_WAKE_ODR_12_5HZ   = 0x80,
	FXLS8974CF_WAKE_ODR_6_25HZ   = 0x90,
	FXLS8974CF_WAKE_ODR_3_125HZ  = 0xa0,
	FXLS8974CF_WAKE_ODR_1_563HZ  = 0xb0,
	FXLS8974CF_WAKE_ODR_0_781HZ  = 0xc0,
	FXLS8974CF_SLEEP_ODR_3200HZ  = 0x00,
	FXLS8974CF_SLEEP_ODR_1600HZ  = 0x01,
	FXLS8974CF_SLEEP_ODR_800HZ   = 0x02,
	FXLS8974CF_SLEEP_ODR_400HZ   = 0x03,
	FXLS8974CF_SLEEP_ODR_200HZ   = 0x04,
	FXLS8974CF_SLEEP_ODR_100HZ   = 0x05,
	FXLS8974CF_SLEEP_ODR_50HZ    = 0x06,
	FXLS8974CF_SLEEP_ODR_25HZ    = 0x07,
	FXLS8974CF_SLEEP_ODR_12_5HZ  = 0x08,
	FXLS8974CF_SLEEP_ODR_6_25HZ  = 0x09,
	FXLS8974CF_SLEEP_ODR_3_125HZ = 0x0a,
	FXLS8974CF_SLEEP_ODR_1_563HZ = 0x0b,
	FXLS8974CF_SLEEP_ODR_0_781HZ = 0x0c
}fxls8974cf_odr_t;

typedef struct fxls8974cf_control
{
    uint8_t range: 2;            /*!< Refer fxls8974cf_range enum. */
    uint8_t le_be:1;             /*!< 1 = Big-endian , 0 = little-Endian. */
	uint8_t fastmode:1;          /*!< 1 = Set the fast mode , 0 = normal mode. */
	uint8_t reservered:4;        /*!< reserved */
}fxls8974cf_control_t;


enum
{
    FXLS8974CF_INT_STATUS = 0x00,
    FXLS8974CF_TEMP_OUT = 0x01,
    FXLS8974CF_VECM_LSB = 0x02,
    FXLS8974CF_VECM_MSB = 0x03,
    FXLS8974CF_OUT_X_LSB = 0x04,
    FXLS8974CF_OUT_X_MSB = 0x05,
    FXLS8974CF_OUT_Y_LSB = 0x06,
    FXLS8974CF_OUT_Y_MSB = 0x07,
    FXLS8974CF_OUT_Z_LSB = 0x08,
    FXLS8974CF_OUT_Z_MSB = 0x09,
    FXLS8974CF_BUF_STATUS = 0x0B,
    FXLS8974CF_BUF_X_LSB = 0x0C,
    FXLS8974CF_BUF_X_MSB = 0x0D,
    FXLS8974CF_BUF_Y_LSB = 0x0E,
    FXLS8974CF_BUF_Y_MSB = 0x0F,
    FXLS8974CF_BUF_Z_LSB = 0x10,
    FXLS8974CF_BUF_Z_MSB = 0x11,
    FXLS8974CF_PROD_REV = 0x12,
    FXLS8974CF_WHO_AM_I = 0x13,
    FXLS8974CF_SYS_MODE = 0x14,
    // FXLS8974CF_SENS_CONFIG1 = 0x15,
    FXLS8974CF_SENS_CONFIG2 = 0x16,
    FXLS8974CF_SENS_CONFIG3 = 0x17,
    FXLS8974CF_SENS_CONFIG4 = 0x18,
    FXLS8974CF_SENS_CONFIG5 = 0x19,
    FXLS8974CF_WAKE_IDLE_LSB = 0x1A,
    FXLS8974CF_WAKE_IDLE_MSB = 0x1B,
    FXLS8974CF_SLEEP_IDLE_LSB = 0x1C,
    FXLS8974CF_SLEEP_IDLE_MSB = 0x1D,
    FXLS8974CF_ASLP_COUNT_LSB = 0x1E,
    FXLS8974CF_ASLP_COUNT_MSB = 0x1F,
    FXLS8974CF_INT_EN = 0x20,
    FXLS8974CF_INT_PIN_SEL = 0x21,
    FXLS8974CF_OFF_X = 0x22,
    FXLS8974CF_OFF_Y = 0x23,
    FXLS8974CF_OFF_Z = 0x24,
    FXLS8974CF_BUF_CONFIG1 = 0x26,
    FXLS8974CF_BUF_CONFIG2 = 0x27,
    FXLS8974CF_ORIENT_STATUS = 0x28,
    FXLS8974CF_ORIENT_CONFIG = 0x29,
    FXLS8974CF_ORIENT_DBCOUNT = 0x2A,
    FXLS8974CF_ORIENT_BF_ZCOMP = 0x2B,
    FXLS8974CF_ORIENT_THS_REG = 0x2C,
    FXLS8974CF_SDCD_INT_SRC1 = 0x2D,
    FXLS8974CF_SDCD_INT_SRC2 = 0x2E,
    FXLS8974CF_SDCD_CONFIG1 = 0x2F,
    FXLS8974CF_SDCD_CONFIG2 = 0x30,
    FXLS8974CF_SDCD_OT_DBCNT = 0x31,
    FXLS8974CF_SDCD_WT_DBCNT = 0x32,
    FXLS8974CF_SDCD_LTHS_LSB = 0x33,
    FXLS8974CF_SDCD_LTHS_MSB = 0x34,
    FXLS8974CF_SDCD_UTHS_LSB = 0x35,
    FXLS8974CF_SDCD_UTHS_MSB = 0x36,
    FXLS8974CF_SELF_TEST_CONFIG1 = 0x37,
    FXLS8974CF_SELF_TEST_CONFIG2 = 0x38,
};

typedef union {
    struct {
        uint8_t            trg_orient : 1; /*  Orientation change event trigger enable                                    */

        uint8_t _reserved_            : 1;
        uint8_t           trg_sdcd_ot : 1; /*  SDCD outside-of-thresholds event buffer trigger enable                     */

        uint8_t           trg_sdcd_wt : 1; /*  SDCD within-thresholds event trigger enable                                */

        uint8_t              buf_gate : 1; /*  Output data buffer gate enable                                             */

        uint8_t              buf_mode : 2; /*  Buffer data collection mode                                                */

        uint8_t              buf_type : 1; /*  Buffer data read out order                                                 */

    } b;
    uint8_t w;
} FXLS8974CF_BUF_CONFIG1_t;


typedef enum sensor_error_type
{
    SENSOR_SUCCESS           = 0,  /*!< Success value returned by sensor APIs. */
    SENSOR_INVALIDPARAM_ERR  = 1,  /*!< Invalid Param Error value by SENSOR APIs. */
    SENSOR_INIT_ERR          = 2,  /*!< SENSOR Init Error value returned by Init API. */
    SENSOR_WRITE_ERR         = 3,  /*!< SENSOR Write Error value returned by Write API. */
    SENSOR_READ_ERR          = 4,  /*!< SENSOR Read Error value returned by Read API. */
	SENSOR_BAD_ADDRESS       = 5,  /*!< SENSOR Error value returned for bad address access. */
} fxos8700_error_type_t;



typedef union {
    struct {
        uint8_t          buf_wmrk : 6; /*  Buffer sample count watermark                                              */

        uint8_t          wake_src_buf : 1; /*  Buffer WAKE-to-SLEEP transition source enable                              */

        uint8_t             buf_flush : 1; /*  Buffer flush enable                                                        */

    } b;
    uint8_t w;
} FXLS8974CF_BUF_CONFIG2_t;


typedef struct fxls8974cf_fifo{
	FXLS8974CF_BUF_CONFIG1_t config1;
	FXLS8974CF_BUF_CONFIG2_t config2;
}fxls8974cf_fifo_t;



typedef struct fxls8974cf_accel_config
{
	fxls8974cf_control_t control;  /*!< control bits for accel. */
	fxls8974cf_fifo_t fifosetup;   /*!< fifo setup. */
}fxls8974cf_accel_config_t;

typedef union
{
    struct
    {
        uint8_t active : 1; /*  Standby/Active mode selection                                              */

        uint8_t fsr : 2; /*  Full-scale measurement range (FSR) selection.                              */

        uint8_t spi_m : 1; /*  SPI interface mode selection; selects between 3- and 4-wire operating      */
                           /*  modes for the SPI interface.                                               */

        uint8_t st_pol : 1; /*  Self-Test Displacement Polarity                                            */

        uint8_t st_axis_sel : 2; /*  Self-Test Axis Selection                                                   */

        uint8_t rst : 1; /*  The RST bit may be used to initiate a software reset.                      */

    } b;
    uint8_t w;
} FXLS8974CF_SENS_CONFIG1_t;

/*
** SENS_CONFIG1 - Bit field mask definitions
*/
#define FXLS8974CF_SENS_CONFIG1_ACTIVE_MASK ((uint8_t)0x01)
#define FXLS8974CF_SENS_CONFIG1_ACTIVE_SHIFT ((uint8_t)0)

#define FXLS8974CF_SENS_CONFIG1_FSR_MASK ((uint8_t)0x06)
#define FXLS8974CF_SENS_CONFIG1_FSR_SHIFT ((uint8_t)1)

#define FXLS8974CF_SENS_CONFIG1_SPI_M_MASK ((uint8_t)0x08)
#define FXLS8974CF_SENS_CONFIG1_SPI_M_SHIFT ((uint8_t)3)

#define FXLS8974CF_SENS_CONFIG1_ST_POL_MASK ((uint8_t)0x10)
#define FXLS8974CF_SENS_CONFIG1_ST_POL_SHIFT ((uint8_t)4)

#define FXLS8974CF_SENS_CONFIG1_ST_AXIS_SEL_MASK ((uint8_t)0x60)
#define FXLS8974CF_SENS_CONFIG1_ST_AXIS_SEL_SHIFT ((uint8_t)5)

#define FXLS8974CF_SENS_CONFIG1_RST_MASK ((uint8_t)0x80)
#define FXLS8974CF_SENS_CONFIG1_RST_SHIFT ((uint8_t)7)

/*
** SENS_CONFIG1 - Bit field value definitions
*/
#define FXLS8974CF_SENS_CONFIG1_RST_RST ((uint8_t)0x80) /*  Trigger Reset                                   */
#define FXLS8974CF_SENS_CONFIG1_ST_AXIS_SEL_DISABLED \
    ((uint8_t)0x00)                                            /*  Self-Test function is disabled                  */
#define FXLS8974CF_SENS_CONFIG1_ST_AXIS_SEL_EN_X ((uint8_t)0x20) /*  Self-Test function is enabled for X-axis        */
#define FXLS8974CF_SENS_CONFIG1_ST_AXIS_SEL_EN_Y ((uint8_t)0x40) /*  Self-Test function is enabled for Y-axis        */
#define FXLS8974CF_SENS_CONFIG1_ST_AXIS_SEL_EN_Z ((uint8_t)0x60) /*  Self-Test function is enabled for Z-axis        */
#define FXLS8974CF_SENS_CONFIG1_ST_POL_POSITIVE ((uint8_t)0x00)  /*  Proof mass displacement for the selected axis   */
                                                               /*  is in the positive direction.                   */
#define FXLS8974CF_SENS_CONFIG1_ST_POL_NEGATIVE ((uint8_t)0x10)  /*  Proof mass displacement for the selected axis   */
                                                               /*  is in the negative direction.                   */
#define FXLS8974CF_SENS_CONFIG1_SPI_M_FOUR ((uint8_t)0x00)       /*  4-wire interface mode is selected.              */
#define FXLS8974CF_SENS_CONFIG1_SPI_M_THREE ((uint8_t)0x08)      /*  3-wire interface mode is selected.              */
#define FXLS8974CF_SENS_CONFIG1_FSR_2G ((uint8_t)0x00)           /*  ±2g; 0.98 mg/LSB (1024 LSB/g) nominal           */
                                                               /*  sensitivity.                                    */
#define FXLS8974CF_SENS_CONFIG1_FSR_4G ((uint8_t)0x02)           /*  ±4g; 1.95 mg/LSB (512 LSB/g) nominal            */
                                                               /*  sensitivity.                                    */
#define FXLS8974CF_SENS_CONFIG1_FSR_8G ((uint8_t)0x04)           /*  ±8g; 3.91 mg/LSB (256 LSB/g) nominal            */
                                                               /*  sensitivity.                                    */
#define FXLS8974CF_SENS_CONFIG1_FSR_16G ((uint8_t)0x06)          /*  ±16g; 7.81 mg/LSB (128 LSB/g) nominal           */
                                                               /*  sensitivity.                                    */
#define FXLS8974CF_SENS_CONFIG1_ACTIVE_STANDBY ((uint8_t)0x00)   /*  Standby mode.                                   */
#define FXLS8974CF_SENS_CONFIG1_ACTIVE_ACTIVE ((uint8_t)0x01)    /*  Active mode.                                    */
                                                               /*------------------------------*/


typedef union
{
    struct
    {
        uint8_t f_read : 1; /*  Fast-read mode selection.                                                  */

        uint8_t anic_temp : 1; /*  Temperature output data auto-increment control.                            */

        uint8_t _reserved_ : 1;
        uint8_t le_be : 1; /*  Little/Big-endian output mode selection.                                   */

        uint8_t sleep_pm : 2; /*  SLEEP power mode selection.                                                */

        uint8_t wake_pm : 2; /*  WAKE power mode selection.                                                 */

    } b;
    uint8_t w;
} FXLS8974CF_SENS_CONFIG2_t;

/*
** SENS_CONFIG2 - Bit field mask definitions
*/
#define FXLS8974CF_SENS_CONFIG2_F_READ_MASK ((uint8_t)0x01)
#define FXLS8974CF_SENS_CONFIG2_F_READ_SHIFT ((uint8_t)0)

#define FXLS8974CF_SENS_CONFIG2_ANIC_TEMP_MASK ((uint8_t)0x02)
#define FXLS8974CF_SENS_CONFIG2_ANIC_TEMP_SHIFT ((uint8_t)1)

#define FXLS8974CF_SENS_CONFIG2_LE_BE_MASK ((uint8_t)0x08)
#define FXLS8974CF_SENS_CONFIG2_LE_BE_SHIFT ((uint8_t)3)

#define FXLS8974CF_SENS_CONFIG2_SLEEP_PM_MASK ((uint8_t)0x30)
#define FXLS8974CF_SENS_CONFIG2_SLEEP_PM_SHIFT ((uint8_t)4)

#define FXLS8974CF_SENS_CONFIG2_WAKE_PM_MASK ((uint8_t)0xC0)
#define FXLS8974CF_SENS_CONFIG2_WAKE_PM_SHIFT ((uint8_t)6)


/*
** SENS_CONFIG2 - Bit field value definitions
*/
#define FXLS8974CF_SENS_CONFIG2_WAKE_PM_LOW_POWER ((uint8_t)0x00)  /*  Low Power mode is selected.                     */
#define FXLS8974CF_SENS_CONFIG2_WAKE_PM_HIGH_PERF ((uint8_t)0x40)  /*  High Performance Mode is selected.              */
#define FXLS8974CF_SENS_CONFIG2_WAKE_PM_FLEX_PERF ((uint8_t)0x80)  /*  Flexible Performance Mode is selected.          */
#define FXLS8974CF_SENS_CONFIG2_SLEEP_PM_LOW_POWER ((uint8_t)0x00) /*  Low Power mode is selected. */
#define FXLS8974CF_SENS_CONFIG2_SLEEP_PM_HIGH_PERF ((uint8_t)0x10) /*  High Performance Mode is selected. */
#define FXLS8974CF_SENS_CONFIG2_SLEEP_PM_FLEX_PERF ((uint8_t)0x20) /*  Flexible Performance Mode is selected. */
#define FXLS8974CF_SENS_CONFIG2_LE_BE_LE ((uint8_t)0x00)           /*  Little-endian output mode is selected.          */
#define FXLS8974CF_SENS_CONFIG2_LE_BE_BE ((uint8_t)0x08)           /*  Big-endian output mode is selected.             */
#define FXLS8974CF_SENS_CONFIG2_ANIC_TEMP_DIS ((uint8_t)0x00)      /*  TEMP_OUT register content is not included in    */
                                                                 /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG2_ANIC_TEMP_EN ((uint8_t)0x02)       /*  TEMP_OUT register content is included in        */
                                                                 /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG2_F_READ_NORMAL ((uint8_t)0x00)      /*  Normal read mode.                               */
#define FXLS8974CF_SENS_CONFIG2_F_READ_FAST ((uint8_t)0x01)        /*  Fast read mode.                                 */



/*--------------------------------
** Register: SENS_CONFIG3
** Enum: FXLS8974CF_SENS_CONFIG3
** --
** Offset : 0x17 Configuration register 3.
** ------------------------------*/
typedef union
{
    struct
    {
        uint8_t sleep_odr : 4; /*  Sleep ODR                                                                  */

        uint8_t wake_odr : 4; /*  Wake ODR                                                                   */

    } b;
    uint8_t w;
} FXLS8974CF_SENS_CONFIG3_t;

/*
** SENS_CONFIG3 - Bit field mask definitions
*/
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_MASK ((uint8_t)0x0F)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_SHIFT ((uint8_t)0)

#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_MASK ((uint8_t)0xF0)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_SHIFT ((uint8_t)4)

/*
** SENS_CONFIG3 - Bit field value definitions
*/
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_3200HZ ((uint8_t)0x00)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_1600HZ ((uint8_t)0x10)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_800HZ ((uint8_t)0x20)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_400HZ ((uint8_t)0x30)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_200HZ ((uint8_t)0x40)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_100HZ ((uint8_t)0x50)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_50HZ ((uint8_t)0x60)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_25HZ ((uint8_t)0x70)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_12_5HZ ((uint8_t)0x80)
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_6_25HZ ((uint8_t)0x90)  /*  6.25 HZ                                         */
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_3_125HZ ((uint8_t)0xa0) /*  3.125 HZ                                        */
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_1_563HZ ((uint8_t)0xb0) /*  1.563 HZ                                        */
#define FXLS8974CF_SENS_CONFIG3_WAKE_ODR_0_781HZ ((uint8_t)0xc0) /*  0.781 HZ                                        */
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_3200HZ ((uint8_t)0x00)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_1600HZ ((uint8_t)0x01)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_800HZ ((uint8_t)0x02)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_400HZ ((uint8_t)0x03)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_200HZ ((uint8_t)0x04)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_100HZ ((uint8_t)0x05)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_50HZ ((uint8_t)0x06)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_25HZ ((uint8_t)0x07)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_12_5HZ ((uint8_t)0x08)
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_6_25HZ ((uint8_t)0x09)  /*  6.25 HZ                                         */
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_3_125HZ ((uint8_t)0x0a) /*  3.125 HZ                                        */
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_1_563HZ ((uint8_t)0x0b) /*  1.563 HZ                                        */
#define FXLS8974CF_SENS_CONFIG3_SLEEP_ODR_0_781HZ ((uint8_t)0x0c) /*  0.781 HZ                                        */
                                                                /*------------------------------*/


/*--------------------------------
** Register: SENS_CONFIG5
** Enum: FXLS8974CF_SENS_CONFIG5
** --
** Offset : 0x19 Configuration register 5.
** ------------------------------*/
typedef union
{
    struct
    {
        uint8_t hibernate_en : 1; /*  Hibernate mode enable.                                                     */

        uint8_t z_dis : 1; /*  Z-axis auto-increment disable.                                             */

        uint8_t y_dis : 1; /*  Y-axis auto-increment disable.                                             */

        uint8_t x_dis : 1; /*  X-axis auto-increment disable.                                             */

        uint8_t vecm_en : 1; /*  Vector Magnitude calculation enable.                                       */

    } b;
    uint8_t w;
} FXLS8974CF_SENS_CONFIG5_t;

/*
** SENS_CONFIG5 - Bit field mask definitions
*/
#define FXLS8974CF_SENS_CONFIG5_HIBERNATE_EN_MASK ((uint8_t)0x01)
#define FXLS8974CF_SENS_CONFIG5_HIBERNATE_EN_SHIFT ((uint8_t)0)

#define FXLS8974CF_SENS_CONFIG5_Z_DIS_MASK ((uint8_t)0x02)
#define FXLS8974CF_SENS_CONFIG5_Z_DIS_SHIFT ((uint8_t)1)

#define FXLS8974CF_SENS_CONFIG5_Y_DIS_MASK ((uint8_t)0x04)
#define FXLS8974CF_SENS_CONFIG5_Y_DIS_SHIFT ((uint8_t)2)

#define FXLS8974CF_SENS_CONFIG5_X_DIS_MASK ((uint8_t)0x08)
#define FXLS8974CF_SENS_CONFIG5_X_DIS_SHIFT ((uint8_t)3)

#define FXLS8974CF_SENS_CONFIG5_VECM_EN_MASK ((uint8_t)0x10)
#define FXLS8974CF_SENS_CONFIG5_VECM_EN_SHIFT ((uint8_t)4)

/*
** SENS_CONFIG5 - Bit field value definitions
*/
#define FXLS8974CF_SENS_CONFIG5_VECM_EN_DIS ((uint8_t)0x00)      /*  12-bit vector magnitude result is not           */
                                                               /*  calculated on every ODR cycle.                  */
#define FXLS8974CF_SENS_CONFIG5_VECM_EN_EN ((uint8_t)0x10)       /*  12-bit vector magnitude result is calculated on */
                                                               /*  every ODR cycle.                                */
#define FXLS8974CF_SENS_CONFIG5_X_DIS_EN ((uint8_t)0x00)         /*  X-axis measurement is included in the           */
                                                               /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG5_X_DIS_DIS ((uint8_t)0x08)        /*  X-axis measurement is excluded from the         */
                                                               /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG5_Y_DIS_EN ((uint8_t)0x00)         /*  Y-axis measurement is included in the           */
                                                               /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG5_Y_DIS_DIS ((uint8_t)0x04)        /*  Y-axis measurement is excluded from the         */
                                                               /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG5_Z_DIS_EN ((uint8_t)0x00)         /*  Z-axis measurement is included in the           */
                                                               /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG5_Z_DIS_DIS ((uint8_t)0x02)        /*  Z-axis measurement is excluded from the         */
                                                               /*  auto-increment address range.                   */
#define FXLS8974CF_SENS_CONFIG5_HIBERNATE_EN_DIS ((uint8_t)0x00) /*  Hibernate mode not enabled.                     */
#define FXLS8974CF_SENS_CONFIG5_HIBERNATE_EN_EN ((uint8_t)0x01)  /*  Commands device to enter Hibernate mode.        */
 

typedef enum fxls8974cf_mode
{
	STANDBY                = 0,  /*!<STANDBY Mode*/
	ACTIVE                 = 1,  /*!<Active  Mode.*/
}fxls8974cf_mode_t;



typedef struct fxls8974cf_int_control
{
	uint8_t                 pp_od : 1; /*!<  - Push-Pull/Open Drain selection on interrupt pad for INT1/INT2
                                             0: Push-pull (default)
											 1: Open-drain.	*/
	uint8_t                 ipol  : 1; /*!<  - Interrupt polarity ACTIVE high, or ACTIVE low for INT1/INT2.
	                                         0: Active low (default)
											 1: Active high. */
	uint8_t                 reserved: 6;
}fxls8974cf_int_control_t;

/*--------------------------------
** Register: INT_EN
** Enum: FXLS8974CF_INT_EN
** --
** Offset : 0x20 Interrupt output enable register.
** ------------------------------*/
typedef union
{
    struct
    {
        uint8_t wake_out_en : 1; /*  WAKE power state output enable.                                            */

        uint8_t boot_dis : 1; /*  Boot interrupt output disable.                                             */

        uint8_t aslp_en : 1; /*  Auto-WAKE/SLEEP interrupt output enable.                                   */

        uint8_t orient_en : 1; /*  Orientation interrupt output enable.                                       */

        uint8_t sdcd_wt_en : 1; /*  SDCD within thresholds interrupt output enable.                            */

        uint8_t sdcd_ot_en : 1; /*  SDCD outside of thresholds interrupt output enable.                        */

        uint8_t buf_en : 1; /*  Output data buffer interrupt output enable                                        */
        uint8_t drdy_en : 1; /*  Data Ready interrupt output enable.                                        */

    } b;
    uint8_t w;
} FXLS8974CF_INT_EN_t;

/*--------------------------------
** Register: INT_PIN_SEL
** Enum: FXLS8974CF_INT_PIN_SEL
** --
** Offset : 0x21 Interrupt output pin routing register, INT1 or INT2.
** ------------------------------*/
typedef union
{
    struct
    {
        uint8_t wk_out_int2 : 1; /*  WAKE power state interrupt routing.                                        */

        uint8_t boot_int2 : 1; /*  BOOT event interrupt routing.                                              */

        uint8_t aslp_int2 : 1; /*  Auto-WAKE/SLEEP event interrupt routing.                                   */

        uint8_t orient_int2 : 1; /*  ORIENT event interrupt routing.                                            */

        uint8_t sdcd_wt_int2 : 1; /*  SDCD within thresholds event interrupt routing.                            */

        uint8_t sdcd_ot_int2 : 1; /*  SDCD outside of thresholds event interrupt routing.                        */

        uint8_t buf_int2 : 1; /* Output buffer interrupt routing. */

        uint8_t drdy_int2 : 1; /*  Data Ready interrupt routing.                                              */

    } b;
    uint8_t w;
} FXLS8974CF_INT_PIN_SEL_t;


typedef struct fxls8974cf_interrupt_config
{
	fxls8974cf_int_control_t control;


	FXLS8974CF_INT_EN_t  intSources;      /*!<  Sources to be configured.
	                                         0 to a specific source field bit -disable the interrupt
											 1 to a specific source field bit -Enable the interrupt
											 eg. int_en_ff_mt bit to zero disable the interrupt, int_en_ff_mt bit to 1 enable the interrupt. */
	FXLS8974CF_INT_PIN_SEL_t  int1_2;          /*!< INT1 or INT2 Routing configuration for specified source
	                                         0 to a bit configures interrupt for specified source to INT2 pin 
											 1 to a bit configures interrupt for specified source to INT1 pin
											 eg. int_cfg_pulse bit to '0' configures pulse interrupt to INT2, int_cfg_pulse bit to '1' configures pulse interrupt to INT1 */
											      
}fxls8974cf_interrupt_config_t;


/*! @brief This structure defines variables to compute self-test output change (STOC) and self-test offset (STOF). */
typedef struct fxls8974cf_selftest
{
	int16_t x_stoc;
	int16_t y_stoc;
	int16_t z_stoc;
	int16_t x_stof;
	int16_t y_stof;
	int16_t z_stof;
	// fxls8974cf_selftest_range_t valid_range;
	uint8_t fsr;
  	uint8_t selftest_complete;
} fxls8974cf_selftest_t;

typedef struct fxls8974cf_selftest_range
{
	int16_t min_stoc[NUM_AXES];
	int16_t max_stoc[NUM_AXES];
	int16_t min_stof[NUM_AXES];
	int16_t max_stof[NUM_AXES];
} fxls8974cf_selftest_range_t;


#endif /* ZEPHYR_DRIVERS_SENSOR_FXLS8974CF_FXLS8974CF_H_ */
