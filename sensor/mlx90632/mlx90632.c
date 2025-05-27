 #define DT_DRV_COMPAT melexis_mlx90632


#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#include "mlx90632.h"

#define POW10 10000000000LL

#ifndef VERSION
#define VERSION "test"
#endif

static const char mlx90632version[] __attribute__((used)) = { VERSION };

#ifndef STATIC
#define STATIC static
#endif

//........................................................
int16_t ambient_new_raw_dbg1;
int16_t ambient_old_raw_dbg1;
int16_t object_new_raw_dbg1;
int16_t object_old_raw_dbg1;

int16_t ambient_new_raw_dbg2;
int16_t ambient_old_raw_dbg2;
uint32_t P_T_dbg2;
uint32_t P_R_dbg2;
uint32_t P_G_dbg2;
uint32_t P_O_dbg2;
uint32_t Gb_dbg2;
double ambient_dbg2;

int16_t ambient_new_raw_dbg3;
int16_t ambient_old_raw_dbg3;
uint32_t Gb_dbg3;

int16_t object_new_raw_dbg4;
int16_t object_old_raw_dbg4;
int16_t ambient_new_raw_dbg4;
int16_t ambient_old_raw_dbg4;
uint32_t Ka_dbg4;

double pre_object_dbg5;
double pre_ambient_dbg5;
uint32_t Ea_dbg5;
uint32_t Eb_dbg5;
uint32_t Ga_dbg5;
uint32_t Fa_dbg5;
uint32_t Fb_dbg5;
uint32_t Ha_dbg5;
uint32_t Hb_dbg5;
double object_dbg5;
//........................................................


int32_t mlx90632_trigger_measurement(const struct device *dev)
{
    uint16_t reg_status;
    int32_t ret;

    ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);
    if (ret < 0)
        return ret;

    ret = mlx90632_i2c_write(dev, MLX90632_REG_STATUS, reg_status & (~MLX90632_STAT_DATA_RDY));

    return ret;
}

int32_t mlx90632_wait_for_measurement(const struct device *dev)
{
    int tries = MLX90632_MAX_NUMBER_MESUREMENT_READ_TRIES;
    uint16_t reg_status;
    int32_t ret;

    while (tries-- > 0)
    {
        ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);
        if (ret < 0)
            return ret;
        if (reg_status & MLX90632_STAT_DATA_RDY)
            break;
        /* minimum wait time to complete measurement
         * should be calculated according to refresh rate
         * atm 10ms - 11ms
         */
        usleep(10000, 11000);
    }

    if (tries < 0)
    {
        // data not ready
        return -ETIMEDOUT;
    }

    return (reg_status & MLX90632_STAT_CYCLE_POS) >> 2;
}

int32_t mlx90632_start_measurement(const struct device *dev)
{
    int32_t ret = mlx90632_trigger_measurement(dev);

    if (ret < 0)
        return ret;

    ret = mlx90632_wait_for_measurement(dev);

    return ret;
}

/** Based on @link mlx90632_start_measurement @endlink return value fill channel_new and channel_old
 * variables with proper values. This is to provide a bit more flexibility in case other channels are
 * returned and need a bit more mingeling than usual maths can provide.
 *
 * So far there are just two use cases. If returned value is not 1 or 2, it will leave channel_new
 * and channel_old unassigned.
 *
 * @param[in] ret @link mlx90632_start_measurement @endlink return value
 * @param[out] *channel_new Pointer to memory location where new channel value is stored
 * @param[out] *channel_old Pointer to memory location where old channel value is stored
 *
 * @retval 0 When both memory locations are updated as per ret
 * @retval -EINVAL channel_new and channel_old were not updated
 */
STATIC int32_t mlx90632_channel_new_select(const struct device *dev, int32_t ret, uint8_t *channel_new, uint8_t *channel_old)
{
    switch (ret)
    {
        case 1:
            *channel_new = 1;
            *channel_old = 2;
            break;

        case 2:
            *channel_new = 2;
            *channel_old = 1;
            break;

        default:
            return -EINVAL;
    }
    return 0;
}

/** Read ambient raw old and new values based on @link mlx90632_start_measurement @endlink return value.
 *
 * Two i2c_reads are needed to obtain necessary raw ambient values from the sensor, as they are then
 * preprocessed before going to calculation functions. Because one value is newer than other (see @link
 * mlx90632_start_measurement @endlink) this function provides dynamics based on return value of the
 * @link mlx90632_start_measurement @endlink.
 *
 * @param[out] *ambient_new_raw Pointer to memory location where new ambient value from sensor is stored
 * @param[out] *ambient_old_raw Pointer to memory location where old ambient value from sensor is stored
 *
 * @retval 0 Successfully read both values
 * @retval <0 Something went wrong. Check errno.h for more details.
 */
STATIC int32_t mlx90632_read_temp_ambient_raw(const struct device *dev, int16_t *ambient_new_raw, int16_t *ambient_old_raw)
{
    int32_t ret;
    uint16_t read_tmp;

    ret = mlx90632_i2c_read(dev, MLX90632_RAM_3(1), &read_tmp);
    if (ret < 0)
        return ret;
    *ambient_new_raw = (int16_t)read_tmp;

    ret = mlx90632_i2c_read(dev, MLX90632_RAM_3(2), &read_tmp);
    if (ret < 0)
        return ret;
    *ambient_old_raw = (int16_t)read_tmp;

    return ret;
}

/** Read object raw old and new values based on @link mlx90632_start_measurement @endlink return value.
 *
 * Four i2c_reads are needed to obtain necessary raw object values from the sensor. These values are grouped per new
 * and old and then averaged before return of the function. After that they are then preprocessed before going to
 * calculation functions. Because one value is newer than other (see @link mlx90632_start_measurement @endlink) this
 * function provides dynamics based on return value of the @link mlx90632_start_measurement @endlink.
 *
 * @param[in] channel_position Channel position where new (recently updated) measurement can be found,
 *                             usually return value of @link mlx90632_start_measurement @endlink or
 *                             @link mlx90632_wait_for_measurement @endlink or @link
 *                             mlx90632_get_channel_position @endlink in case of continuous mode,
 *                             2 in case of burst mode, and return value of @link
 *                             mlx90632_wait_for_measurement @endlink or @link
 *                             mlx90632_get_channel_position @endlink in case of single mode
 * @param[out] *object_new_raw Pointer to memory location where average of new object values from sensor is stored
 * @param[out] *object_old_raw Pointer to memory location where average of old object values from sensor is stored
 *
 * @retval 0 Successfully read both values
 * @retval <0 Something went wrong. Check errno.h for more details.
 */
STATIC int32_t mlx90632_read_temp_object_raw(const struct device *dev, int32_t channel_position,
                                             int16_t *object_new_raw, int16_t *object_old_raw)
{
    int32_t ret;
    uint16_t read_tmp;
    int16_t read;
    uint8_t channel, channel_old;

    ret = mlx90632_channel_new_select(dev, channel_position, &channel, &channel_old);
    if (ret != 0)
        return -EINVAL;

    ret = mlx90632_i2c_read(dev, MLX90632_RAM_2(channel), &read_tmp);
    if (ret < 0)
        return ret;

    read = (int16_t)read_tmp;

    ret = mlx90632_i2c_read(dev, MLX90632_RAM_1(channel), &read_tmp);
    if (ret < 0)
        return ret;
    *object_new_raw = (read + (int16_t)read_tmp) / 2;

    ret = mlx90632_i2c_read(dev, MLX90632_RAM_2(channel_old), &read_tmp);
    if (ret < 0)
        return ret;
    read = (int16_t)read_tmp;

    ret = mlx90632_i2c_read(dev, MLX90632_RAM_1(channel_old), &read_tmp);
    if (ret < 0)
        return ret;
    *object_old_raw = (read + (int16_t)read_tmp) / 2;

    return ret;
}

int32_t mlx90632_read_temp_raw_wo_wait(const struct device *dev, int32_t channel_position,
                                       int16_t *ambient_new_raw, int16_t *ambient_old_raw,
                                       int16_t *object_new_raw, int16_t *object_old_raw)
{
    /** Read new and old **ambient** values from sensor */
    int32_t ret = mlx90632_read_temp_ambient_raw(dev, ambient_new_raw, ambient_old_raw);

    if (ret < 0)
        return ret;

    /** Read new and old **object** values from sensor */
    ret = mlx90632_read_temp_object_raw(dev, channel_position, object_new_raw, object_old_raw);

    return ret;
}

int32_t mlx90632_read_temp_raw(const struct device *dev, int16_t *ambient_new_raw, int16_t *ambient_old_raw,
                               int16_t *object_new_raw, int16_t *object_old_raw)
{
    // trigger and wait for measurement to complete
    int32_t start_measurement_ret = mlx90632_start_measurement(dev);

    if (start_measurement_ret < 0)
        return start_measurement_ret;

    /** Read raw ambient and object temperature */
    return mlx90632_read_temp_raw_wo_wait(dev, start_measurement_ret, ambient_new_raw, ambient_old_raw,
                                          object_new_raw, object_old_raw);
}

int32_t mlx90632_read_temp_raw_burst(const struct device *dev, int16_t *ambient_new_raw, int16_t *ambient_old_raw,
                                     int16_t *object_new_raw, int16_t *object_old_raw)
{
    int32_t start_measurement_ret;

    // trigger and wait for measurement to complete
    start_measurement_ret = mlx90632_start_measurement_burst(dev);
    if (start_measurement_ret < 0)
        return start_measurement_ret;

    /** Read raw ambient and object temperature */
    return mlx90632_read_temp_raw_wo_wait(dev, 2, ambient_new_raw, ambient_old_raw,
                                          object_new_raw, object_old_raw);
}

// no read
/* DSPv5 */
double mlx90632_preprocess_temp_ambient(int16_t ambient_new_raw, int16_t ambient_old_raw, int16_t Gb)
{
    double VR_Ta, kGb;

    kGb = ((double)Gb) / 1024.0;

    VR_Ta = ambient_old_raw + kGb * (ambient_new_raw / (MLX90632_REF_3));
    return ((ambient_new_raw / (MLX90632_REF_3)) / VR_Ta) * 524288.0;
}
// no read
double mlx90632_preprocess_temp_object(int16_t object_new_raw, int16_t object_old_raw,
                                       int16_t ambient_new_raw, int16_t ambient_old_raw,
                                       int16_t Ka)
{
    double VR_IR, kKa;

    kKa = ((double)Ka) / 1024.0;

    VR_IR = ambient_old_raw + kKa * (ambient_new_raw / (MLX90632_REF_3));
    return ((((object_new_raw + object_old_raw) / 2) / (MLX90632_REF_12)) / VR_IR) * 524288.0;
}

// no read
double mlx90632_calc_temp_ambient(int16_t ambient_new_raw, int16_t ambient_old_raw, int32_t P_T,
                                  int32_t P_R, int32_t P_G, int32_t P_O, int16_t Gb)
{
    double Asub, Bsub, Ablock, Bblock, Cblock, AMB;

    AMB = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);

    Asub = ((double)P_T) / (double)17592186044416.0;
    Bsub = (double)AMB - ((double)P_R / (double)256.0);
    Ablock = Asub * (Bsub * Bsub);
    Bblock = (Bsub / (double)P_G) * (double)1048576.0;
    Cblock = (double)P_O / (double)256.0;

    return Bblock + Ablock + Cblock;
}

// no read
/** Iterative calculation of object temperature
 *
 * DSPv5 requires 3 iterations to reduce noise for object temperature. Since
 * each iteration requires same calculations this helper function is
 * implemented.
 *
 * @param[in] prev_object_temp previously calculated object temperature. If
 *                              there is no previously calculated temperature
 *                              input 25.0
 * @param[in] object object temperature from @link mlx90632_preprocess_temp_object @endlink
 * @param[in] TAdut ambient temperature coefficient
 * @param[in] Ga Register value on @link MLX90632_EE_Ga @endlink
 * @param[in] Fa Register value on @link MLX90632_EE_Fa @endlink
 * @param[in] Fb Register value on @link MLX90632_EE_Fb @endlink
 * @param[in] Ha Register value on @link MLX90632_EE_Ha @endlink
 * @param[in] Hb Register value on @link MLX90632_EE_Hb @endlink
 * @param[in] emissivity Value provided by user of the object emissivity
 *
 * @return Calculated object temperature for current iteration in milliCelsius
 */
STATIC double mlx90632_calc_temp_object_iteration(double prev_object_temp, int32_t object, double TAdut,
                                                  int32_t Ga, int32_t Fa, int32_t Fb, int16_t Ha, int16_t Hb,
                                                  double emissivity)
{
    double calcedGa, calcedGb, calcedFa, TAdut4, first_sqrt;
    // temp variables
    double KsTAtmp, Alpha_corr;
    double Ha_customer, Hb_customer;


    Ha_customer = Ha / ((double)16384.0);
    Hb_customer = Hb / ((double)1024.0);
    calcedGa = ((double)Ga * (prev_object_temp - 25)) / ((double)68719476736.0);
    KsTAtmp = (double)Fb * (TAdut - 25);
    calcedGb = KsTAtmp / ((double)68719476736.0);
    Alpha_corr = (((double)(Fa * POW10)) * Ha_customer * (double)(1 + calcedGa + calcedGb)) /
                 ((double)70368744177664.0);
    calcedFa = object / (emissivity * (Alpha_corr / POW10));
    TAdut4 = (TAdut + 273.15) * (TAdut + 273.15) * (TAdut + 273.15) * (TAdut + 273.15);

    first_sqrt = sqrt(calcedFa + TAdut4);

    return sqrt(first_sqrt) - 273.15 - Hb_customer;
}
// no read
/** Iterative calculation of object temperature  when the environment temperature differs from the sensor temperature
 *
 * DSPv5 requires 3 iterations to reduce noise for object temperature. Since
 * each iteration requires same calculations this helper function is
 * implemented.
 *
 * @param[in] prev_object_temp previously calculated object temperature. If
 *                              there is no previously calculated temperature
 *                              input 25.0
 * @param[in] object object temperature from @link mlx90632_preprocess_temp_object @endlink
 * @param[in] TAdut ambient temperature coefficient
 * @param[in] TaTr4 compensation coefficient for reflected (environment) temperature. The compensation
 *                  coefficient is calculated from ambient temperature either from a sensor different than the MLX90632 or
 *                  acquired by other means.
 * @param[in] Ga Register value on @link MLX90632_EE_Ga @endlink
 * @param[in] Fa Register value on @link MLX90632_EE_Fa @endlink
 * @param[in] Fb Register value on @link MLX90632_EE_Fb @endlink
 * @param[in] Ha Register value on @link MLX90632_EE_Ha @endlink
 * @param[in] Hb Register value on @link MLX90632_EE_Hb @endlink
 * @param[in] emissivity Value provided by user of the object emissivity
 *
 * @return Calculated object temperature for current iteration in milliCelsius
 */
STATIC double mlx90632_calc_temp_object_iteration_reflected(double prev_object_temp, int32_t object, double TAdut, double TaTr4,
                                                            int32_t Ga, int32_t Fa, int32_t Fb, int16_t Ha, int16_t Hb,
                                                            double emissivity)
{
    double calcedGa, calcedGb, calcedFa, first_sqrt;
    // temp variables
    double KsTAtmp, Alpha_corr;
    double Ha_customer, Hb_customer;

    Ha_customer = Ha / ((double)16384.0);
    Hb_customer = Hb / ((double)1024.0);
    calcedGa = ((double)Ga * (prev_object_temp - 25)) / ((double)68719476736.0);
    KsTAtmp = (double)Fb * (TAdut - 25);
    calcedGb = KsTAtmp / ((double)68719476736.0);
    Alpha_corr = (((double)(Fa * POW10)) * Ha_customer * (double)(1 + calcedGa + calcedGb)) /
                 ((double)70368744177664.0);
    calcedFa = object / (emissivity * (Alpha_corr / POW10));

    first_sqrt = sqrt(calcedFa + TaTr4);

    return sqrt(first_sqrt) - 273.15 - Hb_customer;
}

static double emissivity = 0.0;
// no read
void mlx90632_set_emissivity(double value)
{
    emissivity = value;
}
// no read
double mlx90632_get_emissivity(void)
{
    if (emissivity == 0.0)
    {
        return 1.0;
    }
    else
    {
        return emissivity;
    }
}
// no read
double mlx90632_calc_temp_object(int32_t object, int32_t ambient,
                                 int32_t Ea, int32_t Eb, int32_t Ga, int32_t Fa, int32_t Fb,
                                 int16_t Ha, int16_t Hb)
{
    double kEa, kEb, TAdut;
    double temp = 25.0;
    double tmp_emi = mlx90632_get_emissivity();
    int8_t i;

    kEa = ((double)Ea) / ((double)65536.0);
    kEb = ((double)Eb) / ((double)256.0);
    TAdut = (((double)ambient) - kEb) / kEa + 25;

    //iterate through calculations
    for (i = 0; i < 5; ++i)
    {
        temp = mlx90632_calc_temp_object_iteration(temp, object, TAdut, Ga, Fa, Fb, Ha, Hb, tmp_emi);
    }
    return temp;
}
// no read
double mlx90632_calc_temp_object_reflected(int32_t object, int32_t ambient, double reflected,
                                           int32_t Ea, int32_t Eb, int32_t Ga, int32_t Fa, int32_t Fb,
                                           int16_t Ha, int16_t Hb)
{
    double kEa, kEb, TAdut;
    double temp = 25.0;
    double tmp_emi = mlx90632_get_emissivity();
    double TaTr4;
    double ta4;
    int8_t i;

    kEa = ((double)Ea) / ((double)65536.0);
    kEb = ((double)Eb) / ((double)256.0);
    TAdut = (((double)ambient) - kEb) / kEa + 25;

    TaTr4 = reflected + 273.15;
    TaTr4 = TaTr4 * TaTr4;
    TaTr4 = TaTr4 * TaTr4;
    ta4 = TAdut + 273.15;
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    TaTr4 = TaTr4 - (TaTr4 - ta4) / tmp_emi;

    //iterate through calculations
    for (i = 0; i < 5; ++i)
    {
        temp = mlx90632_calc_temp_object_iteration_reflected(temp, object, TAdut, TaTr4, Ga, Fa, Fb, Ha, Hb, tmp_emi);
    }
    return temp;
}

int32_t mlx90632_init(const struct device *dev)
{
    int32_t ret;
    uint16_t eeprom_version, reg_status;
    struct mlx90632_data *cal_data = dev->data;
    const struct mlx90632_config *cfg = dev->config;

    ret = mlx90632_i2c_read(dev, MLX90632_EE_VERSION, &eeprom_version);
    if (ret < 0) {
        printk("MLX90632: fail to read eeprom\n");
        return ret;
    }

    if ((eeprom_version & 0x00FF) != MLX90632_DSPv5) {
        // this here can fail because of big/little endian of cpu/i2c
        printk("MLX90632: eeprom versionm fail\n");
        return -EPROTONOSUPPORT;
    }

    ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);
    if (ret < 0) {
        printk("MLX90632: fail to read status\n");
        return ret;
    }

    // Prepare a clean start with setting NEW_DATA to 0
    ret = mlx90632_i2c_write(dev, MLX90632_REG_STATUS, reg_status & ~(MLX90632_STAT_DATA_RDY));
    if (ret < 0) {
        printk("MLX90632: fail to update status\n");
        return ret;
    }

    /*
    // Load full calibration block
    uint8_t write_buff[2] = {
        (uint8_t)(MLX90632_EE_P_R >> 8),
        (uint8_t)(MLX90632_EE_P_R & 0x00FF)
    };
    uint8_t read_buf[76];

    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 76);
    if (ret < 0) {
        printk("MLX90632: EEPROM read failed: %d\n", ret);
        return ret;
    }

    cal_data->P_R = (read_buf[0] << 24) | (read_buf[1] << 16) | (read_buf[2] << 8) | read_buf[3];
    cal_data->P_G = (read_buf[4] << 24) | (read_buf[5] << 16) | (read_buf[6] << 8) | read_buf[7];
    cal_data->P_T = (read_buf[8] << 24) | (read_buf[9] << 16) | (read_buf[10] << 8) | read_buf[11];
    cal_data->P_O = (read_buf[12] << 24) | (read_buf[13] << 16) | (read_buf[14] << 8) | read_buf[15];
    cal_data->Aa  = (read_buf[16] << 24) | (read_buf[17] << 16) | (read_buf[18] << 8) | read_buf[19];
    cal_data->Ab  = (read_buf[20] << 24) | (read_buf[21] << 16) | (read_buf[22] << 8) | read_buf[23];
    cal_data->Ba  = (read_buf[24] << 24) | (read_buf[25] << 16) | (read_buf[26] << 8) | read_buf[27];
    cal_data->Bb  = (read_buf[28] << 24) | (read_buf[29] << 16) | (read_buf[30] << 8) | read_buf[31];
    cal_data->Ca  = (read_buf[32] << 24) | (read_buf[33] << 16) | (read_buf[34] << 8) | read_buf[35];
    cal_data->Cb  = (read_buf[36] << 24) | (read_buf[37] << 16) | (read_buf[38] << 8) | read_buf[39];
    cal_data->Da  = (read_buf[40] << 24) | (read_buf[41] << 16) | (read_buf[42] << 8) | read_buf[43];
    cal_data->Db  = (read_buf[44] << 24) | (read_buf[45] << 16) | (read_buf[46] << 8) | read_buf[47];
    cal_data->Ea  = (read_buf[48] << 24) | (read_buf[49] << 16) | (read_buf[50] << 8) | read_buf[51];
    cal_data->Eb  = (read_buf[52] << 24) | (read_buf[53] << 16) | (read_buf[54] << 8) | read_buf[55];
    cal_data->Fa  = (read_buf[56] << 24) | (read_buf[57] << 16) | (read_buf[58] << 8) | read_buf[59];
    cal_data->Fb  = (read_buf[60] << 24) | (read_buf[61] << 16) | (read_buf[62] << 8) | read_buf[63];
    cal_data->Ga  = (read_buf[64] << 24) | (read_buf[65] << 16) | (read_buf[66] << 8) | read_buf[67];
    cal_data->Gb  = (read_buf[68] << 8)  | read_buf[69];
    cal_data->Ka  = (read_buf[70] << 8)  | read_buf[71];
    cal_data->Ha  = (read_buf[72] << 8)  | read_buf[73];
    cal_data->Hb  = (read_buf[74] << 8)  | read_buf[75];
    */

    /*
    uint8_t read_buf[4];
    uint8_t write_buff[2] ;

    write_buff[0] = (uint8_t)(MLX90632_EE_P_R >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_P_R & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_P_R read failed: %d\n", ret);
        return ret;
    }
    cal_data->P_R = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_P_G >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_P_G & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_P_G read failed: %d\n", ret);
        return ret;
    }
    cal_data->P_G = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_P_T >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_P_T & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_P_T read failed: %d\n", ret);
        return ret;
    }
    cal_data->P_T = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_P_O >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_P_O & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_P_O read failed: %d\n", ret);
        return ret;
    }
    cal_data->P_O = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Aa >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Aa & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Aa read failed: %d\n", ret);
        return ret;
    }
    cal_data->Aa = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ab >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ab & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ab read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ab = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ba >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ba & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ba read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ba = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Bb >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Bb & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Bb read failed: %d\n", ret);
        return ret;
    }
    cal_data->Bb = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ca >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ca & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ca read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ca = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Cb >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Cb & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Cb read failed: %d\n", ret);
        return ret;
    }
    cal_data->Cb = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Da >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Da & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Da read failed: %d\n", ret);
        return ret;
    }
    cal_data->Da = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Db >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Db & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Db read failed: %d\n", ret);
        return ret;
    }
    cal_data->Db = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ea >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ea & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ea read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ea = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Eb >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Eb & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Eb read failed: %d\n", ret);
        return ret;
    }
    cal_data->Eb = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Fa >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Fa & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Fa read failed: %d\n", ret);
        return ret;
    }
    cal_data->Fa = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Fb >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Fb & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Fb read failed: %d\n", ret);
        return ret;
    }
    cal_data->Fb = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ga >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ga & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 4);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ga read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ga = (read_buf[2] << 24) | (read_buf[3] << 16) |
                    (read_buf[0] << 8)  |  read_buf[1];


    write_buff[0] = (uint8_t)(MLX90632_EE_Gb >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Gb & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 2);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Gb read failed: %d\n", ret);
        return ret;
    }
    cal_data->Gb = (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ka >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ka & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 2);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ka read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ka = (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Ha >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Ha & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 2);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Ha read failed: %d\n", ret);
        return ret;
    }
    cal_data->Ha = (read_buf[0] << 8)  |  read_buf[1];

    write_buff[0] = (uint8_t)(MLX90632_EE_Hb >> 8);
    write_buff[1] = (uint8_t)(MLX90632_EE_Hb & 0x00FF);
    ret = i2c_write_read_dt(&cfg->i2c, write_buff, 2, read_buf, 2);
    if (ret < 0) {
        printk("MLX90632: MLX90632_EE_Hb read failed: %d\n", ret);
        return ret;
    }
    cal_data->Hb = (read_buf[0] << 8)  |  read_buf[1];
    */


    
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_P_R, &cal_data->P_R);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_P_G, &cal_data->P_G);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_P_T, &cal_data->P_T);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_P_O, &cal_data->P_O);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Aa, &cal_data->Aa);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Ab, &cal_data->Ab);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Ba, &cal_data->Ba);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Bb, &cal_data->Bb);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Ca, &cal_data->Ca);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Cb, &cal_data->Cb);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Da, &cal_data->Da);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Db, &cal_data->Db);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Ea, &cal_data->Ea);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Eb, &cal_data->Eb);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Fa, &cal_data->Fa);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Fb, &cal_data->Fb);
    ret = mlx90632_i2c_read32(dev, MLX90632_EE_Ga, &cal_data->Ga);
    ret = mlx90632_i2c_read(dev, MLX90632_EE_Gb, &cal_data->Gb); 
    ret = mlx90632_i2c_read(dev, MLX90632_EE_Ka, &cal_data->Ka);
    ret = mlx90632_i2c_read(dev, MLX90632_EE_Ha, &cal_data->Ha);
    ret = mlx90632_i2c_read(dev, MLX90632_EE_Hb, &cal_data->Hb);
    

    // !dbg! - start
    /*
    printk("P_R: %u\n", cal_data->P_R);
    printk("P_G: %u\n", cal_data->P_G);
    printk("P_T: %u\n", cal_data->P_T);
    printk("P_O: %u\n", cal_data->P_O);
    printk("Aa: %u\n", cal_data->Aa);
    printk("Ab: %u\n", cal_data->Ab);
    printk("Ba: %u\n", cal_data->Ba);
    printk("Bb: %u\n", cal_data->Bb);
    printk("Ca: %u\n", cal_data->Ca);
    printk("Cb: %u\n", cal_data->Cb);
    printk("Da: %u\n", cal_data->Da);
    printk("Db: %u\n", cal_data->Db);
    printk("Ea: %u\n", cal_data->Ea);
    printk("Eb: %u\n", cal_data->Eb);
    printk("Fa: %u\n", cal_data->Fa);
    printk("Fb: %u\n", cal_data->Fb);
    printk("Ga: %u\n", cal_data->Ga);
    printk("Gb: %u\n", cal_data->Gb);
    printk("Ka: %u\n", cal_data->Ka);
    printk("Ha: %u\n", cal_data->Ha);
    printk("Hb: %u\n", cal_data->Hb);
    */
    printk("P_R: 0x%08x\n", cal_data->P_R);
    printk("P_G: 0x%08x\n", cal_data->P_G);
    printk("P_T: 0x%08x\n", cal_data->P_T);
    printk("P_O: 0x%08x\n", cal_data->P_O);
    printk("Aa: 0x%08x\n", cal_data->Aa);
    printk("Ab: 0x%08x\n", cal_data->Ab);
    printk("Ba: 0x%08x\n", cal_data->Ba);
    printk("Bb: 0x%08x\n", cal_data->Bb);
    printk("Ca: 0x%08x\n", cal_data->Ca);
    printk("Cb: 0x%08x\n", cal_data->Cb);
    printk("Da: 0x%08x\n", cal_data->Da);
    printk("Db: 0x%08x\n", cal_data->Db);
    printk("Ea: 0x%08x\n", cal_data->Ea);
    printk("Eb: 0x%08x\n", cal_data->Eb);
    printk("Fa: 0x%08x\n", cal_data->Fa);
    printk("Fb: 0x%08x\n", cal_data->Fb);
    printk("Ga: 0x%08x\n", cal_data->Ga);
    printk("Gb: 0x%08x\n", cal_data->Gb);
    printk("Ka: 0x%08x\n", cal_data->Ka);
    printk("Ha: 0x%08x\n", cal_data->Ha);
    printk("Hb: 0x%08x\n", cal_data->Hb);

    // !dbg! - end


    // !gb! Added here, as taken from Niall's working project
    mlx90632_set_emissivity(0.98f);

    // !gb! moved here
    if ((eeprom_version & 0x7F00) == MLX90632_XTD_RNG_KEY) {
        return ERANGE;
    }

    return 0;
}

int32_t mlx90632_addressed_reset(const struct device *dev)
{
    int32_t ret;
    uint16_t reg_ctrl;
    uint16_t reg_value;

    ret = mlx90632_i2c_read(dev, MLX90632_REG_CTRL, &reg_value);
    if (ret < 0)
        return ret;

    reg_ctrl = reg_value & ~MLX90632_CFG_PWR_MASK;
    reg_ctrl |= MLX90632_PWR_STATUS_STEP;
    ret = mlx90632_i2c_write(dev, MLX90632_REG_CTRL, reg_ctrl);
    if (ret < 0)
        return ret;

    ret = mlx90632_i2c_write(dev, 0x3005, MLX90632_RESET_CMD);
    if (ret < 0)
        return ret;

    usleep(150, 200);

    ret = mlx90632_i2c_write(dev, MLX90632_REG_CTRL, reg_value);

    return ret;
}

int32_t mlx90632_get_measurement_time(const struct device *dev, uint16_t meas)
{
    int32_t ret;
    uint16_t reg;

    ret = mlx90632_i2c_read(dev, meas, &reg);
    if (ret < 0)
        return ret;

    reg &= MLX90632_EE_REFRESH_RATE_MASK;
    reg = reg >> 8;

    return MLX90632_MEAS_MAX_TIME >> reg;
}

int32_t mlx90632_calculate_dataset_ready_time(const struct device *dev)
{
    int32_t ret;
    int32_t refresh_time;

    ret = mlx90632_get_meas_type(dev);
    if (ret < 0)
        return ret;

    if ((ret != MLX90632_MTYP_MEDICAL_BURST) && (ret != MLX90632_MTYP_EXTENDED_BURST))
        return -EINVAL;

    if (ret == MLX90632_MTYP_MEDICAL_BURST)
    {
        ret = mlx90632_get_measurement_time(dev, MLX90632_EE_MEDICAL_MEAS1);
        if (ret < 0)
            return ret;

        refresh_time = ret;

        ret = mlx90632_get_measurement_time(dev, MLX90632_EE_MEDICAL_MEAS2);
        if (ret < 0)
            return ret;

        refresh_time = refresh_time + ret;
    }
    else
    {
        ret = mlx90632_get_measurement_time(dev, MLX90632_EE_EXTENDED_MEAS1);
        if (ret < 0)
            return ret;

        refresh_time = ret;

        ret = mlx90632_get_measurement_time(dev, MLX90632_EE_EXTENDED_MEAS2);
        if (ret < 0)
            return ret;

        refresh_time = refresh_time + ret;

        ret = mlx90632_get_measurement_time(dev, MLX90632_EE_EXTENDED_MEAS3);
        if (ret < 0)
            return ret;

        refresh_time = refresh_time + ret;
    }

    return refresh_time;
}

int32_t mlx90632_trigger_measurement_burst(const struct device *dev)
{
    uint16_t reg;
    int32_t ret;

    ret = mlx90632_i2c_read(dev, MLX90632_REG_CTRL, &reg);
    if (ret < 0)
        return ret;

    reg |= MLX90632_START_BURST_MEAS;

    ret = mlx90632_i2c_write(dev, MLX90632_REG_CTRL, reg);

    return ret;
}

int32_t mlx90632_wait_for_measurement_burst(const struct device *dev)
{
    int tries = MLX90632_MAX_NUMBER_MESUREMENT_READ_TRIES;
    uint16_t reg_status;
    int32_t ret;

    while (tries-- > 0)
    {
        ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);
        if (ret < 0)
            return ret;
        if ((reg_status & MLX90632_STAT_BUSY) == 0)
            break;
        /* minimum wait time to complete measurement
         * should be calculated according to refresh rate
         * atm 10ms - 11ms
         */
        usleep(10000, 11000);
    }

    if (tries < 0)
    {
        // data not ready
        return -ETIMEDOUT;
    }

    return 0;
}

int32_t mlx90632_start_measurement_burst(const struct device *dev)
{
    int32_t ret = mlx90632_trigger_measurement_burst(dev);

    if (ret < 0)
        return ret;

    ret = mlx90632_calculate_dataset_ready_time(dev);
    if (ret < 0)
        return ret;
    msleep(ret); /* Waiting for refresh of all the measurement tables */

    ret = mlx90632_wait_for_measurement_burst(dev);

    return ret;
}

int32_t mlx90632_trigger_measurement_single(const struct device *dev)
{
    uint16_t reg;
    int32_t ret;

    // Clear NEW_DATA flag
    ret = mlx90632_trigger_measurement(dev);
    if (ret < 0)
        return ret;

    ret = mlx90632_i2c_read(dev, MLX90632_REG_CTRL, &reg);
    if (ret < 0)
        return ret;

    reg |= MLX90632_START_SINGLE_MEAS;

    ret = mlx90632_i2c_write(dev, MLX90632_REG_CTRL, reg);

    return ret;
}

STATIC int32_t mlx90632_unlock_eeprom(const struct device *dev)
{
    return mlx90632_i2c_write(dev, 0x3005, MLX90632_EEPROM_WRITE_KEY);
}

STATIC int32_t mlx90632_wait_for_eeprom_not_busy(const struct device *dev)
{
    uint16_t reg_status;
    int32_t ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);

    while (ret >= 0 && reg_status & MLX90632_STAT_EE_BUSY)
    {
        ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);
    }

    return ret;
}

STATIC int32_t mlx90632_erase_eeprom(const struct device *dev, uint16_t address)
{
    int32_t ret = mlx90632_unlock_eeprom(dev);

    if (ret < 0)
        return ret;

    ret = mlx90632_i2c_write(dev, address, 0x00);
    if (ret < 0)
        return ret;

    ret = mlx90632_wait_for_eeprom_not_busy(dev);
    return ret;
}

STATIC int32_t mlx90632_write_eeprom(const struct device *dev, uint16_t address, uint16_t data)
{
    int32_t ret = mlx90632_erase_eeprom(dev, address);

    if (ret < 0)
        return ret;

    ret = mlx90632_unlock_eeprom(dev);
    if (ret < 0)
        return ret;

    ret = mlx90632_i2c_write(dev, address, data);
    if (ret < 0)
        return ret;

    ret = mlx90632_wait_for_eeprom_not_busy(dev);
    return ret;
}

int32_t mlx90632_set_refresh_rate(const struct device *dev, mlx90632_meas_t measRate)
{
    uint16_t meas1, meas2;

    int32_t ret = mlx90632_i2c_read(dev, MLX90632_EE_MEDICAL_MEAS1, &meas1);

    if (ret < 0)
        return ret;

    uint16_t new_value = MLX90632_NEW_REG_VALUE(meas1, measRate, MLX90632_EE_REFRESH_RATE_START, MLX90632_EE_REFRESH_RATE_SHIFT);

    if (meas1 != new_value)
    {
        ret = mlx90632_write_eeprom(dev, MLX90632_EE_MEDICAL_MEAS1, new_value);
        if (ret < 0)
            return ret;
    }

    ret = mlx90632_i2c_read(dev, MLX90632_EE_MEDICAL_MEAS2, &meas2);
    if (ret < 0)
        return ret;

    new_value = MLX90632_NEW_REG_VALUE(meas2, measRate, MLX90632_EE_REFRESH_RATE_START, MLX90632_EE_REFRESH_RATE_SHIFT);
    if (meas2 != new_value)
    {
        ret = mlx90632_write_eeprom(dev, MLX90632_EE_MEDICAL_MEAS2, new_value);
    }

    return ret;
}

mlx90632_meas_t mlx90632_get_refresh_rate(const struct device *dev)
{
    int32_t ret;
    uint16_t meas1;

    ret = mlx90632_i2c_read(dev, MLX90632_EE_MEDICAL_MEAS1, &meas1);
    if (ret < 0)
        return MLX90632_MEAS_HZ_ERROR;

    return (mlx90632_meas_t)MLX90632_REFRESH_RATE(meas1);
}

int32_t mlx90632_get_channel_position(const struct device *dev)
{
    uint16_t reg_status;
    int32_t ret;

    ret = mlx90632_i2c_read(dev, MLX90632_REG_STATUS, &reg_status);
    if (ret < 0)
        return ret;

    return (reg_status & MLX90632_STAT_CYCLE_POS) >> 2;
}

int32_t mlx90632_i2c_read(const struct device *dev, int16_t register_address, uint16_t *value)
{
    //printk("READING FROM MLX TEMP\n");
    uint8_t buffer[2]; // Buffer to store 2 bytes read from the I2C device
    uint8_t i2c_write_buff[2];
    int32_t ret;
    const struct mlx90632_config *cfg = dev->config;


	i2c_write_buff[0] = ( register_address >> 8 ) & 0xFF;
	i2c_write_buff[1] = ( register_address & 0xFF);

    ret = i2c_write_read_dt(&cfg->i2c, i2c_write_buff, 2, &buffer, 2);
    
    if (ret < 0) {
        return ret; // Return error code if i2c_read fails
    }

    // Combine the two bytes into a 16-bit value (assuming big-endian order from the device)
    *value = ((uint16_t)buffer[0] << 8) | buffer[1];

    return 0; // Success
}
int32_t mlx90632_i2c_read32(const struct device *dev, int16_t register_address, uint32_t *value)
{
    //printk("READING FROM MLX TEMP\n");
    uint8_t buffer[4]; // Buffer to store 4 bytes read from the I2C device
    uint8_t i2c_write_buff[2];
    int32_t ret;

    const struct mlx90632_config *cfg = dev->config;


	i2c_write_buff[0] = ( register_address >> 8 ) & 0xFF;
	i2c_write_buff[1] = ( register_address & 0xFF);

    ret = i2c_write_read_dt(&cfg->i2c, i2c_write_buff, 2, &buffer, 4);
    if (ret < 0) {
        return ret; // Return error code if i2c_read fails
    }

    // Combine the four bytes into a 32-bit value (assuming big-endian order from the device)
    *value = ((uint32_t)buffer[2] << 24) | ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[0] << 8) | (uint32_t)buffer[1];

    return 0; // Success
}
int32_t mlx90632_i2c_write(const struct device *dev, int16_t register_address, uint16_t value)
{

    uint8_t data[4];
	int ret;
    const struct mlx90632_config *cfg = dev->config;

	data[0] = (uint8_t)(register_address >> 8);
	data[1] = (uint8_t)(register_address & 0xFF);

	data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)(value & 0xFF);


    ret = i2c_write_dt(&cfg->i2c, data, 4);

	return ret;
}

void usleep(int min_range, int max_range)
{
    k_sleep(K_USEC(min_range));

}

void msleep(int msecs)
{
    k_sleep(K_MSEC(msecs));
    return;
}

static int mlx90632_driver_init(const struct device *dev)
{
    const struct mlx90632_config *cfg = dev->config;
    struct mlx90632_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        printk("MLX90632: I2C bus not ready\n");
        return -ENODEV;
    }

    data->initialized = false;
    return 0;  // Don't talk to the sensor yet
}

static int mlx90632_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct mlx90632_data *data = dev->data;
    int ret = 0;

    if (!data->initialized) {
        // This code was moved here, to be calle don the first fetch, rather than
        // on initialisation, because the library initialisation occurs BEFORE
        // the i2c bus / devices are ready.

        // !gb! trying this
        usleep(150, 200);        printk("MLX90632: first fetch\n");
        
        int ret = mlx90632_init(dev);  // EEPROM read, DSP version check, etc.
        if (ret < 0) {
            printk("MLX90632: lazy init failed: %d\n", ret);
            return ret;
        }

        // !gb! added these two lines, similar to Niall's project
        data->ambient_old_raw = 25;
        data->object_old_raw = 25;
        // !gb! trying this
        usleep(150, 200);


        data->initialized = true;
    }

    ret = mlx90632_read_temp_raw(dev, &data->ambient_new_raw, &data->ambient_old_raw,
                                       &data->object_new_raw, &data->object_old_raw);

    ambient_new_raw_dbg1 = data->ambient_new_raw;
    ambient_old_raw_dbg1 = data->ambient_old_raw;
    object_new_raw_dbg1 = data->object_new_raw;
    object_old_raw_dbg1 = data->object_old_raw;
    return ret;
}

static int mlx90632_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct mlx90632_data *data = dev->data;

    ambient_new_raw_dbg2 = data->ambient_new_raw;
    ambient_old_raw_dbg2 = data->ambient_old_raw;
    P_T_dbg2 = data->P_T;
    P_R_dbg2 = data->P_R;
    P_G_dbg2 = data->P_G;
    P_O_dbg2 = data->P_O;
    Gb_dbg2 = data->Gb;
    double ambient = mlx90632_calc_temp_ambient(
        data->ambient_new_raw, data->ambient_old_raw,
        data->P_T, data->P_R, data->P_G, data->P_O, data->Gb);
    ambient_dbg2 = ambient;

    ambient_new_raw_dbg3 = data->ambient_new_raw;
    ambient_old_raw_dbg3 = data->ambient_old_raw;
    Gb_dbg3 = data->Gb;
    double pre_ambient = mlx90632_preprocess_temp_ambient(data->ambient_new_raw,
        data->ambient_old_raw, data->Gb);

    object_new_raw_dbg4 = data->object_new_raw;
    object_old_raw_dbg4 = data->object_old_raw;
    ambient_new_raw_dbg4 = data->ambient_new_raw;
    ambient_old_raw_dbg4 = data->ambient_old_raw;
    Ka_dbg4 = data->Ka;
    double pre_object = mlx90632_preprocess_temp_object(data->object_new_raw,
        data->object_old_raw, data->ambient_new_raw, data->ambient_old_raw, data->Ka);

    pre_object_dbg5 = pre_object;
    pre_ambient_dbg5 = pre_ambient;
    Ea_dbg5 = data->Ea;
    Eb_dbg5 = data->Eb;
    Ga_dbg5 = data->Ga;
    Fa_dbg5 = data->Fa;
    Fb_dbg5 = data->Fb;
    Ha_dbg5 = data->Ha;
    Hb_dbg5 = data->Hb;
    double object = mlx90632_calc_temp_object(pre_object, pre_ambient,
        data->Ea, data->Eb, data->Ga, data->Fa, data->Fb, data->Ha, data->Hb);
    object_dbg5 = object;

    //printk("mlx90632_channel_get: chan = %d\n", chan);

    switch (chan) {
    case SENSOR_CHAN_DIE_TEMP:
        val->val1 = (int)object;
        val->val2 = (int)((object - val->val1) * 1000000);
//        val->val1 = (int16_t)object;
        //val->val2 = (int16_t)((object - val->val1) * 1000000);
//        val->val2 = (int16_t)(((int16_t)object - (int16_t)val->val1) * 1000000);
        return 0;

    case SENSOR_CHAN_AMBIENT_TEMP:
        val->val1 = (int)ambient;
        val->val2 = (int)((ambient - val->val1) * 1000000);
//        val->val1 = (int16_t)ambient;
        //val->val2 = (int16_t)((ambient - val->val1) * 1000000);
//        val->val2 = (int16_t)(((int16_t)ambient - (int16_t)val->val1) * 1000000);
        return 0;

    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api mlx90632_api = 
{
    .sample_fetch = mlx90632_sample_fetch,
    .channel_get = mlx90632_channel_get,
};

#define MLX90632_DEFINE(inst)												\
	static struct mlx90632_data mlx90632_data##inst;                   		\
																			\
	static const struct mlx90632_config mlx90632_config##inst = {			\
		.i2c = I2C_DT_SPEC_INST_GET(inst),                          		\
	};  																	\
	DEVICE_DT_INST_DEFINE(inst,												\
                mlx90632_driver_init,												\
				NULL,														\
				&mlx90632_data##inst,										\
				&mlx90632_config##inst,										\
				POST_KERNEL, 												\
				CONFIG_SENSOR_INIT_PRIORITY, 								\
				&mlx90632_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90632_DEFINE)

///@}