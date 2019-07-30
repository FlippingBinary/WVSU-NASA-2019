#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

enum Baudrate
{
    B50 = 50,
    B110 = 110,
    B150 = 150,
    B300 = 300,
    B1200 = 1200,
    B2400 = 2400,
    B4800 = 4800,
    B9600 = 9600,
    B19200 = 19200,
    B38400 = 38400,
    B57600 = 57600,
    B115200 = 115200,
    B230400 = 230400,
    B460800 = 460800,
    B500000 = 500000,
    B1000000 = 1000000
};

enum Stopbits
{
    one = ONESTOPBIT,
    onePointFive = ONE5STOPBITS,
    two = TWOSTOPBITS
};

enum Paritycheck
{
    off = NOPARITY,
    odd = ODDPARITY,
    even = EVENPARITY,
    mark = MARKPARITY
};

HANDLE openSerialPort(LPCSTR portname, enum Baudrate baudrate, enum Stopbits stopbits, enum Paritycheck parity)
{
    DWORD accessdirection = GENERIC_READ | GENERIC_WRITE;
    HANDLE hSerial = CreateFile(portname,
                                accessdirection,
                                0,    // exclusive access
                                NULL, // no security
                                OPEN_EXISTING,
                                0,     // no overlapped I/O
                                NULL); // null template
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        return hSerial;
    };
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb))
    {
        //could not get the state of the comport
    };
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = stopbits;
    dcb.Parity = parity;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    if (!SetCommState(hSerial, &dcb))
    {
        //analyse error
        fprintf(stdout,"Error setting SetCommState\n");
        fflush(stdout);
    };
    COMMTIMEOUTS timeouts = {0};

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        //handle error
        fprintf(stdout,"Error setting SetCommTimeouts\n");
        fflush(stdout);
    };
    return hSerial;
};

void closeSerialPort(HANDLE hSerial)
{
    CloseHandle(hSerial);
};

DWORD readFromSerialPort(HANDLE hSerial, uint8_t *buffer, int buffersize)
{
    DWORD dwBytesRead = 0;
    if (!ReadFile(hSerial, buffer, buffersize, &dwBytesRead, NULL))
    {
        //handle error
        printf("Failed to read from serial port.\n");
        fflush(stdout);
        exit(0);
    };
    return dwBytesRead;
};

enum read_status_t
{
    STATE_RESTART = 0,
    STATE_W,
    STATE_V,
    STATE_S,
    STATE_U,
    STATE_MICROS_1,
    STATE_MICROS_2,
    STATE_MICROS_3,
    STATE_MICROS_4,

    STATE_CLOCK_1,
    STATE_CLOCK_2,
    STATE_CLOCK_3,
    STATE_CLOCK_4,
    STATE_CLOCK_5,
    STATE_CLOCK_6,
    STATE_CLOCK_7,
    STATE_CLOCK_8,

    STATE_CYCLE_COUNT_M,
    STATE_CYCLE_COUNT_L,
    STATE_DWELL_1_M,
    STATE_DWELL_1_L,
    STATE_DWELL_2_M,
    STATE_DWELL_2_L,
    STATE_DWELL_3_M,
    STATE_DWELL_3_L,
    STATE_DWELL_4_M,
    STATE_DWELL_4_L,

    STATE_OPTICAL_1,
    STATE_OPTICAL_2,
    STATE_OPTICAL_3,

    STATE_GEIGER_1,
    STATE_GEIGER_2,
    STATE_GEIGER_3,
    STATE_GEIGER_4,
    STATE_GEIGER_5,

    STATE_PARTICLE_X_S,
    STATE_PARTICLE_Y_S,
    STATE_PARTICLE_Z_S,
    STATE_PARTICLE_X_N,
    STATE_PARTICLE_Y_N,
    STATE_PARTICLE_Z_N,

    STATE_FXOS8700_1,
    STATE_FXOS8700_2,
    STATE_FXOS8700_3,
    STATE_FXOS8700_4,
    STATE_FXOS8700_5,
    STATE_FXOS8700_6,
    STATE_FXOS8700_7,
    STATE_FXOS8700_8,
    STATE_FXOS8700_9,
    STATE_FXOS8700_10,
    STATE_FXOS8700_11,
    STATE_FXOS8700_12,
    STATE_FXOS8700_13,

    STATE_FXAS21002C_1,
    STATE_FXAS21002C_2,
    STATE_FXAS21002C_3,
    STATE_FXAS21002C_4,
    STATE_FXAS21002C_5,
    STATE_FXAS21002C_6,
    STATE_FXAS21002C_7,
    /* enable if the micro controller is transmitting new calibration data
    STATE_BMP280_T1_M,
    STATE_BMP280_T1_L,
    STATE_BMP280_T2_M,
    STATE_BMP280_T2_L,
    STATE_BMP280_T3_M,
    STATE_BMP280_T3_L,
    STATE_BMP280_P1_M,
    STATE_BMP280_P1_L,
    STATE_BMP280_P2_M,
    STATE_BMP280_P2_L,
    STATE_BMP280_P3_M,
    STATE_BMP280_P3_L,
    STATE_BMP280_P4_M,
    STATE_BMP280_P4_L,
    STATE_BMP280_P5_M,
    STATE_BMP280_P5_L,
    STATE_BMP280_P6_M,
    STATE_BMP280_P6_L,
    STATE_BMP280_P7_M,
    STATE_BMP280_P7_L,
    STATE_BMP280_P8_M,
    STATE_BMP280_P8_L,
    STATE_BMP280_P9_M,
    STATE_BMP280_P9_L,
    //*/
    STATE_BMP280_TEMP_1,
    STATE_BMP280_TEMP_2,
    STATE_BMP280_TEMP_3,
    STATE_BMP280_PRESS_1,
    STATE_BMP280_PRESS_2,
    STATE_BMP280_PRESS_3,

    STATE_MESSAGE_COMPLETE
};

#define ACCEL_MG_LSB_2G (0.000244F)
#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)

#define GYRO_SENSITIVITY_250DPS (0.0078125F)
#define SENSORS_DPS_TO_RADS (0.017453293F) /**< Degrees/s to rad/s multiplier */

typedef struct
{
    uint16_t dig_T1; /**< dig_T1 cal register. */
    int16_t dig_T2;  /**<  dig_T2 cal register. */
    int16_t dig_T3;  /**< dig_T3 cal register. */

    uint16_t dig_P1; /**< dig_P1 cal register. */
    int16_t dig_P2;  /**< dig_P2 cal register. */
    int16_t dig_P3;  /**< dig_P3 cal register. */
    int16_t dig_P4;  /**< dig_P4 cal register. */
    int16_t dig_P5;  /**< dig_P5 cal register. */
    int16_t dig_P6;  /**< dig_P6 cal register. */
    int16_t dig_P7;  /**< dig_P7 cal register. */
    int16_t dig_P8;  /**< dig_P8 cal register. */
    int16_t dig_P9;  /**< dig_P9 cal register. */

} bmp280_calib_data;
const bmp280_calib_data _bmp280_calib = {
    27681,
    25691,
    50,
    37500,
    -10810,
    3024,
    7817,
    -72,
    -7,
    15500,
    -14600,
    6000
};

enum read_status_t read_status = STATE_W;
uint32_t micros = 0;
int16_t tmp_int16 = 0;
uint16_t tmp_uint16 = 0;
int32_t tmp_int32 = 0;
uint32_t tmp_uint32 = 0;
float tmp_float = 0.0;
int32_t t_fine;
struct timespec tv;
uint16_t total_rad_1[50];
uint8_t total_rad1_marker = 0;
uint16_t total_rad_2[50];
uint8_t total_rad2_marker = 0;
uint16_t total_rad_3[50];
uint8_t total_rad3_marker = 0;
uint16_t total_rad_4[50];
uint8_t total_rad4_marker = 0;
uint16_t total_rad_5[50];
uint8_t total_rad5_marker = 0;

int stateMachine(uint8_t sym, FILE* dest)
{
    switch (read_status)
    {
    case STATE_W:
    {
        if (sym != 'W')
        {
            fprintf(dest,"Not W: %X  \n", sym);
            fflush(dest);
            return 1;
        }
        break;
    }
    case STATE_V:
    {
        if (sym != 'V')
        {
            fprintf(dest,"Not V: %X  \n", sym);
            fflush(dest);
            read_status = STATE_W;
            return 1;
        }
        break;
    }
    case STATE_S:
    {
        if (sym != 'S')
        {
            fprintf(dest,"Not S: %X  \n", sym);
            fflush(dest);
            read_status = STATE_W;
            return 1;
        }
        break;
    }
    case STATE_U:
    {
        if (sym != 'U')
        {
            fprintf(dest,"Not U: %X  \n", sym);
            fflush(dest);
            read_status = STATE_W;
            return 1;
        }
        break;
    }
    case STATE_MICROS_1:
    {
        micros = sym;
        break;
    }
    case STATE_MICROS_2:
    {
        micros <<= 8;
        micros |= sym;
        break;
    }
    case STATE_MICROS_3:
    {
        micros <<= 8;
        micros |= sym;
        break;
    }
    case STATE_MICROS_4:
    {
        micros <<= 8;
        micros |= sym;
        fprintf(dest,"T:%11u\t", micros);
        break;
    }

    case STATE_CLOCK_1:
    {
        tv.tv_sec = (uint64_t)sym;
        break;
    }
    case STATE_CLOCK_2:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        break;
    }
    case STATE_CLOCK_3:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        break;
    }
    case STATE_CLOCK_4:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        break;
    }
    case STATE_CLOCK_5:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        break;
    }
    case STATE_CLOCK_6:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        break;
    }
    case STATE_CLOCK_7:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        break;
    }
    case STATE_CLOCK_8:
    {
        tv.tv_sec = (uint64_t)(tv.tv_sec << 8) | sym;
        fprintf(dest, "W:%010u\t", tv.tv_sec);
        break;
    }

    case STATE_CYCLE_COUNT_M:
    {
        tmp_uint16 = sym;
        //                    fprintf(dest,"C:%03d\t", sym);
        break;
    }
    case STATE_CYCLE_COUNT_L:
    {
        //                    tmp_uint16 <<= 8;
        tmp_uint16 = (uint16_t)(tmp_uint16 << 8) | sym;
        fprintf(dest,"C:%03u\t", tmp_uint16);
        break;
    }
    case STATE_DWELL_1_M:
    {
        tmp_uint16 = sym;
        break;
    }
    case STATE_DWELL_1_L:
    {
        tmp_uint16 = (uint16_t)(tmp_uint16 << 8) | sym;
        fprintf(dest,"D:%03u", tmp_uint16);
        break;
    }
    case STATE_DWELL_2_M:
    {
        tmp_uint16 = sym;
        break;
    }
    case STATE_DWELL_2_L:
    {
        tmp_uint16 = (uint16_t)(tmp_uint16 << 8) | sym;
        fprintf(dest,"/%03u", tmp_uint16);
        break;
    }
    case STATE_DWELL_3_M:
    {
        tmp_uint16 = sym;
        break;
    }
    case STATE_DWELL_3_L:
    {
        tmp_uint16 = (uint16_t)(tmp_uint16 << 8) | sym;
        fprintf(dest,"/%03u", tmp_uint16);
        break;
    }
    case STATE_DWELL_4_M:
    {
        tmp_uint16 = sym;
        break;
    }
    case STATE_DWELL_4_L:
    {
        tmp_uint16 = (uint16_t)(tmp_uint16 << 8) | sym;
        fprintf(dest,"/%03u\t", tmp_uint16);
        break;
    }

    case STATE_OPTICAL_1:
    {
        fprintf(dest,"O:%04u", sym);
        break;
    }
    case STATE_OPTICAL_2:
    {
        fprintf(dest,"/%04u", sym);
        break;
    }
    case STATE_OPTICAL_3:
    {
        fprintf(dest,"/%04u\t", sym);
        break;
    }

    case STATE_GEIGER_1:
    {
        fprintf(dest,"G:%03u", sym);
        total_rad_1[total_rad1_marker++] = sym;
        total_rad1_marker %= 50;
        uint16_t sum = 0;
        for ( int x = 0; x < 50; x++ )
        {
            sum += total_rad_1[x];
        }
        fprintf(dest,"(%03u)",sum);
        break;
    }
    case STATE_GEIGER_2:
    {
        fprintf(dest,"/%03u", sym);
        total_rad_2[total_rad2_marker++] = sym;
        total_rad2_marker %= 50;
        uint16_t sum = 0;
        for (int x = 0; x < 50; x++)
        {
            sum += total_rad_2[x];
        }
        fprintf(dest, "(%03u)", sum);
        break;
    }
    case STATE_GEIGER_3:
    {
        fprintf(dest,"/%03u", sym);
        total_rad_3[total_rad3_marker++] = sym;
        total_rad3_marker %= 50;
        uint16_t sum = 0;
        for (int x = 0; x < 50; x++)
        {
            sum += total_rad_3[x];
        }
        fprintf(dest, "(%03u)", sum);
        break;
    }
    case STATE_GEIGER_4:
    {
        fprintf(dest,"/%03u", sym);
        total_rad_4[total_rad4_marker++] = sym;
        total_rad4_marker %= 50;
        uint16_t sum = 0;
        for (int x = 0; x < 50; x++)
        {
            sum += total_rad_4[x];
        }
        fprintf(dest, "(%03u)", sum);
        break;
    }
    case STATE_GEIGER_5:
    {
        fprintf(dest,"/%03u", sym);
        total_rad_5[total_rad5_marker++] = sym;
        total_rad5_marker %= 50;
        uint16_t sum = 0;
        for (int x = 0; x < 50; x++)
        {
            sum += total_rad_5[x];
        }
        fprintf(dest, "(%03u)\t", sum);
        break;
    }

    case STATE_PARTICLE_X_S:
    {
        fprintf(dest,"PS:%03u", sym);
        break;
    }
    case STATE_PARTICLE_Y_S:
    {
        fprintf(dest,"/%03u", sym);
        break;
    }
    case STATE_PARTICLE_Z_S:
    {
        fprintf(dest,"/%03u\t", sym);
        break;
    }
    case STATE_PARTICLE_X_N:
    {
        fprintf(dest,"PN:%03u", sym);
        break;
    }
    case STATE_PARTICLE_Y_N:
    {
        fprintf(dest,"/%03u", sym);
        break;
    }
    case STATE_PARTICLE_Z_N:
    {
        fprintf(dest,"/%03u\t", sym);
        break;
    }

    case STATE_FXOS8700_1:
    {
        fprintf(dest,"I%02X", sym);
        break;
    }
    case STATE_FXOS8700_2:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXOS8700_3:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym) >> 2;
        tmp_float = tmp_int16 * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
        fprintf(dest,":% 05.1f", tmp_float);
        break;
    }
    case STATE_FXOS8700_4:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXOS8700_5:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym) >> 2;
        tmp_float = tmp_int16 * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
        fprintf(dest,"/% 05.1f", tmp_float);
        break;
    }
    case STATE_FXOS8700_6:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXOS8700_7:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym) >> 2;
        tmp_float = tmp_int16 * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
        fprintf(dest,"/% 05.1f", tmp_float);
        break;
    }
    case STATE_FXOS8700_8:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXOS8700_9:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym);
        tmp_float = tmp_int16 * MAG_UT_LSB;
        fprintf(dest,":% 05.1f", tmp_float);
        break;
    }
    case STATE_FXOS8700_10:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXOS8700_11:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym);
        tmp_float = tmp_int16 * MAG_UT_LSB;
        fprintf(dest,"/% 05.1f", tmp_float);
        break;
    }
    case STATE_FXOS8700_12:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXOS8700_13:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym);
        tmp_float = tmp_int16 * MAG_UT_LSB;
        fprintf(dest,"/% 05.1f\t", tmp_float);
        break;
    }

    case STATE_FXAS21002C_1:
    {
        fprintf(dest,"G%02X", sym);
        break;
    }
    case STATE_FXAS21002C_2:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXAS21002C_3:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym);
        tmp_float = tmp_int16 * GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
        fprintf(dest,":% 05.1f", tmp_float);
        break;
    }
    case STATE_FXAS21002C_4:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXAS21002C_5:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym);
        tmp_float = tmp_int16 * GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
        fprintf(dest,":% 05.1f", tmp_float);
        break;
    }
    case STATE_FXAS21002C_6:
    {
        tmp_int16 = (int16_t)sym;
        break;
    }
    case STATE_FXAS21002C_7:
    {
        tmp_int16 = (int16_t)((tmp_int16 << 8) | sym);
        tmp_float = tmp_int16 * GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
        fprintf(dest,":% 05.1f\t", tmp_float);
        break;
    }
        /* enable this section if micro controller is sending calibration data
                case STATE_BMP280_T1_M:
                {
                    tmp_uint16 = (uint16_t)sym;
                    break;
                }
                case STATE_BMP280_T1_L:
                {
                    tmp_uint16 = (uint16_t)((sym << 8) | tmp_uint16);
                    fprintf(dest,"CAL:%03d(1)", tmp_uint16);
                    break;
                }
                case STATE_BMP280_T2_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_T2_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(2)", tmp_int16);
                    break;
                }
                case STATE_BMP280_T3_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_T3_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(3)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P1_M:
                {
                    tmp_uint16 = (uint16_t)sym;
                    break;
                }
                case STATE_BMP280_P1_L:
                {
                    tmp_uint16 = (uint16_t)((sym << 8) | tmp_uint16);
                    fprintf(dest,"/%03d(4)", tmp_uint16);
                    break;
                }
                case STATE_BMP280_P2_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P2_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(5)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P3_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P3_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(6)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P4_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P4_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(7)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P5_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P5_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(8)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P6_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P6_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(9)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P7_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P7_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(10)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P8_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P8_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(11)", tmp_int16);
                    break;
                }
                case STATE_BMP280_P9_M:
                {
                    tmp_int16 = (int16_t)sym;
                    break;
                }
                case STATE_BMP280_P9_L:
                {
                    tmp_int16 = (int16_t)((sym << 8) | tmp_int16);
                    fprintf(dest,"/%03d(12)\t", tmp_int16);
                    break;
                }
//*/
    case STATE_BMP280_TEMP_1:
    {
        tmp_int32 = (int32_t)sym;
        break;
    }
    case STATE_BMP280_TEMP_2:
    {
        tmp_int32 = (int32_t)((tmp_int32 << 8) | sym);
        break;
    }
    case STATE_BMP280_TEMP_3:
    {
        int32_t var1, var2;
        tmp_int32 = (int32_t)((tmp_int32 << 8) | sym);

        tmp_int32 >>= 4;

        var1 = ((((tmp_int32 >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
                ((int32_t)_bmp280_calib.dig_T2)) >>
               11;

        var2 = (((((tmp_int32 >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
                  ((tmp_int32 >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
                 12) *
                ((int32_t)_bmp280_calib.dig_T3)) >>
               14;

        t_fine = var1 + var2;

        float T = (t_fine * 5 + 128) >> 8;
        T /= 100;
        fprintf(dest,"T:%05.1f\t", T);
        break;
    }
    case STATE_BMP280_PRESS_1:
    {
        tmp_int32 = (int32_t)sym;
        break;
    }
    case STATE_BMP280_PRESS_2:
    {
        tmp_int32 = (int32_t)((tmp_int32 << 8) | sym);
        break;
    }
    case STATE_BMP280_PRESS_3:
    {
        tmp_int32 = (int32_t)((tmp_int32 << 8) | sym);
        int64_t var1, var2, p;

        tmp_int32 >>= 4;

        var1 = ((int64_t)t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
        var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
        var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
               ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
        var1 =
            (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

        if (var1 == 0)
        {
            fprintf(dest,"[div zero]");
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576 - tmp_int32;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

        p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
        float P = p / 256;
        fprintf(dest,"P:%09.1f\t", P);
        break;
    }

    case STATE_MESSAGE_COMPLETE:
    {
        fprintf(dest,"X:%02X\n", sym);
        read_status = STATE_RESTART;
        break;
    }
    }
    read_status++;
    fflush(dest);
}

int main(int argc, char **argv)
{
    uint8_t buf[512];
    DWORD len = 0;
    
    if ( argc < 2 || argc > 3 ) {
        printf("This program requires one or two arguments.\n First argument should be the source filename or COM port name.\n Second argument is optional and can be a filename to write CSV version of source data.\n\n");
        exit(0);
    };

    HANDLE hSerial;
    FILE *dest;

    if ( argc == 3 ) {
        dest = fopen(argv[2],"w");
        if ( !dest )
        {
            printf("Error opening output file.\n");
            exit(0);
        };
    } else
    {
        dest = stdout;
    };
    
    if ( argv[1][0] == 'C' && argv[1][1] == 'O' && argv[1][2] == 'M' )
    {
        hSerial = openSerialPort(argv[1], B57600, one, off);
        if (hSerial == INVALID_HANDLE_VALUE)
        {
            printf("Error in opening serial port\n");
            exit(0);
        }
        else
        {
            printf("opening serial port successful\n");
            fflush(stdout);
            while (len = readFromSerialPort(hSerial, buf, 512) )// || 1 )
            {
                for (int i = 0; i < len; i++)
                {
                    if (stateMachine(buf[i], dest) )
                        continue;
                }
            };
            closeSerialPort(hSerial);
        }
    } else
    {
        FILE *source = fopen(argv[1],"rb");
        if ( !source ) {
            printf("ERROR: argument did not describe a COM port and no file by that name could be opened.\n");
            fflush(stdout);
            exit(0);
        };
        while (len = fread(buf, 1, 512, source))
        {
            for ( int i = 0; i < len; i++ )
            {
                if (stateMachine(buf[i], dest) )
                    continue;
            }
        }
        fclose(source);
    }
};
