#ifndef MINO_MOT_REG_
#define MINO_MOT_REG_


/* Motor module registers */
#define REG_FLAGS_0           0x00
#define REG_BITS_0            0x01
#define REG_MODEL             0x04
#define REG_VERSION           0x05
#define REG_ADDRESS           0x06
#define REG_CHIP_ID           0x07
#define REG_MOT_FREQUENCY_L   0x08
#define REG_MOT_FREQUENCY_H   0x09
#define REG_MOT_MAX_RPM_DEV   0x0A
#define REG_MOT_FLG           0x10
#define REG_MOT_MAGNET        0x11
#define REG_MOT_REDUCER_L     0x12
#define REG_MOT_REDUCER_C     0x13
#define REG_MOT_REDUCER_H     0x14
#define REG_MOT_SET_PWM_L     0x15
#define REG_MOT_SET_PWM_H     0x16
#define REG_MOT_SET_RPM_L     0x17
#define REG_MOT_SET_RPM_H     0x18
#define REG_MOT_GET_RPM_L     0x19
#define REG_MOT_GET_RPM_H     0x1A
#define REG_MOT_GET_REV_L     0x1B
#define REG_MOT_GET_REV_C     0x1C
#define REG_MOT_GET_REV_H     0x1D
#define REG_MOT_STOP_REV_L    0x1E
#define REG_MOT_STOP_REV_C    0x1F
#define REG_MOT_STOP_REV_H    0x20
#define REG_MOT_STOP_TMR_L    0x21
#define REG_MOT_STOP_TMR_C    0x22
#define REG_MOT_STOP_TMR_H    0x23
#define REG_MOT_STOP          0x24
#define REG_BITS_2            0x25
#define REG_MOT_VOLTAGE       0x26
#define REG_MOT_NOMINAL_RPM_L 0x27
#define REG_MOT_NOMINAL_RPM_H 0x28

/* REG_FLAGS_0 register bits */
#define FLG_I2C_UP    0x04
#define FLG_SELF_TEST 0x40
#define FLG_RESET     0x80

/* REG_BITS_0 register bits */
#define SAVE_ADR_EN   0x02
#define SET_I2C_UP    0x04
#define BLOK_ADR      0x08
#define SET_SELF_TEST 0x40
#define SET_RESET     0x80

/* REG_ADDRESS register bits */
#define SAVE_FLASH 0x01

/* REG_MOT_FLG register bits */
#define FLG_NEUTRAL 0x01
#define FLG_STOP    0x02
#define FLG_DRV_ERR 0x10
#define FLG_RPM_ERR 0x20
#define FLG_RPM_EN  0x80

/* REG_MOT_STOP register bits */
#define BIT_NEUTRAL 0x01
#define BIT_STOP    0x02

/* REG_BITS_2 register bits */
#define BIT_INV_PIN 0x01
#define BIT_INV_RDR 0x02
#define BIT_DIR_CKW 0x04


#endif // MINO_MOT_REG_