/*
 * Copyright 2019 Solomon Ltd. All rights reserved.
 *
 * SPD2010 device Touch device driver
 *
 * Date: 2021.01.13
 * modify ver: 0010-1
 */
#ifndef __spd2010_H
#define __spd2010_H

#include <linux/semaphore.h>

#define DRIVER_VRESION "1.00"

#define SUPPORT_BOOTUP_FW_UPGRADE_BINFILE
#define SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE

#define SUPPORT_ES2
#define SUPPORT_PROTOCOL_6BYTES

#define SUPPORT_SSL

#define SUPPORT_TMC_I2C_LENGTH

#define SUPPORT_LPM

#define SUPPORT_AUX			/* for support of AUX bit of SnL */
#define SUPPORT_KEY_BUTTON		/* for key H/W button */


/* Gesture proc control */
#define ENABLE_GESTURE
/* Report gesture as long as it's still in LPM */
#define SUPPORT_REPEAT_GESTURE_REPORT

#define TMC_UNSTALL_DELAY	5
#define TMC_READ_DELAY		200		/* I2C packet delay */
#define HDP_CHECK_DELAY		500

#define TOUCH_POINT_MODE		0
#define RAW_DATA_MODE			1

#define AGING_RAW_DATA_MODE		31	/* for rawdata of aging */
#define AGING_BASELINED_DATA_MODE	32	/* for baselined of aging */
#define AGING_BASELINE_DATA_MODE	33	/* for baseline of aging */
#define AGING_MODIFY_DATA_MODE		34	/* for modify of aging */

/* for MP Test */
#define MPTEST_START		0x0001
#define MPTEST_STOP			0x0000
#define MPTEST_READY_STOP	0x8000
#define MPTEST_READY_START	0x4000
#define MPTEST_LPM_NC_RANGE	0x0006	/* Normal CELL */
#define MPTEST_LPM_NC_JITTER	0x0007	/* Normal CELL */
#define MPTEST_LPM_LC_RANGE	0x0008	/* Large CELL */
#define MPTEST_LPM_LC_JITTER	0x0009	/* Large CELL */

#define MAX_RAWDATA_BUFFER_SIZE	2048
#define MAX_DEVICE_KEYMAP_SIZE	16

#define AFE_MAX_LIMIT 2700
#define AFE_MIN_LIMIT 50

#define HIDDEN_VERSION_FACTORY		0x7F0000F7	/* for force update */

#define FW_FORCE_FULL_PATH			("/sdcard/mgd_ssl_fw.img")

#define SPD2010_NAME				("ep0700")
#define SPD2010_I2C_ADDR			(0x53)	/* I2C slave address */
#define SPD2010_X_MAX				(356)	/* resolution X */
#define SPD2010_Y_MAX				(400)	/* resolution Y */
#define SPD2010_MAX_X_NODE			(6)
#define SPD2010_MAX_Y_NODE			(7)
#define SPD2010_MAX_PRESSURE	    255
#define SPD2010_MAX_MAJOR	        30

/* The maximum supported number of fingers. */
#define MAX_POINT_NUMBER			(5)

#define POINT_HEADER_Cnt			4
#define HDP_ADD_Cnt					2
#define PACKET_Length_Cnt			2
#define ContactID_Cnt				1
#define ReportID_Cnt				1
#define POINT_INFO_Cnt				6
#define SCANTIME_Cnt				6
#define KEY_Packet_Cnt				6
#define GESTURE_Packet_Cnt			6
#define AUX_Packet_Cnt				6

#define SPD2010_KEY_ID				0x7A
#define SPD2010_GESTURE_ID			0x7C
#define SPD2010_AUX_ID				0x78

/* rawdata type */
#define MUTUAL						0
#define SELF						1
#define NMS							2

#define TYPE_NC						0x01
#define TYPE_LC						0x02
#define	TYPE_KEY					0x04
#define TYPE_NMS					0x08

#define DATA_INFO_LOC				1
#define FULL_HEADER_LOC				8
#define FULL_BYTE_LOW				10
#define FULL_BYTE_HIGH				11
#define FIRST_PART_HEADER			16
#define FULL_HEADER_SIZE			8
#define PART_HEADER_SIZE			8

#define SPD2010_MAX_NODE	(SPD2010_MAX_X_NODE \
							* SPD2010_MAX_Y_NODE)

#define SPD2010_MAX_RAWDATA_QUEUE	11

#define SPD2010_RESET_PIN			(EXYNOS5410_GPE0(3))
#define SPD2010_INT_PIN				(EXYNOS5410_GPX0(2))
#define SPD2010_IRQ					(IRQ_EINT(2))
#define SPD2010_I2C_SCL				(EXYNOS5410_GPA2(3))

#define DELAY_FOR_SIGNAL_DELAY		30	/* us */

#define DELAY_FOR_TRANSCATION		TMC_READ_DELAY
#define DELAY_FOR_POST_TRANSCATION	TMC_READ_DELAY

#define RETRY_TIME_INT_PIN			200
/* Gesture size
 * size = Gesture(2bytes)+Length(2bytes)+points(40bytes)
 * points = (pos_x(2bytes) + pos_y(2bytes))*Max_coord(10)
 */
#define LENGTH_GESTURE_DATA			(2+2+2*2*10)

#define PASS_DATA_ID_BYTE			0	/* array index */
#define PASS_DATA_DEVICEID_BYTE		5	/* array index */
#define PASS_DATA_DEVICEID_MASK		0x0F
#define PASS_DATA_FORCE_DOWN_MASK	0x80	/* force down */
#define PASS_DATA_FORCE_UP_MASK		0x40	/* force up */

#define PASS_DATA_ID_AXIS_FIRST		0x00
#define PASS_DATA_ID_AXIS_LAST		0x0E
#define PASS_DATA_ID_KEY			0xF5
#define PASS_DATA_ID_GESTURE		0xF6
#define PASS_DATA_ID_AUX			0xF8	/* aux */

#define I2C_SUCCESS					0
#define I2C_FAIL					1

#define POWER_ON					0
#define POWER_OFF					1
#define POWER_ON_SEQUENCE			2
#define POWER_RESET					3

/* KEY_GESTURE 15 */
#define  KEY_GESTURE_UP				KEY_UP
#define  KEY_GESTURE_DOWN			KEY_DOWN
#define  KEY_GESTURE_LEFT			KEY_RIGHT
#define  KEY_GESTURE_RIGHT			KEY_LEFT
#define  KEY_GESTURE_DOUBLECLICK	KEY_D
#define  KEY_GESTURE_O				KEY_O
#define  KEY_GESTURE_W				KEY_W
#define  KEY_GESTURE_M				KEY_M
#define  KEY_GESTURE_E				KEY_E
#define  KEY_GESTURE_S				KEY_S
#define  KEY_GESTURE_Z				KEY_Z
#define  KEY_GESTURE_C				KEY_C
#define  KEY_GESTURE_U				KEY_U

#define GESTURE_UP					0x54
#define GESTURE_DOWN				0x42
#define GESTURE_LEFT				0x52
#define GESTURE_RIGHT				0x4c

#define GESTURE_DOUBLECLICK			0x400
#define GESTURE_O					0x6F
#define GESTURE_W					0x77
#define GESTURE_M					0x6D
#define GESTURE_E					0x65
#define GESTURE_S					0x73
#define GESTURE_Z					0x7A
#define GESTURE_C					0x63
#define GESTURE_U					0x76
#define GESTURE_U_DOWN				0x5E
#define GESTURE_U_RIGHT				0x3E

/* for DS16 */
#define DS_EFLASH_WRITE				0x0004
#define DS_EFLASH_READ				0x0005
#define DS_COMMAND_01				0xF002
#define DS_COMMAND_02				0xE008
#define DS_COMMAND_031				0xA001
#define DS_COMMAND_03				0xA002
#define DS_CUP_CONTROL				0x0002
#define DS_ERASE_MACRO				0x000B
#define DS_BOOTUP_MULTIFW			0x000E

#define DS_EFLASH_READ_01			0x0009
#define DS_EFLASH_WRITE_01			0x0008
#define DS_EFLASH_DOWNLOAD_ADDR1	0x7E00
#define DS_EFLASH_DOWNLOAD_ADDR2	0x0000

#define DS_COMMAND_READ_VERSION	    0x000F

#define DS_WRITE_PTR				0x9004
#define DS_READ_PTR					0x9000

/*-----------------------------------------------------
 *	Create gesture device node msg
 *-----------------------------------------------------
 */
#ifdef ENABLE_GESTURE
#define SSD_PROC_NODE_CLASS "class"
#define SSD_PROC_NODE_DEVICE "gesture_ssd"
#define SSD_PROC_NODE_GESTURE_WAKEUP_MODE "gesture_switch_ssd"
#define SSD_PROCFS_AUTHORITY (0666)

ssize_t ssdDrvMainProcfsGestureWakeupModeRead(struct file *pFile,
		char __user *pBuffer, size_t nCount, loff_t *pPos);
ssize_t ssdDrvMainProcfsGestureWakeupModeWrite(struct file *pFile,
		const char __user *pBuffer, size_t nCount, loff_t *pPos);
#endif

/*-----------------------------------------------------
 *	Debug msg
 *-----------------------------------------------------
 */
#define ssl_project_debug_msg		1
#define ssl_project_warnning_msg		1

#if ssl_project_debug_msg
#define ssl_info_msg(fmt, args...)	\
	pr_info("[SSL-INFO : %-18s] "fmt"\n",	\
			__func__, ##args)
#else
#define ssl_info_msg(fmt, args...) \
	do {} while (0)
#endif

#if  ssl_project_warnning_msg
#define ssl_dbg_msg(fmt, args...) \
	pr_info("[SSL-WARN : %-18s] "fmt"\n",	\
			__func__, ##args)
#else
#define ssl_dbg_msg(fmt, args...) \
	do {} while (0)
#endif

/*-----------------------------------------------------
 *	ESD TIMER
 *-----------------------------------------------------
 */
/* Support ESD Timer when value is 1. */
#define ESD_TIMER_ENABLE		1

#define	SPD2010_ESD_INTERVAL		1
#define SPD2010_SCAN_RATE_HZ		60
#define SPD2010_CHECK_ESD_TIMER		1


/*-----------------------------------------------------
 *	CMD & Reg Addr
 *-----------------------------------------------------
 */
#define SPD2010_INT_CLEAR_CMD		0x0002
#define SPD2010_GOTO_BIOS			0x0004
#define SPD2010_RUN_CPU				0x0004
#define ACC_ICREG					0x0006
#define SPD2010_BIOS_CLR_PROTECT	0x0008
#define SPD2010_SEEPROM_ACCESS		0x0020
#define SPD2010_BIOS_STATUS			0x0028
#define SPD2010_POWER_MODE			0x002F

#define SPD2010_GET_STATUS			0x0020
#define SPD2010_PACKET_LENGTH		0x0042
#define SPD2010_HDP_STATUS			0x0044
#define SPD2010_SW_COMMAND			0x0046

#define SPD2010_SWRESET_CMD			0x0044

#define SPD2010_SW_CALIBRATION		0x0040
#define SPD2010_TEST_MODE			0x0045
#define SPD2010_HW_CAL				0x0046

#define SPD2010_TMC_PAUSE			0x0041
#define SPD2010_TDFE_SCAN			0x0038

#define SPD2010_MP_TEST				0x004A
#define SPD2010_SELF_TEST			0x004E
#define SPD2010_GET_REGISTER		0x0050

#define SPD2010_GET_RAW_LEN			0x0074

#define SPD2010_GET_AUX				0x0378
#define SPD2010_GET_KEY				0x037A
#define SPD2010_GET_GESTURE			0x037C
#define SPD2010_DISPLAY_VER			0x0380
#define SPD2010_GRAPH				0x03A0

#define SPD2010_HDP					0x0300
#define SPD2010_TOUCH_MODE			0x0050

#define SPD2010_INT_FLAG			0x0062
#define SPD2010_TOTAL_X_NODE		0x006A
#define SPD2010_TOTAL_Y_NODE		0x0068
#define SPD2010_X_RESOLUTION		0x006C
#define SPD2010_Y_RESOLUTION		0x006E
#define SPD2010_ORIENTATION			0x0070
#define SPD2010_USING_POINT_NUM		0x0072
#define SPD2010_SENSITIVITY			0x007E
#define SPD2010_ESD_INT_INTERVAL	0x029C

#ifdef SUPPORT_TMC_I2C_LENGTH
#define SPD2010_TMC_I2C_LENGTH		0x005A
#endif
#define SPD2010_HW_CAL_INFO			(SPD2010_TOUCH_MODE+0x12B)
#define SPD2010_AFE_MAX_LIMIT		0x01C4
#define SPD2010_AFE_MIN_LIMIT		0x01C6
#define SPD2010_ESD_TIME			0x029C

#define SPD2010_CHECK_HDP			0x02FC
#define SPD2010_POINT_DATA			0x0300
#define SPD2010_RAW_DATA			0x0300

/* HDP Status */
#define KEEP_POLL					1
#define INT_CLEAR					2
#define UPLOAD_PACKET				4
#define LOAD_FAIL					-1

#define SPD2010_STATUS_LENGTH		0x0AF0
#define SPD2010_DEBUG_MODE			0x0AF3
#define SPD2010_GRAPH_MODE			0x0AF4
#define SPD2010_GET_KEYDATA			0x0AF5
#define SPD2010_GET_GESTURE_COORDINATE	0x0AF7
#define SPD2010_AUX					0x0AF8
#define SPD2010_DOWNLOAD_MODE		0x0AF9
#define SPD2010_FW_VER_WRITE		0x0AFA
#define SPD2010_DATA_VER_WRITE		0x0AFB
#define SPD2010_SAVE_REG			0x0AFD
#define SPD2010_TCI1_FAIL_REASON	0x0B03
#define SPD2010_TCI2_FAIL_REASON	0x0B04

#define SSL_SLEEP_IN				0x0000
#define SSL_SLEEP_OUT				0x0000

/*-----------------------------------------------------
 *	AUX mode
 *-----------------------------------------------------
 */
#define AUX_ESD_DETECT			0x00AA
#define AUX_CS_ERROR			0x00F0
#ifdef SUPPORT_ES2
#define AUX_BOOTUP_RESET		0x0001
#define AUX_WDTDS_INT			0x0002
#endif
#define AUX_NEED_C1_TMC_RESET		0x00C1
#define AUX_NEED_C2_DIC_RESET		0x00C2
#define AUX_NEED_C3_DIC_POWER		0x00C3
#define AUX_NEED_C4_DSV_POWER_RESET	0x00C4

/*-----------------------------------------------------
 *	POWER mode
 *-----------------------------------------------------
 */
#define POWER_MODE_NM				0x0001
#define POWER_MODE_LPM				0x0002

#define CHECK_GESTURE_KNOCK_ON		0x0400

#define STATUS_CHECK_PALM_GESTURE	0x10
#define STATUS_CHECK_KEY			0x20
#define STATUS_CHECK_AUX			0x40
#define STATUS_CHECK_BOOT_ST		0x80

#define MAX_GESTURE_COORDINATE_SIZE		(64*4)
/*	gesture status(1word) + legnth(1word); */
#define GESTURE_READ_SIZE				2

#define GESTURE_STATUS_KNOCK_CODE		0x0800
#define GESTURE_STATUS_KNOCK_ON			0x0400
#define GESTURE_STATUS_PALM_REJECT		0x0200
#define GESTURE_STATUS_LARGE_PALM		0x0100
#define GESTURE_STATUS_GESTURES			0x00FF

#define GESTURE_STATUS_KNOCK_ALL	(GESTURE_STATUS_KNOCK_CODE |	\
		GESTURE_STATUS_KNOCK_ON |	\
		GESTURE_STATUS_GESTURES)
/*-----------------------------------------------------
 *	boot status failure bit
 *-----------------------------------------------------
 */
#define BOOT_STATUS_ERR_CPUCFG_CPUINST_CHECKSUM_FAIL	0x0100
#define BOOT_STATUS_ERR_CPUCFG_CPUDM_CHECKSUM_FAIL		0x0200
#define BOOT_STATUS_ERR_CPUCFG_TABLE_CHECK_FAIL			0x0400
#define BOOT_STATUS_ERR_SYS_CFG_FAIL					0x0800
#define BOOT_STATUS_ERR_INF_LDORDAC_INVALID				0x1000
#define BOOT_STATUS_ERR_INF_IDCO32K_INVALID				0x2000
#define BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID			0x4000

#define BOOT_STATUS_ERR_CPUCFG_ALL			\
	(BOOT_STATUS_ERR_CPUCFG_CPUINST_CHECKSUM_FAIL |	\
	 BOOT_STATUS_ERR_CPUCFG_CPUDM_CHECKSUM_FAIL	|	\
	 BOOT_STATUS_ERR_CPUCFG_TABLE_CHECK_FAIL)

#define BOOT_STATUS_ERR_INF_ALL				\
	(BOOT_STATUS_ERR_INF_LDORDAC_INVALID |		\
	 BOOT_STATUS_ERR_INF_IDCO32K_INVALID |		\
	 BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID)

#define BOOT_STATUS_ERR_ALL			\
	(BOOT_STATUS_ERR_CPUCFG_ALL |		\
	 BOOT_STATUS_ERR_CPUCFG_SYS_CFG_FAIL |	\
	 BOOT_STATUS_ERR_INF_ALL)

/*-----------------------------------------------------
 *	DOWNLOAD
 *-----------------------------------------------------
 */
#define LDM_RUN				0x00
#define APM_RUN				0x02
#define	FW_LOAD_FINISH		0x04
#define	FW_LOAD_START		0x01
#define	FW_READ_START		0x08
#define DATA_LOAD_FINISH	0x07
#define DATA_LOAD_START		0x05
#define	DATA_READ_START		0x09

#define HCMD_ADDRESS		0x0008
#define RHDP_ADDRESS		0x0040
#define HQARG_ADDRESS		0x0010
#define PROGRAM_PAGE_SIZE	256
#define READ_MAX_LENGTH		960 /* 1024 - 64 */

/*-----------------------------------------------------
 *	IOCTL
 *-----------------------------------------------------
 */
#define TOUCH_IOCTL_BASE				0xbc
#define TOUCH_IOCTL_GET_FW_VERSION		_IO(TOUCH_IOCTL_BASE, 0)
#define TOUCH_IOCTL_GET_DATA_VERSION	_IO(TOUCH_IOCTL_BASE, 1)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 2)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 3)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM	_IO(TOUCH_IOCTL_BASE, 4)
#define TOUCH_IOCTL_SET_TOUCH_MODE		_IO(TOUCH_IOCTL_BASE, 5)
#define TOUCH_IOCTL_GET_RAW_DATA		_IO(TOUCH_IOCTL_BASE, 6)
#define TOUCH_IOCTL_GET_X_RESOLUTION	_IO(TOUCH_IOCTL_BASE, 7)
#define TOUCH_IOCTL_GET_Y_RESOLUTION	_IO(TOUCH_IOCTL_BASE, 8)
#define TOUCH_IOCTL_GET_REG				_IO(TOUCH_IOCTL_BASE, 9)
#define TOUCH_IOCTL_SET_REG				_IO(TOUCH_IOCTL_BASE, 10)
#define TOUCH_IOCTL_SET_DOWNLOAD		_IO(TOUCH_IOCTL_BASE, 11)
#define TOUCH_IOCTL_GET_GRAPH_DATA		_IO(TOUCH_IOCTL_BASE, 12)
#define TOUCH_IOCTL_QUEUE_CLEAR			_IO(TOUCH_IOCTL_BASE, 13)
#define TOUCH_IOCTL_GET_GESTURE			_IO(TOUCH_IOCTL_BASE, 14)
#define TOUCH_IOCTL_MP_TEST				_IO(TOUCH_IOCTL_BASE, 15)
#define TOUCH_IOCTL_SW_RESET			_IO(TOUCH_IOCTL_BASE, 16)
#define TOUCH_IOCTL_HW_RESET			_IO(TOUCH_IOCTL_BASE, 17)
#define TOUCH_IOCTL_GET_I2C_UPLOAD_SIZE \
						_IO(TOUCH_IOCTL_BASE, 18)
#define TOUCH_IOCTL_TINT_OUTPUT			_IO(TOUCH_IOCTL_BASE, 19)
#define TOUCH_IOCTL_GET_ALL_REG			_IO(TOUCH_IOCTL_BASE, 20)

int spd2010_reset(void);
void spd2010_reset_pin_test(void);
int spd2010_force_bios(void);
int spd2010_goto_bios(void);
int spd2010_bootup_multifw(void);
int ts_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length);
int ts_read_data_ex(struct i2c_client *client, u8 *reg, u16 regLen,
		u8 *values, u16 length);
int ts_write_data(struct i2c_client *client,	u16 reg, u8 *values,
		u16 length);
void ts_write_data_burst(struct i2c_client *client,	u16 reg, u8 *values,
		u16 length);
int ds_read_boot_st(struct i2c_client *client, u16 *value);
int ds_eflash_write(struct i2c_client *client, int addr, u16 data);
int ds_eflash_read(struct i2c_client *client, int addr, u8 *rd, int rLen);

int spd2010_suspend_ex(void);
int spd2010_resume_ex(void);

/*-----------------------------------------------------
 *	firmware update
 *-----------------------------------------------------
 */
#define SUPPORT_TEST_MODE

#define CONTENT_HEADER_SIZE		12		/* bytes */

#define FW_MAX_RETRY_COUNT		3
#define FW_MAX_I2C_DATA_COUNT	256
#define FW_MAX_DATA_INFO_SIZE	8
#define FW_ERASE_ALL_PAGENUM	128

#define BOOT_UPDATE_ALL			1
#define BOOT_UPDATE_EACH		0

#define BOOT_UPDATE_OK			1
#define BOOT_UPDATE_NONE		0

/* ERROR NUMBER */
#define ERROR_TYPE_PARSING           0x81000000
#define ERROR_TYPE_UPDATE            0x82000000
#define ERROR_TYPE_VERSION           0x84000000
#define ERROR_TYPE_VERIFY            0x88000000
#define ERROR_TYPE_EFLASH            0x90000000
#define ERROR_TYPE_SYSTEM            0xA0000000

#define ERROR_SUCCESS			0
#define ERROR_PARSING_FILENAME_IS_NULL	(ERROR_TYPE_PARSING | 0x00000001)
#define ERROR_PARSING_FILE_OPEN_FAIL	(ERROR_TYPE_PARSING | 0x00000002)
#define ERROR_PARSING_FORMAT_INVALID	(ERROR_TYPE_PARSING | 0x00000003)
#define ERROR_PARSING_CHECKSUM_FAIL		\
				(ERROR_TYPE_PARSING | 0x00000004)
#define ERROR_PARSING_MALLOC_FAIL		\
				(ERROR_TYPE_PARSING | 0x00000005)
#define ERROR_PARSING_CONTENT_SIZE_FAIL	\
				(ERROR_TYPE_PARSING | 0x00000006)
#define ERROR_PARSING_DATA_CNT_FAIL		\
				(ERROR_TYPE_PARSING | 0x00000007)
#define ERROR_PARSING_HEADER_DATA_INVALID_LENGTH	\
	(ERROR_TYPE_PARSING | 0x00000008)

#define ERROR_PARSING_INVALID_DATATYPE	(ERROR_TYPE_PARSING | 0x00000009)

#define ERROR_UPDATE_INIT_FAIL		(ERROR_TYPE_UPDATE | 0x00000001)
#define ERROR_UPDATE_ERASE_FAIL		(ERROR_TYPE_UPDATE | 0x00000002)
#define ERROR_UPDATE_WRITE_FAIL		(ERROR_TYPE_UPDATE | 0x00000003)
#define ERROR_UPDATE_READ_FAIL		(ERROR_TYPE_UPDATE | 0x00000004)
#define ERROR_UPDATE_VERIFY_FAIL	(ERROR_TYPE_UPDATE | 0x00000005)

#define ERROR_EFLAH_ERASE_FAIL		(ERROR_TYPE_EFLASH | 0x00000001)
#define ERROR_EFLAH_WRITE_FAIL		(ERROR_TYPE_EFLASH | 0x00000002)
#define ERROR_EFLAH_READ_FAIL		(ERROR_TYPE_EFLASH | 0x00000003)
#define ERROR_EFLAH_VERIFY_FAIL		(ERROR_TYPE_EFLASH | 0x00000004)

#define ERROR_SYSTEM_FAIL			(ERROR_TYPE_SYSTEM | 0x00000001)

#define ERROR_VERSION_CHECK_FAIL	(ERROR_TYPE_VERSION | 0x00000001)

#define ERROR_VERIFY_VERIFY_FAIL	(ERROR_TYPE_VERIFY | 0x00000001)

/*-----------------------------------------------------
 *	key data
 *-----------------------------------------------------
 */
#ifdef SUPPORT_KEY_BUTTON
#define SSL_KEYDATA_BACK_DOWN		0x0001
#define SSL_KEYDATA_MENU_DOWN		0x0002
#define SSL_KEYDATA_HOME_DOWN		0x0004
#define SSL_KEYDATA_HOMEPAGE_DOWN	0x0008

#define SSL_KEYDATA_BACK_UP			0x0100
#define SSL_KEYDATA_MENU_UP			0x0200
#define SSL_KEYDATA_HOME_UP			0x0400
#define SSL_KEYDATA_HOMEPAGE_UP		0x0800
#endif	/* SUPPORT_KEY_BUTTON */

#ifdef SUPPORT_ES2
#define DS_VERSION_ES1		1
#define DS_VERSION_ES2		2
#endif
/* to support I2C read/write */
struct I2C_func {
	int (*ts_read_data)(struct i2c_client *, u16, u8 *, u16);
	int (*ts_read_data_ex)(struct i2c_client *, u8 *, u16, u8 *, u16);
	int (*ts_write_data)(struct i2c_client *, u16, u8 *, u16);
};

typedef union {
	struct {
		u8 id;
		u8 x_lsb;
		u8 y_lsb;
		u8 x_y_msb;
		u8 weight;
		u8 reserved01;
	} point;

	struct {
		u8 reserved01;
		u8 scantime_low_L;
		u8 scantime_low_H;
		u8 scantime_hi_L;
		u8 scantime_hi_H;
		u8 contact_count;
	} time;

	/* key data structure */
	struct {
		u8 id;
		u8 keyL;
		u8 keyH;
		u8 reserved01;
		u8 reserved02;
		u8 reserved03;
	} key;

	/* gesure data structure */
	struct {
		u8 id;
		u8 gestureL;
		u8 gestureH;
		u8 reserved01;
		u8 reserved02;
		u8 reserved03;
	} gesture;

	struct {
		u8 id;
		u8 codeL;
		u8 codeH;
		u8 reserved01;
		u8 reserved02;
		u8 reserved03;
	} aux;
} ssd_data;

struct spd2010_data {
	int packet_length;
	ssd_data points[MAX_POINT_NUMBER];
	s16 rawData[SPD2010_MAX_RAWDATA_QUEUE]
		[SPD2010_MAX_NODE +
		(SPD2010_MAX_X_NODE + SPD2010_MAX_Y_NODE + 2) +
		((SPD2010_MAX_Y_NODE * 4) + 2) +
		(6 * MAX_POINT_NUMBER) + 2];
	int data_info;
	int queue_front;
	int queue_rear;
	int queue_size;
	int validPointCnt;
	int lastValidCnt;
#ifdef SUPPORT_KEY_BUTTON
	u32	keydata;
#endif	/* SUPPORT_KEY_BUTTON */
	int point_info;
};

struct spd2010_config {
	u8 fw_ver[24];
	u16 data_ver;
	u16 max_x;		/* x resolution */
	u16 max_y;		/* y resolution */
	u16 max_weight;
	u16 using_point;
	u16 x_node;
	u16 y_node;
	bool check;
	u16 total_size;
#ifdef SUPPORT_TMC_I2C_LENGTH
	u16 i2c_length;
#endif	/* SUPPORT_TMC_I2C_LENGTH */
};

struct ssl_version {
	u32 display_version;
	u32 hidden_version;
	u32 productID01;
	u32 productID02;
	u32 ICName01;
	u32 ICName02;
};

struct ssl_device {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct atc260x_dev *atc260x;
	struct spd2010_data *ftdata;
	struct spd2010_config *ftconfig;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	int irq;
	int int_pin;
	int reset_pin;

	u16 touch_mode;
	u16 mptest_mode;
	u8 update;
	struct semaphore	raw_data_lock;

	u8 work_procedure;
	struct semaphore work_procedure_lock;

	/* ESD TIMER */
#if ESD_TIMER_ENABLE
	struct work_struct tmr_work;
	struct workqueue_struct *tmr_workqueue;
	u8 use_esd_tmr;
	bool in_esd_tmr;
	struct timer_list esd_tmr;
	struct timer_list *p_esd_tmr;
	unsigned long esd_check_time;
#endif
	struct I2C_func i2c_func;
	struct ssl_version fw_version;
	u16		boot_flag;
	u16		checksum_flag;
#ifdef SUPPORT_ES2
	u16		es_version;
#endif
#ifdef SUPPORT_ESD_CHECKSUM
	u16		state_flag;
#endif
	unsigned long tmc_start_time;
};

struct ssl_fw {
	int address;
	int byte_cnt;
	int erase_page_cnt;
	int version;
	unsigned int checksum;
	int reserved_01;
	int reserved_02;
	int reserved_03;
	unsigned char *content;
};

struct ssl_fw_info {
	int DVersion;
	int HVersion;
	int FWPart_Num;

	u32 PID;
	u32 ICName;
	u32 FW_FilePath;
	u32 FW_Name;
};

struct ssl_mem_info {
	int CPU;
	int CPUCFG;
	int SYSCFG;
	int TMCREG;
	int FDM;
	int MPFDM;
	int IRAM;
	int DRAM;

	int FPM;
	int MPFPM;
	int DCSW;

	int ERASE_SECTOR_SIZE;
};

/* structure for update */
struct ssl_fw_group {
	struct ssl_fw section;
	struct ssl_fw_group *next;
};

/* structure for header */
struct ssl_fw_group_header {
	struct ssl_version fw_version;
	struct ssl_fw_group *fw_group;
};

int ssl_firmware_update_byfile(struct ssl_device *dev,
		char *filename);
u8 *ssl_get_version(struct ssl_device *dev, u8 *ver_buff);
int ssl_free_header(struct ssl_device *dev);
#endif
