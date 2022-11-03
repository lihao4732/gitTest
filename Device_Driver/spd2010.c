/*
 * Copyright 2019 Solomon Ltd. All rights reserved.
 *
 * SPD2010 Touch device driver
 *
 * Date: 2020.02.12
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/mm.h>
#include <linux/firmware.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/version.h>
#include <linux/proc_fs.h>

#include "spd2010.h"

unsigned int spd2010_tp_report_panel_dead;

void __iomem *exynos_map;

int init_ds_flag;
int init_tmc_flag;
int probe_done_flag;
int init_startup_flag;

#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
int found_force_bin_file;
#endif

enum _ts_work_procedure {
	TS_NO_WORK = 0,
	TS_NORMAL_WORK,
	TS_ESD_TIMER_WORK,
	TS_IN_EALRY_SUSPEND,
	TS_IN_SUSPEND,
	TS_IN_RESUME,
	TS_IN_LATE_RESUME,
	TS_IN_UPGRADE,
	TS_REMOVE_WORK,
	TS_SET_MODE,
	TS_GET_RAW_DATA,
	TS_IN_INITIALIZE,
};

static int spd2010_send_key_du(struct input_dev *input, unsigned int key);
static int ssl_report(struct ssl_device *ftdev);
static int ssl_report_release(struct ssl_device *);
static int ssl_init_config(struct ssl_device *ftdev);
static int ssl_pre_init(struct ssl_device *ftdev);
static int ssl_init(struct ssl_device *ftdev);
static int ssl_power_control(struct ssl_device *ftdev, u8 mode);
static int ssl_hw_init(void);
static void ssl_hw_deint(void);
static inline s32 int_clear_cmd(struct i2c_client *client);
static int spd2010_startup(struct ssl_device *ftdev);
static int ssl_TINT_forcebios(void);
static void ssl_SCL_swingtest(void);
static int ssl_mptest(struct ssl_device *ftdev,
		struct spd2010_data *data);

struct _raw_ioctl {
	int	sz;
	u8	*buf;
};

struct _reg_ioctl {
	int	addr;
	int	*val;
};

struct _down_ioctl {
	int	sz;
	char *file;
};

struct _mp_ioctl {
	int mode;
	int count;
	int calldelay;
	int needback;
};


static u32 m_key_map[MAX_DEVICE_KEYMAP_SIZE] = {
	KEY_BACK, KEY_MENU, KEY_HOMEPAGE, KEY_HOMEPAGE,
	KEY_BACK, KEY_BACK, KEY_BACK, KEY_BACK,
	KEY_BACK, KEY_BACK, KEY_BACK, KEY_BACK,
	KEY_BACK, KEY_BACK, KEY_BACK, KEY_BACK,
};

#define KEY_MPTEST		KEY_KPSLASH

static int lpm_gesture_keys[] = {
	KEY_POWER, KEY_BACK, KEY_WAKEUP, KEY_MENU, KEY_HOMEPAGE
};

#define LPM_GESTURE_KEY_CNT	(sizeof(lpm_gesture_keys) /	\
		sizeof(lpm_gesture_keys[0]))

#ifdef SUPPORT_LPM
#define POWER_STATUS_NM		0x8000
#define POWER_STATUS_LPM	0x4000
#define LPM_RESUME		(POWER_STATUS_NM | 0x0001)
#define LPM_SUSPEND		(POWER_STATUS_LPM | 0x0001)
#define LPM_SUSPEND_DELAY	(POWER_STATUS_LPM | 0x0002)

int m_power_status = LPM_RESUME;	/* 0:resume , 1:suspend */

static int ssl_read_points(struct ssl_device *ftdev,
	struct spd2010_data *data);
#endif	/* SUPPORT_LPM */

int m_mp_total_count = -1;
int m_mp_cur_count;
int m_mp_skip_count;
int m_mp_calldealy;
int m_mp_needback;
int m_mp_status;

unsigned long m_point_skip_time;

struct ssl_device *misc_dev;

int mTotal_size;

static struct class *touchscreen_class;
u8 m_up_event[MAX_POINT_NUMBER] = {0,};

u8 rcv_buf[9000] = {0,};
u8 tempBuf[9000] = {0,};

/* Point Info status check function */
static int SNL_Read(struct i2c_client *client, int *status,
	int *nByte)
{
	int err = 0;
	int retry_cnt = 2;
	u8 SNL_buf[4] = {0};

	memset(SNL_buf, 0, 4);

snl_retry:
	err = ts_read_data(client, SPD2010_GET_STATUS,
		(u8 *)(SNL_buf), 4);

	if (err < 0) {
		ssl_dbg_msg("SnL read fail");
		if (retry_cnt-- > 0) {
			udelay(5);
			goto snl_retry;
		} else {
			return err;
		}
	} else {
		*status = SNL_buf[0] | (SNL_buf[1] << 8);
		*nByte = SNL_buf[2] | (SNL_buf[3] << 8);
	}

	return err;
}


/* HDP status check function */
static int HDP_Done_Check(struct i2c_client *client,
	int *nByte, int *status)
{
	int err = 0;
	int retry_cnt = 2;
	u8 HDP_Status[8] = {0};

	memset(HDP_Status, 0, 8);

hdp_retry:
	err = ts_read_data(client,
		SPD2010_CHECK_HDP, (u8 *)(HDP_Status), 8);

	if (err < 0) {
		ssl_dbg_msg("error : read HDP Status");
		if (retry_cnt-- > 0) {
			udelay(5);
			goto hdp_retry;
		} else {
			return err;
		}
	} else {
		/* Next Packet Length */
		*nByte = HDP_Status[2] | (HDP_Status[3] << 8);
		/* Status */
		*status = HDP_Status[5];
	}

	return err;
}

/* rawdata queue process */
static char *get_rear_queue_buff(struct spd2010_data *ftdata)
{
	char *ptr = NULL;

	if (ftdata->queue_size == SPD2010_MAX_RAWDATA_QUEUE)
		return NULL;

	ptr = (char *)&(ftdata->rawData[(ftdata->queue_rear + 1) %
			SPD2010_MAX_RAWDATA_QUEUE][0]);
	return ptr;
}

static int put_queue(struct spd2010_data *ftdata)
{
	if (ftdata->queue_size == SPD2010_MAX_RAWDATA_QUEUE)
		return -1;

	ftdata->queue_rear = (ftdata->queue_rear + 1) %
		SPD2010_MAX_RAWDATA_QUEUE;
	ftdata->queue_size += 1;

	return 0;
}

static u8 *get_front_queue_buff(struct spd2010_data *ftdata)
{
	int tmp = ftdata->queue_front;

	if (ftdata->queue_size == 0)
		return NULL;

	tmp = (ftdata->queue_front + 1) % SPD2010_MAX_RAWDATA_QUEUE;

	return (u8 *)&(ftdata->rawData[tmp][0]);
}

static int get_queue(struct spd2010_data *ftdata)
{
	if (ftdata->queue_size == 0)
		return -1;

	ftdata->queue_front = (ftdata->queue_front + 1) %
		SPD2010_MAX_RAWDATA_QUEUE;
	ftdata->queue_size -= 1;

	return 0;
}

/************************************************************
* irq Interface
*************************************************************/
/*
 * ts_enable_irq - enable irq function.
 */
void ts_enable_irq(void)
{
	enable_irq(misc_dev->irq);
}

/**
 * ts_irq_disable - disable irq function.
 *
 */
void ts_disable_irq(void)
{
	disable_irq(misc_dev->irq);
}

#ifdef SUPPORT_ESD_CHECKSUM
/* It calculate the checksum for ESD */
static int esd_checksum(u8 *data, int length, u8 *xor, u8 *sum)
{
	int ret = -1;
	int i = 0;

	if (data == NULL) {
		ssl_dbg_msg("data is null!!!");
		goto out;
	}

	if (length < 1) {
		ssl_dbg_msg("length(%d) is smaller than 1!!", length);
		goto out;
	}

	if (xor == NULL || sum == NULL) {
		ssl_dbg_msg(" xor or sum is null!!");
		goto out;
	}

	*xor = 0;
	*sum = 0;

	for (i = 0; i < length; i++) {
		*xor ^= data[i];
		*sum += data[i];
	}

	ret = 0;
out:
	return ret;
}
#endif	/* SUPPORT_ESD_CHECKSUM */

/* I2C read function. */
int ts_read_data(struct i2c_client *client, u16 reg,
	u8 *values, u16 length)
{
	s32 ret;

	/* select register*/
	ret = i2c_master_send(client, (u8 *)&reg, 2);
	if (ret < 0) {
		ssl_dbg_msg("I2C READ CMD FAIL (%d)", ret);
		return ret;
	}

	/* for setup tx transaction. */
	ret = i2c_master_recv(client, values, length);

	if (ret < 0) {
		ssl_dbg_msg("I2C READ DATA FAIL (%d) (0x%04X)",
			ret, reg);
		return ret;
	}

	udelay(60);

	return length;
}

/*
 * values : Buffer to store data
 * total_cnt : Size of data to be stored
 * length : Full frame length
*/

int ts_read_data_raw(struct ssl_device *ftdev,
	struct spd2010_data *data, u16 reg,
	u8 *values, int *total_cnt, u16 length)
{
	int i = 0;
	int final_flag = 0;
	s32 ret = 0;

	memset(rcv_buf, 0, 9000);

	/* HDP status read */
	ret = i2c_master_send(ftdev->client, (u8 *)&reg, 2);
	if (ret < 0) {
		ssl_dbg_msg("I2C READ CMD FAIL");
		return ret;
	}

	ret = i2c_master_recv(ftdev->client, rcv_buf, length);
	if (ret < 0) {
		ssl_dbg_msg("I2C READ DATA FAIL (%d) (0x%04X)",
			ret, reg);
		return ret;
	}

	/* Data Info */
	data->data_info = rcv_buf[DATA_INFO_LOC];
	if (((rcv_buf[DATA_INFO_LOC] >> 6) & 0x01) == 1)
		final_flag = 1;

	ftdev->ftconfig->total_size =
		rcv_buf[FULL_BYTE_LOW] | (rcv_buf[FULL_BYTE_HIGH] << 8);

	for (i = FULL_HEADER_LOC + FULL_HEADER_SIZE; i < length; i++)
		values[i - (FULL_HEADER_LOC + FULL_HEADER_SIZE)] = rcv_buf[i];

	udelay(20);

	if (ftdev->ftconfig->total_size ==
		length - (FULL_HEADER_LOC + FULL_HEADER_SIZE)) {
		mTotal_size = ftdev->ftconfig->total_size;

		if (final_flag == 1)
			return 1;
		else
			return ret;
	} else {
		ssl_dbg_msg("Error - total size : %d, full packet size : %d",
			ftdev->ftconfig->total_size, length);
		return ret;
	}
}

/* I2C read function. It support several byte for the write. */
int ts_read_data_ex(struct i2c_client *client, u8 *reg,
	u16 regLen,	u8 *values, u16 length)
{
	s32 ret;

	ssl_dbg_msg("read data (ex)");

	/* select register */
	ret = i2c_master_send(client, reg, regLen);
	if (ret < 0) {
		ssl_dbg_msg("[ts_read_data_ex] I2C READ CMD FAIL");
		return ret;
	}

	/* for setup tx transaction. */
	ret = i2c_master_recv(client, values, length);

	if (ret < 0) {
		ssl_dbg_msg("I2C READ DATA FAIL (%d)", ret);
		return ret;
	}

	return length;
}

/* I2C write function. */
int ts_write_data(struct i2c_client *client, u16 reg,
	u8 *values, u16 length)
{
	s32 ret;
	s32 retry = 2;
	u8 pkt[64];				/* max packet */

	memset(pkt, 0, 64);

	pkt[0] = (reg) & 0xff;	/* reg addr */
	pkt[1] = (reg >> 8) & 0xff;
	memcpy((u8 *)&pkt[2], values, length);

again:
	ret = i2c_master_send(client, pkt, length + 2);
	if (ret < 0) {
		ssl_dbg_msg("I2C WRITE FAIL : 0x%x / 0x%x",
			reg, pkt[2]);
		if ((retry--) > 0)
			goto again;

		return ret;
	}

	udelay(20);

	return length;
}

void ts_write_data_burst(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	s32 retry = 2;
	u8 pkt[520]; /* max packet */

	memset(pkt, 0, 520);

	pkt[0] = (reg)&0xff;	/* reg addr */
	pkt[1] = (reg >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

again:
	ret = i2c_master_send(client, pkt, length + 2);
	if (ret < 0) {
		ssl_dbg_msg("I2C WRITE FAIL : 0x%x / 0x%x",
			reg, pkt[2]);
		if ((retry--) > 0)
			goto again;

	}
}

#ifdef SUPPORT_TMC_I2C_LENGTH
static inline s32 ts_read_tmc_i2c(struct i2c_client *client,
	u16 *i2c_len)
{
	int err = 0;

	ssl_dbg_msg("read data (tmc)");

	err = ts_read_data(client, SPD2010_TMC_I2C_LENGTH,
		(u8 *)(i2c_len), 2);

	if (err < 0) {
		ssl_dbg_msg("error : read TMC I2C Length");
		return -EAGAIN;
	}

	ssl_info_msg("read TMC I2C Length : 0x%04x", *i2c_len);
	return err;
}

static inline s32 ts_write_tmc_i2c(struct i2c_client *client,
	u16 i2c_len)
{
	int err = 0;

	ssl_dbg_msg("write data (tmc)");

	err = ts_write_data(client, SPD2010_TMC_I2C_LENGTH,
			(u8 *)(&i2c_len), 2);

	if (err < 0) {
		ssl_dbg_msg("error : write TMC I2C Length");
		return -EAGAIN;
	}

	return err;
}
#endif	/* SUPPORT_TMC_I2C_LENGTH */

/* create gesture switch node */
#ifdef ENABLE_GESTURE
static struct proc_dir_entry *_ssdgProcGestureWakeupModeEntry;
static struct proc_dir_entry *_ssdgProcDeviceEntry;
static struct proc_dir_entry *_ssdgProcClassEntry;
u32 spd2010_gesture_status;
static const struct file_operations _ssdgProcGestureWakeupMode = {
	.read = ssdDrvMainProcfsGestureWakeupModeRead,
	.write = ssdDrvMainProcfsGestureWakeupModeWrite,
};

ssize_t ssdDrvMainProcfsGestureWakeupModeRead(struct file *pFile,
	char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;
	char nUserTempBuffer[16];

	if (*pPos != 0)
		return 0;

	nLength = sprintf(nUserTempBuffer, "%d",
		spd2010_gesture_status);

	if (copy_to_user(pBuffer, nUserTempBuffer, nLength))
		return -EFAULT;

	*pPos += nLength;

	return nLength;
}

ssize_t ssdDrvMainProcfsGestureWakeupModeWrite(struct file *pFile,
	const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	char gbuffer[16];

	memset(gbuffer, 0, sizeof(gbuffer));

	if (pBuffer != NULL) {
		if (copy_from_user(gbuffer, pBuffer, nCount))
			return -EFAULT;

		spd2010_gesture_status =
			simple_strtol(gbuffer, NULL, 0);
	}

	return nCount;
}

void ssdcreate_gesture_status_node(void)
{
	_ssdgProcClassEntry =
		proc_mkdir(SSD_PROC_NODE_CLASS, NULL);
	_ssdgProcDeviceEntry =
		proc_mkdir(SSD_PROC_NODE_DEVICE, _ssdgProcClassEntry);
	_ssdgProcGestureWakeupModeEntry =
		proc_create(SSD_PROC_NODE_GESTURE_WAKEUP_MODE,
		SSD_PROCFS_AUTHORITY, _ssdgProcDeviceEntry,
		&_ssdgProcGestureWakeupMode);
}
#endif
/* create gesture switch node */

/* get gesture */
static inline s32 ts_read_gesture(struct i2c_client *client,
		u16 *gesture, u16 length)
{
	int err = 0;

	ssl_dbg_msg("read data (gesture)");

	err = ts_read_data(client,
			SPD2010_GET_GESTURE, (u8 *)(gesture), length);

	if (err < 0) {
		ssl_dbg_msg("error : read Gesture");
		return -EAGAIN;
	}

	ssl_info_msg("read gesture : 0x%04x", *gesture);
	return err;
}

/* get key touch */
static inline s32 ts_read_keydata(struct i2c_client *client, u8 *keydata)
{
	int err = 0;

	ssl_dbg_msg("read data (key)");

	err = ts_read_data(client, SPD2010_GET_KEYDATA, (u8 *)(keydata), 2);

	if (err < 0) {
		ssl_dbg_msg("error : read key data");
		return -EAGAIN;
	}

	ssl_info_msg("read key data : 0x%02x%02x", keydata[1], keydata[0]);

	return err;
}

int ssl_set_intflag(struct i2c_client *client, int flag)
{
	return ts_write_data(client, SPD2010_INT_FLAG,
		(u8 *)&flag, 2);
}

int ssl_set_mpmode(struct i2c_client *client, int mode)
{
	return ts_write_data(client, SPD2010_MP_TEST,
		(u8 *)&mode, 2);
}

int ssl_chk_mpmode(struct i2c_client *client, int mode)
{
	int err = 0;
	int retry_cnt = 2;
	u8 data[2] = {0};

	memset(data, 0, 2);

rd_retry:
	err = ts_read_data(client,
		SPD2010_MP_TEST, (u8 *)(data), 2);

	if (err < 0) {
		ssl_dbg_msg("error : read MPTest mode");
		if (retry_cnt-- > 0) {
			udelay(5);
			goto rd_retry;
		} else {
			return err;
		}
	} else {
		if (data[0] == mode)
			return 1;
		else
			return 0;
	}

	return err;
}

int ssl_set_swcmd(struct i2c_client *client, int cmd)
{
	return ts_write_data(client, SPD2010_SW_COMMAND,
		(u8 *)&cmd, 2);
}

int ssl_gesture_init(struct ssl_device *ftdev)
{
#ifdef KEY_GESTURE_UP
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_UP);
#endif	/* KEY_GESTURE_UP */

#ifdef KEY_GESTURE_DOWN
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_DOWN);
#endif	/* KEY_GESTURE_DOWN */

#ifdef KEY_GESTURE_LEFT
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_LEFT);
#endif	/* KEY_GESTURE_LEFT */

#ifdef KEY_GESTURE_RIGHT
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_RIGHT);
#endif	/* KEY_GESTURE_RIGHT */

#ifdef KEY_GESTURE_DOUBLECLICK
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_DOUBLECLICK);
#endif	/* KEY_GESTURE_DOUBLECLICK */

#ifdef KEY_GESTURE_O
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_O);
#endif	/* KEY_GESTURE_O */

#ifdef KEY_GESTURE_W
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_W);
#endif	/* KEY_GESTURE_W */

#ifdef KEY_GESTURE_M
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_M);
#endif	/* KEY_GESTURE_M */

#ifdef KEY_GESTURE_E
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_E);
#endif	/* KEY_GESTURE_E */

#ifdef KEY_GESTURE_S
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_S);
#endif	/* KEY_GESTURE_S */

#ifdef KEY_GESTURE_Z
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_Z);
#endif	/* KEY_GESTURE_Z */

#ifdef KEY_GESTURE_C
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_C);
#endif	/* KEY_GESTURE_C */

#ifdef KEY_GESTURE_U
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_U);
#endif	/* KEY_GESTURE_U */

#ifdef KEY_GESTURE_U_DOWN
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_U_DOWN);
#endif	/* KEY_GESTURE_U_DOWN */

#ifdef KEY_GESTURE_U_RIGHT
	input_set_capability(ftdev->input_dev, EV_KEY, KEY_GESTURE_U_RIGHT);
#endif	/* KEY_GESTURE_U_RIGHT */

	return 0;
}

void ssl_gesture_report(struct ssl_device *ftdev,
	int gesture_code)
{
	ssl_dbg_msg("---> gesture_code = 0x%x",
		gesture_code);

	switch (gesture_code) {
#ifdef KEY_GESTURE_UP
	case GESTURE_UP:
		input_report_key(ftdev->input_dev, KEY_GESTURE_UP, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_UP, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_UP");
		break;
#endif	/* KEY_GESTURE_UP */
#ifdef KEY_GESTURE_DOWN
	case GESTURE_DOWN:
		input_report_key(ftdev->input_dev, KEY_GESTURE_DOWN, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_DOWN, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_DOWN");
		break;
#endif	/* KEY_GESTURE_DOWN */
#ifdef KEY_GESTURE_LEFT
	case GESTURE_LEFT:
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_LEFT, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_LEFT, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg(
			"---> reporting KEY_GESTURE_LEFT");
		break;
#endif	/* KEY_GESTURE_LEFT */
#ifdef KEY_GESTURE_RIGHT
	case GESTURE_RIGHT:
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_RIGHT, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_RIGHT, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg(
			"---> reporting KEY_GESTURE_RIGHT");
		break;
#endif	/* KEY_GESTURE_RIGHT */
#ifdef KEY_GESTURE_DOUBLECLICK
	case GESTURE_DOUBLECLICK:
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_DOUBLECLICK, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_DOUBLECLICK, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg(
		"---> reporting KEY_GESTURE_DOUBLECLICK");
		break;
#endif	/* KEY_GESTURE_DOUBLECLICK */
#ifdef KEY_GESTURE_O
	case GESTURE_O:
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_O, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_O, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg(
			"---> reporting KEY_GESTURE_O");
		break;
#endif	/* KEY_GESTURE_O */
#ifdef KEY_GESTURE_W
	case GESTURE_W:
		input_report_key(ftdev->input_dev, KEY_GESTURE_W, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_W, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_W");
		break;
#endif	/* KEY_GESTURE_W */
#ifdef KEY_GESTURE_M
	case GESTURE_M:
		input_report_key(ftdev->input_dev, KEY_GESTURE_M, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_M, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_M");
		break;
#endif	/* KEY_GESTURE_M */
#ifdef KEY_GESTURE_E
	case GESTURE_E:
		input_report_key(ftdev->input_dev, KEY_GESTURE_E, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_E, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_E");
		break;
#endif	/* KEY_GESTURE_E */
#ifdef KEY_GESTURE_S
	case GESTURE_S:
		input_report_key(ftdev->input_dev, KEY_GESTURE_S, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_S, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_S");
		break;
#endif	/* KEY_GESTURE_S */
#ifdef KEY_GESTURE_Z
	case GESTURE_Z:
		input_report_key(ftdev->input_dev, KEY_GESTURE_Z, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_Z, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_Z");
		break;
#endif	/* KEY_GESTURE_Z */
#ifdef KEY_GESTURE_C
	case GESTURE_C:
		input_report_key(ftdev->input_dev, KEY_GESTURE_C, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_C, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_C");
		break;
#endif	/* KEY_GESTURE_C */
#ifdef KEY_GESTURE_U
	case GESTURE_U:
		input_report_key(ftdev->input_dev, KEY_GESTURE_U, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev, KEY_GESTURE_U, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg("---> reporting KEY_GESTURE_U");
		break;
#endif	/* KEY_GESTURE_U */
#ifdef KEY_GESTURE_U_DOWN
	case GESTURE_U_DOWN:
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_U_DOWN, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_U_DOWN, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg(
			"---> reporting KEY_GESTURE_U_DOWN");
		break;
#endif	/* KEY_GESTURE_U_DOWN */
#ifdef KEY_GESTURE_U_RIGHT
	case GESTURE_U_RIGHT:
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_U_RIGHT, 1);
		input_sync(ftdev->input_dev);
		input_report_key(ftdev->input_dev,
			KEY_GESTURE_U_RIGHT, 0);
		input_sync(ftdev->input_dev);
		ssl_dbg_msg(
			"---> reporting KEY_GESTURE_U_RIGHT");
		break;
#endif	/* KEY_GESTURE_U_RIGHT */
	default:
		break;
	}
}

/* ESD TIMER */
#if ESD_TIMER_ENABLE
static void esd_checktime_init(struct ssl_device *ftdev)
{
	ftdev->esd_check_time = jiffies;
}

static void esd_timeout_handler(unsigned long data)
{
	struct ssl_device *ftdev = (struct ssl_device *)data;

	if (ftdev == NULL)
		return;

	ftdev->p_esd_tmr = NULL;
	queue_work(ftdev->tmr_workqueue, &ftdev->tmr_work);
}

static void esd_timer_start(u16 sec, struct ssl_device *ftdev)
{
	if (ftdev == NULL)
		return;

	if (ftdev->p_esd_tmr != NULL)
		del_timer(ftdev->p_esd_tmr);

	ftdev->p_esd_tmr = NULL;

	init_timer(&(ftdev->esd_tmr));
	ftdev->esd_tmr.data = (unsigned long)(ftdev);
	ftdev->esd_tmr.function = esd_timeout_handler;
	ftdev->esd_tmr.expires = jiffies + HZ*sec;
	ftdev->p_esd_tmr = &ftdev->esd_tmr;
	add_timer(&ftdev->esd_tmr);
}

static void esd_timer_stop(struct ssl_device *ftdev)
{
	unsigned int ret;

	if (ftdev == NULL)
		return;

	if (ftdev->p_esd_tmr)
		ret = del_timer(ftdev->p_esd_tmr);

	ftdev->p_esd_tmr = NULL;
}

static void touch_esd_tmr_work(struct work_struct *work)
{
	struct ssl_device *ftdev =
		container_of(work, struct ssl_device, tmr_work);

	ssl_info_msg("tmr queue work ++");
	if (ftdev == NULL) {
		ssl_dbg_msg("touch dev == NULL ?");
		goto fail_time_out_init;
	}

	if (down_trylock(&ftdev->work_procedure_lock)) {
		ssl_dbg_msg("fail to occupy sema");
		esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
		return;
	}

	if (ftdev->work_procedure != TS_NO_WORK) {
		ssl_dbg_msg("other process occupied (%d)",
				ftdev->work_procedure);
		up(&ftdev->work_procedure_lock);
		return;
	}

	ts_disable_irq();
	ftdev->work_procedure = TS_ESD_TIMER_WORK;

	ssl_dbg_msg("ESD TIMER(%d) %lu - %lu = %lu", HZ, jiffies,
			ftdev->esd_check_time, jiffies-ftdev->esd_check_time);

	if ((jiffies - ftdev->esd_check_time) > HZ * SPD2010_CHECK_ESD_TIMER) {
		ssl_dbg_msg("ESD Timer expired");

		ssl_report_release(ftdev);
		spd2010_tp_report_panel_dead = 1;
		ssl_dbg_msg("TP report panel dead, request recovery");
	}

	ftdev->work_procedure = TS_NO_WORK;

	ts_enable_irq();
	up(&ftdev->work_procedure_lock);
	ssl_info_msg("tmr queue work ----");

	esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
	return;

fail_time_out_init:
	ssl_info_msg("tmr work : restart error");
	ftdev->work_procedure = TS_NO_WORK;
	ts_enable_irq();
	up(&ftdev->work_procedure_lock);
	esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
}
#endif	/* ESD_TIMER_ENABLE */

/* get TCI1 fail reason */
static inline s32 ts_read_TCI1_Fail_reason(struct ssl_device *dev)
{
	int err = 0;
	u16 buff[3] = {0,};
	u8 *tmp = NULL;
	int fail_count = 0;
	int i = 0;

	ssl_dbg_msg("read data (fail reason)");

	err = ts_read_data(dev->client, SPD2010_TCI1_FAIL_REASON,
		(u8 *)buff, 6);

	if (err < 0) {
		ssl_info_msg("error : TCI1 Fail reason");
		return -EAGAIN;
	}

	tmp = (u8 *)buff;
	fail_count = tmp[0];
	ssl_dbg_msg("Fail Count : %d", fail_count);

	for (i = 0; i < fail_count; i++)
		ssl_dbg_msg("[%d] %d", i, tmp[i+1]);

	return err;
}

/* get TCI2 fail reason */
static inline s32 ts_read_TCI2_Fail_reason(struct ssl_device *dev)
{
	int err = 0;
	u16 buff[3] = {0,};
	u8 *tmp = NULL;
	int fail_count = 0;
	int i = 0;

	ssl_dbg_msg("read data (fail reason)");

	err = ts_read_data(dev->client, SPD2010_TCI2_FAIL_REASON,
		(u8 *)buff, 6);

	if (err < 0) {
		ssl_info_msg("error : TCI2 Fail reason");
		return -EAGAIN;
	}

	tmp = (u8 *)buff;
	fail_count = tmp[0];
	ssl_dbg_msg("Fail Count : %d", fail_count);

	for (i = 0; i < fail_count; i++)
		ssl_dbg_msg("[%d] %d", i, tmp[i+1]);

	return err;
}

static ssize_t ssl_fail_reason(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ssl_device *ftdev = i2c_get_clientdata(client);

	if (count > 0)
		if (buf[0] == 0x31)
			ts_read_TCI1_Fail_reason(ftdev);
		else
			ts_read_TCI2_Fail_reason(ftdev);

	return count;
}

static DEVICE_ATTR(fail_reason, S_IRUGO, NULL, ssl_fail_reason);

/* get the version info use kernel command. */
static ssize_t ssl_read_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char tmp_str[128] = {0};
	char ssd_return_str[512] = {0};
	char hex_str1[5] = {0};
	char hex_str2[5] = {0};
	int ret;

	memset(ssd_return_str, 0, sizeof(ssd_return_str));
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "################################\n");
	strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, " Display Version : 0x%08x\n",
		misc_dev->fw_version.display_version);
	strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "  Driver Version : %s\n", DRIVER_VRESION);
	strcat(ssd_return_str, tmp_str);
	hex_str1[0] = (misc_dev->fw_version.productID01
		& 0xFF000000) >> 24;
	hex_str1[1] = (misc_dev->fw_version.productID01
		& 0x00FF0000) >> 16;
	hex_str1[2] = (misc_dev->fw_version.productID01
		& 0x0000FF00) >> 8;
	hex_str1[3] = (misc_dev->fw_version.productID01
		& 0x000000FF) >> 0;
	hex_str2[0] = (misc_dev->fw_version.productID02
		& 0xFF000000) >> 24;
	hex_str2[1] = (misc_dev->fw_version.productID02
		& 0x00FF0000) >> 16;
	hex_str2[2] = (misc_dev->fw_version.productID02
		& 0x0000FF00) >> 8;
	hex_str2[3] = (misc_dev->fw_version.productID02
		& 0x000000FF) >> 0;
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "      Product ID : 0x%08x(%4s) 0x%08x(%4s)\n",
		misc_dev->fw_version.productID01, hex_str1,
		misc_dev->fw_version.productID02, hex_str2);
	strcat(ssd_return_str, tmp_str);
	hex_str1[0] = (misc_dev->fw_version.ICName01
		& 0xFF000000) >> 24;
	hex_str1[1] = (misc_dev->fw_version.ICName01
		& 0x00FF0000) >> 16;
	hex_str1[2] = (misc_dev->fw_version.ICName01
		& 0x0000FF00) >> 8;
	hex_str1[3] = (misc_dev->fw_version.ICName01
		& 0x000000FF) >> 0;
	hex_str2[0] = (misc_dev->fw_version.ICName02
		& 0xFF000000) >> 24;
	hex_str2[1] = (misc_dev->fw_version.ICName02
		& 0x00FF0000) >> 16;
	hex_str2[2] = (misc_dev->fw_version.ICName02
		& 0x0000FF00) >> 8;
	hex_str2[3] = (misc_dev->fw_version.ICName02
		& 0x000000FF) >> 0;
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "     IC Name : 0x%08x(%4s) 0x%08x(%4s)\n",
		misc_dev->fw_version.ICName01, hex_str1,
		misc_dev->fw_version.ICName02, hex_str2);
	strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "     Resolution : %d X %d\n",
		misc_dev->ftconfig->max_x, misc_dev->ftconfig->max_y);
	strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "     Node Info : %d X %d\n",
		misc_dev->ftconfig->x_node, misc_dev->ftconfig->y_node);
	strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "     Point Count : %d\n",
		misc_dev->ftconfig->using_point);
	strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0, sizeof(tmp_str));
	sprintf(tmp_str, "#################################\n");
	strcat(ssd_return_str, tmp_str);

	ret = snprintf(buf, PAGE_SIZE, "%s\n", ssd_return_str);
	return ret;
}
static DEVICE_ATTR(version, S_IRUGO, ssl_read_version, NULL);

static ssize_t class_ts_info_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "SPD2010 FW version: 0x%x\n",
		misc_dev->fw_version.display_version);
}

static CLASS_ATTR(ts_info, S_IRUSR, class_ts_info_show, NULL);

static ssize_t ssl_esd_time(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
#if ESD_TIMER_ENABLE
	struct i2c_client *client = to_i2c_client(dev);
	struct ssl_device *ftdev = i2c_get_clientdata(client);

	if (count > 0) {
		if (buf[0] == '0') {
			if (ftdev->use_esd_tmr) {
				ssl_dbg_msg("esd timer stop");
				esd_timer_stop(ftdev);
				ftdev->use_esd_tmr = 0;
			}
		} else {
			ssl_dbg_msg("esd timer start");
			ftdev->use_esd_tmr = 1;
			esd_checktime_init(ftdev);
			esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
		}
	}
#endif
	return count;
}

static DEVICE_ATTR(esdtime, S_IRUGO, NULL, ssl_esd_time);

/* get x, y node count */
static inline s32 ts_read_node(struct i2c_client *client, u8 *node_x,
		u8 *node_y)
{
	int err = 0;

	ssl_dbg_msg("read data (node)");

	err = ts_read_data(client, SPD2010_TOTAL_X_NODE, node_x, 2);

	if (err < 0) {
		ssl_dbg_msg("error : read x node");
		return -EAGAIN;
	}
	ssl_info_msg("x node : %d", *node_x);

	err = ts_read_data(client, SPD2010_TOTAL_Y_NODE, node_y, 2);
	if (err < 0) {
		ssl_dbg_msg("error : read y node");
		return -EAGAIN;
	}
	ssl_info_msg("y node : %d", *node_y);

	return err;
}

/* set resolution */
static inline s32 ts_write_resolution(struct i2c_client *client, u16 resol_x,
		u16 resol_y)
{
	int err = 0;

	ssl_dbg_msg("write data (res)");

	err = ts_write_data(client, SPD2010_X_RESOLUTION, (u8 *)&(resol_x), 2);

	if (err < 0) {
		ssl_dbg_msg("error : write x resolution!!");
		return -EAGAIN;
	}

	err = ts_write_data(client, SPD2010_Y_RESOLUTION, (u8 *)&(resol_y), 2);

	if (err < 0) {
		ssl_dbg_msg("error : write y resolution!!");
		return -EAGAIN;
	}

	return err;
}

/* set point num */
static inline s32 ts_write_point_num(struct i2c_client *client, u16 point)
{
	int err = 0;

	ssl_dbg_msg("write data (pt num)");

	err = ts_write_data(client, SPD2010_USING_POINT_NUM,
			(u8 *)&(point), 2);

	if (err < 0) {
		ssl_dbg_msg("error : write point num!!");
		return -EAGAIN;
	}

	return err;
}

int ds_read_boot_st(struct i2c_client *client, u16 *value)
{
	int ret = 0;
	u8 wd[10];
	u8 rd[10];

	wd[0] = 0x00;
	wd[1] = 0x00;
	wd[2] = 0x00;
	wd[3] = 0x00;

	ssl_dbg_msg("read data (st)");

	ret = ts_read_data_ex(client, wd, 4, rd, 2);

	if (ret < 0) {
		ssl_dbg_msg("read boot st i2c read fail(1)!!");
		return ret;
	}

	*value = rd[0] | (rd[1] << 8);

	return ret;
}

int ds_eflash_write(struct i2c_client *client, int addr, u16 data)
{
	int ret = 0;
	u8 wd[10];

	wd[0] = addr & 0xFF;
	wd[1] = (addr >> 8) & 0xFF;
	wd[2] = data & 0xFF;
	wd[3] = (data >> 8) & 0xFF;

	ssl_dbg_msg("write data (ds)");

	ret = ts_write_data(client, DS_EFLASH_WRITE, wd, 4);

	if (ret < 0) {
		ssl_dbg_msg("0x%04X i2c read fail(2)!!", DS_COMMAND_01);
		return ret;
	}

	return ret;
}

int ds_eflash_read(struct i2c_client *client, int addr, u8 *rd, int rLen)
{
	int ret = 0;
	u8 wd[10];

	wd[0] = DS_EFLASH_READ & 0xff;
	wd[1] = (DS_EFLASH_READ >> 8) & 0xff;
	wd[2] = addr & 0xff;
	wd[3] = (addr >> 8) & 0xff;

	ssl_dbg_msg("read data (ds)");

	ret = ts_read_data_ex(client, wd, 4, rd, rLen);

	if (ret < 0) {
		ssl_dbg_msg("0x%04X i2c read fail(1)!!", addr);
		return ret;
	}

	return ret;
}

static int ds_init_code(struct i2c_client *client)
{
	int ret = 0;

	ret = ds_eflash_write(client, 0xE003, 0x0007);

	if (ret < 0)
		return ret;

	ret = ds_eflash_write(client, 0xE000, 0x0048);

	if (ret < 0)
		return ret;

	return ret;
}

static int sint_unstall(struct i2c_client *client)
{
	s32 ret = 0;
	u32 temp_flag = 0x00;

	temp_flag = 0x0000;

	ssl_dbg_msg("write data (unstall)");

	ret = ts_write_data(client, DS_CUP_CONTROL, (u8 *)&temp_flag, 2);

	if (ret < 0) {
		ssl_dbg_msg("0x%04X i2c write fail!!",
			DS_CUP_CONTROL);
		return ret;
	}

	return ret;
}
/* send the clear command to TMC */
static inline s32 int_clear_cmd(struct i2c_client *client)
{
	int val = 0x1;

	return ts_write_data(client, SPD2010_INT_CLEAR_CMD,
		(u8 *)&val, 2);
}

/* Check interrupt pin */
static s32 int_pin_check(struct ssl_device *ftdev, int retry)
{
	int ret = 0;

	if (ftdev == NULL)
		return 0;

	if (retry == 0)
		return 0;

	do {
		if (gpio_get_value(ftdev->int_pin) == 0)
			break;

		mdelay(1);
	} while ((retry--) > 0);

	if (retry < 1)
		ret = -1;

	return ret;
}

#if ESD_TIMER_ENABLE
/* ESD Timer set to TMC
 * value : frame count. if you want to set 2sec, you should input 120(60x2).
 */
static s32 ssl_set_esdtime(struct ssl_device *ftdev, u16 value)
{
	int err;

	ssl_dbg_msg("ESD Time : %d", value);

	err = ts_write_data(ftdev->client,
			 SPD2010_ESD_TIME, (u8 *)&(value), 2);

	if (err < 0)
		ssl_dbg_msg("Fail to set ESD Time.");

	return err;
}
#endif

static s32 ssl_set_AFE_limit(struct ssl_device *ftdev,
	u16 max_value, u16 min_value)
{
	int err;

	ssl_dbg_msg("AFE limit : %d to %d",
		min_value, max_value);

	err = ts_write_data(ftdev->client,
		SPD2010_AFE_MAX_LIMIT, (u8 *)&(max_value), 2);

	if (err < 0)
		ssl_dbg_msg("Fail to set AFE Max limit");

	err = ts_write_data(ftdev->client,
		SPD2010_AFE_MIN_LIMIT, (u8 *)&(min_value), 2);

	if (err < 0)
		ssl_dbg_msg("Fail to set AFE Min limit");

	return err;
}

#ifdef SUPPORT_LPM
static inline s32 lpm_end(struct ssl_device *ftdev)
{
	s32 ret = 0;
	u32 temp_flag = 0x0000;
	int retry1 = 5;

	ssl_dbg_msg("write data (lpm)");

again:
	ret = ts_write_data(ftdev->client, DS_CUP_CONTROL,
			(u8 *)&temp_flag, 2);

	if (ret < 0)
		ssl_dbg_msg("0x%04X i2c write fail!!",
			DS_CUP_CONTROL);

	ssl_dbg_msg("retry = %d \t ret : 0x%04x",
		retry1, ret);

	retry1--;

	if (ret < 0 && retry1 > 0)
		goto again;

	return ret;
}

static inline s32 lpm_end_clear(struct ssl_device *ftdev)
{
	int err = 0;
	int retry = 500;

	err = int_pin_check(ftdev, retry);
	int_clear_cmd(ftdev->client);

	return err;
}

static inline s32 lpm_end2(struct ssl_device *ftdev)
{
	s32 ret = 0;
	u32 temp_flag = 0x0000;
	int retry = 5;

again:
	ssl_dbg_msg("lpm end2");
	ts_write_data(ftdev->client, DS_CUP_CONTROL,
		(u8 *)&temp_flag, 2);

	mdelay(1);

	ret = ts_write_data(ftdev->client, DS_CUP_CONTROL,
			(u8 *)&temp_flag, 2);

	if (ret < 0)
		ssl_dbg_msg("0x%04X i2c write fail!!",
			DS_CUP_CONTROL);

	retry--;

	if (ret < 0 && retry > 0)
		goto again;

	return ret;
}
#endif	/* SUPPORT_LPM */
static s32 ssl_set_mptest(struct ssl_device *ftdev, u16 value)
{
	int err;

	ssl_dbg_msg("set mptest");

	err = ts_write_data(ftdev->client,
			 SPD2010_MP_TEST, (u8 *)&(value), 2);
	if (err < 0)
		ssl_dbg_msg("Fail to set MP_TEST %d."
			, ftdev->mptest_mode);

	ftdev->mptest_mode = value;
	ftdev->ftdata->queue_front = 0;
	ftdev->ftdata->queue_rear = 0;
#if 0
	err = ts_read_data(ftdev->client,
			 SPD2010_MP_TEST, (u8 *)&(ftdev->mptest_mode), 2);
	if (err < 0)
		ssl_dbg_msg("err : read MP_TEST");

	ssl_info_msg("touch mode %d %d", value, ftdev->mptest_mode);
#endif

	return err;
}

static s32 ssl_set_mptest_alone(struct ssl_device *ftdev, u16 value)
{
	int err = 0;

	disable_irq(ftdev->irq);

	down(&ftdev->work_procedure_lock);

	if (ftdev->work_procedure != TS_NO_WORK) {
		ssl_info_msg("other process occupied.. (%d)\n",
			ftdev->work_procedure);
		enable_irq(ftdev->irq);
		up(&ftdev->work_procedure_lock);
		return -1;
	}

	ftdev->work_procedure = TS_SET_MODE;

	ssl_set_mptest(ftdev, value);

	ftdev->work_procedure = TS_NO_WORK;
	enable_irq(ftdev->irq);
	up(&ftdev->work_procedure_lock);

	return err;
}


static ssize_t spd2010_attr_mptest(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ssl_device *ftdev = i2c_get_clientdata(client);
	u16 mode = 0;

	if (count > 0) {
		mode = buf[0] - '0';

		ssl_dbg_msg("MODE : %d", mode);

		if (mode == 0) {
			ftdev->mptest_mode = (ftdev->mptest_mode |
						MPTEST_READY_STOP);
			m_mp_total_count = -1;
			m_mp_cur_count = 0;
			m_mp_calldealy = 0;
		} else {
			ftdev->ftdata->queue_front = 0;
			ftdev->ftdata->queue_rear = 0;
			ssl_set_mptest_alone(ftdev, mode);

			m_mp_total_count = -1;
			m_mp_cur_count = 0;
			if (mode >= MPTEST_LPM_NC_RANGE ||
				mode <= MPTEST_LPM_LC_RANGE)
				m_mp_total_count = 10;

			if (mode >= MPTEST_LPM_NC_RANGE &&
				mode <= MPTEST_LPM_LC_JITTER)
				m_mp_skip_count = 100;
			else
				m_mp_skip_count = 0;
		}
	}

	return count;
}

static DEVICE_ATTR(mptest, S_IRUGO, NULL, spd2010_attr_mptest);

static s32 spd2010_set_touchmode(u16 value)
{
	int err;
	int retry = 3;

	ts_disable_irq();

	ssl_dbg_msg("write data (set touchmode)");

	misc_dev->work_procedure = TS_NO_WORK;

	down(&misc_dev->work_procedure_lock);

	if (misc_dev->work_procedure != TS_NO_WORK) {
		ssl_dbg_msg("other process occupied.. (%d)\n",
				misc_dev->work_procedure);
		ts_enable_irq();
		up(&misc_dev->work_procedure_lock);
		return -1;
	}

	misc_dev->work_procedure = TS_SET_MODE;
retry:
	err = ts_write_data(misc_dev->client,
			SPD2010_TOUCH_MODE, (u8 *)&(value), 2);
	if (err < 0)
		ssl_dbg_msg("Fail to set TOUCH_MODE %d."
				, misc_dev->touch_mode);

	err = ts_read_data(misc_dev->client,
			SPD2010_TOUCH_MODE, (u8 *)&(misc_dev->touch_mode), 2);
	if (err < 0)
		ssl_dbg_msg("err : read touch_mode");

	ssl_info_msg("touch mode %d %d", value, misc_dev->touch_mode);
	if (value != misc_dev->touch_mode) {
		ssl_dbg_msg("touch mode set fail!!!");
		if ((retry--) > 0) {
			ssl_dbg_msg("retry!!");
			goto retry;
		}
	}

	m_point_skip_time = jiffies;

	misc_dev->work_procedure = TS_NO_WORK;
	ts_enable_irq();
	up(&misc_dev->work_procedure_lock);

	return err;
}

static ssize_t ssl_touchmode(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	u16 t = 0, i = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct ssl_device *ftdev = i2c_get_clientdata(client);

	if (count > 0) {
		for (i = 0; i < count-1; i++)
			t = t*10 + buf[i] - '0';

		ssl_dbg_msg("\ntouch mode : %d", t);
		ftdev->ftdata->queue_front = 0;
		ftdev->ftdata->queue_rear = 0;

		spd2010_set_touchmode(t);
	}

	return count;
}

static DEVICE_ATTR(touchmode, S_IRUGO, NULL, ssl_touchmode);


/* send sleep-in command to TMC */
static s32 spd2010_set_sleepin(struct ssl_device *ftdev)
{
	int err = 0x00;
#if 0
	u16 value = 0x00;
	int retry = 5;

	do {
		err = ts_write_data(misc_dev->client,
				SSL_SLEEP_IN, (u8 *)&(value), 2);

		if (err >= 0)
			break;

		ssl_dbg_msg("Fail to set SSL_SLEEP_IN.");
		mdelay(1);
	} while ((retry--) > 0);
#endif
	return err;
}

/* send sleep-out command to TMC */
static s32 spd2010_set_sleepout(struct ssl_device *ftdev)
{
	int err = 0x00;
#if 0
	u16 value = 0x00;
	int retry = 5;

	do {
		err = ts_write_data(misc_dev->client,
				SSL_SLEEP_OUT, (u8 *)&(value), 2);
		if (err >= 0)
			break;

		ssl_dbg_msg("Fail to set SSL_SLEEP_OUT.");
		mdelay(1);
	} while ((retry--) > 0);
#endif
	return err;
}

/* send stop scanning command to TMC */
static s32 spd2010_stop_scanning(struct ssl_device *ftdev)
{
	int err = 0x00;

	u16 value = 0x01;
	int retry = 5;

	ssl_dbg_msg("write data (stop scan)");

	do {
		err = ts_write_data(misc_dev->client,
				SPD2010_TMC_PAUSE, (u8 *)&(value), 2);

		if (err >= 0)
			break;

		ssl_dbg_msg("Fail to set SPD2010_TMC_PAUSE.");
		mdelay(1);
	} while ((retry--) > 0);

	value = 0x00;
	do {
		err = ts_write_data(misc_dev->client,
				SPD2010_TDFE_SCAN, (u8 *)&(value), 2);

		if (err >= 0)
			break;

		ssl_dbg_msg("Fail to set SPD2010_TDFE_SCAN.");
		mdelay(1);
	} while ((retry--) > 0);

	return err;
}

/* send start scanning command to TMC */
static s32 spd2010_start_scanning(struct ssl_device *ftdev)
{
	int err = 0x00;

	u16 value = 0x01;
	int retry = 5;

	ssl_dbg_msg("write data (start scan)");

	do {
		err = ts_write_data(misc_dev->client,
				SPD2010_TDFE_SCAN, (u8 *)&(value), 2);

		if (err >= 0)
			break;

		ssl_dbg_msg("Fail to set SPD2010_TDFE_SCAN.");
		mdelay(1);
	} while ((retry--) > 0);

	value = 0x00;
	do {
		err = ts_write_data(misc_dev->client,
				SPD2010_TMC_PAUSE, (u8 *)&(value), 2);

		if (err >= 0)
			break;

		ssl_dbg_msg("Fail to set SPD2010_TMC_PAUSE.");
		mdelay(1);
	} while ((retry--) > 0);

	return err;
}

/* IOCTL Driver */
static int ts_misc_fops_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ts_misc_fops_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	struct _raw_ioctl raw_ioctl;
	struct _reg_ioctl reg_ioctl;
	struct _down_ioctl down_ioctl;
	struct _mp_ioctl mp_ioctl;

	u8 *u8Data;
	int ret = 0;
	u16 val;
	int nval = 0;
	u8 send_cmd[2] = {0x00, 0x00};
	u8 dp_ver[24];
	char data[256];
	u8 reg_buf[600];
	u8 rdata[1152];
	int i;
	int retry_cnt = 10;

	for (i = 0; i < 1152; i++)
		rdata[i] = 1;

	if (misc_dev == NULL) {
		pr_info("misc_dev is NULL");
		return -1;
	}

	switch (cmd) {
	case TOUCH_IOCTL_TINT_OUTPUT:
		ssl_dbg_msg("============> goto BIOS\n");
		ssl_TINT_forcebios();
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = ts_read_data(misc_dev->client,
			SPD2010_DISPLAY_VER,
			(u8 *)(dp_ver), 24);

		if (ret < 0) {
			ssl_dbg_msg("display version read fail");
			return -1;
		}

		mdelay(10);

		if (copy_to_user(argp, dp_ver, sizeof(dp_ver)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_GESTURE:
		break;

	case TOUCH_IOCTL_GET_DATA_VERSION:
		ret = misc_dev->ftconfig->data_ver;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_dev->ftconfig->max_x;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_dev->ftconfig->max_y;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_dev->ftconfig->x_node;
		pr_info("send to apk : x node %d", ret);
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_dev->ftconfig->y_node;
		pr_info("send to apk : y node %d", ret);
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;

		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_dev->ftconfig->total_size;
		pr_info("send to apk : total size %d", ret);
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_I2C_UPLOAD_SIZE:
		ret = misc_dev->ftconfig->i2c_length;
		pr_info("send to apk : i2c size %d", ret);
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;

		break;

	case TOUCH_IOCTL_SET_TOUCH_MODE:
		if (copy_from_user(&nval, argp, 2))
			return -1;
		ssl_dbg_msg("Touch Mode : %d", nval);
		return spd2010_set_touchmode((u16)nval);

	case TOUCH_IOCTL_SW_RESET:
		sint_unstall(misc_dev->client);	/* TMC sw reset */
		ssl_dbg_msg("SW reset");
		break;

	case TOUCH_IOCTL_HW_RESET:
		ssl_dbg_msg("  SPD2010 Init cmd send");
		if (probe_done_flag == 1) {
RECHK_TINT:
			init_startup_flag = 0;

			spd2010_reset(); /* device reset */

			if (gpio_get_value(misc_dev->int_pin)) {
				/*interrupt pin is high, not valid data.*/
				ssl_info_msg("touch interrupt pin is high");
				return -1;
			} else {
				ret = spd2010_startup(misc_dev);
			}
			if (ret < 0) {
				ssl_dbg_msg(
					"  spd2010 startup failed");
				return ret;
			}
		} else {
			ssl_dbg_msg("  device driver load fail");
			return -1;
		}
		break;

	case TOUCH_IOCTL_MP_TEST:
		ts_disable_irq();
		misc_dev->work_procedure = TS_SET_MODE;

		if (copy_from_user(&mp_ioctl,
			argp, sizeof(struct _mp_ioctl)))
			return -1;

		ssl_dbg_msg("MP TEST Mode : %d",
			mp_ioctl.mode);
		ssl_dbg_msg("MP TEST Count : %d",
			mp_ioctl.count);
		ssl_dbg_msg("MP TEST delay : %d",
			mp_ioctl.calldelay);
		ssl_dbg_msg("MP TEST needback : %d",
			mp_ioctl.needback);

		if (mp_ioctl.mode == 0) {
			ssl_dbg_msg("------------ Stop mp mode");

			/* change MP mode */
			if (misc_dev->mptest_mode != MPTEST_STOP)
				ssl_set_mptest(misc_dev, MPTEST_STOP);

			m_mp_total_count = -1;
			m_mp_cur_count = 0;
			m_mp_calldealy = 0;
			m_mp_needback = 0;
			ret = 0;
		} else {
			misc_dev->mptest_mode = (mp_ioctl.mode |
						MPTEST_READY_START);
			m_mp_total_count = mp_ioctl.count;
			m_mp_calldealy = mp_ioctl.calldelay;
			m_mp_needback = mp_ioctl.needback;

			misc_dev->ftdata->queue_front = 0;
			misc_dev->ftdata->queue_rear = 0;

			m_mp_cur_count = m_mp_total_count;
			m_mp_skip_count = 10;

			/* sint_unstall(misc_dev->client); */
			int_clear_cmd(misc_dev->client);
			mdelay(m_mp_calldealy);

			ssl_dbg_msg("MP1 Mode:%d",
				misc_dev->mptest_mode);
			ssl_dbg_msg("MP1 Count : %d",
				m_mp_total_count);
			ssl_dbg_msg("MP1 delay : %d",
				m_mp_calldealy);
			ssl_dbg_msg("MP1 needback : %d",
				m_mp_needback);

			/* mp initialize status */
			m_mp_status = 0;
			ssl_mptest(misc_dev, misc_dev->ftdata);
		}

		misc_dev->work_procedure = TS_NO_WORK;
		ts_enable_irq();
		return ret;

	case TOUCH_IOCTL_GET_REG:
		down(&misc_dev->work_procedure_lock);
		if (misc_dev->work_procedure != TS_NO_WORK) {
			ssl_info_msg("other process occupied.. (%d)\n",
				misc_dev->work_procedure);
			up(&misc_dev->work_procedure_lock);
			return -1;
		}

		misc_dev->work_procedure = TS_SET_MODE;

		if (copy_from_user(&reg_ioctl,
					argp, sizeof(struct _reg_ioctl))) {
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ssl_dbg_msg("error : copy_from_user\n");
			return -1;
		}

		val = 0;
		if (ts_read_data(misc_dev->client,
					reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		nval = (int)val;

		if (copy_to_user(reg_ioctl.val,
			(u8 *)&nval, sizeof(nval))) {
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ssl_dbg_msg("error : copy_to_user\n");
			return -1;
		}

		ssl_info_msg("read : reg addr = 0x%x, val = 0x%x",
				reg_ioctl.addr, nval);

		misc_dev->work_procedure = TS_NO_WORK;
		up(&misc_dev->work_procedure_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:
		down(&misc_dev->work_procedure_lock);
		if (misc_dev->work_procedure != TS_NO_WORK) {
			copy_from_user(&reg_ioctl,
					argp, sizeof(struct _reg_ioctl));
			ssl_dbg_msg("other process (%d), 0x%x, 0x%x",
					misc_dev->work_procedure,
					reg_ioctl.addr, val);
			up(&misc_dev->work_procedure_lock);
			return -1;
		}

		misc_dev->work_procedure = TS_SET_MODE;

		if (copy_from_user(&reg_ioctl,
					argp, sizeof(struct _reg_ioctl))) {
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ssl_dbg_msg("error : copy_from_user\n");
			return -1;
		}

		if (copy_from_user(&val, reg_ioctl.val, 2)) {
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ssl_dbg_msg("error : copy_from_user\n");
			return -1;
		}

		if (ts_write_data(misc_dev->client,
					reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		ssl_info_msg("write : reg addr = 0x%x, val = 0x%x",
				reg_ioctl.addr, val);

		misc_dev->work_procedure = TS_NO_WORK;
		up(&misc_dev->work_procedure_lock);
		return ret;

	case TOUCH_IOCTL_GET_GRAPH_DATA:
		down(&misc_dev->work_procedure_lock);
		if (misc_dev->work_procedure != TS_NO_WORK) {
			ssl_dbg_msg("other process occupied.. (%d)\n",
					misc_dev->work_procedure);
			up(&misc_dev->work_procedure_lock);
			return -1;
		}

		misc_dev->work_procedure = TS_SET_MODE;

		if (copy_from_user(&reg_ioctl,
					argp, sizeof(struct _reg_ioctl))) {
			ssl_dbg_msg("error : copy_from_user\n");
			ret = -1;
			goto out_graph;
		}

		val = 0;
		if (ts_read_data(misc_dev->client,
					reg_ioctl.addr, (u8 *)data, 64) < 0) {
			ret = -1;
			goto out_graph;
		}

		if (copy_to_user(reg_ioctl.val, (u8 *)&data, 64)) {
			ssl_dbg_msg("error : copy_to_user\n");
			ret = -1;
		}

		ssl_info_msg("read : reg addr = 0x%x", reg_ioctl.addr);
out_graph:
		misc_dev->work_procedure = TS_NO_WORK;
		up(&misc_dev->work_procedure_lock);
		return ret;

	case TOUCH_IOCTL_QUEUE_CLEAR:
		misc_dev->ftdata->queue_front = 0;
		misc_dev->ftdata->queue_rear = 0;
		/* renewal queue */
		misc_dev->ftdata->queue_size = 0;

		ssl_dbg_msg("Rawdata queue clear!!!\n");
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_dev->touch_mode == TOUCH_POINT_MODE) {
			ssl_dbg_msg("error : POINT mode");
			return -1;
		}

		if (copy_from_user(&raw_ioctl,
					argp, sizeof(raw_ioctl))) {
			ssl_dbg_msg("error : copy_from_user");
			return -1;
		}

		if (misc_dev->ftdata->queue_size == 0)
			return -1;

		u8Data = get_front_queue_buff(misc_dev->ftdata);

		/*printk("-%x,%x,%d(%d)-", u8Data[8], u8Data[9],
		 *	(u8Data[9] << 8 | u8Data[8]), mTotal_size);
		 */

		if (copy_to_user(raw_ioctl.buf,
					u8Data, mTotal_size)) {
			ssl_dbg_msg("error : copy_to_user %d",
					mTotal_size);
			return -1;
		}

		ret = get_queue(misc_dev->ftdata);
		if (ret == -1)
			ssl_dbg_msg("get_queue() - error return");

		return mTotal_size;

	case TOUCH_IOCTL_SET_DOWNLOAD:
		ssl_dbg_msg("enter ioctl download mode");
		ts_disable_irq();
		down(&misc_dev->work_procedure_lock);
		misc_dev->work_procedure = TS_IN_UPGRADE;

#if ESD_TIMER_ENABLE
		if (misc_dev->use_esd_tmr)
			esd_timer_stop(misc_dev);
#endif

		memset(&down_ioctl, 0, sizeof(struct _down_ioctl));

		if (copy_from_user(&down_ioctl,
					argp, sizeof(struct _down_ioctl))) {

			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ts_enable_irq();
			ssl_dbg_msg("error : copy_from_user\n");
			return -1;
		}

		if (copy_from_user(&data,
					down_ioctl.file, 256)) {
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ts_enable_irq();
			ssl_dbg_msg("error : copy_from_user\n");
			return -1;
		}

		ssl_dbg_msg("NAME : %s", down_ioctl.file);
		ssl_dbg_msg("SIZE : %d", down_ioctl.sz);

		/* HW Reset & Wait BIOS Interrupt */
		spd2010_reset();

		/* Enter the download sequence */
		ret = ssl_firmware_update_byfile(misc_dev,
				down_ioctl.file);

		if (ret > 0)
			ssl_dbg_msg("firmware update by file failed");
		else
			ssl_dbg_msg(
		"===== firmware update by file succeed =====");

		/* CPU Restart */
		init_startup_flag = 0;
		spd2010_startup(misc_dev);

		misc_dev->work_procedure = TS_NO_WORK;
		up(&misc_dev->work_procedure_lock);
		ts_enable_irq();

		return ret;

	case TOUCH_IOCTL_GET_ALL_REG:
		ssl_dbg_msg("Get all register");

		if (copy_from_user(&reg_ioctl,
					argp, sizeof(struct _reg_ioctl))) {
			ssl_dbg_msg("error : copy_from_user\n");
			ret = -1;
		}

		if (ts_read_data(misc_dev->client,
					SPD2010_GET_REGISTER,
					(u8 *)reg_buf, 600) < 0) {
			ssl_dbg_msg("error : get register fail");
			ret = -1;
		}

		if (copy_to_user(reg_ioctl.val, (u8 *)&reg_buf, 600)) {
			ssl_dbg_msg("error : copy_to_user");
			ret = -1;
		}
		break;

	default:
		break;
	}

	return 0;
}

static const struct file_operations ts_misc_fops = {
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_release,
	.unlocked_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ssl_touch_misc",
	.fops = &ts_misc_fops,
};

/* power control */
static int ssl_power_control(struct ssl_device *ftdev, u8 mode)
{
	int err = 0;

	if (mode == POWER_OFF) {
	} else if (mode == POWER_ON) {
	} else if (mode == POWER_ON_SEQUENCE) {
		ssl_info_msg("POWER_ON_SEQUENCE");
#ifdef SUPPORT_ESD_CHECKSUM
		ftdev->state_flag = 0;
#endif
		spd2010_reset();
	} else if (mode == POWER_RESET) {
		ssl_info_msg("POWER_RESET");
#ifdef SUPPORT_ESD_CHECKSUM
		ftdev->state_flag = 0;
#endif
		spd2010_reset();
	}

	return err;
}

int ssl_TINT_forcebios(void)
{
	ssl_dbg_msg("forcebios test\n");
	ts_disable_irq();
	gpio_direction_output(SPD2010_RESET_PIN, 0);
	/*gpio_direction_output(SPD2010_INT_PIN, 0);*/
	gpio_direction_output(SPD2010_I2C_SCL, 0);

	gpio_direction_input(SPD2010_RESET_PIN);
	mdelay(3);

	/*gpio_direction_input(SPD2010_INT_PIN);*/
	gpio_direction_input(SPD2010_I2C_SCL);

	ts_enable_irq();
}

void ssl_SCL_swingtest(void)
{
	int i = 0;

	for (i = 0; i < 10; i++) {
		ssl_dbg_msg("SCL pin test");
		gpio_direction_output(SPD2010_I2C_SCL, 0);
		mdelay(50);
		gpio_direction_input(SPD2010_I2C_SCL);
		mdelay(50);
	}
}

void spd2010_reset_pud_set(void)
{
	int verify = 0;

	exynos_map = ioremap(0x13400000, SZ_4K);
	writel(0x5515, exynos_map + 0x1A8);
	verify = readl(exynos_map + 0x1A8);
}

void spd2010_reset_pin_test(void)
{
/****************************************************
*	int retry = 50;
*	int test = 0;
*
*	exynos_map = ioremap(0x13400000, SZ_4K);
*	test = readl(exynos_map + 0x1A8);
*	writel(0x5515, exynos_map + 0x1A8);
*	test = readl(exynos_map + 0x1A8);
*/
	int retry = 50;

	while (retry-- != 0) {
		gpio_direction_input(SPD2010_RESET_PIN);
		msleep(1200);
		gpio_direction_output(SPD2010_RESET_PIN, 0);
		msleep(1200);
	}
/******************************************************/
}

int spd2010_reset(void)
{
	int retry = 100;

	gpio_direction_output(SPD2010_RESET_PIN, 0);
	msleep(4);
	gpio_direction_input(SPD2010_RESET_PIN);

	/* wait tic init; */
	if (misc_dev != NULL)
		int_pin_check(misc_dev, retry * 2);

	return 0;
}

int spd2010_force_bios(void)
{
	gpio_direction_output(SPD2010_I2C_SCL, 0);
	/*gpio_direction_output(SPD2010_INT_PIN, 0);*/
	gpio_direction_output(SPD2010_RESET_PIN, 0);
	msleep(4);
	gpio_direction_input(SPD2010_RESET_PIN);
	mdelay(4);
	gpio_direction_input(SPD2010_I2C_SCL);
	/*gpio_direction_input(SPD2010_INT_PIN);*/
	msleep(10);
}

int spd2010_goto_bios(void)
{
	int retry = 100;
	int val = 0x10;

	ssl_dbg_msg("TP go to bios");

	ts_write_data(misc_dev->client,
		SPD2010_GOTO_BIOS, (u8 *)&val, 2);

	int_pin_check(misc_dev, retry * 2);

	return 0;
}

int spd2010_bootup_multifw(void)
{
	int ret = 0;
	u8 wd[4];

	wd[0] = 0x00;
	wd[1] = 0x00;
	wd[2] = 0x00;
	wd[3] = 0x00;

	ssl_dbg_msg("write data (bootup multifw)");

	ret = ts_write_data(misc_dev->client,
		DS_BOOTUP_MULTIFW, wd, 4);

	if (ret < 0) {
		ssl_dbg_msg("0x%04X i2c write fail(2)!!",
			DS_BOOTUP_MULTIFW);
		return ret;
	}

	return ret;
}

/* Read HW info use I2C */
int ssl_read_hw_info(struct i2c_client *client, u16 *info)
{
	int err = 0;

	ssl_dbg_msg("read data (hw info)");

	err = ts_read_data(client, SPD2010_HW_CAL_INFO,
		(u8 *)(info), 2);

	if (err < 0)
		ssl_dbg_msg("error read HW cal info using i2c.-");

	return err;
}

/* Write HW info use I2c */
int ssl_write_hw_cal(struct i2c_client *client)
{
	int err = 0;
	u16 info = 0x01;

	ssl_dbg_msg("write data (hw cal)");

	err = ts_write_data(client, SPD2010_HW_CAL,
		(u8 *)&(info), 2);

	if (err < 0)
		ssl_dbg_msg("error HW cal info write (0x%04x)!!",
			info);

	return err;
}

int ssl_hw_check(struct ssl_device *ftdev)
{
	int err = 0;
	int retry1 = 3;
	int limit = 0;
	u16 info = 0;

	ssl_dbg_msg("read data (hw chk)");

	err = ssl_read_hw_info(ftdev->client, &info);

	if (err >= 0) {
		ssl_dbg_msg("HW Cal info = %d", info);

		if (info == 0) {
			do {
				limit = 5;
				err = ssl_write_hw_cal(ftdev->client);

				if (err < 0)
					continue;

				do {
					ssl_read_hw_info(ftdev->client,
							&info);
					if (info != 0)
						break;

					mdelay(10);
				} while ((limit--) > 0);
			} while (info == 0 && retry1-- > 0);

			if (info == 1)
				ssl_dbg_msg("HW Cal Success!!");
			else
				ssl_dbg_msg("HW Cal Fail!!");
		}
	}

	return 0;
}

static int ssl_pre_init(struct ssl_device *ftdev)
{
	int ret = 0;

	return ret;
}

static int ssl_init(struct ssl_device *ftdev)
{
	int ret = 0;

	ret = ssl_init_config(ftdev);
	if (ret < 0) {
		ssl_dbg_msg("Read config failed");
		return ret;
	}

	return ret;
}

static int ssl_hw_init(void)
{
	int ret = 0;

	if (gpio_is_valid(misc_dev->int_pin)) {
		ret = gpio_request(misc_dev->int_pin, "int_pin");
		if (ret) {
			pr_err("Unable to request int_pin [%d]\n",
				misc_dev->int_pin);
			ret = -EINVAL;
			goto err_pwr_off;
		}
		ret = gpio_direction_input(misc_dev->int_pin);
		if (ret) {
			pr_err("Unable to set direction for int_pin [%d]\n",
				misc_dev->int_pin);
			goto err_free_int_pin;
		}
	} else {
		pr_err("Invalid int_pin [%d]!\n", misc_dev->int_pin);
		ret = -EINVAL;
		goto err_pwr_off;
	}

#if defined(SUPPORT_TOUCH_RESET_PIN_CTL)
	if (gpio_is_valid(misc_dev->reset_pin)) {
		ret = gpio_request(misc_dev->reset_pin, "reset_pin");
		if (ret) {
			pr_err("Unable to request reset_pin [%d]\n",
				misc_dev->reset_pin);
			goto err_free_int_pin;
		}

		ret = gpio_direction_output(misc_dev->reset_pin, 1);
		if (ret) {
			pr_err("Unable to set direction for reset gpio [%d]\n",
				misc_dev->reset_pin);
			goto err_free_reset_pin;
		}
	} else {
		pr_err("Invalid reset_pin [%d]!\n", misc_dev->reset_pin);
		ret = -EINVAL;
		goto err_free_int_pin;
	}
#endif

	return ret;

#if defined(SUPPORT_TOUCH_RESET_PIN_CTL)
err_free_reset_pin:
	if (gpio_is_valid(misc_dev->reset_pin))
		gpio_free(misc_dev->reset_pin);
#endif
err_free_int_pin:
	if (gpio_is_valid(misc_dev->int_pin))
		gpio_free(misc_dev->int_pin);
err_pwr_off:
	return ret;
}

static void ssl_hw_deint(void)
{
#if defined(SUPPORT_TOUCH_RESET_PIN_CTL)
	if (gpio_is_valid(misc_dev->reset_pin))
		gpio_free(misc_dev->reset_pin);
#endif
	if (gpio_is_valid(misc_dev->int_pin))
		gpio_free(misc_dev->int_pin);
}

static int ssl_get_rawdata(struct ssl_device *ftdev,
		u8 *rawdata, int size)
{
	int err = 0;
	int sz = 0, len = 0;
	u8 *buffer = NULL;
	int max = 50;

	buffer = rawdata;
	sz = size;
	len = ftdev->ftconfig->i2c_length;	/* I2C length */

	ssl_dbg_msg("read data (get raw)");

	while (sz > 0) {
		if (sz <= ftdev->ftconfig->i2c_length)
			len = sz;

		sz -= ftdev->ftconfig->i2c_length;
		err = ts_read_data(ftdev->client,
			SPD2010_RAW_DATA, buffer, len);

		if (err < 0) {
			ssl_info_msg("error : read raw data");
			break;
		}

		buffer = buffer + len;
		if (--max < 0) {
			err = -EAGAIN;
			break;
		}
	}

	return err;
}

static int ssl_get_rawdata_by_queue(struct ssl_device *ftdev)
{
	int err = 0;
	u8 *buffer = NULL;
	int sz = 0;
	struct spd2010_data *data = NULL;

	data = ftdev->ftdata;

	buffer = get_rear_queue_buff(data);

	if (buffer == NULL) {
		ssl_dbg_msg("rawdata queue is full");
		err = -EAGAIN;
		goto out;
	}

	sz = (ftdev->ftconfig->x_node * ftdev->ftconfig->y_node) * 2;

	err = ssl_get_rawdata(ftdev, buffer, sz);

	put_queue(data);

out:
	return err;
}

static int ssl_read_mptest(struct ssl_device *ftdev)
{
	int err = 0;

	ssl_get_rawdata_by_queue(ftdev);

	if (m_mp_skip_count > 0) {
		ssl_dbg_msg("rawdata skip!!");
		get_queue(ftdev->ftdata);
		if (m_mp_skip_count == 1) {
			ftdev->ftdata->queue_front = 0;
			ftdev->ftdata->queue_rear = 0;
		}
		m_mp_skip_count--;
		return err;
	}

	if ((ftdev->mptest_mode & MPTEST_READY_STOP) ==
		MPTEST_READY_STOP) {
		ssl_dbg_msg("STOP MPTEST");
		/* change MP mode */
		ssl_set_mptest(ftdev, MPTEST_STOP);
		/* TMC sw reset */
		sint_unstall(ftdev->client);
	} else {
		if (m_mp_cur_count == 0) {
			ssl_dbg_msg("UP key (count:%d)",
				m_mp_cur_count);
			if (m_mp_needback > 0) {
				/* wake up AP */
				spd2010_send_key_du(ftdev->input_dev,
								KEY_BACK);
				mdelay(m_mp_needback);
			} else {
				m_mp_cur_count = m_mp_total_count;
			}

			/* send get rawdata event key to APK */
			spd2010_send_key_du(ftdev->input_dev, KEY_MPTEST);

			m_mp_cur_count--;
		} else if (m_mp_cur_count > 0) {
			m_mp_cur_count--;
		}
	}

	return err;
}

static int ssl_read_mpdata(struct ssl_device *ftdev,
		struct spd2010_data *data)
{
	int err = 0;
	int loop_status = 1;
	int total_cnt = 0;
	int hdp_nByte = 0;
	int get_status = 0;
	int snl_status = 0;
	int snl_length = 0;
	u32 total_size = 0;
	u32 total_node = ftdev->ftconfig->x_node * ftdev->ftconfig->y_node;

	char *buffer;

	memset(tempBuf, 0, 9000);

	if (!ftdev || !data)
		return -EFAULT;

	if (gpio_get_value(ftdev->int_pin)) {
		/*interrupt pin is high, not valid data.*/
		ssl_info_msg("read rawdata... inturrpt pin is high");
		return 0;
	}

	if (down_trylock(&ftdev->raw_data_lock)) {
		ssl_dbg_msg("fail to occupy sema(%s)", __func__);
		goto out;
	}

	/* SnL Read */
	err = SNL_Read(ftdev->client, &snl_status, &snl_length);

	if (snl_length > 0) {
		while (loop_status > 0) {
			/* read hdp status */
			err = HDP_Done_Check(ftdev->client,
				&hdp_nByte, &get_status);

			if (err < 0) {
				ssl_dbg_msg("%s: fail to get HDP status",
					__func__);
				goto out;
			}

			if (get_status == 0x82) {
				/* Clear INT */
				loop_status = 0;
				int_clear_cmd(ftdev->client);
			} else if (get_status == 0x00) {
				if (data->queue_size ==
					SPD2010_MAX_RAWDATA_QUEUE) {
					printk("f");

					data->packet_length = hdp_nByte;
					err = ts_read_data_raw(ftdev,
						data, SPD2010_HDP,
						(u8 *)tempBuf, &total_cnt,
						hdp_nByte);

					udelay(20);
					ftdev->update = 1;
				} else {
					buffer = get_rear_queue_buff(data);

					data->packet_length = hdp_nByte;
					err = ts_read_data_raw(ftdev,
						data, SPD2010_HDP,
						(u8 *)buffer, &total_cnt,
						hdp_nByte);

					total_size = 0;

					/* calcutate total size */
					if (((data->data_info >> MUTUAL)
						& 0x01) == 1)
					total_size += ftdev->ftconfig->x_node *
					ftdev->ftconfig->y_node;

					if (((data->data_info >> SELF)
						& 0x01) == 1)
					total_size += (ftdev->ftconfig->x_node +
					ftdev->ftconfig->y_node) + 2;

					if (((data->data_info >> NMS)
						& 0x01) == 1)
					total_size +=
						(ftdev->ftconfig->y_node * 4) + 2;

					mTotal_size = total_size * 2;

					if ((total_size * 2) >
					ftdev->ftconfig->i2c_length &&
					err == 1) {
						printk("+");
						put_queue(data);
						ftdev->update = 1;
					} else if ((total_size * 2) >
					ftdev->ftconfig->i2c_length &&
					err != 1) {
						printk("_");
					} else {
						printk(".");
						put_queue(data);
						ftdev->update = 1;
					}
				}
			}
		}
	}

out:
	up(&ftdev->raw_data_lock);
	int_clear_cmd(ftdev->client); /* add by Eddie (20.12.15) */
	return err;
}

static int ssl_mptest(struct ssl_device *ftdev,
	struct spd2010_data *data)
{
	int err = 0;
	int limit_cnt = 0;
	int mp_item = misc_dev->mptest_mode ^ MPTEST_READY_START;
	u8 status_data[2];

	if (!ftdev || !data)
		return -EFAULT;

	if (down_trylock(&ftdev->raw_data_lock)) {
		ssl_dbg_msg("fail to occupy sema(%s)", __func__);
		goto out_2;
	}

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		ssl_info_msg("esd timer stop");
		esd_timer_stop(ftdev);
	}
#endif	/* ESD_TIMER_ENABLE */

	ssl_dbg_msg("set mp mode to STOP");
	err = ssl_set_mpmode(ftdev->client, 0x00);
	if (err < 0)
		ssl_dbg_msg("error in ssl_set_mpmode");

	mdelay(20);

	err = ssl_chk_mpmode(ftdev->client, 0x00);
	if (err != 1) {
		ssl_dbg_msg("fail to set mpmode");
		goto out_2;
	}

	ssl_dbg_msg("clear INT");
	err = int_clear_cmd(ftdev->client);

	mdelay(20);

	ssl_dbg_msg("send 0x0b01 sw_cmd");
	err = ssl_set_swcmd(ftdev->client, 0x0b01);
	if (err < 0) {
		ssl_dbg_msg("fail to send sw cmd 0x0b01");
		goto out_2;
	}

	mdelay(20);

	ssl_dbg_msg("check status");
	do {
		err = ts_read_data(ftdev->client,
			SPD2010_GET_STATUS, (u8 *)(&status_data), 2);

		if (err < 0) {
			ssl_dbg_msg("error read mpdata info using i2c.-");
			goto out_2;
		} else {
			if (limit_cnt >= 100) {
				ssl_dbg_msg("status receive timeout");
				goto out_2;
			} else {
				limit_cnt += 1;
			}
		}
	} while ((!(status_data[1] == 0x08)) && (limit_cnt <= 100));

	ssl_dbg_msg("status check OK");

	ssl_dbg_msg("set mp mode to %d", mp_item);
	err = ssl_set_mpmode(ftdev->client, mp_item);
	if (err < 0)
		ssl_dbg_msg("error in ssl_set_mpmode");

	mdelay(20);

	err = ssl_chk_mpmode(ftdev->client, mp_item);
	if (err != 1) {
		ssl_dbg_msg("fail to set mpmode");
		goto out_2;
	}

	ssl_dbg_msg("clear INT");
	err = int_clear_cmd(ftdev->client);
	mdelay(20);

	ssl_dbg_msg("send 0x0901 sw_cmd");
	err = ssl_set_swcmd(ftdev->client, 0x0901);
	if (err < 0) {
		ssl_dbg_msg("fail to send sw cmd 0x0901");
		goto out_2;
	}

	mdelay(20);

	ssl_set_intflag(ftdev->client, 0x030f);
	if (err < 0)
		ssl_dbg_msg("error in ssl_set_intflag");
	else
		goto out_3;

out:
#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
		ssl_info_msg("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */
out_2:
	int_clear_cmd(ftdev->client);
	udelay(DELAY_FOR_SIGNAL_DELAY);
out_3:
	up(&ftdev->raw_data_lock);
	mdelay(20);
	ssl_dbg_msg("ssl_mptest work done");
	return err;
}

static int ssl_read_rawdata(struct ssl_device *ftdev,
		struct spd2010_data *data, u8 *check)
{
	int err = 0;
	int ret = 0;
	int loop_status = 1;
	int total_cnt = 0;
	int hdp_nByte = 0;
	int get_status = 0;
	int snl_status = 0;
	int snl_length = 0;
	char *buffer;
	*check = 0x00;

	memset(tempBuf, 0, 9000);

	if (!ftdev || !data)
		return -EFAULT;

	if (gpio_get_value(ftdev->int_pin)) {
		/*interrupt pin is high, not valid data.*/
		ssl_info_msg("read rawdata... inturrpt pin is high");
		return 0;
	}

	if (down_trylock(&ftdev->raw_data_lock)) {
		ssl_dbg_msg("fail to occupy sema(%s)", __func__);
		goto out;
	}

	/* SnL Read */
	err = SNL_Read(ftdev->client, &snl_status, &snl_length);

	if (snl_length > 0) {
		if (data->queue_size ==
			SPD2010_MAX_RAWDATA_QUEUE) {
			printk("f");

			data->packet_length = hdp_nByte;
			ret = ts_read_data_raw(ftdev, data,
				SPD2010_HDP, (u8 *)tempBuf,
				&total_cnt, snl_length);

			udelay(20);
			ftdev->update = 1;
		} else {
			buffer = get_rear_queue_buff(data);

			data->packet_length = snl_length;
			ret = ts_read_data_raw(ftdev, data,
				SPD2010_HDP, (u8 *)buffer,
				&total_cnt, snl_length);

			if (ret == 1) {
				printk("*");
				put_queue(data);
			} else {
				printk(".");
			}
		}
	}

	while (loop_status > 0) {
		/* read hdp status */
		err = HDP_Done_Check(ftdev->client,
			&hdp_nByte, &get_status);

		if (get_status == 0x82) {
			/* Clear INT */
			loop_status = 0;
			int_clear_cmd(ftdev->client);
		} else if (get_status == 0x00) {
			if (data->queue_size ==
				SPD2010_MAX_RAWDATA_QUEUE) {
				printk("f");

				data->packet_length = hdp_nByte;

				if (hdp_nByte == 0)
					goto out;

				err = ts_read_data_raw(ftdev,
					data, SPD2010_HDP,
					(u8 *)tempBuf, &total_cnt,
					hdp_nByte);

				udelay(20);
				ftdev->update = 1;
			} else {
				buffer = get_rear_queue_buff(data);

				data->packet_length = hdp_nByte;
				err = ts_read_data_raw(ftdev,
					data, SPD2010_HDP,
					(u8 *)buffer, &total_cnt,
					hdp_nByte);
				if (err == 1) {
					printk("*");
					put_queue(data);
				} else {
					printk(".");
				}
			}
		}
	}

out:
	up(&ftdev->raw_data_lock);

	return err;
}

/* send android key event to OS */
static int spd2010_send_key(struct input_dev *input,
	unsigned int key, int level)
{
	if (input == NULL)
		return -1;

	input_report_key(input, key, level);
	input_sync(input);

	return 0;
}

static int spd2010_send_key_du(struct input_dev *input,
	unsigned int key)
{
	if (input == NULL)
		return -1;

	spd2010_send_key(input, key, 1);
	spd2010_send_key(input, key, 0);

	return 0;
}

#ifdef SUPPORT_KEY_BUTTON
static int ssl_send_key(struct ssl_device *ftdev,
	u16 keydown, u16 keyup)
{
	int err = 0;
	int i = 0;
	u16 key = 0x00;
	struct input_dev *input = NULL;
	struct spd2010_data *data = NULL;

	data = ftdev->ftdata;
	input = ftdev->input_dev;

	ssl_dbg_msg("KEYDOWN : 0x%04x, KEYUP : 0x%04x",
			keydown, keyup);

	/* key down */
	key = keydown;

	for (i = 0; i < MAX_DEVICE_KEYMAP_SIZE; i++) {
		if ((key & 0x01)) {
			ssl_dbg_msg("down key = 0x%02x", m_key_map[i]);
			spd2010_send_key(input, m_key_map[i], 1);
			data->keydata |= (key << i);
		}

		key >>= 1;
	}

	/* key up */
	key = keyup;

	for (i = 0; i < MAX_DEVICE_KEYMAP_SIZE; i++) {
		if ((key & 0x01)) {
			if ((data->keydata&(key << i)) == 0)
				ssl_dbg_msg("up[0x%02x] error!",
					m_key_map[i]);
			else {
				ssl_dbg_msg("up[0x%02x]", m_key_map[i]);
				spd2010_send_key(input, m_key_map[i], 0);
				data->keydata &= ~(key << i);
			}
		}

		key >>= 1;
	}

	return err;
}

#ifndef SUPPORT_PROTOCOL_6BYTES
static int spd2010_read_keydata_android(struct ssl_device *ftdev)
{
	int err = 0;
	u16 keydata = 0x00;

	ssl_dbg_msg("key start");
	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	err = ts_read_keydata(ftdev->client, (u8 *)(&keydata));

	if (err < 0) {
		ssl_dbg_msg("Fail read keydata!!");
		return err;
	}

	err = ssl_send_key(ftdev, (keydata & 0xff),
		((keydata >> 8) & 0xff));

	return err;
}
#endif	/* SUPPORT_PROTOCOL_6BYTES */

#endif	/* SUPPORT_KEY_BUTTON */

#ifdef SUPPORT_LPM
static ssize_t ssl_gesture_android(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ssl_device *ftdev = i2c_get_clientdata(client);

	input_report_key(ftdev->input_dev, KEY_BACK, 1);
	input_sync(ftdev->input_dev);

	return count;
}

static DEVICE_ATTR(gesture, S_IRUGO, NULL, ssl_gesture_android);

static int spd2010_gesture(struct ssl_device *ftdev,
	u16 gesture, int length)
{
	int err = 0;
	unsigned int gesture_key = 0x00;
	unsigned int gesture_key2 = 0x00;

	ssl_info_msg("KNOCK CODE : 0x%04x\n",
		(gesture&0x00FF));
	ssl_info_msg("KNOCK STATUS : 0x%04x\n",
		((gesture&0xFF00)>>8));

	if ((gesture & GESTURE_STATUS_KNOCK_ON) > 0) {
		gesture_key = KEY_POWER;
	} else if ((gesture &
				(GESTURE_STATUS_GESTURES |
				 GESTURE_STATUS_KNOCK_CODE)) > 0) {
		gesture_key2 = gesture & 0xFF;
	} else if ((gesture & GESTURE_STATUS_PALM_REJECT) > 0) {
		ssl_report_release(ftdev);
		goto out;
	} else {
		err = -1;
		goto out;
	}

#ifndef SUPPORT_REPEAT_GESTURE_REPORT
	if (lpm_end(ftdev) < 0)
		goto out;

	m_power_status = LPM_SUSPEND_DELAY;
#endif

		spd2010_send_key(ftdev->input_dev, gesture_key, 1);
		spd2010_send_key(ftdev->input_dev, gesture_key, 0);

	if (gesture_key2 > 0) {
		/* It is just the demo. You must change keymap */

		ssl_info_msg("GESTURE : 0x%02x",
			(int)gesture_key2);

		/* android do missing a event. So, need to delay */
		mdelay(100);
		ssl_gesture_report(ftdev, gesture_key2);

		}
out:
	return err;
}

#ifndef SUPPORT_PROTOCOL_6BYTES
static int ssl_read_gesture(struct ssl_device *ftdev)
{
	int err = 0;
	u16 arrGesture[GESTURE_READ_SIZE] = {0,};

	ssl_dbg_msg("read data (gesture)");

	err = ts_read_gesture(ftdev->client,
		arrGesture, GESTURE_READ_SIZE * 2);

	if (err > 0)
		err = spd2010_gesture(ftdev, arrGesture[0],
			arrGesture[1]);

	return err;
}
#endif
#endif	/* SUPPORT_LPM */

#ifdef SUPPORT_AUX
static int ssl_read_aux(struct i2c_client *client, u16 *aux)
{
	int err = 0;
	*aux = 0x00;

	err = ts_read_data(client, SPD2010_AUX, (u8 *)(aux), 2);
	if (err < 0)
		ssl_dbg_msg("error read AUX info using i2c.-");

	ssl_info_msg("Read AUX info : 0x%04x", *aux);

	return err;

}
#endif	/* SUPPORT_AUX */

static int ssl_check_skiptime(unsigned long msecs)
{
	int err = 0;

	ssl_info_msg("jiffies : %lu, skip : %lu, msec : %d",
		jiffies, m_point_skip_time,
		jiffies_to_msecs(jiffies - m_point_skip_time));

	if (jiffies_to_msecs(jiffies - m_point_skip_time) < msecs) {
		ssl_dbg_msg("point1 data skip time(%lu)",
			m_point_skip_time);
		m_point_skip_time = 0;
		err = -1;
	} else {
		m_point_skip_time = 0;
	}

	return err;
}


static int ssl_touch_down_up(int id, int xpos, int ypos,
	int width, int isdown)
{
	if (isdown) {
		input_mt_slot(misc_dev->input_dev, id);
		input_mt_report_slot_state(misc_dev->input_dev,
			MT_TOOL_FINGER, true);
		input_report_abs(misc_dev->input_dev,
			ABS_MT_POSITION_X, xpos);
		input_report_abs(misc_dev->input_dev,
			ABS_MT_POSITION_Y, ypos);
		input_report_abs(misc_dev->input_dev,
			ABS_MT_WIDTH_MAJOR, width);
		input_report_abs(misc_dev->input_dev,
			ABS_MT_PRESSURE, width);
		input_report_abs(misc_dev->input_dev,
			ABS_MT_TOUCH_MAJOR, width);
	} else {
		input_mt_slot(misc_dev->input_dev, id);
		input_mt_report_slot_state(misc_dev->input_dev,
			MT_TOOL_FINGER, false);
	}
	return 0;
}

#ifdef SUPPORT_PROTOCOL_6BYTES
static int spd2010_send_axis(struct ssl_device *ftdev, ssd_data axis)
{
	struct spd2010_data *data = ftdev->ftdata;
	u8 id, w;
	u16 x, y;

	id = (axis.point.id);
	x = (((axis.point.x_y_msb & 0xf0) << 4) | axis.point.x_lsb);
	y = (((axis.point.x_y_msb & 0x0f) << 8) | axis.point.y_lsb);
	w = (axis.point.weight);

	if (w) {
		ssl_touch_down_up(id, x, y, w, 1);
		data->lastValidCnt++;
		m_up_event[id] = 1;
	} else {
		ssl_touch_down_up(id, x, y, w, 0);
		m_up_event[id] = 0;
	}

	return 0;
}

static int ssl_report_ex(struct ssl_device *ftdev)
{
	int i = 0;
	int count = 0, axisCnt = 0;
	ssd_data *points = NULL;

	struct input_dev *input = ftdev->input_dev;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	points = ftdev->ftdata->points;

	count = ftdev->ftdata->packet_length / sizeof(ssd_data);

	for (i = 0; i < count; i++) {
		if (points[i].point.id >= PASS_DATA_ID_AXIS_FIRST &&
			points[i].point.id <= PASS_DATA_ID_AXIS_LAST) {
			/* data ID is axis */
			spd2010_send_axis(ftdev, points[i]);
			axisCnt++;
		}
	}

	if (axisCnt > 0) {
		/* ssl_dbg_msg("report to host"); */
		input_mt_report_pointer_emulation(input, true);
		input_sync(input);
	}

	/*
	 * if (key_data.key.id == SPD2010_KEY_ID)
	 *	spd2010_send_key(input, key_data.key.keyL, key_data.key.keyH);
     *
	 * if (gesture_data.key.id == SPD2010_GESTURE_ID)
	 * 	spd2010_gesture(ftdev, gesture_data.gesture.gestureL,
	 *		gesture_data.gesture.gestureH);
	 */
	return 0;
}

static int ssl_report(struct ssl_device *ftdev)
{
#ifndef SUPPORT_SSL
	int i = 0;
#endif
	int count = 0, axisCnt = 0;
	ssd_data *points = NULL;
	struct spd2010_data *data = ftdev->ftdata;
	struct input_dev *input = ftdev->input_dev;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	points = ftdev->ftdata->points;

	if (points == NULL) {
		ssl_dbg_msg("data for parse is null");
		return -1;
	}

	count = (ftdev->ftdata->point_info & 0x00ff) / sizeof(ssd_data);
	ssl_info_msg("union spd2010_data size = %d",
		(int)sizeof(ssd_data));

	if (count == 0) {
		ssl_info_msg("length zero release");
		ssl_report_release(ftdev);
		return -1;
	}
	data->lastValidCnt = 0;

#ifndef SUPPORT_SSL
	for (i = 0; i < count; i++) {
		if (points[i].gesture.id == PASS_DATA_ID_GESTURE) {

#ifdef SUPPORT_LPM
			/* data ID is gesture */
			spd2010_gesture(ftdev, ((points[i].gesture.status<<8)
				| points[i].gesture.gesture),
				((points[i].gesture.coordLengthH<<8)
				| points[i].gesture.coordLengthL));
#endif
		} else if (points[i].key.id == PASS_DATA_ID_KEY) {
			/* data ID is key */
#ifdef SUPPORT_KEY_BUTTON
			ssl_send_key(ftdev, ((points[i].key.keydownH << 8)
				| points[i].key.keydownL),
				((points[i].key.keyupH << 8)
				| points[i].key.keyupL));
#endif
		} else if (points[i].point.id >= PASS_DATA_ID_AXIS_FIRST
			&& points[i].point.id <= PASS_DATA_ID_AXIS_LAST) {
			/* data ID is axis */
			spd2010_send_axis(ftdev, points[i]);
			axisCnt++;
		} else if (points[i].aux.id == PASS_DATA_ID_AUX) {
			/* data ID is AUX. It doesn't support yet. */
			ssl_dbg_msg("AUX ID detected!!");
		} else {
			ssl_dbg_msg("%d doesn't supported ID(0x%02x)",
			(points[i].point.id &
			PASS_DATA_DEVICEID_MASK),
			points[i].point.id);
		}
	}
#endif /* !SUPPORT_SSL */

	if (axisCnt > 0) {
		input_mt_report_pointer_emulation(input, true);
		input_sync(input);
	}

	return 0;
}
#else /* NOT_SUPPORT_PROTOCOL_6BYTES */
/* send a touch point to AP */
static int ssl_report(struct ssl_device *ftdev)
{
	int i = 0;
	u8 status, length, id, w;
	u16 x, y;
	struct spd2010_data *data = ftdev->ftdata;
	struct input_dev *input = ftdev->input_dev;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	status = (data->point_info & 0xff00) >> 8;
	length = (data->point_info & 0x00ff) / 6;

	if (length == 0) {
		ssl_info_msg("length zero release");
		ssl_report_release(ftdev);
	} else {
		data->lastValidCnt = 0;

		for (i = 0; i < length; i++) {
			id = (data->points[i].point.id);
			x = (((data->points[i].point.x_y_msb & 0xf0) << 4) |
				data->points[i].point.x_lsb);
			y = (((data->points[i].point.x_y_msb & 0x0f) << 8) |
				data->points[i].point.y_lsb);
			w = (data->points[i].point.weight);

			ssl_info_msg("%d %dx%d %d", id, x, y, w);
			if (w) {
				ssl_touch_down_up(id, x, y, w, 1);
				data->lastValidCnt++;
				m_up_event[id] = 1;
			} else {
				if (m_up_event[id] == 1) {
					ssl_info_msg(
						"[ID:%d] UP event", id);
					ssl_touch_down_up(id, x, y, w, 0);
				} else {
					ssl_info_msg(
						"[ID:%d]unknown UP", id);
				}
				memset(&data->points[i], 0x0,
						sizeof(ssd_data));
			}
		}
		input_mt_report_pointer_emulation(input, true);
		input_sync(input);
	}

	return 0;
}
#endif	/* SUPPORT_PROTOCOL_6BYTES */

static int ssl_report_release(struct ssl_device *ftdev)
{
	int i = 0;
	struct spd2010_data *data = ftdev->ftdata;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	if (data->lastValidCnt <= 0)
		return 0;

	ssl_info_msg("Point release(%d)", data->lastValidCnt);

	for (i = 0; i < MAX_POINT_NUMBER; i++) {
		if (m_up_event[i] == 1) {
			ssl_info_msg("[ID:%d] UP event", i);
			ssl_touch_down_up(i, 0, 0, 0, 0);
			m_up_event[i] = 0;
		} else {
			ssl_info_msg("[ID:%d] unknown UP event", i);
		}
	}

	input_sync(ftdev->input_dev);

	data->lastValidCnt = 0;

	return 0;
}

/*
static void ssl_pt_set_scantime(struct spd2010_data *data,
	int pt_idx)
{
	int i;
	int p_add = HDP_ADD_Cnt + PACKET_Length_Cnt;

	for (i = 0; i <	(data->Contact_cnt + 1); i++) {
		if (i == data->Contact_cnt) {
			pt_idx = (i * 6) + 2;
			data->scantime.time.scantime_low_L =
				data->point_data[p_add + pt_idx];

			data->scantime.time.scantime_low_H =
				data->point_data[p_add + pt_idx + 1];

			data->scantime.time.scantime_hi_L =
				data->point_data[p_add + pt_idx + 2];

			data->scantime.time.scantime_hi_H =
				data->point_data[p_add + pt_idx + 3];
		} else {
			pt_idx = (i * 6) + 2;

			data->points[i].point.tip_switch =
				data->point_data[p_add + pt_idx];
			data->points[i].point.id =
				data->point_data[p_add + pt_idx + 1];
			data->points[i].point.x_low =
				data->point_data[p_add + pt_idx + 2];
			data->points[i].point.x_high =
				data->point_data[p_add + pt_idx + 3];
			data->points[i].point.y_low =
				data->point_data[p_add + pt_idx + 4];
			data->points[i].point.y_high =
				data->point_data[p_add + pt_idx + 5];
		}
	}
}
*/

/*
static int ssl_pt_set_other(struct ssl_device *ftdev,
	struct spd2010_data *data, int pt_idx)
{
	int i;
	int err = 0;
	u8 status;
#ifdef SUPPORT_AUX
	u16 aux = 0;
#endif
	int p_add = HDP_ADD_Cnt + PACKET_Length_Cnt;

	for (i = 0; i < data->packet_length; i += 6) {
		if (data->point_data[p_add + i] == SPD2010_KEY_ID) {
			data->key_data.key.id =
				data->point_data[p_add + i];
			data->key_data.key.keyL =
				data->point_data[p_add + i + 1];
			data->key_data.key.keyH =
				data->point_data[p_add + i + 2];
		} else if (data->point_data[p_add + i] == SPD2010_GESTURE_ID) {
			data->gesture_data.gesture.id =
				data->point_data[p_add + i];
			data->gesture_data.gesture.gestureL =
				data->point_data[p_add + i + 1];
			data->gesture_data.gesture.gestureH =
				data->point_data[p_add + i + 2];
		} else if (data->point_data[p_add + i] == SPD2010_AUX_ID) {
			aux = data->point_data[p_add + i + 1] |
				(data->point_data[p_add + i + 2] >> 8);
			ssl_info_msg("Read AUX info : 0x%04x", aux);
		}
	}

#if defined(SUPPORT_KEY_BUTTON) && !defined(SUPPORT_PROTOCOL_6BYTES)
	if ((status & STATUS_CHECK_KEY) == STATUS_CHECK_KEY) {
		ssl_info_msg("KEY bit set!!");
		spd2010_read_keydata_android(ftdev);
	}
#endif

#ifdef STATUS_CHECK_BOOT_ST
	if ((status & STATUS_CHECK_BOOT_ST) == STATUS_CHECK_BOOT_ST) {
		ssl_dbg_msg("Need TMC config initialize.");
		ssl_report_release(ftdev);
		err = ssl_init_config(ftdev);

		if (err < 0)
			ssl_dbg_msg("TMC config initialize failed");
		else {
			if (init_tmc_flag == 1) {
				input_set_abs_params(misc_dev->input_dev,
					ABS_MT_POSITION_X, 0,
					ftdev->ftconfig->max_x, 0, 0);
				input_set_abs_params(misc_dev->input_dev,
					ABS_MT_POSITION_Y, 0,
					ftdev->ftconfig->max_y, 0, 0);
				input_set_abs_params(misc_dev->input_dev,
					ABS_MT_PRESSURE, 0, 1000, 0, 0);
				err = input_register_device(ftdev->input_dev);

				if (err) {
					ssl_dbg_msg(
					"Register input failed");
					err = -1;
				}
				init_tmc_flag = 0;
			}
		}
		return -1;
	}
	return 0;
}
*/

static int ssl_read_points(struct ssl_device *ftdev,
		struct spd2010_data *data)
{
	int err = 0;
	int loop_status = 1;
	int limit_cnt = 6;
	int hdp_status = 0;
	int hdp_nByte = 0;
	int snl_status = 0;
	int snl_length = 0;
	u8 p_data[POINT_HEADER_Cnt + (POINT_INFO_Cnt * MAX_POINT_NUMBER)];
#ifdef SUPPORT_AUX
	u16 aux = 0;
#endif	/* SUPPORT_AUX */

	if (!ftdev || !data)
		return -EFAULT;

	if (gpio_get_value(ftdev->int_pin)) {
		/*interrupt pin is high, not valid data.*/
		ssl_info_msg("touch interrupt pin is high");
		return -EAGAIN;
	}

	/* SnL Read */
	err = SNL_Read(ftdev->client, &snl_status, &snl_length);
	if (err < 0) {
		ssl_dbg_msg("SnL read fail");
		goto out;
	}

	if (snl_length > 0) {
		/* HDP Read */
		data->packet_length = snl_length;
		err = ts_read_data(ftdev->client, SPD2010_HDP,
			(u8 *)(&p_data), data->packet_length);

		memcpy(data->points, &p_data[POINT_HEADER_Cnt],
				data->packet_length);

		if (err < 0) {
			ssl_dbg_msg("error read point info using i2c.\n");
			goto out;
		}
	}

	while (loop_status > 0) {
		/* read hdp_status */
		err = HDP_Done_Check(ftdev->client,
			&hdp_nByte, &hdp_status);
		if (err < 0) {
			loop_status = 0;
			ssl_dbg_msg("HDP status read fail");
			goto out;
		}

		/* Interrupt Clear */
		if (hdp_status == 0x82) {
			loop_status = 0;
			int_clear_cmd(ftdev->client);
			goto out;
		} else if (hdp_status == 0x00) {
			/* ready upload data */
			data->packet_length = hdp_nByte;
			err = ts_read_data(ftdev->client, SPD2010_HDP,
				(u8 *)(&p_data),
				data->packet_length);

			memcpy(data->points, &p_data[POINT_HEADER_Cnt],
					hdp_nByte);

			if (err < 0) {
				loop_status = 0;
				ssl_dbg_msg(
				"error read point info using i2c.\n");
				goto out;
			}
		}

		/* delay & exception (exit loop) */
		if (limit_cnt > 0) {
			limit_cnt--;
			udelay(HDP_CHECK_DELAY);
		} else {
			ssl_dbg_msg(
			"Timeout for obtaining point information");
			loop_status = 0;
			int_clear_cmd(ftdev->client);
		}
	}

	if (ftdev->touch_mode >= AGING_RAW_DATA_MODE)
		ssl_get_rawdata_by_queue(ftdev);
out:

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
		ssl_info_msg("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */

	return err;

out_reset:
	ssl_power_control(ftdev, POWER_RESET);
	/* spd2010_reset(); */
	return -1;
}

/* TMC initialize */
static int ssl_init_config(struct ssl_device *ftdev)
{
	int err = 0;
	u16 val = 0x0000;
	struct i2c_client *client = ftdev->client;
	struct spd2010_config *ftconfig = ftdev->ftconfig;

	ssl_dbg_msg("config setting");

	if (!client || !ftconfig) {
		ssl_dbg_msg("error : I2C client fail!!!");
		return -EINVAL;
	}

	/* mp test mode */
/*	if (m_mp_total_count < 0)	*/
	misc_dev->mptest_mode = MPTEST_STOP;

	m_point_skip_time = 0;

	/* touch mode */
	val = TOUCH_POINT_MODE;
	err = ts_write_data(misc_dev->client,
		SPD2010_TOUCH_MODE, (u8 *)&(val), 2);

	if (err < 0)
		ssl_dbg_msg("Fail to set TOUCH_MODE %d.", val);

	ssl_get_version(ftdev, ftconfig->fw_ver);

	err = ts_read_node(client, (u8 *)&ftconfig->x_node,
			(u8 *)&ftconfig->y_node);

	if (err < 0) {
		ssl_dbg_msg("Get node x/y failed");
		goto init_config_failed;
	}

	err = ts_read_data(client,
			SPD2010_X_RESOLUTION, (u8 *)&(ftconfig->max_x), 2);

	if (err < 0) {
		err = -EAGAIN;
		ssl_info_msg("error : read x resolution");
	}
	ssl_info_msg("X resolution : %d", ftconfig->max_x);

	err = ts_read_data(client,
			SPD2010_Y_RESOLUTION, (u8 *)&(ftconfig->max_y), 2);

	if (err < 0) {
		err = -EAGAIN;
		ssl_info_msg("error : read y resolution");
	}
	ssl_info_msg("Y resolution : %d", ftconfig->max_y);

	ftconfig->max_weight = (ftconfig->max_x * ftconfig->max_y) >> 1;

	err = ts_read_data(client,
			SPD2010_USING_POINT_NUM,
			(u8 *)&(ftconfig->using_point), 2);

	if (err < 0) {
		err = -EAGAIN;
		ssl_info_msg("error : read using point");
	}
	ssl_info_msg("using point num : %d",
		ftconfig->using_point);

#ifdef SUPPORT_TMC_I2C_LENGTH
	err = ts_read_tmc_i2c(client, &val);
	if (err < 0) {
		err = -EAGAIN;
		ssl_info_msg("error : read I2C Length");
		ftconfig->i2c_length = MAX_RAWDATA_BUFFER_SIZE;
	} else {
		ftconfig->i2c_length = val * 2;
	}
	ssl_dbg_msg("I2C Length : %d",
		ftconfig->i2c_length);
#else
	ftconfig->i2c_length = MAX_RAWDATA_BUFFER_SIZE;
#endif	/* SUPPORT_TMC_I2C_LENGTH */

#if ESD_TIMER_ENABLE
	ftdev->use_esd_tmr = 0;
	if (SPD2010_CHECK_ESD_TIMER > 0) {
		if (SPD2010_CHECK_ESD_TIMER == 1)
			val = 30;
		else
			val = (SPD2010_CHECK_ESD_TIMER - 1) * 60;

		err = ssl_set_esdtime(ftdev, val);

		if (err >= 0) {
			ftdev->use_esd_tmr = 1;
			esd_checktime_init(ftdev);
			esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
			ssl_info_msg("esd timer start");
		}

	}
#endif	/* ESD_TIMER_ENABLE */

	err = ssl_set_AFE_limit(ftdev,
		AFE_MAX_LIMIT, AFE_MIN_LIMIT);

init_config_failed:
	ftconfig->check = true;
	return err;
}

/* Interrupt service routine */
static void ssl_int_work(struct work_struct *work)
{
	struct ssl_device *ftdevice =
		container_of(work, struct ssl_device, work);
	int ret = 0;
	u8 check = 0x00;

	if (init_startup_flag == 0) {
		ssl_info_msg("not yet startup");
		return;
	}

	if (ftdevice->work_procedure == TS_IN_UPGRADE) {
		ssl_info_msg("upgrade..");
		return;
	}

	if (ftdevice->work_procedure == TS_SET_MODE) {
		ssl_info_msg("prepare MP Test..\n");

		ssl_read_mpdata(ftdevice,
				ftdevice->ftdata);

		goto out;
	}

	if (ftdevice->work_procedure != TS_NO_WORK) {
		ssl_info_msg("other process occupied..(%d)\n",
					misc_dev->work_procedure);
		udelay(DELAY_FOR_SIGNAL_DELAY);
		ret = int_clear_cmd(ftdevice->client);
		if (ret < 0)
			ssl_info_msg("send int clear cmd failed\n");
		goto out;
	}

	ftdevice->work_procedure = TS_NORMAL_WORK;

	if (ftdevice->mptest_mode > MPTEST_STOP) {
		if (m_mp_status == 1) {
			ssl_mptest(ftdevice, ftdevice->ftdata);
			m_mp_status = 0;
		} else {
			ssl_read_mpdata(ftdevice,
				ftdevice->ftdata);
		}
	} else if (ftdevice->touch_mode > TOUCH_POINT_MODE &&
			ftdevice->touch_mode < AGING_RAW_DATA_MODE) {
		ssl_read_rawdata(ftdevice,
			ftdevice->ftdata, &check);

#ifndef SUPPORT_SSL
		if (check > 0)
			ssl_report(ftdevice);
#endif
	} else {
		ret = ssl_read_points(ftdevice,
				ftdevice->ftdata);

		if (ret > 0)
#ifdef SUPPORT_SSL
		ssl_report_ex(ftdevice);
#else
		ssl_report(ftdevice);
#endif
	}

out:
	if (ftdevice->work_procedure == TS_NORMAL_WORK)
		ftdevice->work_procedure = TS_NO_WORK;

	up(&ftdevice->work_procedure_lock);
}

static irqreturn_t ssl_interrupt(int irq, void *devid)
{
	struct ssl_device *ftdevice =
		(struct ssl_device *)devid;

#ifdef SUPPORT_LPM__
	/* This is not use more */
	if ((m_power_status & LPM_SUSPEND_DELAY) != LPM_SUSPEND_DELAY)
#endif	/* SUPPORT_LPM */
		ssl_int_work(&ftdevice->work);
	return IRQ_HANDLED;
}

static int ssl_get_rawdata_str(struct ssl_device *ftdev,
	u16 touchmode, char *buff)
{
	int err = 0;
	int sz = 0;
	int total_node = 0;
	unsigned long retry_time = 0;
	int len = MAX_RAWDATA_BUFFER_SIZE;
	char *buffer = NULL;

	ssl_dbg_msg("read data (raw str)");

	down(&misc_dev->work_procedure_lock);
	if (misc_dev->work_procedure != TS_NO_WORK) {
		ssl_info_msg("other process occupied.. (%d)\n",
			misc_dev->work_procedure);
		err = -1;
		goto out;
	}

	misc_dev->work_procedure = TS_SET_MODE;
	err = ts_write_data(misc_dev->client, SPD2010_TOUCH_MODE,
		(u8 *)&(touchmode), 2);
	if (err < 0) {
		ssl_info_msg("Fail to set TOUCH_MODE %d.",
			misc_dev->touch_mode);
		goto out_work;
	}

	retry_time = 40;
	err = -1;
	do {
		if (gpio_get_value(misc_dev->int_pin) == 0) {
			err = 0;
			break;
		}
		msleep(50);
	} while (retry_time-- > 0);

	if (err < 0)
		goto out_clear;

	buffer = buff;
	total_node = misc_dev->ftconfig->x_node *
		misc_dev->ftconfig->y_node;
	sz = total_node * 2;
	len = ftdev->ftconfig->i2c_length;

	while (sz > 0) {
		if (sz <= ftdev->ftconfig->i2c_length)
			len = sz;
		sz -= ftdev->ftconfig->i2c_length;
		err = ts_read_data(ftdev->client,
			SPD2010_RAW_DATA, buffer, len);
		if (err < 0) {
			ssl_info_msg("error : read raw data");
			goto out_clear;
		}

		buffer = buffer + len;
	}

out_clear:
	int_clear_cmd(misc_dev->client);
	touchmode = TOUCH_POINT_MODE;
	err = ts_write_data(misc_dev->client,
		SPD2010_TOUCH_MODE, (u8 *)&(touchmode), 2);

	if (err < 0) {
		ssl_info_msg("Fail to set TOUCH_MODE %d.",
			misc_dev->touch_mode);
		goto out_work;
	}
out_work:
	misc_dev->work_procedure = TS_NO_WORK;
out:
	up(&misc_dev->work_procedure_lock);

	return err;
}

static char ssd_return_str[256000] = {0};

#define SSDTOUCH_WRITE					"write"
#define SSDTOUCH_READ					"read"

#define SSDTOUCH_DEBUG					"debug"
#define SSDTOUCH_RESET					"reset"
#define SSDTOUCH_UPDATE					"update"
#define SSDTOUCH_MODE					"mode"
#define SSDTOUCH_STOP					"stop"
#define SSDTOUCH_START					"start"
#define SSDTOUCH_GESTURE				"gesture"
#define SSDTOUCH_VERSION				"version"

static ssize_t spd2010_set_ssdtouch_attr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	char *value = NULL;
	unsigned int reg = 0;
	unsigned int data = 0;

	unsigned short send_reg = 0;
	unsigned short send_data = 0;
	char tmp_str[64] = {0};
	char hex_str1[5] = {0};
	char hex_str2[5] = {0};
	short *rawdata;
	int i, j, ret;

	const char *ssd_receive_buf = buf;

	if ((strstr(ssd_receive_buf, SSDTOUCH_WRITE)) != NULL) {
		value = strstr(ssd_receive_buf, SSDTOUCH_WRITE);
		value += strlen(SSDTOUCH_WRITE);
		if (strlen(value) < 14)	{
			sprintf(ssd_return_str, "%s error!\n", SSDTOUCH_WRITE);
			return -EFAULT;
		}
		value += strlen(" ");
		ret = sscanf(value, "0x%x 0x%x", &reg, &data);
		send_reg = reg;
		send_data = data;

		ts_write_data(misc_dev->client, send_reg,
			(unsigned char *)&(send_data), 2);

		memset(ssd_return_str, 0, sizeof(ssd_return_str));
		sprintf(ssd_return_str, "write [0x%04X]=0x%04X\n",
			reg, data);
	} else if ((strstr(ssd_receive_buf, SSDTOUCH_READ)) != NULL) {
		value = strstr(ssd_receive_buf, SSDTOUCH_READ);
		value += strlen(SSDTOUCH_READ);
		if (strlen(value) < 4) {
			sprintf(ssd_return_str, "%s error!\n",
				SSDTOUCH_READ);
			return -EFAULT;
		}

		value += strlen(" ");
		ret = sscanf(value, "0x%x", &reg);
		send_reg = reg;

		ts_read_data(misc_dev->client, send_reg,
			(unsigned char *)&(send_data), 2);

		memset(ssd_return_str, 0, sizeof(ssd_return_str));

		sprintf(ssd_return_str, "0x%04X\n", send_data);
	} else if ((strstr(ssd_receive_buf, SSDTOUCH_UPDATE)) != NULL) {
		ssl_dbg_msg("spd2010_set_ssdtouch_attr update mode");
		ts_disable_irq();
		misc_dev->work_procedure = TS_IN_UPGRADE;
		ssl_firmware_update_byfile(misc_dev,
			FW_FORCE_FULL_PATH);
		misc_dev->work_procedure = TS_NO_WORK;
		spd2010_bootup_multifw();
		ts_enable_irq();
		sprintf(ssd_return_str, "%s ok!\n", SSDTOUCH_UPDATE);
	} else if ((strstr(ssd_receive_buf, SSDTOUCH_RESET)) != NULL) {
		ssl_init(misc_dev);
		sprintf(ssd_return_str, "%s ok!\n", SSDTOUCH_RESET);
	} else if ((strstr(ssd_receive_buf, SSDTOUCH_STOP)) != NULL) {
		spd2010_stop_scanning(misc_dev);
		sprintf(ssd_return_str, "%s ok!\n", SSDTOUCH_STOP);
	} else if ((strstr(ssd_receive_buf, SSDTOUCH_START)) != NULL) {
		spd2010_start_scanning(misc_dev);
		sprintf(ssd_return_str, "%s ok!\n", SSDTOUCH_START);
	} else if ((strstr(ssd_receive_buf, SSDTOUCH_MODE)) != NULL) {
		value = strstr(ssd_receive_buf, SSDTOUCH_MODE);
		value += strlen(SSDTOUCH_MODE);
		if (strlen(value) < 4) {
			sprintf(ssd_return_str, "%s error!, input str = %s\n",
				SSDTOUCH_MODE, ssd_receive_buf);
			return -EFAULT;
		}
		value += strlen(" ");
		ret = sscanf(value, "0x%x", &reg);

		memset(ssd_return_str, 0, sizeof(ssd_return_str));

		rawdata = (u16 *)&(misc_dev->ftdata->rawData[0][0]);
		if (ssl_get_rawdata_str(misc_dev,
				reg, (char *)rawdata) >= 0) {
			memset(tmp_str, 0, sizeof(tmp_str));
			sprintf(tmp_str, "	  ");
			strcat(ssd_return_str, tmp_str);
			for (j = 0; j < misc_dev->ftconfig->y_node; j++) {
				memset(tmp_str, 0, sizeof(tmp_str));
				sprintf(tmp_str, "[%3d] ", j + 1);
				strcat(ssd_return_str, tmp_str);
			}
			memset(tmp_str, 0, sizeof(tmp_str));
			sprintf(tmp_str, "\n");
			strcat(ssd_return_str, tmp_str);

			for (i = 0; i < misc_dev->ftconfig->x_node;
				i++) {
				memset(tmp_str, 0, sizeof(tmp_str));
				sprintf(tmp_str, "[%3d] ", i + 1);
				strcat(ssd_return_str, tmp_str);
				for (j = 0; j < misc_dev->ftconfig->y_node;
				j++) {
				memset(tmp_str, 0, sizeof(tmp_str));
				sprintf(tmp_str, "%5d ",
				rawdata[i * misc_dev->ftconfig->y_node + j]);
				strcat(ssd_return_str, tmp_str);
				}
				memset(tmp_str, 0, sizeof(tmp_str));
				sprintf(tmp_str, "\n");
				strcat(ssd_return_str, tmp_str);
			}
			memset(tmp_str, 0, sizeof(tmp_str));
			sprintf(tmp_str, "\n");
			strcat(ssd_return_str, tmp_str);
		}
	} else if ((strstr(ssd_receive_buf,
		SSDTOUCH_VERSION)) != NULL) {
		memset(ssd_return_str, 0, sizeof(ssd_return_str));
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, "################################\n");
		strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, " Display Version : 0x%08x\n",
			misc_dev->fw_version.display_version);
		strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, "  Driver Version : %s\n",
			DRIVER_VRESION);
		strcat(ssd_return_str, tmp_str);
		hex_str1[0] = (misc_dev->fw_version.productID01 &
			0xFF000000) >> 24;
		hex_str1[1] = (misc_dev->fw_version.productID01 &
			0x00FF0000) >> 16;
		hex_str1[2] = (misc_dev->fw_version.productID01 &
			0x0000FF00) >> 8;
		hex_str1[3] = (misc_dev->fw_version.productID01 &
			0x000000FF) >> 0;
		hex_str2[0] = (misc_dev->fw_version.productID02 &
			0xFF000000) >> 24;
		hex_str2[1] = (misc_dev->fw_version.productID02 &
			0x00FF0000) >> 16;
		hex_str2[2] = (misc_dev->fw_version.productID02 &
			0x0000FF00) >> 8;
		hex_str2[3] = (misc_dev->fw_version.productID02 &
			0x000000FF) >> 0;
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str,
			"      Product ID : 0x%08x(%4s) 0x%08x(%4s)\n",
			misc_dev->fw_version.productID01, hex_str1,
			misc_dev->fw_version.productID02, hex_str2);
		strcat(ssd_return_str, tmp_str);
		hex_str1[0] = (misc_dev->fw_version.ICName01 &
			0xFF000000) >> 24;
		hex_str1[1] = (misc_dev->fw_version.ICName01 &
			0x00FF0000) >> 16;
		hex_str1[2] = (misc_dev->fw_version.ICName01 &
			0x0000FF00) >> 8;
		hex_str1[3] = (misc_dev->fw_version.ICName01 &
			0x000000FF) >> 0;
		hex_str2[0] = (misc_dev->fw_version.ICName02 &
			0xFF000000) >> 24;
		hex_str2[1] = (misc_dev->fw_version.ICName02 &
			0x00FF0000) >> 16;
		hex_str2[2] = (misc_dev->fw_version.ICName02 &
			0x0000FF00) >> 8;
		hex_str2[3] = (misc_dev->fw_version.ICName02 &
			0x000000FF) >> 0;
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str,
			"         IC Name : 0x%08x(%4s) 0x%08x(%4s)\n",
			misc_dev->fw_version.ICName01, hex_str1,
			misc_dev->fw_version.ICName02, hex_str2);
		strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, "      Resolution : %d X %d\n",
			misc_dev->ftconfig->max_x, misc_dev->ftconfig->max_y);
		strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, "       Node Info : %d X %d\n",
			misc_dev->ftconfig->x_node, misc_dev->ftconfig->y_node);
		strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, "     Point Count : %d\n",
			misc_dev->ftconfig->using_point);
		strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0, sizeof(tmp_str));
		sprintf(tmp_str, "#################################\n");
		strcat(ssd_return_str, tmp_str);
	}
	return 1;
}

static ssize_t spd2010_get_ssdtouch_attr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ssd_return_str);
}
static DEVICE_ATTR(ssdtouch, 0664,
	spd2010_get_ssdtouch_attr, spd2010_set_ssdtouch_attr);

static struct attribute *ssl_attrs[] = {
	&dev_attr_version.attr,
#ifdef SUPPORT_LPM
	&dev_attr_gesture.attr,
#endif	/* SUPPORT_LPM */
#if 0
	&dev_attr_testing.attr,
#endif
	&dev_attr_ssdtouch.attr,
	&dev_attr_touchmode.attr,
	&dev_attr_mptest.attr,
	&dev_attr_esdtime.attr,
	&dev_attr_fail_reason.attr,
	NULL,
};

static const struct attribute_group ssl_attr_group = {
	.attrs = ssl_attrs,
};

static int spd2010_startup(struct ssl_device *ftdev)
{
	int err = 0;
	int retry = 0;
	int retry_cnt = 20;
	u8 TIC_Status[4] = {0xff, 0xff, 0xff, 0xff};
	u8 BIOS_Status[2] = {0xff, 0xff};
	u8 send_cmd[2] = {0,};
	u8 read_buf[2] = {0x00, 0x00};
	u8 reg_buf[600] = {0,};

	struct i2c_client *client = ftdev->client;

	ssl_dbg_msg("==enter the spd2010_startup function");

RETRY:
	err = ts_read_data(client,
		SPD2010_GET_STATUS, TIC_Status, 4);

	if (err < 0) {
		ssl_dbg_msg("status read fail");
		return err;
	}

	/*
	* 4bit : TINT Low
	* 5bit : TIC_IN_CPU
	* 6bit : TIC_IN_BIOS
	* 7bit : TIC_BUSY
	*/
	if (((TIC_Status[1] >> 6) & 1) == 1) {
		/* BUSY check */
		if (((TIC_Status[1] >> 7) & 1) == 0) {
			ssl_dbg_msg("BIOS & BUSY clear (0x%x, 0x%x)",
			TIC_Status[0], TIC_Status[1]);

			/* Clear Int */
			int_clear_cmd(ftdev->client);

			mdelay(1);

			/* CPU Start command */
			send_cmd[0] = 0x01;
			send_cmd[1] = 0x00;
			err = ts_write_data(client,
				 SPD2010_RUN_CPU, send_cmd, 2);

			/* waiting for TINT to low state */
			if (misc_dev != NULL) {
				retry = 200;
				err = int_pin_check(misc_dev, retry * 2);

				if (err != -1) {
					ssl_dbg_msg("TINT to low state");
					if (retry_cnt-- > 0)
						goto RETRY;
				}
			}
		} else {
			if (retry_cnt-- > 0) {
				msleep(50);
				goto RETRY;
			}
		}
	}

	if ((((TIC_Status[1] >> 4) & 1) == 1) &&
		(((TIC_Status[1] >> 5) & 1) == 1)) {
		/* BUSY check */
		if (((TIC_Status[1] >> 7) & 1) == 0) {
			ssl_dbg_msg("CPU mode (0x%x, 0x%x)",
				TIC_Status[0], TIC_Status[1]);

			mdelay(2);

			send_cmd[0] = 0x00;
			send_cmd[1] = 0x00;
			err = ts_write_data(client,
				 SPD2010_TOUCH_MODE, send_cmd, 2);

			if (err < 0) {
				ssl_dbg_msg("touch mode set 0 fail");
				return err;
			}

			mdelay(1);

			err = ts_write_data(client,
				 SPD2010_SW_COMMAND, send_cmd, 2);

			if (err < 0) {
				ssl_dbg_msg("sw command send fail");
				return err;
			}

			/* Clear Int */
			int_clear_cmd(ftdev->client);

			/* end of CPU Start Up flow */

			mdelay(1);

			err = ts_read_data(client,
				SPD2010_GET_REGISTER, reg_buf, 600);

			if (err < 0) {
				ssl_dbg_msg("register read fail");
			} else {
				ftdev->ftconfig->y_node =
					reg_buf[8] | (reg_buf[9] << 8);
				ftdev->ftconfig->x_node =
					reg_buf[10] | (reg_buf[11] << 8);
				ssl_dbg_msg("SPD2010 ch count - %d / %d",
					ftdev->ftconfig->y_node,
					ftdev->ftconfig->x_node);

				/* upload size setting */
				ftdev->ftconfig->i2c_length =
					(reg_buf[20] | (reg_buf[21] << 8));
				ssl_dbg_msg("SPD2010 i2c upload size - %d",
					ftdev->ftconfig->i2c_length);
			}

			init_startup_flag = 1;
		} else {
			if (retry_cnt-- > 0) {
				msleep(50);
				goto RETRY;
			}
		}
	} else {
		ssl_dbg_msg("SPD2010 doesn't ready (0x%x, 0x%x)",
			TIC_Status[0], TIC_Status[1]);

		err = ts_read_data(client,
			SPD2010_BIOS_STATUS, BIOS_Status, 2);

		if (err >= 0) {
			ssl_dbg_msg("BIOS Status : (0x%x, 0x%x)",
				BIOS_Status[0], BIOS_Status[1]);
		} else {
			ssl_dbg_msg("BIOS Status read fail");
			return err;
		}

		if (retry_cnt-- > 0) {
			msleep(50);
			goto RETRY;
		}

		return -1;
	}

	return err;
}

static int ssl_boot_sequence(struct ssl_device *ftdev)
{
	int err = 0;

	ftdev->work_procedure = TS_IN_INITIALIZE;

	ssl_dbg_msg("before power control");

	err = ssl_power_control(ftdev, POWER_ON_SEQUENCE);

	if (err < 0) {
		ssl_dbg_msg("==Reset Fail!!");
		goto out;
	} else {
		ssl_dbg_msg("==Reset okay");
	}

	/* set flag */
	ftdev->boot_flag = 0x00;
	ftdev->checksum_flag = 0x00;
#ifdef SUPPORT_ESD_CHECKSUM
	ftdev->state_flag = 0;
#endif

	err = spd2010_startup(ftdev);
	if (err < 0) {
		ssl_dbg_msg("==spd2010_startup fail");
		goto out;
	}

out:
	if (ftdev->boot_flag != 0x00 ||
		ftdev->checksum_flag != 0x00) {
		return 0;
	}

	return err;
}

static int spd2010_register_powermanager(void);
static int spd2010_unregister_powermanager(void);

static int ssl_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ssl_device *ftdev = NULL;
	struct input_dev *input = NULL;
	struct spd2010_data *ftdata = NULL;
	struct spd2010_config *ftconfig = NULL;
	struct workqueue_struct *wq = NULL;
	int err = 0;
	char phys[32] = {0,};
	int i = 0;

	ssl_info_msg("#### TOUCH DRIVER PROBE 1 ####");

	spd2010_reset_pud_set();

	err = i2c_check_functionality(
		to_i2c_adapter(client->dev.parent),	I2C_FUNC_I2C);

	if (!err) {
		ssl_dbg_msg("I2C bus doesn't support");
		err = -EFAULT;
		goto i2c_check_failed;
	} else {
		ssl_dbg_msg("I2C check functionality succeed");
	}

	ftdev = (struct ssl_device *)
		kzalloc(sizeof(struct ssl_device), GFP_KERNEL);

	if (!ftdev) {
		ssl_dbg_msg("Create spd2010 device failed");
		err = -ENOMEM;
		goto create_ssl_probe_failed;
	} else {
		ssl_dbg_msg("==Create spd2010 device succeed.");
	}
	ftdev->client = client;

	ftdata = (struct spd2010_data *)
		kzalloc(sizeof(struct spd2010_data), GFP_KERNEL);

	if (!ftdata) {
		ssl_dbg_msg("Create spd2010 data failed");
		err = -ENOMEM;
		goto create_data_failed;
	} else {
		ssl_dbg_msg("==Create spd2010 data succeed.");
	}

	/* initial rawdata queue */
	ftdata->queue_front = 0;
	ftdata->queue_rear = 0;
#ifdef SUPPORT_KEY_BUTTON
	ftdata->keydata = 0x00;
#endif	/* SUPPORT_KEY_BUTTON */

	ftconfig = (struct spd2010_config *)
		kzalloc(sizeof(struct spd2010_config), GFP_KERNEL);

	if (!ftconfig) {
		ssl_dbg_msg("Create spd2010 config failed");
		err = -ENOMEM;
		goto create_config_failed;
	} else {
		ssl_dbg_msg("==Create spd2010 config succeed.");
	}

	input = input_allocate_device();
	if (!input) {
		ssl_dbg_msg("Create input device failed");
		err = -ENOMEM;
		goto create_input_failed;
	} else {
		ssl_dbg_msg("==Create input device succeed.");
	}

	wq = create_singlethread_workqueue("spd2010_touch");

	if (!wq) {
		ssl_dbg_msg("Create workqueue failed");
		goto create_workqueue_failed;
	} else {
		ssl_dbg_msg("==Create workqueue succeed.");
	}

	ftdev->touch_mode = TOUCH_POINT_MODE;
	misc_dev = ftdev;

	ftdev->workqueue = wq;
	ftdev->client = client;
	ftdev->ftconfig = ftconfig;
	ftdev->ftdata = ftdata;
	ftdev->input_dev = input;
	ftdev->int_pin = SPD2010_INT_PIN;
	ftdev->irq = SPD2010_IRQ;
	ftdev->reset_pin = SPD2010_RESET_PIN;

	/* for I2C function */
	ftdev->i2c_func.ts_read_data = ts_read_data;
	ftdev->i2c_func.ts_read_data_ex = ts_read_data_ex;
	ftdev->i2c_func.ts_write_data = ts_write_data;

	i2c_set_clientdata(client, ftdev);

	INIT_WORK(&ftdev->work, ssl_int_work);

	ssl_info_msg("#### TOUCH DRIVER PROBE 2 ####");

	mutex_init(&ftdev->lock);

	/* esd timer */
#if ESD_TIMER_ENABLE
	INIT_WORK(&ftdev->tmr_work, touch_esd_tmr_work);
	ftdev->tmr_workqueue =
		create_singlethread_workqueue("tmr_workqueue");

	if (!ftdev->tmr_workqueue) {
		ssl_info_msg(
			"unabled to create touch tmr work queue");
		goto init_config_failed;
	} else {
		ssl_dbg_msg(
			"==create touch timer work queue succeed");
	}
#endif	/* ESD_TIMER_ENABLE */

	if (ssl_boot_sequence(ftdev) < 0) {
		ssl_dbg_msg("==ssl_boot_sequence failed");
		goto init_config_failed;
	} else {
		ssl_dbg_msg("==ssl_boot_sequence succeed");
	}

	input->name = SPD2010_NAME;
	input->phys = phys;
	input->id.bustype = BUS_I2C;
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_MT_POSITION_X,
		0, SPD2010_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
		0, SPD2010_Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE,
		0, SPD2010_MAX_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
		0, SPD2010_MAX_MAJOR, 0, 0);
	ssl_dbg_msg("==max_weight : %d",
		ftdev->ftconfig->max_weight);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(input, MAX_POINT_NUMBER, 0);
#else
	input_mt_init_slots(input, MAX_POINT_NUMBER);
#endif

	err = input_register_device(ftdev->input_dev);

	if (err) {
		ssl_dbg_msg("Register input failed");
		goto input_register_failed;
	} else {
		ssl_dbg_msg("==Register input succeed");
	}

	ftdev->work_procedure = TS_NO_WORK;
	sema_init(&ftdev->work_procedure_lock, 1);

	device_enable_async_suspend(&client->dev);

	err = request_irq(ftdev->irq, ssl_interrupt,
		IRQF_DISABLED | IRQF_TRIGGER_FALLING,
		SPD2010_NAME, ftdev);
	if (err) {
		ssl_dbg_msg("Irq request failed");
		goto irq_request_failed;
	}

	sema_init(&ftdev->raw_data_lock, 1);

	err = misc_register(&touch_misc_dev);
	if (err)
		ssl_dbg_msg(
			"Fail to register touch misc device.");

#if defined(SUPPORT_LPM) || defined(SUPPORT_KEY_BUTTON)
	for (i = 0; i < LPM_GESTURE_KEY_CNT; i++)
		input_set_capability(input, EV_KEY,
			lpm_gesture_keys[i]);

#endif	/* SUPPORT_LPM || SUPPORT_KEY_BUTTON */

	if (sysfs_create_group(&client->dev.kobj,
		&ssl_attr_group)) {
		dev_err(&client->dev, "failed to create sysfs group\n");
		return -EAGAIN;
	}
	ftdev->tmc_start_time = 0;
	probe_done_flag = 1;

	return 0;

irq_request_failed:
	input_unregister_device(input);
input_register_failed:
init_config_failed:
	ssl_dbg_msg("init_config_failed...");
	ssl_free_header(ftdev);
	ssl_dbg_msg("ftdev free complete");
	if (wq != NULL)
		destroy_workqueue(wq);
	ssl_dbg_msg("workqueue destroy complete");
create_workqueue_failed:
	input_free_device(input);
	ssl_dbg_msg("input free complete");
create_input_failed:
	kfree(ftconfig);
	ssl_dbg_msg("ftconfig free complete");
create_config_failed:
	kfree(ftdata);
	ssl_dbg_msg("ftdata free complete");
create_data_failed:
	if (ftdev != NULL)
		kfree(ftdev);
	misc_dev = NULL;
	ssl_dbg_msg("ftdev free complete");
create_ssl_probe_failed:
i2c_check_failed:
	misc_dev = NULL;
	ssl_dbg_msg("spd2010 fw deinit complete");
	return err;
}

static int ssl_remove(struct i2c_client *client)
{
#if ESD_TIMER_ENABLE
	int val;
#endif	/* ESD_TIMER_ENABLE */
	struct ssl_device *ftdev = i2c_get_clientdata(client);

	if (ftdev) {
		down(&ftdev->work_procedure_lock);
		ftdev->work_procedure = TS_REMOVE_WORK;

#if ESD_TIMER_ENABLE
		if (ftdev->use_esd_tmr != 0) {
			flush_work(&ftdev->tmr_work);
			val = 0;
			ts_write_data(client, SPD2010_ESD_INT_INTERVAL,
				(u8 *)&val, 2);
			esd_timer_stop(ftdev);
			ssl_info_msg("esd timer stop");
			destroy_workqueue(ftdev->tmr_workqueue);
		}
#endif	/* ESD_TIMER_ENABLE */
		ssl_hw_deint();
		spd2010_unregister_powermanager();
		ssl_free_header(ftdev);
		free_irq(ftdev->irq, ftdev);
		misc_deregister(&touch_misc_dev);
		input_unregister_device(ftdev->input_dev);
		destroy_workqueue(ftdev->workqueue);
		input_free_device(ftdev->input_dev);
		kfree(ftdev->ftconfig);
		kfree(ftdev->ftdata);
		up(&ftdev->work_procedure_lock);
		kfree(ftdev);
	}

	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ssl_suspend(struct device *dev)
{
	struct ssl_device *ftdev = NULL;
	int err = 0;

	ftdev = misc_dev;
	if (ftdev == NULL) {
		ssl_dbg_msg("ftdev is NULL, exit\n");
		return 0;
	}

#ifndef SUPPORT_LPM
	ts_disable_irq();
#endif	/* SUPPORT_LPM */
	flush_workqueue(ftdev->workqueue);

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		flush_work(&ftdev->tmr_work);
		esd_timer_stop(ftdev);
		ssl_dbg_msg("esd timer stop");
		ftdev->use_esd_tmr = 0;

		err = ssl_set_esdtime(ftdev, 0);
		ssl_dbg_msg("set esd timer interval to 0");
	}
#endif	/* ESD_TIMER_ENABLE */

	ssl_report_release(ftdev);
	ssl_power_control(ftdev, POWER_OFF);

	spd2010_set_sleepin(ftdev);

#ifdef SUPPORT_LPM
	m_power_status = LPM_SUSPEND;
#endif	/* SUPPORT_LPM */

	return 0;
}

static int ssl_resume(struct device *dev)
{
	struct ssl_device *ftdev = NULL;

#ifdef SUPPORT_LPM
	int err = 0;
#endif	/* SUPPORT_LPM */

	if (dev != NULL)
		ftdev = dev_get_drvdata(dev);
	else
		ftdev = misc_dev;

	ssl_dbg_msg(">>>> %s spd2010 resume start!!!",
		__func__);
	spd2010_tp_report_panel_dead = 0;

	spd2010_set_sleepout(ftdev);
#ifdef SUPPORT_LPM
	if (m_power_status == LPM_SUSPEND_DELAY) {
		err = lpm_end_clear(ftdev);

		if (err < 0) {
			err = -EAGAIN;
			ssl_dbg_msg("fail to write reset command");
		}
	}
	m_power_status = LPM_RESUME;
#endif	/* SUPPORT_LPM */

#ifndef SUPPORT_LPM
	ts_enable_irq();
#endif	/* SUPPORT_LPM */

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SPD2010_CHECK_ESD_TIMER, ftdev);
		ssl_dbg_msg("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */

	ssl_info_msg("\n>>>> %s spd2010 resume end!!!",
		__func__);
	return 0;
}

static struct notifier_block ssl_fb_notifier;

static int ssl_fb_notifier_callback(
		struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			pr_err("ssl_resume");

			ssl_resume(&misc_dev->client->dev);
		}
	}
	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			pr_err("ssl_suspend");
			ssl_suspend(&misc_dev->client->dev);
		}
	}
	return 0;
}

static int spd2010_register_powermanager(void)
{
	ssl_fb_notifier.notifier_call =
		ssl_fb_notifier_callback;
	fb_register_client(&ssl_fb_notifier);

	return 0;
}

static int spd2010_unregister_powermanager(void)
{
	fb_unregister_client(&ssl_fb_notifier);

	return 0;
}

int spd2010_suspend_ex(void)
{
	if (misc_dev == NULL)
		return -EINVAL;

	if (misc_dev->client == NULL)
		return -EINVAL;

	return ssl_suspend(NULL);
}

int spd2010_resume_ex(void)
{
	if (misc_dev == NULL)
		return -EINVAL;

	if (misc_dev->client == NULL)
		return -EINVAL;

	return ssl_resume(NULL);
}

int spd2010_pre_on(void)
{
	ssl_dbg_msg("m_power_status=0x%04x!!",
		m_power_status);
#ifdef SUPPORT_LPM
	if (m_power_status == LPM_SUSPEND) {
		if (lpm_end2(misc_dev) >= 0)
			m_power_status = LPM_SUSPEND_DELAY;
	}
#endif	/* SUPPORT_LPM */
	return 0;
}

static const struct dev_pm_ops spd2010_dev_pm_ops = {
	.suspend = ssl_suspend,
	.resume  = ssl_resume,
};
#else
int spd2010_suspend_ex(void)
{
	return 0;
}
int spd2010_resume_ex(void)
{
	return 0;
}
#endif	/* CONFIG_PM */
#ifdef CONFIG_OF
static const struct of_device_id spd2010_match_table[] = {
	{ .compatible = "ssl,spd2010",},
	{ },
};
#else
#define dsx_match_table NULL
#endif

static const struct i2c_device_id ssl_id[] = {
	{SPD2010_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ssl_id);

static struct i2c_driver spd2010_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SPD2010_NAME,
#ifdef CONFIG_OF
		.of_match_table = spd2010_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &spd2010_dev_pm_ops,
#endif	/* CONFIG_PM */
	},

	.probe = ssl_probe,
	.remove = ssl_remove,
	.id_table = ssl_id,
};

static int ssl_touch_init(void)
{
	int err = 0;

	ssl_info_msg("#### add I2C Driver ####");

	err = i2c_add_driver(&spd2010_driver);

	if (err) {
		ssl_dbg_msg("add spd2010 i2c driver failed");
		goto out;
	} else {
		ssl_dbg_msg("add spd2010 i2c driver succeed");
	}
out:
	return err;
}

static void ssl_touch_exit(void)
{
	i2c_del_driver(&spd2010_driver);
}

module_init(ssl_touch_init);
module_exit(ssl_touch_exit);

MODULE_AUTHOR("Solomon Limited");
MODULE_DESCRIPTION("SPD2010 Touchscreen Driver "DRIVER_VRESION);
MODULE_LICENSE("GPL v2");
