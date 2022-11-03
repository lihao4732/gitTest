/*
 * Copyright 2019 Solomon. All rights reserved.
 *
 * SPD2010 Touch upgrade driver
 *
 * Date: 2019.02.19
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>

#include "spd2010.h"
#include "spd2010_upgrade.h"
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
extern int found_force_bin_file;
#endif

/* level of state for parsing */
enum VAL_LEVEL {
	LEVEL_NONE = 0,
	LEVEL_ADDRESS,
	LEVEL_LENGTH,
	LEVEL_ERASE_SIZE,
	LEVEL_VERSION,
	LEVEL_CHECKSUM,
	LEVEL_RESERVE_01,
	LEVEL_RESERVE_02,
	LEVEL_RESERVE_03,
	LEVEL_CONTENT,
};

/* level of state for dollar */
enum VAL_DOLLAR_LEVEL {
	LEVEL_DOLLAR_NONE = 0,
	LEVEL_DOLLAR_ERASE_TYPE,
};

int BIOS_ERASE_Sector(struct ssl_device *dev,
	struct ssl_mem_info TMC_Info, int Add);
int BIOS_WRITE_FW(struct ssl_device *dev,
	int Add, char data[], int bytecnt);
int BIOS_WAIT_Done(struct ssl_device *dev);


struct ssl_fw_group_header fw_header;

int m_fw_head_bin_flag;		/* 0 : header , 1:bin */

/* show section info */
void view_section(struct ssl_fw fw)
{
#if 0
	int i = 0;
	int len = 0;
	unsigned short *tmpContent = NULL;

	len = fw.byte_cnt;

	ssl_dbg_msg("\n\n");
	ssl_dbg_msg("\t address : 0x%08x", fw.address);
	ssl_dbg_msg("\t byte_cnt : 0x%08x", fw.byte_cnt);
	ssl_dbg_msg("\t erase_page_cnt : 0x%08x", fw.erase_page_cnt);
	ssl_dbg_msg("\t version : 0x%08x", fw.version);
	ssl_dbg_msg("\t checksum : 0x%08x", fw.checksum);
	ssl_dbg_msg("\t reserved_01 : 0x%08x", fw.reserved_01);
	ssl_dbg_msg("\t reserved_02 : 0x%08x", fw.reserved_02);
	ssl_dbg_msg("\t reserved_03 : 0x%08x", fw.reserved_03);

	tmpContent = (unsigned short *)fw.content;

	if (fw.content != NULL) {
		for (i = 0; i < 40; i += 4) {
			ssl_dbg_msg(
					"\t0x%02x%02x%02x%02x \t 0x%04x%04x",
					fw.content[i], fw.content[i+1],
					fw.content[i+2], fw.content[i+3],
					tmpContent[i/2], tmpContent[i/2+1]);
		}
		ssl_dbg_msg("\t.");
		ssl_dbg_msg("\t.");
		ssl_dbg_msg("\t.");
		ssl_dbg_msg("\t.");
		for (i = len - 40; i < len; i += 4) {
			ssl_dbg_msg(
					"\t0x%02x%02x%02x%02x \t 0x%04x%04x",
					fw.content[i], fw.content[i+1],
					fw.content[i+2], fw.content[i+3],
					tmpContent[i/2], tmpContent[i/2+1]);
		}
	}
#endif
}

/*	Convert hex string to int.
 *	parameter :
 *		unsigned char hex[] : hex string to convert int
 *		int hexLen : hex string length
 *		unsigned int *iRet : int value converted
 *	return : if 0 then success, else fail
 */
static int conv_hex_str2int(unsigned char hex[], int hexLen, unsigned int *iRet)
{
	int ret = 0;
	int hexIdx = 0;
	int temp = 0;
	int i = 0;

	if (hexLen < 0) {
		pr_info("convert error - invalid length\n");
		return ERROR_PARSING_INVALID_DATATYPE;
	}

	hexIdx = hexLen - 1;
	*iRet = 0;

	pr_info("convert... > ");
	for (i = 0; i < hexLen; i++)
		pr_info("%x", hex[i]);
	pr_info("\n");

	for (i = 0; i < hexLen; i++) {
		if (hex[i] >= 0x30 && hex[i] <= 0x39)
			temp = hex[i] - 0x30;
		else if (hex[i] >= 0x41 && hex[i] <= 0x46)
			temp = hex[i] - 0x37;
		else if (hex[i] >= 0x61 && hex[i] <= 0x66)
			temp = hex[i] - 0x57;
		else {
			pr_info("[%d] %02x - over lange\n", i, hex[i]);
			ret = -2;
			break;
		}

		*iRet += (temp << (4 * hexIdx));
		hexIdx--;
	}

	return ret;
}

static int conv_hex2ascii(unsigned char hex[], int hexLen, unsigned int *iRet)
{
	int ret = 0;
	u8 temp[16] = {0, };
	u8 ascii[8] = {0, };
	int i = 0;
	int idx = 0;

	if (hexLen < 0) {
		pr_info("convert error - invalid length\n");
		return ERROR_PARSING_INVALID_DATATYPE;
	}

	*iRet = 0;

	for (i = 0; i < hexLen; i++) {
		if (hex[i] >= 0x30 && hex[i] <= 0x39)
			temp[i] = (char)hex[i] - 0x30;
		else if (hex[i] >= 0x41 && hex[i] <= 0x46)
			temp[i] = (char)hex[i] - 0x37;
		else if (hex[i] >= 0x61 && hex[i] <= 0x66)
			temp[i] = (char)hex[i] - 0x57;
		else {
			pr_info("[%d] %02x - over lange\n", i, hex[i]);
			ret = -2;
			break;
		}

		if (i % 2 == 1)
			ascii[idx++] = (temp[i - 1]) << 4 | temp[i];
	}

	*iRet = simple_strtol(ascii, NULL, 16);

	return ret;
}

/*	calculate checksum
 *	parameter :
 *		- int len : tmpContent's length to calculate checksum
 *		- unsigned short tmpContent : To calculate checksum
 *		- unsigned int *checksum : result of calculate checksum
 *	return : 0 is success, else fail
 */
int fw_calc_checksum(int len, unsigned short *tmpContent,
		unsigned int *checksum)
{
	int i = 0;
	int ret = ERROR_SUCCESS;
	unsigned short sum = 0x00;
	unsigned short xor = 0x00;

	if (tmpContent == NULL)
		return ERROR_PARSING_CHECKSUM_FAIL;

	ssl_dbg_msg("\n");
	ssl_dbg_msg("0x%04x 0x%04x\n", tmpContent[0], tmpContent[1]);
	ssl_dbg_msg("\n");

	for (i = 0; i < len; i++) {
		sum += tmpContent[i];
		xor ^= tmpContent[i];
	}

	*checksum = (xor << 16) | sum;

	ssl_dbg_msg(">>>> sum:0x%04x, xor:0x%04x, checksum:0x%08x\n",
			sum, xor, *checksum);

	return ret;
}

/*	compare checksum
 *	parameter :
 *		- struct ssl_fw sec : section to calculate checksum
 *	return : If checksum is equal then 0, else fail
 */
int fw_checksum(struct ssl_fw sec)
{
	int ret = ERROR_SUCCESS;
	unsigned int checksum = 0x00;
	unsigned short *tmpContent = NULL;
	int len = 0;

	if (sec.address == 0x00000000 || sec.address == 0x000001c0 ||
			sec.address == 0x00001000) {

		len = sec.byte_cnt / 2 + (sec.byte_cnt & 0x01);
		tmpContent = (unsigned short *)sec.content;
	} else {
		len = (sec.byte_cnt - CONTENT_HEADER_SIZE) / 2 +
			((sec.byte_cnt - CONTENT_HEADER_SIZE) & 0x01);
		tmpContent = (u16 *)(sec.content + CONTENT_HEADER_SIZE);
	}

	ret = fw_calc_checksum(len, tmpContent, &checksum);

	if (ret == ERROR_SUCCESS) {
		if (sec.checksum != checksum)
			ret = ERROR_PARSING_CHECKSUM_FAIL;
	}

	return ret;
}

#ifdef SUPPORT_TEST_MODE
/* view error message */
void view_error_msg(int errnum)
{
	if ((errnum & ERROR_TYPE_PARSING) == ERROR_TYPE_PARSING)
		ssl_dbg_msg("[parsing");
	else if ((errnum & ERROR_TYPE_PARSING) == ERROR_TYPE_UPDATE)
		ssl_dbg_msg("[update]");

	ssl_dbg_msg("errnum : 0x%08x", errnum);

	if (errnum == ERROR_SUCCESS)
		ssl_dbg_msg("SUCCESS!!");
	else if (errnum == ERROR_PARSING_FILENAME_IS_NULL)
		ssl_dbg_msg("File nmae is fail!!");
	else if (errnum == ERROR_PARSING_FILE_OPEN_FAIL)
		ssl_dbg_msg("File open error!!");
	else if (errnum == ERROR_PARSING_FORMAT_INVALID)
		ssl_dbg_msg("Merge file format error!!");
	else if (errnum == ERROR_PARSING_CHECKSUM_FAIL)
		ssl_dbg_msg("Checksum fail!!");
	else if (errnum == ERROR_PARSING_MALLOC_FAIL)
		ssl_dbg_msg("Malloc fail!!");
	else if (errnum == ERROR_PARSING_CONTENT_SIZE_FAIL)
		ssl_dbg_msg("Content size fail!!");
	else if (errnum == ERROR_PARSING_DATA_CNT_FAIL)
		ssl_dbg_msg("Data count fail!!");
	else if (errnum == ERROR_PARSING_HEADER_DATA_INVALID_LENGTH)
		ssl_dbg_msg("The header data length invalid!!");
	else if (errnum == ERROR_PARSING_INVALID_DATATYPE)
		ssl_dbg_msg("The merge file have invalid data type!!");
	else if (errnum == ERROR_UPDATE_INIT_FAIL)
		ssl_dbg_msg("Update init fail!!");
	else if (errnum == ERROR_UPDATE_ERASE_FAIL)
		ssl_dbg_msg("Update erase fail!!");
	else if (errnum == ERROR_UPDATE_WRITE_FAIL)
		ssl_dbg_msg("Update write fail!!");
	else if (errnum == ERROR_UPDATE_READ_FAIL)
		ssl_dbg_msg("Update read fail!!");
	else if (errnum == ERROR_UPDATE_VERIFY_FAIL)
		ssl_dbg_msg("Update verify fail!!");
	else if (errnum == ERROR_EFLAH_ERASE_FAIL)
		ssl_dbg_msg("Eflash erase fail!!");
	else if (errnum == ERROR_EFLAH_WRITE_FAIL)
		ssl_dbg_msg("Eflash write fail!!");
	else if (errnum == ERROR_EFLAH_READ_FAIL)
		ssl_dbg_msg("Eflash read fail!!");
	else if (errnum == ERROR_EFLAH_VERIFY_FAIL)
		ssl_dbg_msg("Eflash verify fail!!");
	else if (errnum == ERROR_SYSTEM_FAIL)
		ssl_dbg_msg("Syste fail!!");
	else if (errnum == ERROR_VERSION_CHECK_FAIL)
		ssl_dbg_msg("Version check fail!!");
	else if (errnum == ERROR_VERIFY_VERIFY_FAIL)
		ssl_dbg_msg("The Verify verify fail!!");
	else
		ssl_dbg_msg("Unknown error!!");
}
#endif

/* SPD2010 Low-level function */
int SE_GAMMA_LL_WAIT_HCMD_CLR(struct i2c_client *client, int address)
{
	int ret = 0;
	int result = 0;
	int limit_cnt = 30;
	u8 values[2] = {0xff, 0xff};

	while (values[0] != 0x00 && values[1] != 0x00) {
		i2c_master_send(client, (u8 *)&address, 2);
		udelay(20);
		ret = i2c_master_recv(client, values, 2);

		if (limit_cnt > 0) {
			limit_cnt -= 1;
			udelay(20);
		} else {
			ssl_dbg_msg("HCMD Clear failed");
			result = -1;
			break;
		}
	}

	return result;
}


int SE_GAMMA_LL_RDSR(struct i2c_client *client, u8 *value)
{
	int ret = 0;
	int hdp_add = RHDP_ADDRESS;
	u8 send_cmd[2] = {0x10, 0x02};

	/* Write HCMD */
	ret = ts_write_data(client,
			HCMD_ADDRESS, send_cmd, 2);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	if (ret != -1) {
		/* Read HDP */
		i2c_master_send(client, (u8 *)&hdp_add, 2);
		udelay(5);
		i2c_master_recv(client, value, 2);
		return 1;
	} else {
		ssl_dbg_msg("Read Status Register failed..");
		return 0;
	}
}


int SE_GAMMA_LL_WRSR(struct i2c_client *client, int data)
{
	int ret = 0;
	u8 send_cmd[2] = {0,};

	send_cmd[0] = data & 0xff;
	send_cmd[1] = (data >> 8) & 0xff;
	ret = ts_write_data(client,
			HQARG_ADDRESS, send_cmd, 2);

	mdelay(1);

	/* Write HCMD */
	send_cmd[0] = 0x20;
	send_cmd[1] = 0x02;
	ret = ts_write_data(client,
			HCMD_ADDRESS, send_cmd, 2);

	udelay(10);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	if (ret != -1)
		return 1;
	else
		return 0;
}

int SE_GAMMA_LL_WAIT_RDY_SR(struct i2c_client *client)
{
	int sr;
	int ret = 0;

	while (1) {
		ret = SE_GAMMA_LL_RDSR(client, (u8 *)&sr);
		if ((sr & 0x01) == 0x00) {
			ssl_dbg_msg("Ready...");
			break;
		}
	}

	return 0;
}

int SE_GAMMA_LL_RDID(struct i2c_client *client, u8 *ID_Info)
{
	int ret = 0;
	int rhdp_add = RHDP_ADDRESS;

	u8 send_cmd[2] = {0x00, 0x02};

	/* Write HCMD */
	ret = ts_write_data(client,
			HCMD_ADDRESS, send_cmd, 2);

	udelay(5);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	if (ret != -1) {
		/* Read HDP */
		i2c_master_send(client, (u8 *)&rhdp_add, 2);
		udelay(5);
		i2c_master_recv(client, ID_Info, 3);
	} else {
		ssl_dbg_msg("HCMD Clear failed..");
		return 0;
	}

	return 1;
}

int SE_GAMMA_LL_WEN(struct i2c_client *client)
{
	int ret = 0;
	u8 send_cmd[2] = {0x30, 0x02};

	ret = ts_write_data(client, HCMD_ADDRESS, send_cmd, 2);

	mdelay(1);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	if (ret != -1)
		return 1;
	else
		return 0;
}

int SE_GAMMA_LL_ERASE_MACRO(struct i2c_client *client)
{
	int ret = 0;
	u8 send_cmd[2] = {0x00, 0x03};

	/* Write HCMD */
	ret = ts_write_data(client,
			HCMD_ADDRESS, send_cmd, 2);

	mdelay(1);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	if (ret != -1)
		return 1;
	else
		return 0;
}

int SE_GAMMA_LL_READ_DATA(struct i2c_client *client,
		int add, int nb, u8 *rm8)
{
	int ret = 0;
	int length = 0;
	int rhdp_add = RHDP_ADDRESS;
	u8 pkt[10] = {0,};
	u8 send_cmd[2] = {0x30, 0x03};

	memset(pkt, 0, 10);

	/* Write HQARG */
	pkt[length++] = (char)(add & 0xff);
	pkt[length++] = (char)((add >> 8) & 0xff);
	pkt[length++] = (char)((add >> 16) & 0xff);
	pkt[length++] = (char)((add >> 24) & 0xff);

	pkt[length++] = (char)(nb & 0xff);
	pkt[length++] = (char)((nb >> 8) & 0xff);
	pkt[length++] = (char)((nb >> 16) & 0xff);
	pkt[length++] = (char)((nb >> 24) & 0xff);

	ret = ts_write_data(client,
			HQARG_ADDRESS, pkt, length);

	mdelay(1);

	/* Write HCMD */
	ret = ts_write_data(client,
			HCMD_ADDRESS, send_cmd, 2);

	mdelay(1);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	mdelay(1);

	/* Read HDP */
	i2c_master_send(client, (u8 *)&rhdp_add, 2);
	udelay(20);
	i2c_master_recv(client, rm8, nb);

	return 0;
}

int SE_GAMMA_LL_PROGRAM_PAGE(struct i2c_client *client,
		int add, int nb, u8 *wm8)
{
	int ret = 0;
	int length = 0;
	u8 pkt[10] = {0,};
	u8 send_cmd[2] = {0x20, 0x03};

	memset(pkt, 0, 10);

	/* Write HQARG */
	pkt[length++] = (char)(add & 0xff);
	pkt[length++] = (char)((add >> 8) & 0xff);
	pkt[length++] = (char)((add >> 16) & 0xff);
	pkt[length++] = (char)((add >> 24) & 0xff);

	pkt[length++] = (char)(nb & 0xff);
	pkt[length++] = (char)((nb >> 8) & 0xff);
	pkt[length++] = (char)((nb >> 16) & 0xff);
	pkt[length++] = (char)((nb >> 24) & 0xff);

	ret = ts_write_data(client,
			HQARG_ADDRESS, pkt, length);

	udelay(20);

	/* write HDP */
	ts_write_data_burst(client,
			RHDP_ADDRESS, wm8, nb);

	mdelay(1);

	/* Write HCMD */
	ret = ts_write_data(client,
			HCMD_ADDRESS, send_cmd, 2);

	mdelay(1);

	ret = SE_GAMMA_LL_WAIT_HCMD_CLR(client, HCMD_ADDRESS);

	if (ret != -1)
		return 0;
	else
		return 1;
}

/* SPD2010 Software-level function */
void SE_GAMMA_SL_INIT(struct ssl_device *ftdev)
{
	int ret = 0;
	int data = 0x0002;

	ret = SE_GAMMA_LL_WEN(ftdev->client);
	if (ret == 1)
		ssl_dbg_msg("WRITE En");
	else
		ssl_dbg_msg("WRITE En failed...");

	udelay(10);

	/* Write Status Register */
	ret = SE_GAMMA_LL_WRSR(ftdev->client, data);
	if (ret == 1)
		ssl_dbg_msg("Write status register success");
	else
		ssl_dbg_msg("Write status register failed");

	udelay(10);

	ssl_dbg_msg("Wait ready status register");

	/* Wait Ready status register */
	SE_GAMMA_LL_WAIT_RDY_SR(ftdev->client);
}

void SE_GAMMA_SL_RDID(struct ssl_device *ftdev, u8 *ID_Info)
{
	int ret = 0;

	ssl_dbg_msg("Read ID");

	ret = SE_GAMMA_LL_RDID(ftdev->client, ID_Info);

	if (ret == 0)
		ssl_dbg_msg("ID Read fail");
}

void SE_GAMMA_SL_ERASE_MACRO(struct ssl_device *ftdev)
{
	int ret = 0;

	ret = SE_GAMMA_LL_WEN(ftdev->client);
	if (ret == 1)
		ssl_dbg_msg("WRITE En");
	else
		ssl_dbg_msg("WRITE En failed...");

	mdelay(1);

	ret = SE_GAMMA_LL_ERASE_MACRO(ftdev->client);

	mdelay(1);

	/* Wait Ready status register */
	SE_GAMMA_LL_WAIT_RDY_SR(ftdev->client);
}

int SE_GAMMA_SL_PROGRAM_PAGE(struct i2c_client *client,
	int add, int nb, u8 *wm8)
{
	int ret = 0;

	ret = SE_GAMMA_LL_WEN(client);
	if (ret == 1)
		ssl_dbg_msg("WRITE En");
	else
		ssl_dbg_msg("WRITE En failed...");

	udelay(10);

	SE_GAMMA_LL_PROGRAM_PAGE(client, add,
			nb, wm8);

	udelay(10);

	/* Wait Ready status register */
	SE_GAMMA_LL_WAIT_RDY_SR(client);

	return 0;
}

void SE_GAMMA_SL_READ_DATA(struct i2c_client *client,
	int add, int nb, u8 *rm8)
{
	SE_GAMMA_LL_READ_DATA(client, add,
			nb, rm8);
}

/* It is free the SSL firmware struct to malloc */
int fw_free(struct ssl_fw_group *fw, int data_free)
{
	int i = 30;
	struct ssl_fw_group *ptr = fw;

	while (fw != NULL) {
		if ((i--) < 1)
			break;

		ptr = fw->next;

		if (data_free == 1 && fw->section.content != NULL)
			kfree(fw->section.content);

		kfree(fw);
		fw = ptr;
	}

	return 0;
}

/* make F/w linked list. */
static inline int ssl_fw_make_link(struct ssl_fw_group **head,
		struct ssl_fw_group **tail, struct ssl_fw *section)
{
	int err = 0;
	struct ssl_fw_group *ptr = NULL;

	ptr = kmalloc(sizeof(struct ssl_fw_group), GFP_KERNEL);

	if (ptr == NULL)
		return ERROR_PARSING_MALLOC_FAIL;

	ptr->section.address = section->address;
	ptr->section.byte_cnt = section->byte_cnt;
	ptr->section.erase_page_cnt = section->erase_page_cnt;
	ptr->section.version = section->version;
	ptr->section.checksum = section->checksum;
	ptr->section.reserved_01 = section->reserved_01;
	ptr->section.reserved_02 = section->reserved_02;
	ptr->section.reserved_03 = section->reserved_03;
	ptr->section.content = section->content;

	ptr->next = NULL;

	if (*head == NULL) {
		*head = *tail = ptr;
	} else {
		(*tail)->next = ptr;
		*tail = ptr;
	}

	return err;
}

/* Parsing F/W file for update */
static struct ssl_fw_group *parse_uchar2int_arr(const u8 *data,
		size_t data_size, struct ssl_fw_group_header *fw_header,
		int *errnum, int *all)
{
	struct ssl_fw_group *head = NULL, *tail = NULL, *ptr = NULL;
	unsigned char buff[128] = {0,};
	int buff_idx = 0;
	int ret = ERROR_SUCCESS;
	unsigned char ch;
	int bComment = 0;
	enum VAL_LEVEL level = LEVEL_NONE;
	enum VAL_DOLLAR_LEVEL dollar_level = LEVEL_DOLLAR_NONE;
	int iRet = 0;
	int dollar_cnt = 0;
	size_t data_idx = 0;

	do {
		memset(buff, 0, sizeof(buff));
		buff_idx = 0;
		bComment = 0;

		while (data_size > data_idx && buff_idx < 128) {
			ch = data[data_idx++];

			if (ch == '\n')
				break;
			if (ch == '\r' || ch == '\t' || ch == ' ' || ch == 0x09)
				continue;
			else if (ch == ';')
				bComment = 1;
			else if (bComment == 0)
				buff[buff_idx++] = ch;
		}

		if (data_size <= data_idx)
			break;

		if (strlen(buff) < 1)
			continue;

		if (buff[0] == '$') {
			dollar_cnt++;

			if (dollar_cnt == 1) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.display_version = iRet;
				ssl_dbg_msg("display version : 0x%08x",
						iRet);
			} else if (dollar_cnt == 2) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.hidden_version = iRet;
				ssl_dbg_msg("hidden version : 0x%08x",
						iRet);
			} else if (dollar_cnt == 3) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.productID01 = iRet;
				ssl_dbg_msg("productID01 : 0x%08x",
					iRet);
			} else if (dollar_cnt == 4) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.productID02 = iRet;
				ssl_dbg_msg("productID02 : 0x%08x",
					iRet);
			} else if (dollar_cnt == 5) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.ICName01 = iRet;
				ssl_dbg_msg("ICName01 : 0x%08x",
					iRet);
			} else if (dollar_cnt == 6) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.ICName02 = iRet;
				ssl_dbg_msg("ICName02 : 0x%08x",
					iRet);
			}
			if (dollar_level == LEVEL_DOLLAR_NONE &&
					dollar_cnt == 9) {
				ret = conv_hex_str2int((buff + 1), 4, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				*all = iRet;
				ssl_dbg_msg("ALL Update flag : %d",
					*all);
				dollar_level = LEVEL_DOLLAR_ERASE_TYPE;
			}
			continue;
		}

		if (buff[0] == '#') {
			if (strlen(buff + 1) < 1) {
				ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH;
				break;
			}

			if (level != LEVEL_NONE) {
				ret = ERROR_PARSING_FORMAT_INVALID;
				break;
			}

			ptr = kmalloc(sizeof(struct ssl_fw_group),
					GFP_KERNEL);

			if (ptr == NULL) {
				ret = ERROR_PARSING_MALLOC_FAIL;
				break;
			}

			memset(ptr, 0x00, sizeof(struct ssl_fw_group));

			ret = conv_hex_str2int((buff + 1), strlen(buff) - 1,
					&iRet);

			if (ret < ERROR_SUCCESS)
				break;

			ptr->section.address = iRet;
			ssl_dbg_msg("address : 0x%08x\n",
					ptr->section.address);

			if (head == NULL)
				head = tail = ptr;
			else {
				tail->next = ptr;
				tail = ptr;
			}

			level = LEVEL_ADDRESS;
		} else if (buff[0] == '*') {
			if (strlen(buff + 1) < 1) {
				ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH;
				break;
			}

			if (level < LEVEL_ADDRESS) {
				ret = ERROR_PARSING_FORMAT_INVALID;
				break;
			}

			ret = conv_hex_str2int((buff + 1), strlen(buff) - 1,
					&iRet);

			if (ret < ERROR_SUCCESS)
				break;

			if (level == LEVEL_ADDRESS) {
				if (iRet < 1) {
					ret = ERROR_PARSING_DATA_CNT_FAIL;
					break;
				}
				ptr->section.byte_cnt = iRet;
				ssl_dbg_msg("byte_cnt : 0x%08x\n",
						ptr->section.byte_cnt);

				level = LEVEL_LENGTH;
			} else if (level == LEVEL_LENGTH) {
				/* is not used. */
				/* Because CPU_CFG erase_page_cnt = 0. */
				/* if (iRet < 1) {
				 *	ret = ERROR_PARSING_DATA_CNT_FAIL;
				 *	break;
				 * }
				 */
				ptr->section.erase_page_cnt = iRet;
				ssl_dbg_msg("erase_page_cnt : 0x%08x\n",
						ptr->section.erase_page_cnt);
				level = LEVEL_ERASE_SIZE;
			} else if (level == LEVEL_ERASE_SIZE) {
				ptr->section.version = iRet;
				ssl_dbg_msg(">>> version : 0x%08x\n",
						ptr->section.version);
				level = LEVEL_VERSION;
			} else if (level == LEVEL_VERSION) {
				ssl_dbg_msg("%s", buff);
				ssl_dbg_msg("checksum before : 0x%08x",
						iRet);
				ptr->section.checksum = (unsigned int)iRet;
				ssl_dbg_msg("checksum : 0x%08x\n",
						ptr->section.checksum);
				level = LEVEL_CHECKSUM;
			} else if (level == LEVEL_CHECKSUM) {
				ptr->section.reserved_01 = iRet;
				ssl_dbg_msg("reserved_01 : 0x%08x\n",
						ptr->section.reserved_01);
				level = LEVEL_RESERVE_01;
			} else if (level == LEVEL_RESERVE_01) {
				ptr->section.reserved_02 = iRet;
				ssl_dbg_msg("reserved_02 : 0x%08x\n",
						ptr->section.reserved_03);
				level = LEVEL_RESERVE_02;
			} else if (level == LEVEL_RESERVE_02) {
				ptr->section.reserved_03 = iRet;
				ssl_dbg_msg(
						">>> reserved_03 : 0x%08x\n",
						ptr->section.reserved_03);
				ptr->section.content =
					kmalloc(ptr->section.byte_cnt,
							GFP_KERNEL);

				if (ptr->section.content == NULL) {
					ret = ERROR_PARSING_MALLOC_FAIL;
					break;
				}

				memcpy(ptr->section.content, data + data_idx,
						ptr->section.byte_cnt);

				data_idx += ptr->section.byte_cnt;

				ssl_dbg_msg("SIZE %d %d\n",
						ptr->section.byte_cnt, ret);
				/*if (ret != ptr->section.byte_cnt) {
				 *	ret = ERROR_PARSING_CONTENT_SIZE_FAIL;
				 *	break;
				 *}
				 */

				ret = fw_checksum(ptr->section);

				if (ret < ERROR_SUCCESS)
					break;

				level = LEVEL_NONE;
			}
		} else {
			ret = ERROR_PARSING_FORMAT_INVALID;
		}
	} while (ret >= 0);

	*errnum = ret;

	if (ret < 0) {
		fw_free(head, 1);
		head = NULL;
	}

	fw_header->fw_group = head;

	return head;
}

/* make FW linke list for checksum */
static int ssl_make_checksum_link(struct ssl_device *dev,
		struct ssl_fw_group_header *fw_header)
{
	int err = 0;
	int i = 0, len = 0, check = 0;
	int elseChecksum[] = {
		0x00000000
			, 0x000001C0
			, 0x00001000
	};
	struct ssl_fw_group *head = NULL, *chkHead = NULL, *chkTail = NULL;

	len = sizeof(elseChecksum) / sizeof(int);
	ssl_dbg_msg("size of else checksum : %d", len);

	head = fw_header->fw_group;

	while (head) {
		ssl_dbg_msg("address : 0x%08x", head->section.address);
		check = 0;
		for (i = 0; i < len; i++) {
			if (head->section.address == elseChecksum[i]) {
				ssl_dbg_msg("except address");
				check = 1;
				break;
			}
		}

		if (check == 0) {
			ssl_dbg_msg("add address");
			err = ssl_fw_make_link(&chkHead, &chkTail,
					&head->section);

			if (err < 0)
				return err;
		}
		head = head->next;
	}

	return err;
}

/* make FW linke list using FW hex file */
static int ssl_make_header_use_bin(struct ssl_device *dev,
		struct ssl_fw_group_header *fw_header, const u8 *data,
		size_t data_size)
{
	int err = 0;
	int errnum = 0;
	int all = BOOT_UPDATE_EACH;

	fw_header->fw_group = NULL;

	if (parse_uchar2int_arr(data, data_size, fw_header,
				&errnum, &all) != NULL)
		err = ssl_make_checksum_link(dev, fw_header);

	return err;
}

/* make FW linke list using FW Header file */
static int ssl_make_header_use_head(struct ssl_device *dev,
		struct ssl_fw_group_header *fw_header)
{
	int err = 0;
	struct ssl_fw_group *head = NULL, *tail = NULL;

	/* get version from header file */
	fw_header->fw_version.productID01 = SSL_FW_CFG.content[76] |
		(SSL_FW_CFG.content[77] << 8) |
		(SSL_FW_CFG.content[78] << 16) |
		(SSL_FW_CFG.content[79] << 24);

	fw_header->fw_version.productID02 = SSL_FW_CFG.content[80] |
		(SSL_FW_CFG.content[81] << 8) |
		(SSL_FW_CFG.content[82] << 16) |
		(SSL_FW_CFG.content[83] << 24);

	fw_header->fw_version.ICName01 = SSL_FW_CFG.content[84] |
		(SSL_FW_CFG.content[85] << 8) |
		(SSL_FW_CFG.content[86] << 16) |
		(SSL_FW_CFG.content[87] << 24);

	fw_header->fw_version.ICName02 = SSL_FW_CFG.content[88] |
		(SSL_FW_CFG.content[89] << 8) |
		(SSL_FW_CFG.content[90] << 16) |
		(SSL_FW_CFG.content[91] << 24);

	fw_header->fw_version.display_version = SSL_FW_CFG.content[68] |
		(SSL_FW_CFG.content[69] << 8) |
		(SSL_FW_CFG.content[70] << 16) |
		(SSL_FW_CFG.content[71] << 24);

	fw_header->fw_version.hidden_version = SSL_FW_CFG.content[72] |
		(SSL_FW_CFG.content[73] << 8) |
		(SSL_FW_CFG.content[74] << 16) |
		(SSL_FW_CFG.content[75] << 24);

	/* make firmware link */
	err = ssl_fw_make_link(&head, &tail, &SSL_SYS_CFG);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_FW_CFG);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_FW);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_TMC_REG);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_DCSW);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_FDM);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_MPFPM);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_MPFDM);

	if (err < 0)
		return err;

	err = ssl_fw_make_link(&head, &tail, &SSL_FPM);

	if (err < 0)
		return err;

	fw_header->fw_group = head;

	return err;
}

/* DS init */
static int fw_ssl_init(struct ssl_device *dev)
{
	int ret = 0;

	ssl_dbg_msg("initialize start >>>>>>>>>>>>>>>");

	ret = ds_eflash_write(dev->client, 0xE003, 0x0007);

	if (ret < 0)
		return ERROR_SYSTEM_FAIL;

	ret = ds_eflash_write(dev->client, 0xE000, 0x0048);

	if (ret < 0)
		return ERROR_SYSTEM_FAIL;

	ssl_dbg_msg("<<<<<<<<<<<< initialize end(%d)", ret);
	return ret;
}

int ssl_free_header(struct ssl_device *dev)
{
	int err = 0;
	struct ssl_fw_group *head = NULL;

	if (dev == NULL)
		return 0;

	head = fw_header.fw_group;

	if (head != NULL)
		fw_free(head, m_fw_head_bin_flag);

	return err;
}

int BIOS_WAIT_Done(struct ssl_device *dev)
{
	int i = 0;
	int err = 0;
	u8 BIOS_Status[10]  = {0,};

	for (i = 0; i < 100; i++) {
		err = ts_read_data(dev->client,
			SPD2010_BIOS_CLR_PROTECT, BIOS_Status, 2);
		if (err < 0) {
			ssl_dbg_msg("I2C READ CMD FAIL");
			return err;
		}

		if (BIOS_Status[0] == 0 && BIOS_Status[1] == 0)
			return 0;
		else
			printk("bios status : 0x%x, 0x%x\n",
				BIOS_Status[0], BIOS_Status[1]);

		msleep(10);
	}

	ssl_dbg_msg("BIOS Clear timeout");
	return 1;
}

/* check version between SEEPROM and Header.
 * ( display and hidden version, product id, icname )
 */
static int ssl_firmware_version_check(struct ssl_device *dev,
		struct ssl_fw_group_header *fw_header)
{
	ssl_dbg_msg("PRODUCT ID E:0x%08x 0x%08x / H:0x%08x 0x%08x",
			dev->fw_version.productID01,
			dev->fw_version.productID02,
			fw_header->fw_version.productID01,
			fw_header->fw_version.productID02);

	ssl_dbg_msg("ICNAME E:0x%08x 0x%08x / H:0x%08x 0x%08x",
			dev->fw_version.ICName01,
			dev->fw_version.ICName02,
			fw_header->fw_version.ICName01,
			fw_header->fw_version.ICName02);

	ssl_dbg_msg("DISPLAY VERSION  EFLASH : 0x%08x\t0x%08x",
			dev->fw_version.display_version,
			fw_header->fw_version.display_version);

	ssl_dbg_msg("HIDDEN VERSION  EFLASH : 0x%08x\tHEADER : 0x%08x",
			dev->fw_version.hidden_version,
			fw_header->fw_version.hidden_version);

	if (dev->fw_version.display_version == 0xFFFFFFFF &&
			fw_header->fw_version.display_version != 0xFFFFFFFF)
		goto update;

	/* If hidden version of Header or seeprom is HIDDEN_VERSION_FACTORY,
	 * update the SEEPROM from header file.
	 */
	if ((dev->fw_version.hidden_version ==
				HIDDEN_VERSION_FACTORY) &&
			(fw_header->fw_version.hidden_version !=
			 HIDDEN_VERSION_FACTORY)) {
		ssl_dbg_msg("SEEPROM has the factory version in HV");
		goto update;
	}
	if ((dev->fw_version.hidden_version !=
				HIDDEN_VERSION_FACTORY) &&
			(fw_header->fw_version.hidden_version ==
			 HIDDEN_VERSION_FACTORY)) {
		ssl_dbg_msg("Header has the factory version in HV.");
		goto update;
	}

	/* The display version must over 1. If the display version is 0,
	 * then driver update the SEEPROM from header file.
	 */
	if (dev->fw_version.display_version == 0x00 &&
			fw_header->fw_version.display_version != 0x00) {
		ssl_dbg_msg("SEEPROM display version is 0. upgrade.");
		goto update;
	}

	if (dev->fw_version.productID01 !=
		fw_header->fw_version.productID01 ||
		dev->fw_version.productID02 !=
		fw_header->fw_version.productID02) {
		ssl_dbg_msg("Produect ID mismatch!!!");
		goto out;
	}

	if (dev->fw_version.ICName01 !=
		fw_header->fw_version.ICName01 ||
		dev->fw_version.ICName02 !=
		fw_header->fw_version.ICName02) {
		ssl_dbg_msg("IC Name mismatch!!!");
		goto out;
	}

	if (dev->fw_version.display_version <
			fw_header->fw_version.display_version) {
		ssl_dbg_msg("Detect new display version!!");
		goto update;
	} else if (dev->fw_version.display_version ==
			fw_header->fw_version.display_version) {
		if (dev->fw_version.hidden_version <
				fw_header->fw_version.hidden_version) {
			ssl_dbg_msg("Detect new hidden version!!");
			goto update;
		}
	}
out:
	return 0;

update:
	return -1;
}

int FWMergeFile_parsing(struct ssl_device *dev,
	struct ssl_fw *FW_Part, struct ssl_fw_info *FW_Info,
	u8 *merged_fw, int sz)
{
	int i = 0;
	int k = 0;
	int ret = 0;
	int file_read_index = 0;
	int indicate_index = 0;

	u8 strtmp[9] = {0,};
	u8 file_data[100] = {0, };
	int MergeInfo[100] = {0, };
	int file_temp = 0;
	int fw_part_index = 0;
	int mergeinfo_index = 0;
	int temp_data = 0;
	int mergeinfo_idex = 0;
	char charValue[20] = {0,};
	char charValueEx[20] = {0,};

	ssl_dbg_msg("merged file parsing start");

	for (i = 0; i < 14; i++) {
		if (FW_Part[i].byte_cnt > 0)
			FW_Part[i].content = NULL;

		FW_Part[i].address = 0xFFFFFFFF;
		FW_Part[i].byte_cnt = 0;
	}

	for (k = 0; k < sz; k++) {
		for (i = 0; i < 100; i++) {
			file_data[i] = merged_fw[k + i];
			if (merged_fw[k + i] == '\n')
				break;
		}

		/****************************
		* search '$' - New FW Part
		****************************/
		if ((file_data[0] == '$') &&
			(file_read_index == 0) &&
			(MergeInfo != NULL)) {
			int *MergeInfoBuf = MergeInfo;

			k += i;
			file_read_index = 0;
			indicate_index = 0;

			memset(charValue, 0, sizeof(charValue));
			memset(charValueEx, 0, sizeof(charValue));

			charValue[0] = (char)"0";
			charValue[1] = (char)"x";

			for (i = 0; i < 8; i++) {
				sprintf(strtmp, "%02x", file_data[i + 1]);
				strcat(charValue + 2, strtmp);
				strcat(charValueEx, strtmp);
			}

			printk("$ parsing - %x", charValueEx[0]);
			for (i = 1; i < 8; i++)
				printk("%x", charValueEx[i]);
			printk("\n");

			ret = conv_hex2ascii(charValueEx,
				strlen(charValueEx), &temp_data);

			ssl_dbg_msg("$ parsing result : %x", temp_data);

			MergeInfoBuf[mergeinfo_index++] = temp_data;
		} else if ((file_data[0] == '#') && (file_read_index == 0)) {
			/****************************
			* search '#' - New FW Part
			****************************/
			k += i;

			file_read_index = 1;
			indicate_index = 0;

			memset(charValue, 0, sizeof(charValue));
			memset(charValueEx, 0, sizeof(charValue));

			charValue[0] = (char)"0";
			charValue[1] = (char)"x";

			for (i = 0; i < 8; i++) {
				sprintf(strtmp, "%02x", file_data[i + 1]);
				strcat(charValue + 2, strtmp);
				strcat(charValueEx, strtmp);
			}

			ret = conv_hex2ascii(charValueEx,
				strlen(charValueEx), &temp_data);

			ssl_dbg_msg("address : %x", temp_data);
			FW_Part[fw_part_index].address = temp_data;
		} else if ((file_data[0] == '*') && (file_read_index == 1)) {
			/****************************
			* search '*' - New FW Part
			****************************/
			pr_info("* parsing..\n");
			k += i;

			memset(charValue, 0, sizeof(charValue));
			memset(charValueEx, 0, sizeof(charValue));

			charValue[0] = (char)"0";
			charValue[1] = (char)"x";

			for (i = 0; i < 8; i++) {
				sprintf(strtmp, "%02x", file_data[i + 1]);
				strcat(charValue + 2, strtmp);
				strcat(charValueEx, strtmp);
			}

			ret = conv_hex2ascii(charValueEx,
				strlen(charValueEx), &temp_data);
			ssl_dbg_msg("* Parsing result : %x",
				temp_data);

			switch (indicate_index) {
			case 0:
				pr_info("case 0\n");
				FW_Part[fw_part_index].byte_cnt =
					temp_data;
				FW_Part[fw_part_index].content =
				(u8 *)kmalloc(
				FW_Part[fw_part_index].byte_cnt * 2,
				sizeof(u8));
				break;

			case 1:
				pr_info("case 1\n");
				FW_Part[fw_part_index].erase_page_cnt =
					temp_data;
				break;

			case 2:
				pr_info("case 2\n");
				FW_Part[fw_part_index].version =
					temp_data;
				break;

			case 3:
				pr_info("case 3\n");
				FW_Part[fw_part_index].checksum =
					temp_data;
				break;

			case 4:
				pr_info("case 4\n");
				FW_Part[fw_part_index].reserved_01 =
					temp_data;
				break;

			case 5:
				pr_info("case 5\n");
				FW_Part[fw_part_index].reserved_02 =
					temp_data;
				break;

			case 6:
				pr_info("case 6\n");
				FW_Part[fw_part_index].reserved_03 =
					temp_data;
				break;

			default:
				break;
			}

			indicate_index++;

			if (indicate_index == 7)
				file_read_index = 2;
		} else {
			file_read_index = 0;
		}

		if ((file_read_index == 2) && (indicate_index == 7)) {
			ssl_dbg_msg(
				"rd_idx == 2 and indi_idx == 7, byte_cnt : %d",
				FW_Part[fw_part_index].byte_cnt);

			k++;

			for (i = 0; i < FW_Part[fw_part_index].byte_cnt; i++)
				FW_Part[fw_part_index].content[i] =
					merged_fw[k + i];

			k += FW_Part[fw_part_index].byte_cnt;
			k--;
			file_read_index = 0;
			fw_part_index++;
		}
	}

	/***************************
	* FW information parsion
	****************************/
	mergeinfo_idex = 4;

	FW_Info->DVersion =
		(int)((MergeInfo[mergeinfo_index-1] << 24) |
			  (MergeInfo[mergeinfo_idex-2] << 16) |
			  (MergeInfo[mergeinfo_idex-3] << 8) |
			  MergeInfo[mergeinfo_idex-4]);

	ssl_dbg_msg("FW_Info.DVersion : %d",
		FW_Info->DVersion);

	/* TODO : add fw information (20.02.28) */

	mergeinfo_idex = 12;

	FW_Info->FWPart_Num = fw_part_index;
	pr_info("FW_Info.FWPart_Num : %d\n",
			FW_Info->FWPart_Num);

	return false;
}

int check_bios_status(struct ssl_device *dev)
{
	u8 HDP_Data[10] = {0,};
	u8 TIC_Status[10]  = {0,};
	u8 cmd[2] = {0,};
	int err = 0;

	pr_info("Enter the check_bios_status...\n");

	/******************************************************************
	*err = ts_read_data(dev->client,
	*	SPD2010_GET_STATUS, TIC_Status, 2);
	*
	*if (err < 0) {
	*	ssl_dbg_msg("I2C READ CMD FAIL");
	*	return 1;
	*} else {
	*	pr_info("  status get succeed (%x, %x)\n",
	*		TIC_Status[0], TIC_Status[1]);
	*}
	*
	*if ((TIC_Status[1] & 0x40) == 0) {
	*	pr_info("  change to bios mode\n");
	*
	*	udelay(60);
	*
	*	cmd[0] = 0x10;
	*	cmd[1] = 0x00;
	*	err = ts_write_data(dev->client,
	*		SPD2010_GOTO_BIOS, cmd, 2);
	*
	*	if (err >= 0) {
	*		ssl_dbg_msg("I2C WRITE CMD FAIL");
	*		return 1;
	*	}
	*
	*	err = ts_read_data(dev->client,
	*		SPD2010_GET_STATUS, TIC_Status, 2);
	*
	*	if (err < 0) {
	*		ssl_dbg_msg("I2C READ CMD FAIL");
	*		return 1;
	*	}
	*
	*	pr_info("   bios status (%x, %x)\n",
	*		TIC_Status[0], TIC_Status[1]);
	*
	*	if ((TIC_Status[1] & 0x40) == 0) {
	*		ssl_dbg_msg("Failed to enter the BIOS mode");
	*		return 1;
	*	}
	*}
	*************************************************/

	/* use force bios */
	ssl_dbg_msg("force bios");
	spd2010_force_bios();

	pr_info("check current mode...");
	err = ts_read_data(dev->client,
		SPD2010_GET_STATUS, TIC_Status, 2);

	if (err < 0) {
		ssl_dbg_msg("I2C READ CMD FAIL");
		return 1;
	}

	pr_info("   bios status (%x, %x)\n",
		TIC_Status[0], TIC_Status[1]);

	if (((TIC_Status[1] >> 6) & 0x01) == 1) {
		pr_info("BIOS mode...\n");

		udelay(60);

		cmd[0] = 0x00;
		cmd[1] = 0x01;
		err = ts_write_data(dev->client,
			SPD2010_BIOS_CLR_PROTECT, cmd, 2);

		if (err < 0) {
			pr_info("bios clear protect cmd send fail\n");
			return 1;
		}

		pr_info("BIOS_WAIT_Done...\n");
		if (BIOS_WAIT_Done(dev)) {
			pr_info("bios wait done fail");
			return 1;
		}

		/* Read SEEPROM ID */
		pr_info("read hdp...\n");
		err = ts_read_data(dev->client,
			SPD2010_HDP, HDP_Data, 3);

		if (err >= 0) {
			ssl_info_msg("SEEPROM ID : %d-%d-%d",
			HDP_Data[0], HDP_Data[1], HDP_Data[2]);
		} else {
			ssl_dbg_msg("I2C READ CMD FAIL");
			return 1;
		}
	} else {
		ssl_dbg_msg("Failed to enter the BIOS mode");
		return 1;
	}

	return 0;
}

int BIOS_ERASE_Chip(struct ssl_device *dev)
{
	unsigned char HDP_Data[10] = {0,};
	int i = 0;
	int ret = 0;
	u8 cmd[2] = {0,};

	ssl_info_msg("enter BIOS_ERASE_Chip()");
	udelay(10);

	cmd[0] = 0x11;
	cmd[1] = 0x01;
	ret = ts_write_data(dev->client,
		SPD2010_BIOS_CLR_PROTECT, cmd, 2);

	if (ret < 0)
		printk("fail to sent BIOS erase command\n");
	else
		printk("send BIOS erase command...\n");

	udelay(60);

	if (BIOS_WAIT_Done(dev))
		return 1;

	udelay(60);

	cmd[0] = 0x10;
	cmd[1] = 0x02;
	for (i = 0; i < 1000; i++) {
		pr_info("=>BIOS_CLR_PROTECT send...\n");
		ts_write_data(dev->client,
			SPD2010_BIOS_CLR_PROTECT, cmd, 2);

		udelay(60);

		pr_info("=>hdp status read...\n");
		ts_read_data(dev->client, SPD2010_HDP, HDP_Data, 3);

		if ((HDP_Data[0] & 0x0001) == 0) {
			ssl_info_msg("BIOS_ERASE_Chip : check complete\n");
			return 0;
		}

		udelay(60);
	}

	return 1;
}

int BIOS_ERASE_Sector(struct ssl_device *dev,
	struct ssl_mem_info TMC_Info, int Add)
{
	u8 HDP_Data[10] = {0,};
	u8 SendData[1024] = {0,};
	int nbyte = 0;
	int loop_i = 0;
	u8 cmd[2] = {0,};

	/* Generate SEEPROM Access Command */
	SendData[nbyte++] = (char)(Add & 0xFF);
	SendData[nbyte++] = (char)((Add >> 8) & 0xFF);
	SendData[nbyte++] = (char)((Add >> 16) & 0xFF);
	SendData[nbyte++] = (char)((Add >> 24) & 0xFF);

	SendData[nbyte++] =
		(char)(TMC_Info.ERASE_SECTOR_SIZE & 0xFF);
	SendData[nbyte++] =
		(char)((TMC_Info.ERASE_SECTOR_SIZE >> 8) & 0xFF);
	SendData[nbyte++] =
		(char)((TMC_Info.ERASE_SECTOR_SIZE >> 16) & 0xff);
	SendData[nbyte++] =
		(char)((TMC_Info.ERASE_SECTOR_SIZE >> 24) & 0xff);

	/* Send SEEPROM Access Address Set */
	ts_write_data(dev->client,
		SPD2010_SEEPROM_ACCESS, SendData, nbyte);

	/* Send SEEPROM Action Command */
	cmd[0] = 0x21;
	cmd[1] = 0x01;
	ts_write_data(dev->client,
		SPD2010_BIOS_CLR_PROTECT, cmd, 2);

	if (BIOS_WAIT_Done(dev))
		return 1;

	/* Send SEEPROM Action Done check */
	for (loop_i = 0; loop_i < 1000; loop_i++) {
		cmd[0] = 0x10;
		cmd[1] = 0x02;
		ts_write_data(dev->client,
			SPD2010_BIOS_CLR_PROTECT, cmd, 2);

		ts_read_data(dev->client,
			SPD2010_HDP, HDP_Data, 3);

		if ((HDP_Data[0] & 0x0001) == 0) {
			ssl_info_msg(
			"BIOS_ERASE_Chip : check complete\n");
			return 0;
		}
	}

	return 1;
}

int BIOS_READ_FW(struct ssl_device *dev, int Add,
	int bytecnt, char verify_buf[])
{
	const int SEEPROM_READ_SIZEMAX = READ_MAX_LENGTH;
	u8 ReadData[1024] = {0, };
	int idx, loop_k = 0;
	int ReadAdd = 0;
	int ReadMnByte = 0;
	int verify_read_idx = 0;

	for (loop_k = 0; loop_k < bytecnt;
		loop_k += SEEPROM_READ_SIZEMAX) {
		ReadAdd = Add + loop_k;

		if ((bytecnt - loop_k) >= SEEPROM_READ_SIZEMAX)
			ReadMnByte = SEEPROM_READ_SIZEMAX;
		else
			ReadMnByte = (bytecnt - loop_k);

		SE_GAMMA_SL_READ_DATA(dev->client, ReadAdd,
			ReadMnByte, ReadData);

		for (idx = 0; idx < ReadMnByte; idx++)
			verify_buf[verify_read_idx + idx] =
				ReadData[idx];

		verify_read_idx += ReadMnByte;

		mdelay(1);
	}

	return 0;
}

int Download_FW(struct ssl_device *dev, struct ssl_fw *FW_Part,
	struct ssl_mem_info TMC_Info, int min_index, int chip_erase)
{
	int i = 0;
	int fail_cnt = 0;
	static char verify_buf[READ_MAX_LENGTH * 128];
	int verify_goal_cnt = 0;

	ssl_info_msg("----------------------------------------");
	ssl_info_msg("Download Flow");
	ssl_info_msg("----------------------------------------");
	ssl_info_msg("StartAdd : %d", FW_Part[min_index].address);
	ssl_info_msg("ByteCnt : %d", FW_Part[min_index].byte_cnt);

	if (BIOS_WRITE_FW(dev, FW_Part[min_index].address,
		FW_Part[min_index].content,
		FW_Part[min_index].byte_cnt))
		return 1;

	ssl_info_msg("============ Read Data ============\n");

	verify_goal_cnt = FW_Part[min_index].byte_cnt;

	if (BIOS_READ_FW(dev, FW_Part[min_index].address,
		FW_Part[min_index].byte_cnt, verify_buf))
		return 1;

	for (i = 0; i < FW_Part[min_index].byte_cnt; i++) {
		if (verify_buf[i] != FW_Part[min_index].content[i]) {
			ssl_info_msg("Verify Fail! [%x, %x]\n",
				verify_buf[i], FW_Part[min_index].content[i]);
			fail_cnt++;
			if (fail_cnt > 50)
				break;
		}
	}

	if (fail_cnt <= 0)
		ssl_info_msg("Verify result is OK\n");
	else
		return 1;

	ssl_info_msg("Download complete !!\n");

	/* HW Reset */
	spd2010_reset();

	return 0;
}

void Set_TMC_Info(struct ssl_mem_info TMC_Info)
{
	TMC_Info.CPU = 0x0000;
	TMC_Info.CPUCFG = 0x1F400;
	TMC_Info.SYSCFG = 0x1F000;
	TMC_Info.TMCREG = 0x1C000;
	TMC_Info.DCSW = 0xFFFFF;
	TMC_Info.FPM = 0xFFFFF;
	TMC_Info.FDM = 0x1D00;
	TMC_Info.MPFPM = 0xFFFFF;
	TMC_Info.MPFDM = 0x1B000;

	TMC_Info.ERASE_SECTOR_SIZE = 4096;
}

/* Update using F/W file. */
int ssl_firmware_update_byfile(struct ssl_device *dev,
	char *filename)
{
	int i, k, min_value, min_index;
	int errnum = 0;
	int retry = FW_MAX_RETRY_COUNT;
	struct file *src;
	mm_segment_t oldfs;
	size_t fw_size = 0, read_size = 0;
	u8 *fw_data = NULL;
	int allerease_set = 1;
	struct ssl_fw FW_Part[14];
	struct ssl_fw_info FW_Info;
	struct ssl_mem_info TMC_Info;
	u8 ID_Info[3] = {0,};

	if (filename == NULL)
		return ERROR_PARSING_FILENAME_IS_NULL;

	ssl_dbg_msg("=== download start ===");

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	Set_TMC_Info(TMC_Info);

	src = filp_open(filename, O_RDONLY,
		S_IRUSR|S_IRGRP|S_IROTH);

	if (IS_ERR(src)) {
		ssl_dbg_msg("[%s] file open error!!",
			filename);
		return ERROR_PARSING_FILE_OPEN_FAIL;
	} else {
		ssl_dbg_msg("[%s] file open success",
			filename);
	}

	fw_size = src->f_path.dentry->d_inode->i_size;

	if (fw_size <= 0) {
		ssl_dbg_msg("fw size error (%d)", fw_size);
		return ERROR_PARSING_FILE_OPEN_FAIL;
	}

	fw_data = kzalloc(fw_size, GFP_KERNEL);
	read_size = vfs_read(src,
			(char __user *)fw_data, fw_size,
			&src->f_pos);

	ssl_dbg_msg(
		"file path %s, size %lu Bytes\n",
		filename, (unsigned long int)fw_size);

	if (read_size != fw_size) {
		ssl_dbg_msg(
			"File copy error (size : %lu : %lu)",
			(unsigned long int)fw_size,
			(unsigned long int)read_size);
		return ERROR_PARSING_FILE_OPEN_FAIL;
	}

	/* FW Merge file parsing */
	FWMergeFile_parsing(dev, FW_Part,
		&FW_Info, fw_data, fw_size);

	/* Read ID */
	SE_GAMMA_SL_RDID(dev, ID_Info);

	udelay(10);

	/* Download Init */
	SE_GAMMA_SL_INIT(dev);

	udelay(10);

	/* Erase Macro (Erase all) */
	SE_GAMMA_SL_ERASE_MACRO(dev);

	udelay(10);

	/******************************
	* FW Download Part
	*******************************/
	pr_info("===> FW_Info.FWPart_Num : %d\n",
		FW_Info.FWPart_Num);
	for (i = 0; i < FW_Info.FWPart_Num; i++) {
		min_value = 0x6FFFFFFF;

		for (k = 0; k < FW_Info.FWPart_Num; k++) {
			if (FW_Part[k].address < min_value) {
				min_value = FW_Part[k].address;
				min_index = k;
			} else if (FW_Part[k].address ==
				min_value) {
				FW_Part[k].address = 0x7FFFFFFF;
				FW_Part[k].content = NULL;
			}
		}

		if (min_value == 0x6FFFFFFF)
			continue;

		/* Download */
		if (Download_FW(dev, FW_Part,
			TMC_Info, min_index, allerease_set)) {
			pr_info("Download_FW fail...\n");
			for (k = 0; k < FW_Info.FWPart_Num;
				k++) {
				FW_Part[k].content = NULL;
				errnum = 1;
			}
			break;
		}

		FW_Part[min_index].address = 0x7FFFFFFF;
		FW_Part[min_index].content = NULL;
	}
	kfree(fw_data);

	filp_close(src, NULL);
	set_fs(oldfs);

	return errnum;
}

int BIOS_WRITE_PAGE_EFLASH(struct ssl_device *dev,
	int Add, char data[], int st_idx, int bytecnt)
{
	char SendData[1024] = {0,};
	int nbyte = 0;
	int i = 0;
	u8 cmd[2] = {0,};

	/* Generate SEEPROM Access Command */
	SendData[nbyte++] =
		(char)(Add & 0xff);
	SendData[nbyte++] =
		(char)((Add >> 8) & 0xff);
	SendData[nbyte++] =
		(char)((Add >> 16) & 0xff);
	SendData[nbyte++] =
		(char)((Add >> 24) & 0xff);

	SendData[nbyte++] =
		(char)(bytecnt & 0xff);
	SendData[nbyte++] =
		(char)((bytecnt >> 8) & 0xff);
	SendData[nbyte++] =
		(char)((bytecnt >> 16) & 0xff);
	SendData[nbyte++] =
		(char)((bytecnt >> 24) & 0xff);

	/* Send SEEPROM Access Command */
	ts_write_data(dev->client,
		SPD2010_SEEPROM_ACCESS, SendData, nbyte);

	/* Generate SEEPROM Data */
	nbyte = 0;

	for (i = 0; i < bytecnt; i++)
		SendData[i] = data[st_idx + i];

	ts_write_data_burst(dev->client,
		SPD2010_HDP, SendData, bytecnt);

	cmd[0] = 0x31;
	cmd[1] = 0x01;

	ts_write_data(dev->client,
		SPD2010_BIOS_CLR_PROTECT, cmd, 2);

	msleep(1);

	return BIOS_WAIT_Done(dev);
}

int BIOS_WRITE_FW(struct ssl_device *dev,
	int Add, char data[], int bytecnt)
{
	int i = 0;
	int loop_i = 0;
	const int WRITE_SIZEMAX = PROGRAM_PAGE_SIZE;
	int sendnbyte = 0;
	char sliced_data[PROGRAM_PAGE_SIZE];

	memset(sliced_data, 0, PROGRAM_PAGE_SIZE);

	ssl_info_msg("BIOS_WRITE_FW() byte count %d",
		bytecnt);

	for (loop_i = 0; loop_i < bytecnt; loop_i +=
		WRITE_SIZEMAX) {
		if ((bytecnt - loop_i) >= WRITE_SIZEMAX)
			sendnbyte = WRITE_SIZEMAX;
		else
			sendnbyte = (bytecnt - loop_i);

		pr_info("BWF loop index : %d, sendnbyte : %d\n",
			loop_i, sendnbyte);

		for (i = 0; i < sendnbyte; i++)
			sliced_data[i] = data[loop_i + i];

		if (SE_GAMMA_SL_PROGRAM_PAGE(dev->client,
			Add + loop_i, sendnbyte, sliced_data))
			return 1;
	}

	return 0;
}

u8 *ssl_get_version(struct ssl_device *dev,
		u8 *ver_buff)
{
	memcpy(ver_buff, (u8 *)&(dev->fw_version),
			sizeof(struct ssl_version));
	return ver_buff;
}

