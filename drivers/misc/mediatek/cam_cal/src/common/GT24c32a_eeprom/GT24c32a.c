/*
 * Driver for EEPROM
 *
 *
 */

#if 0
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "eeprom.h"
#include "eeprom_define.h"
#include "GT24c32a.h"
#include <asm/system.h>  /* for SMP */
#else

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
//#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "GT24c32a.h"
//#include <asm/system.h>  /* for SMP */
#endif
/* #define EEPROMGETDLT_DEBUG */
#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
#define EEPROMDB printk//pr_debug
#else
#define EEPROMDB(x, ...)
#endif

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

static DEFINE_SPINLOCK(g_EEPROMLock); /* for SMP */
#define EEPROM_I2C_BUSNUM 1
static struct i2c_board_info kd_eeprom_dev __initdata = { I2C_BOARD_INFO("CAM_CAL_DRV", 0xAA >> 1)};

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1 /* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "CAM_CAL_DRV"
#define EEPROM_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
/* #define FM50AF_EEPROM_I2C_ID 0x28 */
#define FM50AF_EEPROM_I2C_ID 0xA1

/*******************************************************************************
/* define LSC data for M24C08F EEPROM on L10 project */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 2
#define Boundary_Address 256
#define EEPROM_Address_Offset 0xC


/*******************************************************************************
*
********************************************************************************/
static struct i2c_client * g_pstI2Cclient = NULL;

/* 81 is used for V4L driver */
static dev_t g_EEPROMdevno = MKDEV(EEPROM_DEV_MAJOR_NUMBER, 0);
static struct cdev * g_pEEPROM_CharDrv = NULL;
/* static spinlock_t g_EEPROMLock; */
/* spin_lock(&g_EEPROMLock); */
/* spin_unlock(&g_EEPROMLock); */

static struct class *EEPROM_class = NULL;
static atomic_t g_EEPROMatomic;
/* static DEFINE_SPINLOCK(kdeeprom_drv_lock); */
/* spin_lock(&kdeeprom_drv_lock); */
/* spin_unlock(&kdeeprom_drv_lock); */
static unsigned short EEPROM_Address[2] = {0x0,0x0} ;

/*******************************************************************************
*
********************************************************************************/
/*
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iBurstReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadReg(u16 a_u2Addr , u8 *a_puBuff , u16 i2cId);
*/
/*******************************************************************************
*
********************************************************************************/
/* maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt6516.c which is 8 bytes */
int iWriteEEPROM(u16 a_u2Addr  , u32 a_u4Bytes, u8 *puDataInBytes)
{
	u32 u4Index;
	int i4RetValue;
	char puSendCmd[8] = {
		(char)(a_u2Addr >> 8) ,
		(char)(a_u2Addr & 0xFF) ,
		0, 0, 0, 0, 0, 0
	};
	if (a_u4Bytes + 2 > 8) {
		EEPROMDB("[S24EEPROM] exceed I2c-mt65xx.c 8 bytes limitation (include address 2 Byte)\n");
		return -1;
	}

	for (u4Index = 0; u4Index < a_u4Bytes; u4Index += 1)
		puSendCmd[(u4Index + 2)] = puDataInBytes[u4Index];

	i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2));
	if (i4RetValue != (a_u4Bytes + 2)) {
		EEPROMDB("[S24EEPROM] I2C write  failed!!\n");
		return -1;
	}
	mdelay(10); /* for tWR singnal --> write data form buffer to memory. */

	/* EEPROMDB("[EEPROM] iWriteEEPROM done!!\n"); */
	return 0;
}


/* maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes */
int iReadEEPROM(u16 a_u2Addr, u32 ui4_length, u8 *a_puBuff)
{
	int  i4RetValue = 0;
	char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

	/* EEPROMDB("[EEPROM] iReadEEPROM!!\n"); */

	if (ui4_length > 8) {
		EEPROMDB("[S24EEPROM] exceed I2c-mt65xx.c 8 bytes limitation\n");
		return -1;
	}
	spin_lock(&g_EEPROMLock); /* for SMP */
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_EEPROMLock); /* for SMP */

	/* EEPROMDB("[EEPROM] i2c_master_send\n"); */
	i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	if (i4RetValue != 2) {
		EEPROMDB("[EEPROM] I2C send read address failed!!\n");
		return -1;
	}

	/* EEPROMDB("[EEPROM] i2c_master_recv\n"); */
	i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
	if (i4RetValue != ui4_length) {
		EEPROMDB("[EEPROM] I2C read data failed!!\n");
		return -1;
	}
	spin_lock(&g_EEPROMLock); /* for SMP */
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & I2C_MASK_FLAG;
	spin_unlock(&g_EEPROMLock); /* for SMP */

	/* EEPROMDB("[EEPROM] iReadEEPROM done!!\n"); */
	return 0;
}


int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char *pinputdata)
{
	int  i4RetValue = 0;
	int  i4ResidueDataLength;
	u32 u4IncOffset = 0;
	u32 u4CurrentOffset;
	u8 *pBuff;

	EEPROMDB("[S24EEPROM] iWriteData\n");

#if 0
	if (ui4_offset + ui4_length >= 0x2000) {
		EEPROMDB("[S24EEPROM] Write Error!! S-24CS64A not supprt address >= 0x2000!!\n");
		return -1;
	}

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;

	EEPROMDB("[S24EEPROM] iWriteData u4CurrentOffset is %d\n", u4CurrentOffset);

	do {
		if (i4ResidueDataLength >= 6) {
			/* i4RetValue = iWriteEEPROM((u16)u4CurrentOffset, 6, pBuff); */
			if (i4RetValue != 0) {
				EEPROMDB("[EEPROM] I2C iWriteData failed!!\n");
				return -1;
			}
			u4IncOffset += 6;
			i4ResidueDataLength -= 6;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
		} else {
			/* i4RetValue = iWriteEEPROM((u16)u4CurrentOffset, i4ResidueDataLength, pBuff); */
			if (i4RetValue != 0) {
				EEPROMDB("[EEPROM] I2C iWriteData failed!!\n");
				return -1;
			}
			u4IncOffset += 6;
			i4ResidueDataLength -= 6;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
			/* break; */
		}
	} while (i4ResidueDataLength > 0);
#endif
	EEPROMDB("[S24EEPROM] iWriteData done\n");

	return 0;
}



int iReadDataFromGT24c32a(unsigned int  ui4_offset, unsigned int  ui4_length,
unsigned char *pinputdata)
{

	char puSendCmd[2];/* = {(char)(ui4_offset & 0xFF) }; */
	unsigned short SampleOffset = (unsigned short)((ui4_offset &0xFFF00000) >> 20) ;//(unsigned short)((ui4_offset) & (0x0000FFFF));
        //unsigned short EEPROM_Address[2] = {0xA0,0xA0} ;
	/* unsigned char address_offset = ((SampleNum *SampleOffset) +
	EEPROM_Address_Offset+ ui4_length) / Boundary_Address; */
	short loop[2], loopCount;
	unsigned short SampleCount;
	u8 *pBuff;
	u32 u4IncOffset = 0;
	int  i4RetValue = 0;


	pBuff = pinputdata;

	EEPROMDB("[EEPROM] ui4_offset=%x ui4_offset(80)=%x ui4_offset(8)=%x\n", ui4_offset,	(unsigned short)((ui4_offset >> 8) & 0x0000FFFF), SampleOffset);

	/* ui4_offset = (char)( (ui4_offset>>8) & 0xFF); */

#if 0
	EEPROM_Address[0] = ((0 < address_offset) ? (EEPROM_Address[0] | (address_offset - 1))
	: EEPROM_Address[0]);
	EEPROM_Address[1] = ((EEPROM_Address[0] & 0xF0) | (address_offset));

	EEPROM_Address[0] = EEPROM_Address[0] << 1;
	EEPROM_Address[1] = EEPROM_Address[1] << 1;
#endif

	EEPROMDB("[EEPROM] EEPROM_Address[0]=%x EEPROM_Address[1]=%x\n", (EEPROM_Address[0] >> 1),(EEPROM_Address[1] >> 1));

	/* loop[0] = (Boundary_Address * address_offset) - ((SampleNum *SampleOffset) +
	EEPROM_Address_Offset); */
	loop[0] = ((ui4_length >> 4) << 4);

	loop[1] = ui4_length - loop[0];


	EEPROMDB("[EEPROM] loop[0]=%d loop[1]=%d\n", (loop[0]) , (loop[1]));

	puSendCmd[0] = (char)(((SampleOffset + u4IncOffset) >> 8) & 0xFF);
	puSendCmd[1] = (char)((SampleOffset + u4IncOffset) & 0xFF);

	for (loopCount = 0; loopCount < Read_NUMofEEPROM; loopCount++) {
		do {
			if (16 <= loop[loopCount]) {

				EEPROMDB("[EEPROM]1 loopCount=%d loop[loopCount]=%d puSendCmd[0]=%x puSendCmd[1]=%x, EEPROM(%x)\n", loopCount , loop[loopCount], puSendCmd[0], puSendCmd[1], EEPROM_Address[loopCount]);
				/* iReadRegI2C(puSendCmd , 2, (u8*)pBuff,16,EEPROM_Address[loopCount]); */
				i4RetValue = iBurstReadRegI2C(puSendCmd , 2, (u8 *)pBuff, 16,
				EEPROM_Address[loopCount]);
				if (i4RetValue != 0) {
					EEPROMDB("[EEPROM] I2C iReadData failed!!\n");
					return -1;
				}
				u4IncOffset += 16;
				loop[loopCount] -= 16;
				/* puSendCmd[0] = (char)( (ui4_offset+u4IncOffset) & 0xFF) ; */
				puSendCmd[0] = (char)(((SampleOffset + u4IncOffset) >> 8) & 0xFF);
				puSendCmd[1] = (char)((SampleOffset + u4IncOffset) & 0xFF);
				pBuff = pinputdata + u4IncOffset;
			} else if (0 < loop[loopCount]) {
				EEPROMDB("[EEPROM]2 loopCount=%d loop[loopCount]=%d puSendCmd[0]=%x puSendCmd[1]=%x\n", loopCount , loop[loopCount], puSendCmd[0], puSendCmd[1]);
				/* iReadRegI2C(puSendCmd , 2, (u8*)pBuff,loop[loopCount],EEPROM_Address[loopCount]); */
				i4RetValue = iBurstReadRegI2C(puSendCmd , 2, (u8 *)pBuff, 16,
				EEPROM_Address[loopCount]);
				if (i4RetValue != 0) {
					EEPROMDB("[EEPROM] I2C iReadData failed!!\n");
					return -1;
				}
				u4IncOffset += loop[loopCount];
				loop[loopCount] -= loop[loopCount];
				/* puSendCmd[0] = (char)( (ui4_offset+u4IncOffset) & 0xFF) ; */
				puSendCmd[0] = (char)(((SampleOffset + u4IncOffset) >> 8) & 0xFF);
				puSendCmd[1] = (char)((SampleOffset + u4IncOffset) & 0xFF);
				pBuff = pinputdata + u4IncOffset;
			}
		} while (loop[loopCount] > 0);
	}

	return 0;
}

int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char *pinputdata)
{
	int  i4RetValue = 0;
	int  i4ResidueDataLength;
	u32 u4IncOffset = 0;
	u32 u4CurrentOffset;
	u8 *pBuff;
	/* EEPROMDB("[S24EEPORM] iReadData\n" ); */

	if (ui4_offset + ui4_length >= 0x2000) {
		EEPROMDB("[S24EEPROM] Read Error!! S-24CS64A not supprt address >= 0x2000!!\n");
		return -1;
	}

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		if (i4ResidueDataLength >= 8) {
			i4RetValue = iReadEEPROM((u16)u4CurrentOffset, 8, pBuff);
			if (i4RetValue != 0) {
				EEPROMDB("[EEPROM] I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += 8;
			i4ResidueDataLength -= 8;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
		} else {
			i4RetValue = iReadEEPROM((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
			if (i4RetValue != 0) {
				EEPROMDB("[EEPROM] I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += 8;
			i4ResidueDataLength -= 8;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
			/* break; */
		}
	} while (i4ResidueDataLength > 0);
	/* EEPROMDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is
	0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff); */
	/* EEPROMDB("[S24EEPORM] iReadData done\n" ); */
	return 0;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->u4Offset);
	err |= put_user(i, &data32->u4Offset);
	err |= get_user(i, &data->u4Length);
	err |= put_user(i, &data32->u4Length);
	/* Assume pointer is not change */
#if 1
	err |= get_user(p, &data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->u4Offset);
	err |= put_user(i, &data->u4Offset);
	err |= get_user(i, &data32->u4Length);
	err |= put_user(i, &data->u4Length);
	err |= get_user(p, &data32->pu1Params);
	err |= put_user(compat_ptr(p), &data->pu1Params);

	return err;
}

static long CAM_CAL_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	int err;

	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;

	CAM_CALDB("[CAMERA SENSOR] CAM_CAL_Ioctl_Compat,%p %p %x ioc size %d\n",
	filp->f_op , filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {

	case COMPAT_CAM_CALIOC_G_READ: {
		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_cal_info_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ, (unsigned long)data);
		err = compat_put_cal_info_struct(data32, data);


		if (err != 0)
			CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	default:
		return -ENOIOCTLCMD;
	}
}
#endif

/*******************************************************************************
*
********************************************************************************/
#if defined( COMMON_CAM_CAL_DRV)
int GT24C32A_EEPROM_Ioctl(
	struct file *file,
	unsigned int a_u4Command,
	unsigned long a_u4Param
)

#else

#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int EEPROM_Ioctl(struct inode *a_pstInode,
			struct file *a_pstFile,
			unsigned int a_u4Command,
			unsigned long a_u4Param)
#else
static long EEPROM_Ioctl(
	struct file *file,
	unsigned int a_u4Command,
	unsigned long a_u4Param
)
#endif

#endif
	
{
	int i4RetValue = 0, ResetCheck;
	u8 *pBuff = NULL;
	u8 *pWorkingBuff = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;
	ssize_t writeSize;
	u8 readTryagain = 0;

#ifdef EEPROMGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif
/*
	if (_IOC_NONE == _IOC_DIR(a_u4Command)) {
	} else {
*/
	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			EEPROMDB("[S24EEPROM] ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/* get input structure address */
				kfree(pBuff);
				EEPROMDB("[S24EEPROM] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
        //LGE_CHANGE_S: [2015-11-18] yonghwan.lym@lge.com, Static_Analysis
        ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
        pWorkingBuff = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
        //LGE_CHANGE_E: [2015-11-18] yonghwan.lym@lge.com, Static_Analysis
	}

	if (NULL == pWorkingBuff) {
		kfree(pBuff);
		EEPROMDB("[S24EEPROM] ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	EEPROMDB("[S24EEPROM] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);


	if (copy_from_user((u8 *)pWorkingBuff , (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pWorkingBuff);
		EEPROMDB("[S24EEPROM] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
	case CAM_CALIOC_S_WRITE:
		EEPROMDB("[S24EEPROM] Write CMD\n");
#ifdef EEPROMGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef EEPROMGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		EEPROMDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;
	case CAM_CALIOC_G_READ:
		EEPROMDB("[S24EEPROM] Read CMD\n");
#ifdef EEPROMGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		EEPROMDB("[EEPROM] offset %x\n", ptempbuf->u4Offset);
		EEPROMDB("[EEPROM] length %x\n", ptempbuf->u4Length);
		EEPROMDB("[EEPROM] Before read Working buffer address 0x%8x\n", (u32)pWorkingBuff);

#if 0
		/* iReadReg(0x0770 , u8 * a_puBuff , u16 i2cId); */
		/* iWriteRegI2C(0xFF ,1,0xA6); */
		if (0) {
			unsigned short addr;
			unsigned short get_byte = 0, loop = 0;
#if 0
			for (loop = 0xc; loop <= 0x06fd; loop++) {
				addr = 0x72;
				iReadReg(loop , (u8 *)&get_byte, addr);

				EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", loop, loop, get_byte);
				get_byte = 0;
			}
#endif
			for (addr = 0; addr < 256; addr++) {
				for (loop = 0x07FC; loop <= 0x07FF; loop++) {
					/* addr = 0x72; */
					get_byte = 0;
					iReadReg(loop , (u8 *)&get_byte, addr);

					EEPROMDB("[EEPROM]enter EEPROM_test function addr(%x) (%x)%x\n",
					addr, loop, get_byte);

				}
			}
		}
#endif

#if 0
		if (1) {
			unsigned short addr;
			unsigned short get_byte = 0, loop = 0;

			iReadReg(0x0000 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0000, get_byte);

			get_byte = 0;

			iReadReg(0x0001 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0001, get_byte);


			get_byte = 0;

			iReadReg(0x0380 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0380, get_byte);

			get_byte = 0;

			iReadReg(0x0381 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0381, get_byte);


			get_byte = 0;

			iReadReg(0x0703 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0703, get_byte);

			get_byte = 0;

			iReadReg(0x0704 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0704, get_byte);

			get_byte = 0;

			iReadReg(0x0705 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0705, get_byte);

			get_byte = 0;

			iReadReg(0x0706 , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x0706, get_byte);

#if 0
			get_byte = 0;

			iReadReg(0x07fc , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x07fc, get_byte);

			get_byte = 0;

			iReadReg(0x07fd , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x05, get_byte);

			get_byte = 0;

			iReadReg(0x07fe , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x05, get_byte);

			get_byte = 0;

			iReadReg(0x07ff , (u8 *)&get_byte, 0xA6);

			EEPROMDB("[EEPROM]enter EEPROM_test function%d (%x)%x\n", ++loop, 0x05, get_byte);

			get_byte = 0;
#endif

		}
#endif

#if 0
		if (ptempbuf->u4Offset == 0x800) {
			*(u16 *)pWorkingBuff = 0x3;

		}

		else if (ptempbuf->u4Offset == 0x0024C32a) {
			*(u32 *)pWorkingBuff = 0x0124C32a;
		} else if (ptempbuf->u4Offset == 0xFFFFFFFF) {
			char puSendCmd[1] = {0, };

			puSendCmd[0] = 0x7E;
			/* iReadRegI2C(puSendCmd , 1, pWorkingBuff,2, (0x53<<1) ); */
			EEPROMDB("[S24EEPORM] Shading CheckSum MSB=> %x %x\n", pWorkingBuff[0], pWorkingBuff[1]);
		} else {
			/* i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff); */

			i4RetValue =  iReadDataFromM24C08F((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);

		}
		EEPROMDB("[S24EEPROM] After read Working buffer data  0x%4x\n", *pWorkingBuff);
#else
#if 0
		if (ptempbuf->u4Offset == 0x0A04C32A) {
			*(u32 *)pWorkingBuff = 0x1A04C32A;
#endif
			if(4 <= ptempbuf->u4Length )
			{
				ResetCheck = *((u32 *)(ptempbuf->pu1Params)) ;
			}
			if(ptempbuf->u4Offset == 0x800)
			{
				*(u16 *)pWorkingBuff = 0x3;

			}
			else if( ((ptempbuf->u4Offset & 0x000FFFFF) == 0x0004C32A) && (EEPROM_Address[0]==0x0) )
			{
				*(u32 *)pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
				EEPROM_Address[0] = (ptempbuf->u4Offset & 0xffff0000) >> 20;
				EEPROM_Address[1] = (ptempbuf->u4Offset & 0xffff0000) >> 20;				
			}
			else if( ((ptempbuf->u4Offset & 0x000FFFFF) == 0x0004C32A) && (ResetCheck == 0xABCDFEDC) )
			{
				*(u32 *)pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
				EEPROMDB("[S24EEPROM] Reset I2C address %x \n", *(u32 *)pWorkingBuff );
			}
			else {
				readTryagain = 3;
				while (0 < readTryagain) {
					i4RetValue =  iReadDataFromGT24c32a(ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
					EEPROMDB("[S24EEPROM] error (%d) Read retry (%d)\n", i4RetValue, readTryagain);
					if (i4RetValue != 0)
						readTryagain--;
					else
						readTryagain = 0;
				}


			}
		EEPROMDB("[S24EEPROM] After read Working buffer data  0x%4x\n", *pWorkingBuff);

#endif

#ifdef EEPROMGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		EEPROMDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif

		break;
	default:
		EEPROMDB("[S24EEPROM] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		EEPROMDB("[S24EEPROM] to user length %d\n", ptempbuf->u4Length);
		EEPROMDB("[S24EEPROM] to user  Working buffer address 0x%8x\n", (u32)pWorkingBuff);
		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pWorkingBuff);
			EEPROMDB("[S24EEPROM] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pWorkingBuff);
	return i4RetValue;
}

static u32 g_u4Opened = 0;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int EEPROM_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	EEPROMDB("[S24EEPROM] EEPROM_Open\n");
	spin_lock(&g_EEPROMLock);
	if (g_u4Opened) {
		spin_unlock(&g_EEPROMLock);
		return -EBUSY;
	} /*else {*//*LukeHu--150720=For check fo*/
	if (!g_u4Opened) {/*LukeHu++150720=For check fo*/
		g_u4Opened = 1;
		atomic_set(&g_EEPROMatomic, 0);
	}
	spin_unlock(&g_EEPROMLock);

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int EEPROM_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_EEPROMLock);

	g_u4Opened = 0;

	atomic_set(&g_EEPROMatomic, 0);

	spin_unlock(&g_EEPROMLock);

	return 0;
}

static const struct file_operations g_stEEPROM_fops = {
	.owner = THIS_MODULE,
	.open = EEPROM_Open,
	.release = EEPROM_Release,
	/* .ioctl = EEPROM_Ioctl */
#if defined( COMMON_CAM_CAL_DRV)
#else
	.unlocked_ioctl = EEPROM_Ioctl
#endif
};

#define EEPROM_DYNAMIC_ALLOCATE_DEVNO 1
static inline int RegisterEEPROMCharDrv(void)
{
	struct device *EEPROM_device = NULL;

#if EEPROM_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_EEPROMdevno, 0, 1, EEPROM_DRVNAME)) {
		EEPROMDB("[S24EEPROM] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_EEPROMdevno , 1 , EEPROM_DRVNAME)) {
		EEPROMDB("[S24EEPROM] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/* Allocate driver */
	g_pEEPROM_CharDrv = cdev_alloc();

	if (NULL == g_pEEPROM_CharDrv) {
		unregister_chrdev_region(g_EEPROMdevno, 1);

		EEPROMDB("[S24EEPROM] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/* Attatch file operation. */
	cdev_init(g_pEEPROM_CharDrv, &g_stEEPROM_fops);

	g_pEEPROM_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pEEPROM_CharDrv, g_EEPROMdevno, 1)) {
		EEPROMDB("[S24EEPROM] Attatch file operation failed\n");

		unregister_chrdev_region(g_EEPROMdevno, 1);

		return -EAGAIN;
	}

	EEPROM_class = class_create(THIS_MODULE, "EEPROMdrv");
	if (IS_ERR(EEPROM_class)) {
		int ret = PTR_ERR(EEPROM_class);

		EEPROMDB("Unable to create class, err = %d\n", ret);
		return ret;
	}
	EEPROM_device = device_create(EEPROM_class, NULL, g_EEPROMdevno, NULL, EEPROM_DRVNAME);

	return 0;
}

static inline void UnregisterEEPROMCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pEEPROM_CharDrv);

	unregister_chrdev_region(g_EEPROMdevno, 1);

	device_destroy(EEPROM_class, g_EEPROMdevno);
	class_destroy(EEPROM_class);
}


/* //////////////////////////////////////////////////////////////////// */
#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int EEPROM_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int EEPROM_i2c_remove(struct i2c_client *);

static const struct i2c_device_id EEPROM_i2c_id[] = {{EEPROM_DRVNAME, 0}, {} };
#if 0 /* test110314 Please use the same I2C Group ID as Sensor */
static unsigned short force[] = {EEPROM_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END,
	I2C_CLIENT_END};
#else
/* static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END}; */
#endif
/* static const unsigned short * const forces[] = { force, NULL }; */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */


static struct i2c_driver EEPROM_i2c_driver = {
	.probe = EEPROM_i2c_probe,
	.remove = EEPROM_i2c_remove,
	/* .detect = EEPROM_i2c_detect, */
	.driver.name = EEPROM_DRVNAME,
	.id_table = EEPROM_i2c_id,
};

#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, EEPROM_DRVNAME);
	return 0;
}
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	EEPROMDB("[S24EEPROM] Attach I2C\n");
	/* spin_lock_init(&g_EEPROMLock); */

	/* get sensor i2c client */
	spin_lock(&g_EEPROMLock); /* for SMP */
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = S24CS64A_DEVICE_ID >> 1;
	spin_unlock(&g_EEPROMLock); /* for SMP */

	EEPROMDB("[EEPROM] g_pstI2Cclient->addr = 0x%8x\n", g_pstI2Cclient->addr);
	/* Register char driver */
	i4RetValue = RegisterEEPROMCharDrv();

	if (i4RetValue) {
		EEPROMDB("[S24EEPROM] register char device failed!\n");
		return i4RetValue;
	}


	EEPROMDB("[S24EEPROM] Attached!!\n");
	return 0;
}

static int EEPROM_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int EEPROM_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&EEPROM_i2c_driver);
}

static int EEPROM_remove(struct platform_device *pdev)
{
	i2c_del_driver(&EEPROM_i2c_driver);
	return 0;
}

/* platform structure */
static struct platform_driver g_stEEPROM_Driver = {
	.probe      = EEPROM_probe,
	.remove = EEPROM_remove,
	.driver     = {
		.name   = EEPROM_DRVNAME,
		.owner  = THIS_MODULE,
	}
};


static struct platform_device g_stEEPROM_Device = {
	.name = EEPROM_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init EEPROM_i2C_init(void)
{
	i2c_register_board_info(EEPROM_I2C_BUSNUM, &kd_eeprom_dev, 1);
	if (platform_driver_register(&g_stEEPROM_Driver)) {
		EEPROMDB("failed to register S24EEPROM driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stEEPROM_Device)) {
		EEPROMDB("failed to register S24EEPROM driver, 2nd time\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit EEPROM_i2C_exit(void)
{
	platform_driver_unregister(&g_stEEPROM_Driver);
}

// LGE_CHANGE_S: [2015-11-09] yonghwan.lym@lge.com, Add Module Vendor ID
void read_moduleID_GT24C32A(unsigned char * pinputdata)
{
unsigned int module_vendor_id = 0x700;
unsigned int offset = 0x4c32a;
unsigned int module_vendor_id_temp = 0;

module_vendor_id_temp = (module_vendor_id<<20)&0xfff00000;
module_vendor_id_temp |= offset;
iReadDataFromGT24c32a(module_vendor_id_temp, 2, pinputdata);

return;
}
// LGE_CHANGE_E: [2015-11-09] yonghwan.lym@lge.com, Add Module Vendor ID

#if defined( COMMON_CAM_CAL_DRV)
EXPORT_SYMBOL(GT24C32A_EEPROM_Ioctl);
EXPORT_SYMBOL(read_moduleID_GT24C32A); // LGE_CHANGE: [2015-11-09] yonghwan.lym@lge.com, Add Module Vendor ID - bug fix
#else
module_init(EEPROM_i2C_init);
module_exit(EEPROM_i2C_exit);
#endif

//MODULE_DESCRIPTION("EEPROM driver");
//MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
//MODULE_LICENSE("GPL");
