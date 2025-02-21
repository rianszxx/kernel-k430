/*
 * DW9714AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9714AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static int g_s4DW9714AF_RELEASE;////[LGE_UPDATE][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int g_sr = 3;

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16) pBuff[0]) << 4) + (pBuff[1] >> 4);

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[2] = { (char)(a_u2Data >> 4), (char)((a_u2Data & 0xF) << 4) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

    if (g_i4MotorStatus == 1)
	    {stMotorInfo.bIsMotorMoving = 1;}
    else 
	    {stMotorInfo.bIsMotorMoving = 0;}

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
	    stMotorInfo.bIsMotorOpen = 0;

	

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

//[LGE_UPDATE_S][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE
#define THRESHOLD 2
static int toMacroSteps(int max_step, int interval, int CurrentPos, int TargetPos)
{
	unsigned short tmpTargetPos;
	printk("[DW9714AF] toMacroSteps from=%d, to=%d \n", CurrentPos, TargetPos);
	if(TargetPos >= CurrentPos){
		return 0;
	}
	while( max_step >= THRESHOLD )
	{
		while( abs(CurrentPos - TargetPos) > max_step )
		{
			if(TargetPos >= CurrentPos){
				printk("Inverse!!\n");
				return 0;
			}
			tmpTargetPos = CurrentPos - max_step;
			printk("tmpTargetPos=%d\n", tmpTargetPos);
			if(s4AF_WriteReg(tmpTargetPos) == 0)
			{
				CurrentPos = tmpTargetPos;
				printk("CurrentPos=%d\n", CurrentPos);
			}
			else
			{
				printk("[DW9714AF] set I2C failed when moving the motor \n");
				g_i4MotorStatus = -1;
				return 0;
			}
			udelay(300);
			//udelay(400); Add after verification
		}
		max_step = (THRESHOLD < max_step)? THRESHOLD : 0;
	}
	if(s4AF_WriteReg(TargetPos) == 0)
	{
		CurrentPos = TargetPos;
	}
	else
	{
		printk("[DW9714AF] set I2C failed when moving the motor \n");
		g_i4MotorStatus = -1;
	}
	g_u4CurrPosition = CurrentPos;
	printk("Leave toMacroSteps\n");
	return 0;
}

static int toInfSteps(int max_step, int interval, int CurrentPos, int TargetPos)
{
	unsigned short tmpTargetPos;
	printk("[DW9714AF] toInfSteps from=%d, to=%d \n", CurrentPos, TargetPos);
	if(TargetPos <= CurrentPos){
		return 0;
	}
	while( max_step >= THRESHOLD )
	{
		while( abs(CurrentPos - TargetPos) > max_step )
		{
			if(TargetPos <= CurrentPos){
				printk("Inverse!!\n");
				return 0;
			}
			tmpTargetPos = CurrentPos + max_step;
			printk("tmpTargetPos=%d\n", tmpTargetPos);
			if(s4AF_WriteReg(tmpTargetPos) == 0)
			{
				CurrentPos = tmpTargetPos;
				printk("CurrentPos=%d\n", CurrentPos);
			}
			else
			{
				printk("[DW9714AF] set I2C failed when moving the motor \n");
				g_i4MotorStatus = -1;
				return 0;
			}
			udelay(300);
			//udelay(400); add after verification
		}
		max_step = (THRESHOLD < max_step)? THRESHOLD : 0;
	}
	if(s4AF_WriteReg(TargetPos) == 0)
	{
		CurrentPos = TargetPos;
	}
	else
	{
		printk("[DW9714AF] set I2C failed when moving the motor \n");
		g_i4MotorStatus = -1;
	}
	g_u4CurrPosition = CurrentPos;
	printk("Leave toInfSteps\n");
	return 0;
}
//[LGE_UPDATE_E][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4AF_ReadReg(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(g_pAF_SpinLock);
        g_i4Dir = 1;
        spin_unlock(g_pAF_SpinLock);
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(g_pAF_SpinLock);
        g_i4Dir = -1;
        spin_unlock(g_pAF_SpinLock);
    }
    else                                        {return 0;}

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

            spin_lock(g_pAF_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(g_pAF_SpinLock);
//[LGE_UPDATE_S][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE
			if(g_s4DW9714AF_RELEASE == 1)
			{
				toMacroSteps(600, 10, g_u4CurrPosition, (g_u4TargetPosition));
			}
			else if( (g_i4Dir == -1) && (g_u4CurrPosition!=g_u4TargetPosition) && (g_u4TargetPosition<700)){
				toMacroSteps(100, 10, g_u4CurrPosition, (g_u4TargetPosition));
			}
			else if( (g_i4Dir == 1) && (g_u4CurrPosition!=g_u4TargetPosition) && (g_u4TargetPosition<700)){
				toInfSteps(100, 10, g_u4CurrPosition, (g_u4TargetPosition));
			}
			else
			{
	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");

	                spin_lock(g_pAF_SpinLock);
	                g_i4MotorStatus = -1;
	                spin_unlock(g_pAF_SpinLock);
	}
			}
	//[LGE_UPDATE_E][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE
    return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
   LOG_INF("Start \n");

	//[LGE_UPDATE_S][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE

	g_s4DW9714AF_RELEASE = 1;
	printk("[DW9714AF] DW9714AF_Release - Start\n");
    if (*g_pAF_Opened)
    {
        LOG_INF("Free \n");
        msleep(1);
        moveAF( (g_u4AF_INF+5) );

        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 0;
		g_s4DW9714AF_RELEASE = 0;
		//[LGE_UPDATE_E][kyunghun.oh@lge.com][2015-10-26] DECREASE_AF_TIC_NOISE
        spin_unlock(g_pAF_SpinLock);
    }

    LOG_INF("End \n");

    return 0;
}

void DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
