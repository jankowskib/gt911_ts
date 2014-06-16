/* drivers/input/touchscreen/goodix_tool.c
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:1.2
 *        V1.0:2012/05/01,create file.
 *        V1.2:2012/06/08,modify some warning.
 *        V1.4:2012/08/28,modified to support GT9XX
 *
 */

#include "gt9xx.h"

#define IC_TYPE_NAME        "GT911" //Default
#define DATA_LENGTH_UINT    512
#define CMD_HEAD_LENGTH     (sizeof(st_cmd_head) - sizeof(u8*))
#define GOODIX_ENTRY_NAME   "goodix_tool"

//#define UPDATE_FUNCTIONS

#ifdef UPDATE_FUNCTIONS
extern s32 gup_enter_update_mode(struct i2c_client *client);
extern void gup_leave_update_mode(void);
extern s32 gup_update_proc(void *dir);
#endif

extern void gtp_irq_disable(struct goodix_ts_data *);
extern void gtp_irq_enable(struct goodix_ts_data *);

#pragma pack(1)
typedef struct{
    u8  wr;         //write read flag£¬0:R  1:W  2:PID 3:  0x00
    u8  flag;       //0:no need flag/int 1: need flag  2:need int 0x01
    u8 flag_addr[2];  //flag address 0x02https://www.codeaurora.org/cgit/quic/la/kernel/msm-3.10/commit/?id=f53bcf29a6e7a66b3d935b8d562fa00829261f05
    u8  flag_val;   //flag val 0x04
    u8  flag_relation;  //flag_val:flag 0:not equal 1:equal 2:> 3:< 0x05
    u16 circle;     //polling cycle 0x06
    u8  times;      //polling times 0x08
    u8  retry;      //I2C retry times 0x09
    u16 delay;      //delay befor read or after write 0xA
    u16 data_len;   //data length 0xC
    u8  addr_len;   //address length 0xE
    u8  addr[2];    //address 0xF
    u8  res[3];     //reserved 0x11
    u8* data;       //data pointer 0x
}st_cmd_head;
#pragma pack()
st_cmd_head cmd_head;

static struct i2c_client *gt_client = NULL;

static struct proc_dir_entry *goodix_proc_entry;
static struct mutex lock; // https://www.codeaurora.org/cgit/quic/la/kernel/msm-3.10/commit/?id=f53bcf29a6e7a66b3d935b8d562fa00829261f05

static s32 goodix_tool_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
static s32 goodix_tool_read( char *page, char **start, off_t off, int count, int *eof, void *data );
static s32 (*tool_i2c_read)(u8 *, u16);
static s32 (*tool_i2c_write)(u8 *, u16);

extern u16 show_len;
extern u16 total_len;

s32 DATA_LENGTH = 0;
s8 IC_TYPE[16] = {0};

static s32 tool_i2c_read_no_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    s32 i = 0;
    struct i2c_msg msgs[2];
    
    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = gt_client->addr;
    msgs[0].len   = cmd_head.addr_len;
    msgs[0].buf   = &buf[0];
    msgs[0].scl_rate = GT9xx_I2C_RATE; 
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = gt_client->addr;
    msgs[1].len   = len;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    msgs[1].scl_rate = GT9xx_I2C_RATE; 
    
    for (i = 0; i < cmd_head.retry; i++)
    {
        ret=i2c_transfer(gt_client->adapter, msgs, 2);
        if (ret > 0)
        {
            break;
        }
    }

	if (i == cmd_head.retry) {
		dev_err(&gt_client->dev, "I2C read retry limit over.\n");
		ret = -EIO;
	}

    return ret;
}

static s32 tool_i2c_write_no_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    s32 i = 0;
	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.addr  = gt_client->addr,
		.len   = len,
		.buf   = buf,
		.scl_rate =  GT9xx_I2C_RATE,
		.udelay = 2000,
	};
    
    for (i = 0; i < cmd_head.retry; i++)
    {
        ret=i2c_transfer(gt_client->adapter, &msg, 1);
        if (ret > 0)
        {
            break;
        }
    }
	if (i == cmd_head.retry) {
		dev_err(&gt_client->dev, "I2C write retry limit over.\n");
		ret = -EIO;
	}
    return ret;
}

static s32 tool_i2c_read_with_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    u8 pre[2] = {0x0f, 0xff};
    u8 end[2] = {0x80, 0x00};

    tool_i2c_write_no_extra(pre, 2);
    ret = tool_i2c_read_no_extra(buf, len);
    tool_i2c_write_no_extra(end, 2);

    return ret;
}

static s32 tool_i2c_write_with_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    u8 pre[2] = {0x0f, 0xff};
    u8 end[2] = {0x80, 0x00};

    tool_i2c_write_no_extra(pre, 2);
    ret = tool_i2c_write_no_extra(buf, len);
    tool_i2c_write_no_extra(end, 2);

    return ret;
}

static void register_i2c_func(void)
{
//    if (!strncmp(IC_TYPE, "GT818", 5) || !strncmp(IC_TYPE, "GT816", 5) 
//        || !strncmp(IC_TYPE, "GT811", 5) || !strncmp(IC_TYPE, "GT818F", 6) 
//        || !strncmp(IC_TYPE, "GT827", 5) || !strncmp(IC_TYPE,"GT828", 5)
//        || !strncmp(IC_TYPE, "GT813", 5))
    if (strncmp(IC_TYPE, "GT8110", 6) && strncmp(IC_TYPE, "GT8105", 6)
        && strncmp(IC_TYPE, "GT801", 5) && strncmp(IC_TYPE, "GT800", 5)
        && strncmp(IC_TYPE, "GT801PLUS", 9) && strncmp(IC_TYPE, "GT811", 5)
        && strncmp(IC_TYPE, "GTxxx", 5))
    {
        tool_i2c_read = tool_i2c_read_with_extra;
        tool_i2c_write = tool_i2c_write_with_extra;
        GTP_DEBUG("I2C function: with pre and end cmd!");
    }
    else
    {
        tool_i2c_read = tool_i2c_read_no_extra;
        tool_i2c_write = tool_i2c_write_no_extra;
        GTP_INFO("I2C function: without pre and end cmd!");
    }
}

static void unregister_i2c_func(void)
{
    tool_i2c_read = NULL;
    tool_i2c_write = NULL;
    GTP_INFO("I2C function: unregister i2c transfer function!");
}


s32 init_wr_node(struct i2c_client *client)
{
    u8 i;

    gt_client = client;
    memset(&cmd_head, 0, sizeof(cmd_head));
    cmd_head.data = NULL;

    i = 5;
    while ((!cmd_head.data) && i)
    {
        cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);
        if (NULL != cmd_head.data)
        {
            break;
        }
        i--;
    }
    if (i)
    {
       // DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
        DATA_LENGTH = i * DATA_LENGTH_UINT;
        GTP_INFO("Applied memory size:%d.", DATA_LENGTH);
    }
    else
    {
        GTP_ERROR("Apply for memory failed.");
        return FAIL;
    }

    cmd_head.addr_len = 2;
    cmd_head.retry = 5;

    register_i2c_func();
	
	mutex_init(&lock);
	
    goodix_proc_entry = create_proc_entry(GOODIX_ENTRY_NAME, 0660, NULL);
    if (goodix_proc_entry == NULL)
    {
        GTP_ERROR("Couldn't create proc entry!");
        return FAIL;
    }
    else
    {
        GTP_INFO("Create proc entry success!");
        goodix_proc_entry->write_proc = goodix_tool_write;
        goodix_proc_entry->read_proc = goodix_tool_read;
    }

    return SUCCESS;
}

void uninit_wr_node(void)
{
    kfree(cmd_head.data);
    cmd_head.data = NULL;
    unregister_i2c_func();
    remove_proc_entry(GOODIX_ENTRY_NAME, NULL);
}

static u8 relation(u8 src, u8 dst, u8 rlt)
{
    u8 ret = 0;
    
    switch (rlt)
    {
    case 0:
        ret = (src != dst) ? true : false;
        break;

    case 1:
        ret = (src == dst) ? true : false;
        GTP_DEBUG("equal:src:0x%02x   dst:0x%02x   ret:%d.", src, dst, (s32)ret);
        break;

    case 2:
        ret = (src > dst) ? true : false;
        break;

    case 3:
        ret = (src < dst) ? true : false;
        break;

    case 4:
        ret = (src & dst) ? true : false;
        break;

    case 5:
        ret = (!(src | dst)) ? true : false;
        break;

    default:
        ret = false;
        break;    
    }

    return ret;
}

/*******************************************************    
Function:
    Comfirm function.
Input:
  None.
Output:
    Return write length.
********************************************************/
static u8 comfirm(void)
{
    s32 i = 0;
    u8 buf[32];
    
//    memcpy(&buf[GTP_ADDR_LENGTH - cmd_head.addr_len], &cmd_head.flag_addr, cmd_head.addr_len);
//    memcpy(buf, &cmd_head.flag_addr, cmd_head.addr_len);//Modified by Scott, 2012-02-17
    memcpy(buf, cmd_head.flag_addr, cmd_head.addr_len);
   
    for (i = 0; i < cmd_head.times; i++)
    {
        if (tool_i2c_read(buf, 1) <= 0)
        {
            GTP_ERROR("Read flag data failed!");
            return FAIL;
        }
        if (true == relation(buf[GTP_ADDR_LENGTH], cmd_head.flag_val, cmd_head.flag_relation))
        {
            GTP_DEBUG("value at flag addr:0x%02x.", buf[GTP_ADDR_LENGTH]);
            GTP_DEBUG("flag value:0x%02x.", cmd_head.flag_val);
            break;
        }

        msleep(cmd_head.circle);
    }

    if (i >= cmd_head.times)
    {
        GTP_ERROR("Didn't get the flag to continue!");
        return FAIL;
    }

    return SUCCESS;
}

/*******************************************************    
Function:
    Goodix tool write function.
Input:
  standard proc write function param.
Output:
    Return write length.
********************************************************/
static s32 goodix_tool_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
    s32 ret = 0;
    GTP_DEBUG_FUNC();
    GTP_DEBUG_ARRAY((u8*)buff, len);
    
    mutex_lock(&lock);
    
    ret = copy_from_user(&cmd_head, buff, CMD_HEAD_LENGTH);
    if(ret)
    {
        GTP_ERROR("copy_from_user failed.");
       	ret = -EACCES;
		goto exit;
    }

    GTP_DEBUG("wr  :0x%02x.", cmd_head.wr);
    GTP_DEBUG("flag:0x%02x.", cmd_head.flag);
    GTP_DEBUG("flag addr:0x%02x%02x.", cmd_head.flag_addr[0], cmd_head.flag_addr[1]);
    GTP_DEBUG("flag val:0x%02x.", cmd_head.flag_val);
    GTP_DEBUG("flag rel:0x%02x.", cmd_head.flag_relation);
    GTP_DEBUG("circle  :%d.", (s32)cmd_head.circle);
    GTP_DEBUG("times   :%d.", (s32)cmd_head.times);
    GTP_DEBUG("retry   :%d.", (s32)cmd_head.retry);
    GTP_DEBUG("delay   :%d.", (s32)cmd_head.delay);
    GTP_DEBUG("data len:%d.", (s32)cmd_head.data_len);
    GTP_DEBUG("addr len:%d.", (s32)cmd_head.addr_len);
    GTP_DEBUG("addr:0x%02x%02x.", cmd_head.addr[0], cmd_head.addr[1]);
    GTP_DEBUG("len:%d.", (s32)len);
    GTP_DEBUG("buf[20]:0x%02x.", buff[CMD_HEAD_LENGTH]);
    
    if (cmd_head.wr == GTP_RW_WRITE)
    {
		if (cmd_head.data_len > (DATA_LENGTH - GTP_ADDR_LENGTH)) 
		{
			GTP_ERROR("data len %d > data buff %d, rejected!\n", cmd_head.data_len, (DATA_LENGTH - GTP_ADDR_LENGTH));
			ret = -EINVAL;
			goto exit;
		}
		if (cmd_head.addr_len > GTP_ADDR_LENGTH) {
			GTP_ERROR(" addr len %d > data buff %d, rejected!\n", cmd_head.addr_len, GTP_ADDR_LENGTH);
			ret = -EINVAL;
			goto exit;
		}
    
      //  copy_from_user(&cmd_head.data[cmd_head.addr_len], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
        ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
        if(ret)
        {
            GTP_ERROR("copy_from_user failed.");
        }
        memcpy(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len], cmd_head.addr, cmd_head.addr_len);

        GTP_DEBUG_ARRAY(cmd_head.data, cmd_head.data_len + cmd_head.addr_len);
        GTP_DEBUG_ARRAY((u8*)&buff[CMD_HEAD_LENGTH], cmd_head.data_len);

        if (cmd_head.flag == GTP_NEED_FLAG)
        {
            if (comfirm() == FAIL)
            {
                GTP_ERROR("[WRITE]Comfirm fail!");
                ret = -EINVAL;
                goto exit;
            }
        }
        else if (2 == cmd_head.flag)
        {
            //Need interrupt!
        }
        if (tool_i2c_write(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],
            cmd_head.data_len + cmd_head.addr_len) <= 0)
        {
            GTP_ERROR("[WRITE]Write data failed!");
            ret = -EIO;
            goto exit;
        }

        GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],cmd_head.data_len + cmd_head.addr_len);
        if (cmd_head.delay)
        {
            msleep(cmd_head.delay);
        }

        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;
    }
    else if (cmd_head.wr == GTP_RW_READ_IC_TYPE)  //Write ic type
    {
		ret = copy_from_user(&cmd_head.data[0], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
      
        if(ret)
        {
            GTP_ERROR("copy_from_user failed.");
        }
        
        if (cmd_head.data_len > sizeof(IC_TYPE))
        {
			pr_err("<<-GTP->> data len %d > data buff %d, rejected!\n",
			cmd_head.data_len, sizeof(IC_TYPE));
			ret = -EINVAL;
			goto exit;
		}
        memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);

        register_i2c_func();

        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;
    }
    else if (cmd_head.wr == GTP_RW_NO_WRITE)
    {
        //memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);

        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;
    }
    else if (7 == cmd_head.wr)//disable irq!
    {
        gtp_irq_disable(i2c_get_clientdata(gt_client));
        
        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;
    }
    else if (9 == cmd_head.wr) //enable irq!
    {
        gtp_irq_enable(i2c_get_clientdata(gt_client));

        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;
    }
    else if(17 == cmd_head.wr)
    {
        struct goodix_ts_data *ts = i2c_get_clientdata(gt_client);
        ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
        if(ret)
        {
            GTP_DEBUG("copy_from_user failed.");
        }
        if(cmd_head.data[GTP_ADDR_LENGTH])
        {
            GTP_DEBUG("gtp enter rawdiff.");
            ts->gtp_rawdiff_mode = true;
        }
        else
        {
            ts->gtp_rawdiff_mode = false;
            GTP_DEBUG("gtp leave rawdiff.");
        }
      
        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;
    }
#ifdef UPDATE_FUNCTIONS
    else if (11 == cmd_head.wr)//Enter update mode!
    {
        if (FAIL == gup_enter_update_mode(gt_client))
        {
			ret = -EBUSY;
            goto exit;
        }
    }
    else if (13 == cmd_head.wr)//Leave update mode!
    {
        gup_leave_update_mode();
    }
    else if (15 == cmd_head.wr) //Update firmware!
    {
        show_len = 0;
        total_len = 0;
        if (cmd_head.data_len + 1 > DATA_LENGTH) 
        {
			GTP_ERROR("data len %d > data buff %d, rejected!\n",
			cmd_head.data_len + 1, DATA_LENGTH);
			ret = -EINVAL;
			goto exit;
		}
        memset(cmd_head.data, 0, cmd_head.data_len + 1);
        memcpy(cmd_head.data, &buff[CMD_HEAD_LENGTH], cmd_head.data_len);

        if (gup_update_proc((void*)cmd_head.data) == FAIL)
        {
			ret = -EBUSY;
            goto exit;
        }
    }
#endif
    else if(cmd_head.wr == GTP_RW_RESET_GUITAR) // -- bj --
    {
		GTP_INFO("Reseting guitar...");
		struct goodix_9xx_platform_data *pdata = gt_client->dev.platform_data;
		GTP_GPIO_OUTPUT(pdata->gpio_reset, 1);
        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		goto exit;	
	}
    ret = CMD_HEAD_LENGTH;
    
exit:
    mutex_unlock(&lock);
    return ret;
}

/*******************************************************    
Function:
    Goodix tool read function.
Input:
  standard proc read function param.
Output:
    Return read length.
********************************************************/
static s32 goodix_tool_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	s32 ret;
    GTP_DEBUG_FUNC();
    
    mutex_lock(&lock);
    
    if (cmd_head.wr % 2)
    {
		GTP_ERROR("[READ]Wrong command. Only evens are allowed to read!");
        ret = -EINVAL;
        goto exit;
    }
    else if (cmd_head.wr == GTP_RW_READ)
    {
        u16 len = 0;
        s16 data_len = 0;
        u16 loc = 0;
        
        if (1 == cmd_head.flag)
        {
            if (FAIL == comfirm())
            {
                GTP_ERROR("[READ]Comfirm fail!");
                ret = -EINVAL;
				goto exit;
            }
        }
        else if (2 == cmd_head.flag)
        {
            //Need interrupt!
        }

        memcpy(cmd_head.data, cmd_head.addr, cmd_head.addr_len);

        GTP_DEBUG("[CMD HEAD DATA] ADDR:0x%02x%02x.", cmd_head.data[0], cmd_head.data[1]);
        GTP_DEBUG("[CMD HEAD ADDR] ADDR:0x%02x%02x.", cmd_head.addr[0], cmd_head.addr[1]);
        
        if (cmd_head.delay)
        {
            msleep(cmd_head.delay);
        }
        
        data_len = cmd_head.data_len;
        while(data_len > 0)
        {
            if (data_len > DATA_LENGTH)
            {
                len = DATA_LENGTH;
            }
            else
            {
                len = data_len;
            }
            data_len -= len;

            if (tool_i2c_read(cmd_head.data, len) <= 0)
            {
                GTP_ERROR("[READ]Read data failed!");
                ret = -EINVAL;
				goto exit;
            }
            memcpy(&page[loc], &cmd_head.data[GTP_ADDR_LENGTH], len);
            loc += len;

            GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH], len);
            GTP_DEBUG_ARRAY(page, len);
        }
    }
    else if (cmd_head.wr == GTP_RW_READ_IC_TYPE)
    {
    //    memcpy(page, "gt8", cmd_head.data_len);
       // memcpy(page, "GT818", 5);
      //  page[5] = 0;

        GTP_DEBUG("Return ic type:%s len:%d.", page, (s32)cmd_head.data_len);
		ret = cmd_head.data_len;
		goto exit;
    }
    else if (4 == cmd_head.wr)
    {
        page[0] = show_len >> 8;
        page[1] = show_len & 0xff;
        page[2] = total_len >> 8;
        page[3] = total_len & 0xff;

		ret = cmd_head.data_len;
		goto exit;
    }
    else if (6 == cmd_head.wr)
    {
        //Read error code!
    }
    else if (8 == cmd_head.wr)  //Read driver version
    {
       // memcpy(page, GTP_DRIVER_VERSION, strlen(GTP_DRIVER_VERSION));
       s32 tmp_len;
       tmp_len = strlen(GTP_DRIVER_VERSION);
       memcpy(page, GTP_DRIVER_VERSION, tmp_len);
       page[tmp_len] = 0;
    }
 
 ret = cmd_head.data_len;
    
exit:
    mutex_unlock(&lock);
    return ret;
}
