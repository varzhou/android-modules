#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/device.h>

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#include <linux/sensor/gsnsr_reg.h>

struct gsnsr_data_t{
	struct input_dev *input_dev;
	int irq;
	int ax;
	int ay;
	int az;
	struct work_struct work;
	struct i2c_client *client;
};

/*
**	、注册sensor的输入子系统
*/

static struct gsnsr_data_t g_gsnsr_data;

static ssize_t enable_sensor(struct device *device, struct device_attribute *attr,
											const char *buf, size_t count){
		
	return 0;
}
		
static ssize_t set_sensortime(struct device *device, struct device_attribute *attr,
											const char *buf, size_t count){
		 
	return 0;
}


static DEVICE_ATTR(enable_sensor, 0644, NULL, enable_sensor);
static DEVICE_ATTR(set_sensortime, 0644, NULL, set_sensortime);


static struct attribute *mpu605x_input_sysfs_entries[] = {
	&dev_attr_enable_sensor.attr,
	&dev_attr_set_sensortime.attr,
	NULL
};


static struct attribute_group mpu605x_input_attr_group = {
	.name = "mpu6050",
	.attrs	= mpu605x_input_sysfs_entries,
};


static int mpu6050_writereg(const struct i2c_client *cli,u8 reg_add,u8 reg_dat)
{
	return i2c_smbus_write_byte_data(cli, reg_dat, reg_add);
}


static void reset_device(struct i2c_client *cli){
	mpu6050_writereg(cli,MPU_PWR_MGMT1_REG,0X80); //复位MPU6050
	msleep(100);
	mpu6050_writereg(cli,MPU_PWR_MGMT1_REG,0X00); //唤醒MPU6050 
}

static void mpu6050_set_lpf(struct i2c_client *cli,u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else 
		data=6; 
	mpu6050_writereg(cli,MPU_CFG_REG,data);//设置数字低通滤波器  
}


static void mpu6050_set_rate(struct i2c_client *cli,u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=mpu6050_writereg(cli,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	mpu6050_set_lpf(cli,rate/2);	//自动设置LPF为采样率的一半
}


/*
**	、初始化硬件设备
*/
static int gsnsr_bsp_Initlation(struct i2c_client *cli,struct gsnsr_platform_data *info){
	int chipid = -1;
	reset_device(cli);
	
	chipid = i2c_smbus_read_byte_data(cli, MPU6050_WHO_AM_I);
	if(chipid != MPU6050_ADDRESS){
		printk("is not support this devices and id = %d\n",chipid);
		return -ENODEV;
	}
	mpu6050_writereg(cli,MPU_SAMPLE_RATE_REG,20 );  //陀螺仪采样率  
    mpu6050_writereg(cli,MPU_CFG_REG,0x06 );            
	
    mpu6050_writereg(cli,MPU_GYRO_CFG_REG,(3<<3));   
    mpu6050_writereg(cli,MPU_ACCEL_CFG_REG,0 );   	//配置加速度传感器工作在16G模式  
    
	mpu6050_writereg(cli,MPU_FIFO_EN_REG,0X00);		//关闭FIFO
	mpu6050_writereg(cli,MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	
    mpu6050_writereg(cli,MPU_GYRO_CFG_REG,0x18);    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)  
	
	mpu6050_writereg(cli,MPU_INTBP_CFG_REG,0x9c);	 //设置为低电平触发
	mpu6050_writereg(cli,MPU_INT_EN_REG,0x01); 		 //选择为数据中断  
	return 0;
}

static u8 mpu_get_accelerometer(struct i2c_client *cli,short *ax,short *ay,short *az)
{
    u8 buf[6];
	int res = -1;  
	res = i2c_smbus_read_i2c_block_data(cli, MPU_ACCEL_XOUTH_REG, 6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}


static irqreturn_t mpu6050_handler_isr(int irq, void *regs){

	disable_irq_nosync(g_gsnsr_data.irq);
	schedule_work(&g_gsnsr_data.work);
	return IRQ_HANDLED ;
}

static void gsensor_data_irq_work(struct work_struct *work) {
	u8 sta = 0;
	u16 ax = 0,ay = 0,az = 0;
	sta = i2c_smbus_read_byte_data(g_gsnsr_data.client,MPU_INT_STA_REG);
	printk("gsensor_data_irq_work %d\n",sta);
	if(sta & 0x01){
		mpu_get_accelerometer(g_gsnsr_data.client,&ax,&ay,&az);
		printk("get mpu6050 data x = %d y = %d z = %d\n",ax,ay,az);
		input_report_abs(g_gsnsr_data.input_dev, ABS_X, ax);
		input_report_abs(g_gsnsr_data.input_dev, ABS_Y, ay);
		input_report_abs(g_gsnsr_data.input_dev, ABS_Z, az);
		input_sync(g_gsnsr_data.input_dev);
	}
	enable_irq(g_gsnsr_data.irq);
}


static int gsensor_i2c_probe(struct i2c_client *cli, const struct i2c_device_id *id){
	int ret = -EINVAL;
	struct gsnsr_platform_data *info;
	printk("hello i found mpu6050 devices\n");
	
	/*首先检查I2C是否使能*/
	if(!i2c_check_functionality(cli->adapter, I2C_FUNC_I2C)){
		printk("i2c0 is not enable\n");
		return -ENODEV;
	}
	g_gsnsr_data.client = cli;
	g_gsnsr_data.input_dev = input_allocate_device();
	if(NULL == g_gsnsr_data.input_dev){
		printk("input_allocate_device fail\n");
		return -ENODEV;
	}
	info = (struct gsnsr_platform_data *)(cli->dev.platform_data);
	if(NULL == info){
		printk("info is NULL\n");
		goto skip_alloc;
	}
	g_gsnsr_data.irq = info->irq;
	if(0 != gsnsr_bsp_Initlation(cli,info)){
		goto skip_alloc;
	}
	INIT_WORK(&g_gsnsr_data.work, gsensor_data_irq_work);

	g_gsnsr_data.input_dev->evbit[0] = BIT_MASK(EV_ABS);//标示input支持的类型
	g_gsnsr_data.input_dev->keybit[BIT_WORD(BTN_X)] = BIT_MASK(BTN_X);
	g_gsnsr_data.input_dev->keybit[BIT_WORD(BTN_Y)] = BIT_MASK(BTN_Y);
	g_gsnsr_data.input_dev->keybit[BIT_WORD(BTN_Z)] = BIT_MASK(BTN_Z);
	
	input_set_abs_params(g_gsnsr_data.input_dev, ABS_X, 0, 0x3FF, 32, 0);
	input_set_abs_params(g_gsnsr_data.input_dev, ABS_Y, 0, 0x3FF, 32, 0);
	input_set_abs_params(g_gsnsr_data.input_dev, ABS_Z, 0, 0x3FF, 32, 0);
	
	g_gsnsr_data.input_dev->name = "MPU605x";
	g_gsnsr_data.input_dev->id.bustype = BUS_I2C;
	g_gsnsr_data.input_dev->id.vendor = 0xDEAD;
	g_gsnsr_data.input_dev->id.product = 0xBEEF;
	g_gsnsr_data.input_dev->id.version = 0x0102;
	printk("irq = %d\n",info->irq);
	
	ret = request_irq(info->irq,mpu6050_handler_isr,
				IRQF_TRIGGER_LOW,
				"mpu6050", cli);
	if(ret){
		printk("request_irq fail %d\n",ret);
	}
	disable_irq(info->irq);
	ret = input_register_device(g_gsnsr_data.input_dev);
	enable_irq(info->irq);
	return 0;
	
skip_alloc:
	input_free_device(g_gsnsr_data.input_dev);
	return 0;
}

static int gsensor_i2c_remove(struct i2c_client *cli){
	struct gsnsr_platform_data *info = (struct gsnsr_platform_data *)cli->dev.platform_data;
	free_irq(info->irq, NULL);
	return 0;
}


static const struct i2c_device_id mpu6050_i2c_id[] = {
	{ "bsp_mpu605x", 0 },
	{ }
};


static struct i2c_driver mpu6050_i2c_driver = {
	.driver = {
		.name = "bsp_mpu605x",
		.owner = THIS_MODULE,
	},
	.probe =    gsensor_i2c_probe,
	.remove =   __devexit_p(gsensor_i2c_remove),
	.id_table = mpu6050_i2c_id,
};


static __init int gsensor_Init(void){
	return i2c_add_driver(&mpu6050_i2c_driver);
}

static __exit void gsensor_Exit(void){
	i2c_del_driver(&mpu6050_i2c_driver);
}

module_init(gsensor_Init);
module_exit(gsensor_Exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zyw");
MODULE_DESCRIPTION("ALL Belong to yuanyang eclec");



