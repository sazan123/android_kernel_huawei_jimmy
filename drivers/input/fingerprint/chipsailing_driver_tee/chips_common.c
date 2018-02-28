

 
 /********************************头文件定义******************************/
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include "./inc/chips_main.h"
#include "./inc/chips_common.h"

/*********************************全局变量定义****************************/
extern struct chips_data *chips_spidev;


/*********************************函数定义********************************/

static void chips_fp_complete(void *arg)
{
	complete(arg);
}


 /**
 *  @brief chips_sync 同步/阻塞SPI数据传输
 *  
 *  @param [in] chips_data chips_data结构体指针
 *  @param [in] message    spi_message结构体指针
 *  
 *  @return 成功返回0，失败返回负数
 */
static int chips_sync(struct chips_data *chips_data,struct spi_message *message)
{
	
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	if(NULL == chips_data || NULL == message){
		chips_dbg("invalid arguments\n");
		return -EINVAL;
	}
	message->complete = chips_fp_complete;
	message->context = &done;

	spin_lock_irq(&chips_data->spin_lock);
	if (chips_data->spi == NULL){
		status = -ESHUTDOWN;
	}
	else{
		status = spi_async(chips_data->spi, message);
	}
		
	spin_unlock_irq(&chips_data->spin_lock);

	if (status == 0){
		wait_for_completion(&done);

		status = message->status;
		//chips_dbg("spi_async call success,message->status = %d\n",status);
		//if (status == 0)
		//	status = message->actual_length;
	}else{
		chips_dbg("Failed to async message,status = %d\n",status);
	}
	return status;
}
 

#define	SPI_BUFSIZ	32
#define PACKET_SIZE 1024

static uint8_t buf[SPI_BUFSIZ] = {0};

/**
 * @func：chips_spi_full_duplex - SPI synchronous write followed by read
 * @txbuf: data to be written (need not be dma-safe)
 * @n_tx: size of txbuf, in bytes
 * @rxbuf: buffer into which data will be read (need not be dma-safe)
 * @n_rx: size of rxbuf, in bytes
 * @return：return 0 on success,negative on failure
 * Context: can sleep
 
 *
 * This performs a half duplex MicroWire style transaction with the
 * device, sending txbuf and then reading rxbuf.  The return value
 * is zero for success, else a negative errno status code.
 * This call may only be used from a context that may sleep.
 *
 * Parameters to this routine are always copied using a small buffer;
 * portable code should never use this for more than 32 bytes.
 * Performance-sensitive or bulk transfer code should instead use
 * spi_{async,sync}() calls with dma-safe buffers.
 */
static int chips_spi_full_duplex(struct chips_data *chips_data,void *txbuf, unsigned n_tx,void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int			status;
	struct spi_message	message;
	//struct spi_transfer	x[2];
	struct spi_transfer x = {0};  
	uint8_t			*local_buf;
	uint32_t    packet_size = 0;

	/* Use preallocated DMA-safe buffer if we can.  We can't avoid
	 * copying here, (as a pure convenience thing), but we can
	 * keep heap costs out of the hot path unless someone else is
	 * using the pre-allocated buffer or the transfer is too large.
	 */
	if((!txbuf && !rxbuf)||(!n_tx && !n_rx)){
		chips_dbg("invalid arguments\n");
		return -EINVAL;
	}
	
	if(n_tx + n_rx < PACKET_SIZE){
		packet_size = n_tx + n_rx;
	}else if((n_tx + n_rx)%PACKET_SIZE){
		packet_size = ((n_tx + n_rx)/PACKET_SIZE + 1)*PACKET_SIZE;
	}else{
		packet_size = n_tx + n_rx;
	}
	
	if (packet_size > SPI_BUFSIZ || !mutex_trylock(&lock)) {
		local_buf = kmalloc(max((unsigned)SPI_BUFSIZ, packet_size),GFP_KERNEL);  // | GFP_DMA
		if (NULL == local_buf){
			chips_dbg("Failed to allocate mem for spi_full_duplex buffer\n");
			return -ENOMEM;
		}
	} else {
		local_buf = buf;
	}
	
	spi_message_init(&message);
	
	//initialize spi_transfer structure
	memset(&x,0,sizeof(x));
	memcpy(local_buf,txbuf,n_tx);
	x.cs_change = 0;
	x.delay_usecs = 1;
	x.speed_hz = 7000000;
	x.tx_buf = local_buf;
	x.rx_buf = local_buf;
	x.len = packet_size;

	spi_message_add_tail(&x, &message);
	status = chips_sync(chips_data,&message);
	if(status == 0){
		memcpy(rxbuf,local_buf+n_tx,n_rx);
	}else{
	  chips_dbg("Failed to sync message,status = %d\n",status);	
	}
	
	if (x.tx_buf == buf)
		mutex_unlock(&lock);
	else
		kfree(local_buf);
	
	return status;

/*
	if ((n_tx + n_rx) > SPI_BUFSIZ || !mutex_trylock(&lock)) {
		local_buf = kmalloc(max((unsigned)SPI_BUFSIZ, n_tx + n_rx),
				    GFP_KERNEL);  // | GFP_DMA
		if (!local_buf)
			return -ENOMEM;
	} else {
		local_buf = buf;
	}
	
	spi_message_init(&message);
	memset(x, 0, sizeof(x));
	if (n_tx) {
		x[0].len = n_tx;
		spi_message_add_tail(&x[0], &message);
	}
	if (n_rx) {		
		x[1].len = n_rx;
		spi_message_add_tail(&x[1], &message);
	}

	memcpy(local_buf, txbuf, n_tx);
	x[0].tx_buf = local_buf;
	x[1].rx_buf = local_buf + n_tx;

	// do the i/o 
	status = chips_sync(&message);
	if (status == 0)
		memcpy(rxbuf, x[1].rx_buf, n_rx);

	if (x[0].tx_buf == buf)
		mutex_unlock(&lock);
	else
		kfree(local_buf);
	
	return status;
	*/
}

#if 0
/**
 *  @brief chips_spi_write spi同步写
 *  
 *  @param [in] chips_data chips_data结构体指针
 *  @param [in] buf  要写入的数据的buffer指针     
 *  @param [in] len  写入的数据的字节数     
 *  
 *  @return 成功返回0,失败返回负数
 */
static int chips_spi_write(struct chips_data *chips_data,void *buf, int len)
{	
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return chips_sync(chips_data,&m);
}


 /**
 *  @brief chips_spi_read   spi同步读
 *  
 *  @param [in] chips_data  chips_data结构体指针
 *  @param [out] buf  成功时存储读取数据的buffer指针      
 *  @param [in] len  要读取的字节数      
 *  
 *  @return 成功返回0,失败返回负数
 */
static int chips_spi_read(struct chips_data *chips_data,void *buf, int len)
{
	struct spi_transfer	t = {
			.rx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return chips_sync(chips_data,&m);
}

#endif

 /**
 *  @brief chips_sfr_read 读SFR寄存器
 *  
 *  @param [in] addr  寄存器起始地址
 *  @param [out] data 读到的数据
 *  @param [in] len   读取的数据长度
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_sfr_read(uint16_t addr,uint8_t *data,uint16_t len)
{
	int status = -1;
	uint8_t tx_buf[2] = {0};

	tx_buf[0] = CHIPS_R_SFR;
	tx_buf[1] = (uint8_t)(addr & 0x00FF);

	status = chips_spi_full_duplex(chips_spidev, tx_buf, 2, data, len);
	if(status < 0){
		chips_dbg("Failed to read SFR from addr = 0x%x,len = %d\n",addr,len);
	}
	
	return status;
}


 /**
 *  @brief chips_sfr_write 写SFR寄存器
 *  
 *  @param [in] addr 寄存器起始地址
 *  @param [in] data 写入的数据
 *  @param [in] len  写入的数据长度
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_sfr_write(uint16_t addr,uint8_t *data,uint16_t len)
{
	
	uint8_t *tx_buf;
	int status = -1;

	tx_buf = (uint8_t *)kmalloc(len+2,GFP_KERNEL);
	if(NULL == tx_buf){
		chips_dbg("Failed to allocate mem for write sfr buffer\n");
		return -ENOMEM;
	}
	
	tx_buf[0] = CHIPS_W_SFR;
	tx_buf[1] = (uint8_t)(addr & 0x00FF);
	memcpy(tx_buf+2,data,len);

	status = chips_spi_full_duplex(chips_spidev,tx_buf,len+2,NULL,0);
	if(status < 0){
		chips_dbg("Failed to write SFR at addr = 0x%x,len = %d\n",addr,len);
	}
	
	if(NULL != tx_buf){
	    kfree(tx_buf);
		tx_buf = NULL;
	}
	
	return status;
}


 /**
 *  @brief chips_sram_read 读SRAM寄存器
 *  
 *  @param [in] addr 寄存器起始地址
 *  @param [in] data 读到的数据
 *  @param [in] len  读取的数据长度
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_sram_read(uint16_t addr,uint8_t *data,uint16_t len)
{
	uint8_t tx_buf[3] = {0};
	int status = -1;
	
	uint8_t *rx_buf;
	rx_buf = (uint8_t *)kmalloc(len+1, GFP_KERNEL);   //first nop
	if(NULL == rx_buf){
		chips_dbg("Failed to allocate mem for read sram buffer\n");
		return -ENOMEM;
	}
	
	tx_buf[0] = CHIPS_R_SRAM;
	tx_buf[1] = (uint8_t)((addr&0xFF00)>>8);
	tx_buf[2] = (uint8_t)(addr&0x00FF);
	
	status = chips_spi_full_duplex(chips_spidev, tx_buf, 3, rx_buf, len+1);
	if(status ==  0){
		memcpy(data,rx_buf+1,len);
	}else{
		chips_dbg("Failed to read SRAM from addr = 0x%x,len = %d\n",addr,len);
	}
		
	if(NULL != rx_buf){
		kfree(rx_buf);
		rx_buf = NULL;
	}
	
	return status;
}


 /**
 *  @brief chips_sram_write 写SRAM寄存器
 *  
 *  @param [in] addr 寄存器起始地址
 *  @param [in] data 写入的数据
 *  @param [in] len  写入的数据长度
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_sram_write(uint16_t addr,uint8_t *data,uint16_t len)
{ 
	uint8_t *tx_buf;
	int status = -1;
	
	tx_buf = (uint8_t *)kmalloc(len+3,GFP_KERNEL);
	if(NULL == tx_buf){
		chips_dbg("Failed to allocate mem for write sram buffer\n");
		return -ENOMEM;
	}

	tx_buf[0] = CHIPS_W_SRAM;
	tx_buf[1] = (uint8_t)((addr&0xFF00)>>8);
	tx_buf[2] = (uint8_t)(addr&0x00FF);        
	memcpy(tx_buf+3,data,len);

	status = chips_spi_full_duplex(chips_spidev,tx_buf,len+3,NULL,0);
	if(status < 0){
		chips_dbg("Failed to write SRAM at addr = 0x%x,len = %d\n",addr,len);
	}
	
	if(NULL != tx_buf){
		kfree(tx_buf);
		tx_buf = NULL;
	}
	
	return status;
}


 /**
 *  @brief chips_spi_send_cmd 发送spi命令
 *  
 *  @param [in] cmd spi命令
 *  @param [in] len spi命令数据长度
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_spi_send_cmd(uint8_t *cmd,uint16_t len)
{
	int status = -1;
	status = chips_spi_full_duplex(chips_spidev,cmd,len,NULL,0);
	if(status < 0){
		chips_dbg("Failed to send spi cmd\n");
	}
	
	return status;
}


 /**
 *  @brief chips_write_configs 给IC配置参数
 *  
 *  @param [in] p_param 寄存器参数结构体指针
 *  @param [in] num 参数个数    
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_write_configs(struct param *p_param, int num)
{
	struct param param;
	uint8_t data;
	int i = 0;
	int retval = 0;
	
	for(i = 0; i < num; i++)
	{
		param = p_param[i];
		
		if(param.cmd == CHIPS_W_SFR) {
			data = (uint8_t)(param.data&0x00FF);
			retval = write_SFR(param.addr,data);
			if(retval < 0){
				chips_dbg("write config err>1\n");
				return retval;
			}
			chips_dbg("param.cmd = %x,param.addr = %x,param.data = %x\n",param.cmd,param.addr,data);
		}else if(param.cmd == CHIPS_W_SRAM){
			retval = write_SRAM(param.addr,param.data);
			if(retval < 0){
				chips_dbg("write config err>2\n");
				return retval;
			}
			chips_dbg("param.cmd = %x,param.addr = %x,param.data = %x\n",param.cmd,param.addr,param.data);
		}else{
			chips_dbg("write config err+3\n");
		}
	}	
	return 0;
}


 /**
 *  @brief chips_probe_sensorID 读取芯片ID，用于兼容其他家IC
 *  
 *  @param [out] sensorid 成功时存储芯片ID
 *  
 *  @return 成功返回0,失败返回负数
 */
int chips_probe_sensorID(uint16_t *sensorid)
{
	int status = -1;
	uint8_t buffer[2] = {0};

	status = chips_sfr_read(0x3E,buffer,2);
	if(status == 0){
		*sensorid = ((buffer[1] << 8)&0xFF00)|(buffer[0]&0x00FF);
	}
	chips_dbg("hwid_h = %x\n",buffer[0]);
	chips_dbg("hwid_l = %x\n",buffer[1]);

	return status;
}


/************************************************************************************
 以下接口函数为spi调试接口，供驱动人员调试spi通信是否正常，在fops的write()函数中使用
 不要在驱动中其他地方使用这些函数，因为在驱动稳定后，这些函数将会被移除 !!!!!!!!!!!!
*************************************************************************************/

int read_SFR(uint16_t addr,uint8_t *data)
{
	return chips_sfr_read(addr,data,1);
}

int write_SFR(uint16_t addr,uint8_t data)
{
	return chips_sfr_write(addr,&data,1);
}

int read_SRAM(uint16_t addr,uint16_t *data)
{
	uint8_t rx_buf[2] = {0};
	int status = -1;
	
	status = chips_sram_read(addr,rx_buf,2);
	if(status == 0)
		*data = ((rx_buf[1] << 8)&0xFF00) | (rx_buf[0]&0x00FF);
	
	return status;
}

int write_SRAM(uint16_t addr,uint16_t data)
{
	uint8_t tx_buf[2] =  {0};
	tx_buf[0] = (uint8_t)(data&0x00FF);  //低8位
	tx_buf[1] = (uint8_t)((data&0xFF00)>>8);  //高8位

	return chips_sram_write(addr,tx_buf,2);
}


 /**
 *  @brief chips_sensor_config 给IC下载参数
 *  
 *  @return 无返回值
 */
void chips_sensor_config(void)
{	
    write_SFR(0x0F, 0x01);
    write_SFR(0x1C, 0x1D);
    write_SFR(0x1F, 0x0A);
    write_SFR(0x42, 0xAA);
    write_SFR(0x60, 0x08);
    write_SFR(0x63, 0x60);
    //write_SFR(0x47, 0x60);//add 20160712
    //write_SFR(0x13, 0x31);//add 20160712
    //chips_sram_write(0xFC1E, 0x0);//add 20160712

    /*****for 3.3V********/
    write_SFR(0x22, 0x07);
    write_SRAM(0xFC8C, 0x0001);
    write_SRAM(0xFC90, 0x0001);
    /*********************/
    write_SRAM(0xFC02, 0x0420);
    write_SRAM(0xFC1A, 0x0C30);
    write_SRAM(0xFC22, 0x085C);//chips_sram_write(0xFC22, 0x0848);打开Normal下8个敏感区域
    write_SRAM(0xFC2E, 0x00F9);//chips_sram_write(0xFC2E, 0x008F);	//chips_sram_write(0xFC2E, 0x00F6); 20160624
    write_SRAM(0xFC30, 0x0270);//chips_sram_write(0xFC30, 0x0260);	//chips_sram_write(0xFC30, 0x0300);	// 20160624
    write_SRAM(0xFC06, 0x0039);
    write_SRAM(0xFC08, 0x0008);//add  times    20160624
    write_SRAM(0xFC0A, 0x0016);
    write_SRAM(0xFC0C, 0x0022);
    write_SRAM(0xFC12, 0x002A);
    write_SRAM(0xFC14, 0x0035);
    write_SRAM(0xFC16, 0x002B);
    write_SRAM(0xFC18, 0x0039);
    write_SRAM(0xFC28, 0x002E);
    write_SRAM(0xFC2A, 0x0018);
    write_SRAM(0xFC26, 0x282D);
    write_SRAM(0xFC82, 0x01FF);
    write_SRAM(0xFC84, 0x0007);
    write_SRAM(0xFC86, 0x0001);
  	write_SRAM(0xFC80, 0x0718); //chips_write_sram_bit(0xFC80, 12,0);
    write_SRAM(0xFC88, 0x380B); //chips_write_sram_bit(0xFC88, 9, 0);
    write_SRAM(0xFC8A, 0x7C8B); //chips_write_sram_bit(0xFC8A, 2, 0); 
    write_SRAM(0xFC8E, 0x3354);
}


 /**
 *  @brief chips_scan_one_image 获取图像数据
 *  
 *  @param [in] addr    图像数据存储地址
 *  @param [out] buffer 成功时存储图像数据
 *  @param [in] len     读取的图像数据长度
 *   
 *  @return 成功返回0，失败返回负数
 */
int chips_scan_one_image(uint16_t addr, uint8_t *buffer, uint16_t len)
{
    int status = -1;
  
	status = write_SRAM(0xFC00,0x0003); 
	if(status < 0){
		chips_dbg("Failed to write 0x0003 to reg_0xFC00\n");
		return status;	
	}
	
	mdelay(3);
	
	status = chips_sram_read(addr,buffer,len);
	if(status < 0){
	 	chips_dbg("Failed to read from addr = 0x%x,status = %d\n",addr,status);
	}
	
	return status;
}


 /**
 *  @brief chips_spi_wakeup spi唤醒
 *  
 *  @return 成功返回0，失败返回负数
 *  
 *  @detail 用于在SLEEP以及DEEP_SLEEP模式下唤醒IC;不可靠函数.
 */
static int chips_spi_wakeup(void)
{
	int status = -1;
	uint8_t wakeup = 0xd5;
	
	status = chips_spi_send_cmd(&wakeup, 1);
	if(status < 0){
		chips_dbg("Failed to send spi cmd, cmd = 0x%x",wakeup);
	}
	
	return status;
}


 /**
 *  @brief chips_force_to_idle 切换IC到IDLE模式
 *  
 *  @return 成功返回0，失败返回负数
 */
static int chips_force_to_idle(void)
{
	uint8_t mode = 0;
	uint8_t status = 0;
	int retval = -1;
	int i = 0;

	retval = read_SFR(0x46, &mode);
	if(retval < 0){
		chips_dbg("Failed to read reg_0x46\n");
		return -1;
	}
	
	retval = read_SFR(0x50,&status);
	if(retval < 0){
		chips_dbg("Failed to read reg_0x50+\n");
		return -1;
	}

	if(mode == 0x70 && status == 0){
		return 0;
	}else if(mode == 0x71){
		;
	}else{
		chips_spi_wakeup();	
		//mdelay(1);
	}

    //注：程序走到这里时，IC可能的模式有SLEEP、NORMAL以及DEEP_SLEEP
	//判断spi_wakeup是否成功,并判断此时IC处于何种模式
	retval = read_SFR(0x46, &mode);            
	if(retval < 0){
		chips_dbg("Failed to read reg_0x46\n");
		return -1;
	}
	retval = read_SFR(0x50,&status);         
	if(retval < 0){
		chips_dbg("Failed to read reg_0x50+1\n");
		return -1;
	}
    //chips_dbg("state before active_idle:reg_0x46 = 0x%x,reg_0x50 = 0x%x\n",mode,status);
  
	retval = write_SFR(0x46,0x70);            //active_idle
	if(retval < 0){
		chips_dbg("Failed to write 0x70 to reg_0x46\n");
		return -1;
	}

	for(i = 0; i < 20; i++)
	{
		retval = read_SFR(0x50, &status);
		if(retval < 0){
			chips_dbg("Failed to read reg_0x50+2\n");
			continue;;
		}		
		
		if(status == 0x01){        //sleep、normal模式下active_idle成功
			break;
		}else if(status == 0){
			return 0;              //deep_sleep模式下active_idle成功，退出
		}
	}

	if(i >= 20){
		chips_dbg("Failed to active idle,reg_0x50 = 0x%x\n",status);
		return -1;
	}
		
	retval = write_SFR(0x50,0x01);   
	if(retval < 0){
		chips_dbg("Failed to write 0x01 to reg_0x50\n");
		return -1;
	}
		
	for(i = 0; i < 5; i++)                //清除idle_irq
	{	
		retval = read_SFR(0x50, &status);
		if(retval < 0){
			chips_dbg("Failed to read reg_0x50+3\n");
			continue;;
		}
		
		if(status == 0x00)                 //清除idle_irq成功
			break;

	}	

	if(i >= 5){
		chips_dbg("Failed to clear active idle irq,reg_0x50 = 0x%x\n",status);
		return -1;
	}
	
	return 0;
}


 /**
 *  @brief chips_set_sensor_mode 设置IC工作模式
 *  
 *  @param [in] mode 要设置的IC工作模式
 *  
 *  @return 成功返回0，失败返回负数
 */
int chips_set_sensor_mode(int mode)
{
	int status = -1;

	status = chips_force_to_idle();
	if(status < 0){
		chips_dbg("Failed to set idle mode\n");
		return status;
	}
	
	switch(mode)
	{
		case IDLE:        //idle
			break;   
    
		case NORMAL:      //normal
			status = write_SFR(0x46, 0x71);
			if(status < 0){
				chips_dbg("Failed to write 0x71 to reg_0x46\n");
			}
			break;
 
		case SLEEP:       //sleep
			status = write_SFR(0x46, 0x72);
			if(status < 0){
				chips_dbg("Failed to write 0x72 to reg_0x46\n");
			}
			break;

		case DEEP_SLEEP:  //deep sleep
			status = write_SFR(0x46,0x76);
			if(status < 0){
				chips_dbg("Failed to write 0x76 to reg_0x46\n");
			}
			break;
	
		default:
			chips_dbg("unrecognized mode,mode = 0x%x\n",mode);
			status = -1;
			break;	 
	}	
	return status;	
}



