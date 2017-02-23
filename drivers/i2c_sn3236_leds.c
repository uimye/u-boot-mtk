#include <common.h>
#include <command.h>
#include <rt_mmap.h>
#include <malloc.h>
#include <configs/rt2880.h>
#include <i2c.h>
#define OPEN    0x01 
#define CLOSE   0x00
#define SN3236_REG_WORK                         0x00 //关断寄存器
#define SN3236_REG_PWM_OUT1     0x01 //输出PWM寄存器
#define SN3236_REG_PWM_OUT2     0x02
#define SN3236_REG_PWM_OUT3     0x03
#define SN3236_REG_PWM_OUT4     0x04
#define SN3236_REG_PWM_OUT5     0x05                  
#define SN3236_REG_PWM_OUT6     0x06
#define SN3236_REG_PWM_OUT7     0x07
#define SN3236_REG_PWM_OUT8     0x08
#define SN3236_REG_PWM_OUT9     0x09
#define SN3236_REG_PWM_OUT10    0x0A
#define SN3236_REG_PWM_OUT11    0x0B
#define SN3236_REG_PWM_OUT12    0x0C
#define SN3236_REG_PWM_OUT13    0x0D
#define SN3236_REG_PWM_OUT14    0x0E
#define SN3236_REG_PWM_OUT15    0x0F
#define SN3236_REG_PWM_OUT16    0x10
#define SN3236_REG_PWM_OUT17    0x11
#define SN3236_REG_PWM_OUT18    0x12
#define SN3236_REG_PWM_OUT19    0x13
#define SN3236_REG_PWM_OUT20    0x14
#define SN3236_REG_PWM_OUT21    0x15
#define SN3236_REG_PWM_OUT22    0x16
#define SN3236_REG_PWM_OUT23    0x17
#define SN3236_REG_PWM_OUT24    0x18
#define SN3236_REG_PWM_OUT25    0x19
#define SN3236_REG_PWM_OUT26    0x1A
#define SN3236_REG_PWM_OUT27    0x1B
#define SN3236_REG_PWM_OUT28    0x1C
#define SN3236_REG_PWM_OUT29    0x1D
#define SN3236_REG_PWM_OUT30    0x1E
#define SN3236_REG_PWM_OUT31    0x1F
#define SN3236_REG_PWM_OUT32    0x20
#define SN3236_REG_PWM_OUT33    0x21
#define SN3236_REG_PWM_OUT34    0x22
#define SN3236_REG_PWM_OUT35    0x23
#define SN3236_REG_PWM_OUT36    0x24
#define SN3236_REG_DATA_UPDATE  0x25 //PWM和LED控制更>新寄存器 
#define SN3236_REG_CTRL_OUT1            0x26 //LED控制寄存器
#define SN3236_REG_CTRL_OUT2            0x27
#define SN3236_REG_CTRL_OUT3            0x28
#define SN3236_REG_CTRL_OUT4            0x29
#define SN3236_REG_CTRL_OUT5            0x2A
#define SN3236_REG_CTRL_OUT6            0x2B
#define SN3236_REG_CTRL_OUT7            0x2C
#define SN3236_REG_CTRL_OUT8            0x2D
#define SN3236_REG_CTRL_OUT9            0x2E
#define SN3236_REG_CTRL_OUT10   0x2F
#define SN3236_REG_CTRL_OUT11   0x30
#define SN3236_REG_CTRL_OUT12   0x31
#define SN3236_REG_CTRL_OUT13   0x32
#define SN3236_REG_CTRL_OUT14   0x33
#define SN3236_REG_CTRL_OUT15   0x34
#define SN3236_REG_CTRL_OUT16   0x35
#define SN3236_REG_CTRL_OUT17   0x36
#define SN3236_REG_CTRL_OUT18   0x37
#define SN3236_REG_CTRL_OUT19   0x38
#define SN3236_REG_CTRL_OUT20   0x39
#define SN3236_REG_CTRL_OUT21   0x3A
#define SN3236_REG_CTRL_OUT22   0x3B
#define SN3236_REG_CTRL_OUT23   0x3C
#define SN3236_REG_CTRL_OUT24   0x3D
#define SN3236_REG_CTRL_OUT25   0x3E
#define SN3236_REG_CTRL_OUT26   0x3F
#define SN3236_REG_CTRL_OUT27   0x40
#define SN3236_REG_CTRL_OUT28   0x41
#define SN3236_REG_CTRL_OUT29   0x42
#define SN3236_REG_CTRL_OUT30   0x43
#define SN3236_REG_CTRL_OUT31   0x44
#define SN3236_REG_CTRL_OUT32   0x45
#define SN3236_REG_CTRL_OUT33   0x46
#define SN3236_REG_CTRL_OUT34   0x47
#define SN3236_REG_CTRL_OUT35   0x48
#define SN3236_REG_CTRL_OUT36   0x49 
#define SN3236_REG_LED_SYNC     0x4A //输出同步寄存器
#define SN3236_REG_RST  0x4F                    //复位寄存器
enum {
	RGB_LED_INIT = 0,
	RGB_LED_CTROL,
	RGB_LED_PWM,
	RGB_LED_STOP_DISP,
};
extern void random_write_one_byte(u32 address, u8 *data);
extern void random_write_nbytes(u32 address,u8 *buf,u8 num);

unsigned char  PWM_RED[36]=
{
    (0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,
    (0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,
    (0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00         ,0x00
};

unsigned char  PWM_GREEN[36]=
{
    0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,
    0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00,
    0x00,(0xFF)>>2,0x00,0x00,(0xFF)>>2,0x00          
};

unsigned char  PWM_YELLOW[36]=
{
    (0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,
    (0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00,
    (0xFF)>>2,(0xFF)>>2,0x00,(0xFF)>>2,(0xFF)>>2,0x00
};

void SN3236_OUT_SW(void);
void SN3236_OUT_PWM(unsigned char *pwm);
void SN_IIC_Write_REG(u8  address,u8 data){
	random_write_one_byte((u32)address,&data);
}
void SN_IIC_Write_REG_Series(u8 address,u8 data,u8 num){
	unsigned char buf[64]={0};
	int i=0;
	for(i=0;i<num;i++){
		buf[i]=data;
	}
	random_write_nbytes((u32)address,buf,num);

	
}
void SN3236_DataUpdate(void){
	SN_IIC_Write_REG(SN3236_REG_DATA_UPDATE,0xff);
}
void SN3236_Soft_SW(unsigned char workmode) {
	SN_IIC_Write_REG(SN3236_REG_WORK,workmode);
}

static void SN3236_Init(unsigned char pwmdata) {
	printf("SN3236_Init\n");
	SN3236_Soft_SW(OPEN);
	SN_IIC_Write_REG_Series(SN3236_REG_PWM_OUT1,pwmdata,36);
	SN_IIC_Write_REG_Series(SN3236_REG_CTRL_OUT1,0x01,36);
	SN3236_DataUpdate();//更新PWM
}
void sn3236_init(void){
	SN3236_Init(0x00);
}

void SN3236_OUT_SW(void) {
	unsigned char out1_8=0xff;
	unsigned char out9_16=0xff;
	unsigned char out17_24=0xff;
	unsigned char out25_32=0xff;
	unsigned char out33_36=0xff;

	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT1,out1_8&0x01);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT2,(out1_8&0x02)>>1);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT3,(out1_8&0x04)>>2);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT4,(out1_8&0x08)>>3);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT5,(out1_8&0x10)>>4);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT6,(out1_8&0x20)>>5);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT7,(out1_8&0x40)>>6);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT8,(out1_8&0x80)>>7);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT9,out9_16&0x01);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT10,(out9_16&0x02)>>1);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT11,(out9_16&0x04)>>2);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT12,(out9_16&0x08)>>3);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT13,(out9_16&0x10)>>4);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT14,(out9_16&0x20)>>5);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT15,(out9_16&0x40)>>6);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT16,(out9_16&0x80)>>7);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT17,out17_24&0x01);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT18,(out17_24&0x02)>>1);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT19,(out17_24&0x04)>>2);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT20,(out17_24&0x08)>>3);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT21,(out17_24&0x10)>>4);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT22,(out17_24&0x20)>>5);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT23,(out17_24&0x40)>>6);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT24,(out17_24&0x80)>>7);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT25,out25_32&0x01);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT26,(out25_32&0x02)>>1);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT27,(out25_32&0x04)>>2);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT28,(out25_32&0x08)>>3);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT29,(out25_32&0x10)>>4);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT30,(out25_32&0x20)>>5);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT31,(out25_32&0x40)>>6);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT32,(out25_32&0x80)>>7);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT33,out33_36&0x01);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT34,(out33_36&0x02)>>1);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT35,(out33_36&0x04)>>2);
	SN_IIC_Write_REG(SN3236_REG_CTRL_OUT36,(out33_36&0x08)>>3);
	SN3236_DataUpdate();
}
void SN3236_OUT_PWM(unsigned char *pwm) {
	
	printf("SN3236_OUT_PWM\n");
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT1,pwm[0]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT2,pwm[1]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT3,pwm[2]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT4,pwm[3]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT5,pwm[4]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT6,pwm[5]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT7,pwm[6]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT8,pwm[7]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT9,pwm[8]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT10,pwm[9]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT11,pwm[10]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT12,pwm[11]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT13,pwm[12]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT14,pwm[13]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT15,pwm[14]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT16,pwm[15]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT17,pwm[16]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT18,pwm[17]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT19,pwm[18]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT20,pwm[19]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT21,pwm[20]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT22,pwm[21]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT23,pwm[22]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT24,pwm[23]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT25,pwm[24]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT26,pwm[25]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT27,pwm[26]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT28,pwm[27]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT29,pwm[28]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT30,pwm[29]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT31,pwm[30]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT32,pwm[31]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT33,pwm[32]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT34,pwm[33]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT35,pwm[34]);
	SN_IIC_Write_REG(SN3236_REG_PWM_OUT36,pwm[35]);
	SN3236_DataUpdate();
}

