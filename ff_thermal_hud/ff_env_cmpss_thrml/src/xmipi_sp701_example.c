//#define DEBUG 1
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xiic.h"
#include "xil_exception.h"
#include "function_prototype.h"
#include "pcam_5C_cfgs.h"
#include "xstatus.h"
#include "sleep.h"
#include "xiic_l.h"
#include "xil_io.h"
#include "xil_types.h"
#include "xv_tpg.h"
#include "xil_cache.h"
#include "stdio.h"
#include "xv_mix_l2.h"
#include "xvidc.h"
#include "xff_monitor_gen.h"
#include "PmodHYGRO.h"
#include "PmodAQS.h"
#include "math.h"
#include "PmodCMPS2.h"
#include "mlx90640_api.h"

#define DDR_BASEADDR XPAR_MIG7SERIES_0_BASEADDR
#define DISPLAY_WIDTH 1920
#define DISPLAY_HEIGHT 1080

/* Memory Layers for Mixer */
#define XVMIX_LAYER2_BASEADDR      (0x84000000U)
#define XVMIX_LAYER_ADDR_OFFSET    (0x01000000U)

#define THERMAL_WIDTH 32
#define THERMAL_HEIGHT 24

// compass labels
#define YCMPSLABEL 730
#define CMPSLABEL_MARGINY 10

// compass scale
#define YCMPS1 770
#define CMPS_MARGINY 8

u8 thermalImgBuff1[768*4];
u8 thermalImgBuff2[768*4];
u8* thermalImageBuffers[]= {thermalImgBuff1, thermalImgBuff2};
int actuaThermalImgBuff = 1;



long map(long x, long in_min, long in_max, long out_min, long out_max);
void MLX90640_GetSensorRaw(uint16_t *frameData, int *result, int *minT, int *maxT);
int initMlx90640(void);
int loopMlx90640Gray(u8* mlx90640Image);
void initHeadsUpDisplay(XFf_monitor_gen* xv_ff_monitor);


uint16_t mlx90640Frame[834];
u8 mlx90640Image[768*4];

// Calibration data struct, track minimum, maximum, and average sample seen for
// each x/y/z channel
typedef struct {
	CMPS2_DataPacket max, min, mid;
} CMPS2_CalibrationData;

PmodCMPS2 cmps2;
CMPS2_CalibrationData myCalibrationData;

void CMPS2Initialize();
void CMPS2ClearCalibration(CMPS2_CalibrationData *calib);
void CMPS2Calibrate(PmodCMPS2 *InstancePtr, CMPS2_CalibrationData *calib,
		CMPS2_DataPacket data);
float CMPS2ConvertDegree(PmodCMPS2 *InstancePtr, CMPS2_CalibrationData calib,
		CMPS2_DataPacket data, int declination);

const int myDeclination = 1; // Magnetic declination for Zaragoza, Spain
const u8 chip_address = 0x30; // Pmod CMPS2 I2C chip address

/*************************** Global Variables ******************************/

#define U32TOBCD(u) ((((u/10000000)%10)<<28)| (((u/1000000)%10)<<24)|\
		(((u/100000)%10)<<20)  | (((u/10000)%10)<<16)|\
		(((u/1000)%10)<<12)    | (((u/100)%10)<<8)|\
		(((u/10)%10)<<4)|(u%10))

#define U16TOBCD(u) ((((u/1000)%10)<<12) | (((u/100)%10)<<8)|\
		(((u/10)%10)<<4)|(u%10))

#define U8TOBCD(u) ((((u/10)%10)<<4)|(u%10))

#ifdef __MICROBLAZE__
#define TIMER_FREQ_HZ XPAR_CPU_M_AXI_DP_FREQ_HZ
#else
#define TIMER_FREQ_HZ 100000000
#endif


/************************** Constant Definitions *****************************/


#define PAGE_SIZE   16


#define IIC_BASE_ADDRESS	XPAR_IIC_2_BASEADDR

#define EEPROM_TEST_START_ADDRESS	0x80

#define IIC_SWITCH_ADDRESS 0x74
#define IIC_ADV7511_ADDRESS 0x39

typedef u8 AddressType;

typedef struct {
	u8 addr;
	u8 data;
	u8 init;
} HDMI_REG;

#define NUMBER_OF_HDMI_REGS  16
HDMI_REG hdmi_iic[NUMBER_OF_HDMI_REGS] = {
		{0x41, 0x00, 0x10},
		{0x98, 0x00, 0x03},
		{0x9A, 0x00, 0xE0},
		{0x9C, 0x00, 0x30},
		{0x9D, 0x00, 0x61},
		{0xA2, 0x00, 0xA4},
		{0xA3, 0x00, 0xA4},
		{0xE0, 0x00, 0xD0},
		{0xF9, 0x00, 0x00},
		{0x18, 0x00, 0xE7},
		{0x55, 0x00, 0x00},
		{0x56, 0x00, 0x28},
		{0xD6, 0x00, 0xC0},
		{0xAF, 0x00, 0x4},
		{0xF9, 0x00, 0x00}
};

u8 EepromIicAddr;		/* Variable for storing Eeprom IIC address */

int IicLowLevelDynEeprom();

u8 EepromReadByte(AddressType Address, u8 *BufferPtr, u8 ByteCount);
u8 EepromWriteByte(AddressType Address, u8 *BufferPtr, u8 ByteCount);
int initVideoSystem(void);
void initVideoMixer(XV_mix* xv_mix);
int startVideoSystem(void);
void initCompassMagnetometer(void);
void initPmodHygro(void);
void initPmodAQS(void);
void startVideoMixer(void);


/****************i************ Type Definitions *******************************/
typedef u8 AddressType;

/************************** Variable Definitions *****************************/
extern XIic IicFmc, IicAdapter, IicMlx ;	/*  IIC device. */
PmodHYGRO hygro;
PmodAQS aqs;
XV_mix xv_mix;
XFf_monitor_gen xv_ff_monitor;



/*****************************************************************************/
/**
 *
 * Main function to initialize interop system and read data from AR0330 sensor

 * @param  None.
 *
 * @return
 *   - XST_SUCCESS if MIPI Interop was successful.
 *   - XST_FAILURE if MIPI Interop failed.
 *
 * @note   None.
 *
 ******************************************************************************/
int main() {
	int Status;
	float temp_degc, hum_perrh;
	CMPS2_DataPacket data;
	int hum = 0;
	int temp = 0;
	u8 buf[5];
	int eCO2 = 0;
	int counter = 0;
	float magXYd = 0;
	float n = 4.0; // average last 4 measurements

	// Video pipeline configuration
    Status = initVideoSystem();
	initVideoMixer(&xv_mix);
	initHeadsUpDisplay(&xv_ff_monitor);
	resetIp();
	EnableCSI();
	Status = startVideoSystem();
	// xil_printf("\n\rPipeline Configuration Completed \n\r");
    initCompassMagnetometer();
    initPmodHygro();
    initPmodAQS();
    // init display data to show e14 e14
	XFf_monitor_gen_Set_angle(&xv_ff_monitor,0) ;
	XFf_monitor_gen_Set_envData(&xv_ff_monitor,  0xFB14FB14);

    startVideoMixer();
    // Get initial Magnetometer data
	data = CMPS2_GetData(&cmps2);
	// start average with actual orientation
	float avgx = data.x;
	float avgy = data.y;
    // Start MLX90640 IIC communication
	Status = initMlx90640();
	if (Status != XST_SUCCESS) {
		xil_printf("MLX90640 IIC programming FAILED\r\n");
		return XST_FAILURE;
	}
	xil_printf("MLX90640 IIC programming PASSED\r\n");

	while(1) {
        // get grayscale image from thermal array sensor
		//loopMlx90640Gray((u8*) XVMIX_LAYER2_BASEADDR);
		// prepare next buffer
		actuaThermalImgBuff = (actuaThermalImgBuff +1) & 0x01;
		// update buffer
		loopMlx90640Gray(thermalImageBuffers[actuaThermalImgBuff]);
		// change buffer
		XV_mix_Set_HwReg_layer2_buf1_V(&xv_mix,(u32)thermalImageBuffers[actuaThermalImgBuff]);

		// read magnetometer data
		data = CMPS2_GetData(&cmps2);

		// moving average for x magnetic component
		avgx = avgx - avgx/n;
		avgx = avgx + data.x/n;

		avgy = avgy - avgy/n;
		avgy = avgy + data.y/n;

		data.x = (u16)avgx;
		data.y = (u16)avgy;

		// recalibrate compass
		CMPS2Calibrate(&cmps2, &myCalibrationData, data);
		// get compass degrees
		magXYd = CMPS2ConvertDegree(&cmps2, myCalibrationData, data,
				myDeclination);
		// display compass with new data
		XFf_monitor_gen_Set_angle(&xv_ff_monitor, (int)(magXYd * 4)) ;

		if(counter% 100 == 0){
			// read temperature & humidity values
			temp_degc = HYGRO_getTemperature(&hygro);
			temp = (int) temp_degc;
			hum_perrh = HYGRO_getHumidity(&hygro);
			hum = (int) hum_perrh;
			AQS_GetData(&aqs, buf);
			eCO2 = (((int)buf[0]) << 8) | ((int)buf[1]);
			// display new temp and humidity values
			XFf_monitor_gen_Set_envData(&xv_ff_monitor, (U16TOBCD(eCO2)<<16) | (U8TOBCD(temp)<<8) | U8TOBCD(hum));
		}
		++counter;
	}

	return XST_SUCCESS;
}

void initHeadsUpDisplay(XFf_monitor_gen* xv_ff_monitor) {
	// Ff monitor
	XFf_monitor_gen_Config* XV_Ff_monitor_cfg;
	XV_Ff_monitor_cfg = XFf_monitor_gen_LookupConfig(XPAR_FF_MONITOR_GEN_1_DEVICE_ID);
	XFf_monitor_gen_CfgInitialize(&*xv_ff_monitor, XV_Ff_monitor_cfg);
	XFf_monitor_gen_Set_row(&*xv_ff_monitor, (u32) DISPLAY_HEIGHT);
	XFf_monitor_gen_Set_column(&*xv_ff_monitor, (u32) DISPLAY_WIDTH);
	XFf_monitor_gen_EnableAutoRestart(&*xv_ff_monitor);
	XFf_monitor_gen_Start(&*xv_ff_monitor);
	XFf_monitor_gen_Set_envData(&*xv_ff_monitor, 0x12345678);
	XFf_monitor_gen_Set_angle(&*xv_ff_monitor, 10);
}

void startVideoMixer(void) {
	XV_mix_Set_HwReg_layerEnable(&xv_mix,(u32)0b111);

	/* Disable Interrupts */
	XV_mix_InterruptDisable(&xv_mix, XVMIX_IRQ_DONE_MASK);
	XV_mix_InterruptGlobalDisable(&xv_mix);

	/* Set autostart bit */
	XV_mix_EnableAutoRestart(&xv_mix);
	XV_mix_Start(&xv_mix);
}

void initPmodAQS(void) {
	xil_printf("\n\rAQS Init Started");
	AQS_begin(&aqs, XPAR_PMODAQS_0_AXI_LITE_IIC_BASEADDR, 0x5B); // Chip address of CS811 0x5A module IIC 0x5B PmocAQS
	xil_printf("\n\rAQS Init Done");
}

void initPmodHygro(void) {
	xil_printf("\n\rHYGRO Init Started");
	HYGRO_begin(
			&hygro,
			XPAR_PMODHYGRO_0_AXI_LITE_IIC_BASEADDR,
			0x40, // Chip address of PmodHYGRO IIC
			XPAR_PMODHYGRO_0_AXI_LITE_TMR_BASEADDR,
			XPAR_PMODHYGRO_0_DEVICE_ID,
			TIMER_FREQ_HZ // Clock frequency of AXI bus, used to convert timer data
	);
	xil_printf("\n\rHYGRO Init Done");
}

void initCompassMagnetometer(void) {
	xil_printf("\n\rCMPS2 Init Started");
	CMPS2Initialize();


	CMPS2ClearCalibration(&myCalibrationData);
	xil_printf("\n\rCMPS2 Init done");
}

// Init Video System
int initVideoSystem(void) {
	int Status;
	Status = IicLowLevelDynEeprom();
	if (Status != XST_SUCCESS) {
		xil_printf("ADV7511 IIC programming FAILED\r\n");
		return XST_FAILURE;
	}
	xil_printf("ADV7511 IIC programming PASSED\r\n");


	//Initialize FMC, Adapter and Sensor IIC
	Status = InitIIC();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\r IIC initialization Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("IIC Initializtion Done \n\r");

	//Initialize FMC Interrupt System
	Status = SetupFmcInterruptSystem(&IicFmc);
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rInterrupt System Initialization Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("FMC Interrupt System Initialization Done \n\r");

	//Set up IIC Interrupt Handlers
	SetupIICIntrHandlers();
	xil_printf("IIC Interrupt Handlers Setup Done \n\r");

	Status =  SetFmcIICAddress();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rFMC IIC Address Setup Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("Fmc IIC Address Set\n\r");

	//Initialize Adapter Interrupt System
	Status = SetupAdapterInterruptSystem(&IicAdapter);
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rInterrupt System Initialization Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("Adapter Interrupt System Initialization Done \n\r");

	//Set Address of Adapter IIC
	Status =  SetAdapterIICAddress();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rAdapter IIC Address Setup Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("Adapter IIC Address Set\n\r");

	Status = InitializeCsiRxSs();
	if (Status != XST_SUCCESS) {
		xil_printf("CSI Rx Ss Init failed status = %x.\r\n", Status);
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

int startVideoSystem(void) {
	int Status;
	int pcam5c_mode = 1;

	Status = demosaic();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rDemosaic Failed \n\r");
		return XST_FAILURE;
	}

	CamReset();

	//Configure Sensor
	Status = SensorPreConfig(pcam5c_mode);
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rSensor PreConfiguration Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("\n\rSensor is PreConfigured\n\r");

	Status = vdma_hdmi();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rVdma_hdmi Failed \n\r");
		return XST_FAILURE;
	}

	Status = vtpg_hdmi();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rVtpg Failed \n\r");
		return XST_FAILURE;
	}

	WritetoReg(0x30, 0x08, 0x02);
	Sensor_Delay();

	return XST_SUCCESS;
}


void initVideoMixer(XV_mix* xv_mix) {
	XV_mix_Config* xv_config;
	xv_config = XV_mix_LookupConfig(XPAR_XV_MIX_0_DEVICE_ID);
	XV_mix_CfgInitialize(&*xv_mix, xv_config, xv_config->BaseAddress);
	XV_mix_Set_HwReg_width(&*xv_mix, (u32) DISPLAY_WIDTH);
	XV_mix_Set_HwReg_height(&*xv_mix, (u32) DISPLAY_HEIGHT);
	XV_mix_Set_HwReg_layerEnable(&*xv_mix, (u32) 0b000);

	XV_mix_Set_HwReg_layerStartX_0(&*xv_mix, (u32) 0);
	XV_mix_Set_HwReg_layerStartY_0(&*xv_mix, 0);
	XV_mix_Set_HwReg_layerWidth_0(&*xv_mix, (u32) DISPLAY_WIDTH);
	XV_mix_Set_HwReg_layerHeight_0(&*xv_mix, (u32) DISPLAY_HEIGHT);
	XV_mix_Set_HwReg_layerAlpha_0(&*xv_mix, 255);

	XV_mix_Set_HwReg_layerStartX_1(&*xv_mix, (u32) 0);
	XV_mix_Set_HwReg_layerStartY_1(&*xv_mix, 0);
	XV_mix_Set_HwReg_layerWidth_1(&*xv_mix, (u32) DISPLAY_WIDTH);
	XV_mix_Set_HwReg_layerHeight_1(&*xv_mix, (u32) DISPLAY_HEIGHT);
	XV_mix_Set_HwReg_layerAlpha_1(&*xv_mix, 255);

	XV_mix_Set_HwReg_layerStartX_2(&*xv_mix, (u32) DISPLAY_WIDTH / 2 - 16 * 4);
	XV_mix_Set_HwReg_layerStartY_2(&*xv_mix, YCMPS1 + 16);
	XV_mix_Set_HwReg_layerWidth_2(&*xv_mix, (u32) THERMAL_WIDTH);
	XV_mix_Set_HwReg_layerHeight_2(&*xv_mix, (u32) THERMAL_HEIGHT);
	XV_mix_Set_HwReg_layerAlpha_2(&*xv_mix, 255);
	XV_mix_Set_HwReg_layerScaleFactor_2(&*xv_mix, XVMIX_SCALE_FACTOR_4X);
	XV_mix_Set_HwReg_layerStride_2(&*xv_mix, THERMAL_WIDTH * 4);
	// XV_mix_Set_HwReg_layer2_buf1_V(&*xv_mix, (u32) (XVMIX_LAYER2_BASEADDR));
	XV_mix_Set_HwReg_layer2_buf1_V(&*xv_mix, (u32) (thermalImageBuffers[actuaThermalImgBuff]));

}


//HDMI IIC
int IicLowLevelDynEeprom()
{
	u8 BytesRead;
	u32 StatusReg;
	u8 Index;
	int Status;
	u32 i;
	EepromIicAddr = IIC_SWITCH_ADDRESS;
	Status = XIic_DynInit(IIC_BASE_ADDRESS);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	xil_printf("\r\nAfter XIic_DynInit\r\n");
	while (((StatusReg = XIic_ReadReg(IIC_BASE_ADDRESS,
			XIIC_SR_REG_OFFSET)) &
			(XIIC_SR_RX_FIFO_EMPTY_MASK |
					XIIC_SR_TX_FIFO_EMPTY_MASK |
					XIIC_SR_BUS_BUSY_MASK)) !=
							(XIIC_SR_RX_FIFO_EMPTY_MASK |
									XIIC_SR_TX_FIFO_EMPTY_MASK)) {

	}


	EepromIicAddr = IIC_ADV7511_ADDRESS;
	for ( Index = 0; Index < NUMBER_OF_HDMI_REGS; Index++)
	{
		EepromWriteByte(hdmi_iic[Index].addr, &hdmi_iic[Index].init, 1);
	}

	for ( Index = 0; Index < NUMBER_OF_HDMI_REGS; Index++)
	{
		BytesRead = EepromReadByte(hdmi_iic[Index].addr, &hdmi_iic[Index].data, 1);
		for(i=0;i<1000;i++) {};	// IIC delay
		if (BytesRead != 1) {
			return XST_FAILURE;
		}
	}
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
 * This function writes a buffer of bytes to the IIC serial EEPROM.
 *
 * @param	BufferPtr contains the address of the data to write.
 * @param	ByteCount contains the number of bytes in the buffer to be
 *		written. Note that this should not exceed the page size of the
 *		EEPROM as noted by the constant PAGE_SIZE.
 *
 * @return	The number of bytes written, a value less than that which was
 *		specified as an input indicates an error.
 *
 * @note		one.
 *
 ******************************************************************************/
u8 EepromWriteByte(AddressType Address, u8 *BufferPtr, u8 ByteCount)
{
	u8 SentByteCount;
	u8 WriteBuffer[sizeof(Address) + PAGE_SIZE];
	u8 Index;

	/*
	 * A temporary write buffer must be used which contains both the address
	 * and the data to be written, put the address in first based upon the
	 * size of the address for the EEPROM
	 */
	if (sizeof(AddressType) == 2) {
		WriteBuffer[0] = (u8) (Address >> 8);
		WriteBuffer[1] = (u8) (Address);
	} else if (sizeof(AddressType) == 1) {
		WriteBuffer[0] = (u8) (Address);
		EepromIicAddr |= (EEPROM_TEST_START_ADDRESS >> 8) & 0x7;
	}

	/*
	 * Put the data in the write buffer following the address.
	 */
	for (Index = 0; Index < ByteCount; Index++) {
		WriteBuffer[sizeof(Address) + Index] = BufferPtr[Index];
	}

	/*
	 * Write a page of data at the specified address to the EEPROM.
	 */
	SentByteCount = XIic_DynSend(IIC_BASE_ADDRESS, EepromIicAddr,
			WriteBuffer, sizeof(Address) + ByteCount,
			XIIC_STOP);

	/*
	 * Return the number of bytes written to the EEPROM.
	 */
	return SentByteCount - sizeof(Address);

}


/******************************************************************************
 *
 * This function reads a number of bytes from the IIC serial EEPROM into a
 * specified buffer.
 *
 * @param	BufferPtr contains the address of the data buffer to be filled.
 * @param	ByteCount contains the number of bytes in the buffer to be read.
 *		This value is constrained by the page size of the device such
 *		that up to 64K may be read in one call.
 *
 * @return	The number of bytes read. A value less than the specified input
 *		value indicates an error.
 *
 * @note		None.
 *
 ******************************************************************************/
u8 EepromReadByte(AddressType Address, u8 *BufferPtr, u8 ByteCount)
{
	u8 ReceivedByteCount;
	u8 SentByteCount;
	u16 StatusReg;

	/*
	 * Position the Read pointer to specific location in the EEPROM.
	 */
	do {
		StatusReg = XIic_ReadReg(IIC_BASE_ADDRESS, XIIC_SR_REG_OFFSET);
		if (!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
			SentByteCount = XIic_DynSend(IIC_BASE_ADDRESS, EepromIicAddr,
					(u8 *) &Address, sizeof(Address), XIIC_REPEATED_START);
		}

	} while (SentByteCount != sizeof(Address));
	/*
	 * Receive the data.
	 */
	ReceivedByteCount = XIic_DynRecv(IIC_BASE_ADDRESS, EepromIicAddr,
			BufferPtr, ByteCount);

	/*
	 * Return the number of bytes received from the EEPROM.
	 */

	return ReceivedByteCount;

}



void CMPS2Initialize() {
	CMPS2_begin(&cmps2, XPAR_PMODCMPS2_0_AXI_LITE_IIC_BASEADDR, chip_address);
	usleep(10000);
	CMPS2_SetSensor(&cmps2);
	usleep(10000);
	CMPS2_SetOutputResolution(&cmps2, 0b00);
}

void CMPS2ClearCalibration(CMPS2_CalibrationData *calib) {
	calib->max.x = 0x8000; // Center point of 0x0000 -> 0xFFFF
	calib->max.y = 0x8000;
	calib->max.z = 0x8000;
	calib->min.x = 0x8000;
	calib->min.y = 0x8000;
	calib->min.z = 0x8000;
	calib->mid.x = 0x8000;
	calib->mid.y = 0x8000;
	calib->mid.z = 0x8000;
}

void CMPS2Calibrate(PmodCMPS2 *InstancePtr, CMPS2_CalibrationData *calib,
		CMPS2_DataPacket data) {
	if (data.x > calib->max.x) calib->max.x = data.x; // Track maximum / minimum
	if (data.y > calib->max.y) calib->max.y = data.y; // value seen per axis
	if (data.z > calib->max.z) calib->max.z = data.z;
	if (data.x < calib->min.x) calib->min.x = data.x;
	if (data.y < calib->min.y) calib->min.y = data.y;
	if (data.z < calib->min.z) calib->min.z = data.z;
	calib->mid.x = (calib->max.x >> 1) + (calib->min.x >> 1); // Find average
	calib->mid.y = (calib->max.y >> 1) + (calib->min.y >> 1);
	calib->mid.z = (calib->max.z >> 1) + (calib->min.z >> 1);
}


float CMPS2ConvertDegree(PmodCMPS2 *InstancePtr, CMPS2_CalibrationData calib,
		CMPS2_DataPacket data, int declination) {
	int tx, ty;
	float deg;

	if (data.x < calib.mid.x)
		tx = (calib.mid.x - data.x);
	else
		tx = (data.x - calib.mid.x);

	if (data.y < calib.mid.y)
		ty = (calib.mid.y - data.y);
	else
		ty = (data.y - calib.mid.y);

	if (data.x < calib.mid.x) {
		if (data.y > calib.mid.y)
			deg = 90.0f - atan2f(ty, tx) * 180.0f / 3.14159f;
		else
			deg = 90.0f + atan2f(ty, tx) * 180.0f / 3.14159f;
	} else {
		if (data.y < calib.mid.y)
			deg = 270.0f - atan2f(ty, tx) * 180.0f / 3.14159f;
		else
			deg = 270.0f + atan2f(ty, tx) * 180.0f / 3.14159f;
	}

	deg += declination;

	while (deg >= 360)
		deg -= 360;
	while (deg < 0)
		deg += 360;

#ifdef DEBUG

	printf("%d, ", (int)deg);

	printf("%d,", data.x);
	printf("%d,", data.y);
	printf("%d", data.z);
	printf("\n\r");
#endif

	return deg;
}

int initMlx90640(void) {
	int Status;


	xil_printf("\n\r******************************************************\n\r");
	Xil_ICacheDisable();
	Xil_DCacheDisable();
	xil_printf("\n\r**                         MLX90640                ***\n\r");


	//Initialize Mlx IIC
	Status = InitIICMlx90640();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\r MLX 90640 IIC initialization Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("MLX 90640 IIC  Initialization Done \n\r");

	//Initialize FMC Interrupt System
	Status = SetupMlxInterruptSystem(&IicMlx);
	if (Status != XST_SUCCESS) {
		xil_printf("MLX 90640 Interrupt System Initialization Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("MLX 90640 Interrupt System Initialization Done \n\r");

	//Set up MLX 90640 IIC Interrupt Handlers
	SetupIICMlxIntrHandlers();
	xil_printf("IIC Interrupt Handlers Setup Done \n\r");

	//Set Address of Adapter IIC
	Status =  SetMlxIICAddress();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rMLX 90640 Address Setup Failed \n\r");
		return XST_FAILURE;
	}
	xil_printf("MLX 90640 IIC Address Set\n\r");

	// Start IIC MLX90640
	Status = start_MLX90640IIC();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rMLX 90640 Start Failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("MLX 90640 IIC Started\n\r");

	MLX90640_SetRefreshRate(0x33, 0x04);
	MLX90640_SetResolution(0x33,0x00);

    return XST_SUCCESS;
}

int loopMlx90640Gray(u8* mlx90640Image) {

	int newMinT = 999999;
	int newMaxT = -999999;
	static int minT = 999999;
	static int maxT =  -999999;

	// Start IIC MLX90640
	int mlx90640Raw[768];


   //	xil_printf("MLX 90640 IIC Started\n\r");

	MLX90640_GetFrameData(0x33, mlx90640Frame);
	MLX90640_GetFrameData(0x33, mlx90640Frame);

    MLX90640_GetSensorRaw(mlx90640Frame, mlx90640Raw, &newMinT, &newMaxT);
   // if(newMinT<minT) {
    	minT = newMinT;
  // }
  //  if(newMaxT>maxT) {
    	maxT = newMaxT;
   // }
	enable_caches();
    int pixelAddr = 0;
    int offsety= 0;
    for(int i = 0; i < THERMAL_HEIGHT; i++) {
    	 for(int j = 0; j < THERMAL_WIDTH; j++) {
        	u8 index = map(mlx90640Raw[(THERMAL_WIDTH - j -1)+offsety], minT, maxT, 0, 255);
     		mlx90640Image[pixelAddr+1] = index;
     		mlx90640Image[pixelAddr+2] = index;
     		mlx90640Image[pixelAddr+3] = index;
     		mlx90640Image[pixelAddr+0] = 0x00;
     		pixelAddr+=4;
         }
    	 offsety += THERMAL_WIDTH;
    }

//	xil_printf("Flush cache %d\r\n");
	Xil_DCacheFlushRange((unsigned int) (&mlx90640Image),  THERMAL_WIDTH * THERMAL_HEIGHT * 4);
	enable_caches();
	return XST_SUCCESS;

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void MLX90640_GetSensorRaw(uint16_t *frameData, int *result, int *minT, int *maxT)
{
    float irData;
    *minT = 999999;
	*maxT = -999999;

    for( int pixelNumber = 0; pixelNumber < THERMAL_WIDTH * THERMAL_HEIGHT; pixelNumber++)
    {
            irData = frameData[pixelNumber];
            result[pixelNumber] = (int16_t)irData;
            if(result[pixelNumber] > *maxT ) {
            	*maxT = result[pixelNumber];
            }
            if(result[pixelNumber] < *minT ) {
            	*minT = result[pixelNumber];
            }

    }
}


