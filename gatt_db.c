/**
 ******************************************************************************
 * @file    App/gatt_db.c
 * @author  SRA Application Team
 * @brief   Functions to build GATT DB and handle GATT events
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bluenrg_def.h"
#include "gatt_db.h"
#include "bluenrg_conf.h"
#include "bluenrg_gatt_aci.h"
#include "hci_tl.h"

/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
		((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
		((buf)[1] =  (uint8_t) (val>>8)  ) , \
		((buf)[2] =  (uint8_t) (val>>16) ) , \
		((buf)[3] =  (uint8_t) (val>>24) ) )

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
		do {\
			uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
			uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
			uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
			uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
		}while(0)

/* Hardware Characteristics Service */
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Software Characteristics Service */
#define COPY_SW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x02,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

uint16_t HWServW2STHandle, EnvironmentalCharHandle, AccGyroMagCharHandle;
uint16_t SWServW2STHandle, QuaternionsCharHandle;

#define COPY_SAMPLING_FREQ_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
uint16_t SamplingFreqCharHandle;

#define COPY_RPI_COMMAND_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct, 0x00,0x00,0x00,0x03,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
uint16_t RPiCommandCharHandle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;

extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;

extern uint16_t connection_handle;
//extern uint32_t start_time;
//extern uint16_t new_sampling_freq;
volatile uint32_t sampling_period;
extern uint32_t sampledTime;
volatile uint32_t freq_write_flag = 0;
volatile uint16_t freq_written_value = 0;

/**
 * @brief  Add the 'HW' service (and the Environmental and AccGyr characteristics).
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
	tBleStatus ret;
	uint8_t uuid[16];

	/* Add_HWServW2ST_Service */
	//cjdbg, 0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b  PRIMARY_SERVICE
	//       0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b  CHAR1(Temperature+Pressure),Notify+Read
	//                                                                                        Notify: Server(STM32) actively notify data to client
	//                                                                                        Read:   Client request to read data from Server
	//       0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b  CHAR2(AccGyroMag),Notify
	//       0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

	COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
	BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
			1+3*5, &HWServW2STHandle);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
	BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
			2+1*3*2,
			CHAR_PROP_NOTIFY,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
			16, 0, &AccGyroMagCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

//	COPY_SAMPLING_FREQ_CHAR_UUID(uuid);
//	BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
	// 2-byte write data request from client, need to call "aci_gatt_write_response()" for acknowledge
//	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//			2,
//			CHAR_PROP_WRITE_WITHOUT_RESP,
//			ATTR_PERMISSION_NONE,
//			GATT_NOTIFY_ATTRIBUTE_WRITE,
//			16, 0, &SamplingFreqCharHandle);
//
//	if (ret != BLE_STATUS_SUCCESS){
//		PRINTF("❌ Failed to add Sampling Freq char: 0x%02X\r\n", ret);
//		return BLE_STATUS_ERROR;
//	}

	// Add RPi Command Char (for frequency updates via notifications)
	COPY_RPI_COMMAND_CHAR_UUID(uuid);
	BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128,
	                        char_uuid.Char_UUID_128,
	                        3,
	                        CHAR_PROP_WRITE_WITHOUT_RESP,
	                        ATTR_PERMISSION_NONE,
	                        GATT_NOTIFY_ATTRIBUTE_WRITE,
	                        16, 0, &RPiCommandCharHandle);

	if (ret != BLE_STATUS_SUCCESS) {
	    PRINTF("❌ Failed to add RPiCommandChar: 0x%02X\r\n", ret);
	} else {
	    PRINTF("✅ Added RPiCommandChar, handle = 0x%04X\r\n", RPiCommandCharHandle);
	}


	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the SW Feature service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_SWServW2ST_Service(void)
{
	tBleStatus ret;
	int32_t NumberOfRecords=1;
	uint8_t uuid[16];

	COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
	BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
			1+3*NumberOfRecords, &SWServW2STHandle);

	if (ret != BLE_STATUS_SUCCESS) {
		goto fail;
	}

	COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
	BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
			2+6*SEND_N_QUATERNIONS,
			CHAR_PROP_NOTIFY,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
			16, 0, &QuaternionsCharHandle);

	if (ret != BLE_STATUS_SUCCESS) {
		goto fail;
	}

	return BLE_STATUS_SUCCESS;

	fail:
	return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration characteristic value
 * @param  AxesRaw_t structure containing acceleration value in mg.
 * @retval tBleStatus Status
 */
tBleStatus Acc_Update(int16_t data[])
{
	uint8_t buff[2+2*1*3];
	tBleStatus ret;

	//HOST_TO_LE_16(buff,(HAL_GetTick()>>3));
	HOST_TO_LE_16(buff,(sampledTime));  // time unit: 0.1sec
	HOST_TO_LE_16(buff+2, data[0]);
	HOST_TO_LE_16(buff+4, data[1]);
	HOST_TO_LE_16(buff+6, data[2]);

	printf("AU: %d %04X %04X %04X\n", *(uint16_t*)buff, data[0], data[1], data[2]);
	ret = aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle,
			0, 2+2*1*3, buff);

	if (ret != BLE_STATUS_SUCCESS){
		PRINTF("Error while updating Acceleration characteristic: 0x%02X\n",ret) ;
		return BLE_STATUS_ERROR ;
	}

	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update quaternions characteristic value
 * @param  SensorAxes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(AxesRaw_t *data)
{
	tBleStatus ret;
	uint8_t buff[2+6*SEND_N_QUATERNIONS];

	HOST_TO_LE_16(buff,(HAL_GetTick()>>3));

#if SEND_N_QUATERNIONS == 1
	HOST_TO_LE_16(buff+2,data[0].AXIS_X);
	HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
	HOST_TO_LE_16(buff+6,data[0].AXIS_Z);
#elif SEND_N_QUATERNIONS == 2
	HOST_TO_LE_16(buff+2,data[0].AXIS_X);
	HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
	HOST_TO_LE_16(buff+6,data[0].AXIS_Z);

	HOST_TO_LE_16(buff+8 ,data[1].AXIS_X);
	HOST_TO_LE_16(buff+10,data[1].AXIS_Y);
	HOST_TO_LE_16(buff+12,data[1].AXIS_Z);
#elif SEND_N_QUATERNIONS == 3
	HOST_TO_LE_16(buff+2,data[0].AXIS_X);
	HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
	HOST_TO_LE_16(buff+6,data[0].AXIS_Z);

	HOST_TO_LE_16(buff+8 ,data[1].AXIS_X);
	HOST_TO_LE_16(buff+10,data[1].AXIS_Y);
	HOST_TO_LE_16(buff+12,data[1].AXIS_Z);

	HOST_TO_LE_16(buff+14,data[2].AXIS_X);
	HOST_TO_LE_16(buff+16,data[2].AXIS_Y);
	HOST_TO_LE_16(buff+18,data[2].AXIS_Z);
#else
#error SEND_N_QUATERNIONS could be only 1,2,3
#endif

	ret = aci_gatt_update_char_value(SWServW2STHandle, QuaternionsCharHandle,
			0, 2+6*SEND_N_QUATERNIONS, buff);
	if (ret != BLE_STATUS_SUCCESS){
		PRINTF("Error while updating Sensor Fusion characteristic: 0x%02X\n",ret) ;
		return BLE_STATUS_ERROR ;
	}

	return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
 * Function Name  : Read_Request_CB.
 * Description    : Update the sensor values.
 * Input          : Handle of the characteristic to update.
 * Return         : None.
 *******************************************************************************/
void Read_Request_CB(uint16_t handle)
{
	tBleStatus ret;

	if(handle == AccGyroMagCharHandle + 1)
	{
		extern int16_t pDataXYZ[3];
		Acc_Update(pDataXYZ);
	}
	else if (handle == EnvironmentalCharHandle + 1)
	{
		float data_t, data_p;
		data_t = 27.0 + ((uint64_t)rand()*5)/RAND_MAX; //T sensor emulation
		data_p = 1000.0 + ((uint64_t)rand()*100)/RAND_MAX; //P sensor emulation
		BlueMS_Environmental_Update((int32_t)(data_p *100), (int16_t)(data_t * 10));
	}

	if(connection_handle !=0)
	{
		ret = aci_gatt_allow_read(connection_handle);
		if (ret != BLE_STATUS_SUCCESS)
		{
			PRINTF("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
		}
	}
}

//void Write_Request_CB(uint16_t handle, uint8_t* data, uint8_t length) {
//	if (length < 2) return;
//
//	if (freq_write_flag ==0) {
//		freq_written_value = ((uint16_t)data[1] << 8) | data[0];
//		if (freq_written_value == 0) freq_written_value = 1;
//
//		freq_write_flag = 1;  // Set flag to be processed elsewhere
//		printf("write req set flag %lu \r\n",freq_write_flag);
//	}
//}






tBleStatus BlueMS_Environmental_Update(int32_t press, int16_t temp)
{
	tBleStatus ret;
	uint8_t buff[8];
	HOST_TO_LE_16(buff, HAL_GetTick()>>3);

	HOST_TO_LE_32(buff+2,press);
	HOST_TO_LE_16(buff+6,temp);

	ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle,
			0, 8, buff);

	if (ret != BLE_STATUS_SUCCESS){
		PRINTF("Error while updating TEMP characteristic: 0x%04X\n",ret) ;
		return BLE_STATUS_ERROR ;
	}

	return BLE_STATUS_SUCCESS;
}
