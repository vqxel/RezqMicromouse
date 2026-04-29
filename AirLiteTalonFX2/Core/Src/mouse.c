/*
 * mouse.c
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */
#include "mouse.h"

void Mouse_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer) {
	uint8_t addr = addressByte & 0x7F;

	HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &addr, 1, 100);
	delay_us(160);

	// Send 0x00 while reading the response
	HAL_SPI_Receive(hspi, buffer, 1, 100);

	HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_SET);
	delay_us(20);
}

void Mouse_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value) {
	uint8_t txData[2] = {addressByte | 0x80, value};

    HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, txData, 2, 100);
    delay_us(35);
    HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_SET);
    delay_us(180);
}

/**
 * @brief  Initializes the PMW3389 Sensor and uploads firmware.
 * @return 1 if successful, 0 if SROM verification failed.
 */
bool Mouse_Init(SPI_HandleTypeDef *hspi) {
    uint8_t data = 0;

    // --- PHASE 1: HARDWARE CHECK & RESET ---

    // 1. Perform Soft Reset (Write 0x5A to 0x3A)
    // Even if it "works" without this, this ensures a clean state every boot.
    Mouse_WriteRegister(hspi, 0x3A, 0x5A);
    HAL_Delay(60); // Wait for chip reboot [cite: 521]

    for (uint8_t i = 2; i < 7; i++) {
    	Mouse_ReadRegister(hspi, i, &data);
    }

    // 2. Verify Product ID (0x00) -> Expect 0x47
    Mouse_ReadRegister(hspi, 0x00, &data);
    if (data != 0x47) {
        return false; // Error: SPI Connection Failed
    }

    // --- PHASE 2: SROM UPLOAD SEQUENCE ---
    // This specific sequence "unlocks" the DSP for firmware upload.

    // 3. Disable Rest Mode (Config2 0x10 -> 0x00)
    Mouse_WriteRegister(hspi, 0x10, 0x00);

    Mouse_WriteRegister(hspi, 0x13, 0x1D);
    HAL_Delay(10);
    Mouse_WriteRegister(hspi, 0x13, 0x18);

    HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_RESET);

	// B. Send Burst Load Address (0x62 | 0x80 for Write)
	uint8_t burstAddr = 0x62 | 0x80;
	HAL_SPI_Transmit(hspi, &burstAddr, 1, 100);

	delay_us(140);

	// D. Send Firmware Loop
	for (int i = 0; i < FIRMWARE_LENGTH; i++) {
		// Read byte from your header array
		uint8_t byte = firmware_data[i];

		// Transmit 1 Byte
		HAL_SPI_Transmit(hspi, &byte, 1, 100);

		// CRITICAL: 15us delay between bytes is required for Burst Mode
		delay_us(30);
	}

	// E. End Transaction
	HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_SET);

	// F. Allow SROM to initialize
	HAL_Delay(10);


	// --- PHASE 3: VERIFICATION ---

	// 8. Check SROM ID (0x2A)
	// If successful, this will NOT be 0x00.
	data = 0x32;
	Mouse_ReadRegister(hspi, 0x2A, &data);

	// 1. Start CRC Test
	Mouse_WriteRegister(hspi, 0x13, 0x15); // Write 0x15 to SROM_Enable [cite: 896, 1446]
	HAL_Delay(15); // Wait at least 10ms [cite: 1447]

	// 2. Read Results
	uint8_t crcLower = 0, crcUpper = 0;
	Mouse_ReadRegister(hspi, 0x25, &crcLower); // Data_Out_Lower [cite: 1449, 1576]
	Mouse_ReadRegister(hspi, 0x26, &crcUpper); // Data_Out_Upper [cite: 1449, 1576]

    if (data == 0x00) {
        return false; // Error: SROM Download Failed
    }


    Mouse_WriteRegister(hspi, 0x3D, 0x80);

    for (int i = 0; i < 55; i++) {
    	Mouse_ReadRegister(hspi, 0x3D, &data);
    	delay_us(820); // Read at 1 ms intervals (180 us taken up by read register)
    	if (data == 0xC0) {
    		break;
    	}
    }

    Mouse_WriteRegister(hspi, 0x3D, 0x00);

    // 9. Force Run Mode (Config2) again to prevent auto-sleep
    Mouse_WriteRegister(hspi, 0x10, 0x00);

    return true; // Success
}

void Mouse_MotionRead(SPI_HandleTypeDef *hspi, MouseData *mouseData) {
	Mouse_WriteRegister(hspi, 0x02, 0x01); //  Could be any value, they use 0x01 not sure why

	uint8_t data = 0x00;
	Mouse_ReadRegister(hspi, 0x02, &data);

	if ((data & (0x01 << 7)) == (0x01 << 7)) {
		// Motion detected
		Mouse_ReadRegister(hspi, 0x03, &(mouseData->deltaXL));
		Mouse_ReadRegister(hspi, 0x04, &(mouseData->deltaXH));
		Mouse_ReadRegister(hspi, 0x05, &(mouseData->deltaYL));
		Mouse_ReadRegister(hspi, 0x06, &(mouseData->deltaYH));
	}

	// TODO: Note this isn't the most efficient way to do it, you don't need to resend motion burst address

	/*HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_RESET);

	uint8_t burstAddr = 0x50 & 0x7F;
	HAL_SPI_Transmit(hspi, &burstAddr, 1, 100);

	delay_us(50);

	// Read 12 bytes
	HAL_SPI_Receive(hspi, (uint8_t *)mouseData, 12, 100);

	// E. End Transaction
	HAL_GPIO_WritePin(MOUSE_CS_GPIO_Port, MOUSE_CS_Pin, GPIO_PIN_SET);

	uint8_t data = 0x67;
	Mouse_ReadRegister(hspi, 0x00, &data);*/
}

void Mouse_GetVelocity(MouseData *data, float32_t dt, float32_t *v, float32_t *w) {
    if (dt <= 0.0f) {
        *v = 0.0f;
        *w = 0.0f;
        return;
    }

    int16_t dx = (int16_t)(((uint16_t)data->deltaXH << 8) | data->deltaXL);
    int16_t dy = (int16_t)(((uint16_t)data->deltaYH << 8) | data->deltaYL);

    *v = ((float32_t)dx / MOUSE_CPI) / dt;
    *w = (((float32_t)dy / MOUSE_CPI) / dt) / MOUSE_OFFSET_INCHES;
}
