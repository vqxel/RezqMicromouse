/*
 * mouse.c
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */
#include "mouse.h"

void Mouse_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer, size_t len) {
    uint8_t addr = addressByte & 0x7F;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &addr, 1, 100);
    delay_us(160);
    HAL_SPI_Receive(hspi, buffer, len, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    delay_us(20);
}

void Mouse_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value) {
	uint8_t txData[2] = {addressByte | 0x80, value};

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, txData, 2, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
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
    HAL_Delay(50); // Wait for chip reboot [cite: 521]

    // 2. Verify Product ID (0x00) -> Expect 0x47
    Mouse_ReadRegister(hspi, 0x00, &data, 1);
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

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

	// B. Send Burst Load Address (0x62 | 0x80 for Write)
	uint8_t burstAddr = 0x62 | 0x80;
	HAL_SPI_Transmit(hspi, &burstAddr, 1, 100);

	delay_us(15);

	// D. Send Firmware Loop
	for (int i = 0; i < FIRMWARE_LENGTH; i++) {
		// Read byte from your header array
		uint8_t byte = firmware_data[i];

		// Transmit 1 Byte
		HAL_SPI_Transmit(hspi, &byte, 1, 100);

		// CRITICAL: 15us delay between bytes is required for Burst Mode
		delay_us(15);
	}

	// E. End Transaction
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	// F. Allow SROM to initialize
	delay_us(200);


	// --- PHASE 3: VERIFICATION ---

	// 8. Check SROM ID (0x2A)
	// If successful, this will NOT be 0x00.
	Mouse_ReadRegister(hspi, 0x2A, &data, 1);

    if (data == 0x00) {
        return false; // Error: SROM Download Failed
    }

    // --- PHASE 4: FINAL CONFIGURATION ---

    // 9. Force Run Mode (Config2) again to prevent auto-sleep
    Mouse_WriteRegister(hspi, 0x10, 0x00);

    // (Optional) Enable Motion Burst in 0x50 if needed later
    // Mouse_WriteRegister(hspi, 0x50, ...);

    return true; // Success
}
