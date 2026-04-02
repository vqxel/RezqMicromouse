/*
 * mouse.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_MOUSE_H_
#define INC_MOUSE_H_


void Mouse_ReadRegister(uint8_t addressByte, uint8_t *buffer, size_t len);

void Mouse_WriteRegister(uint8_t addressByte, uint8_t value);

bool Mouse_Init(void);

#endif /* INC_MOUSE_H_ */
