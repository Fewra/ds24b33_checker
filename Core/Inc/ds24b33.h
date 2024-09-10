/*
 * ds24b33.h
 *
 *  Created on: 14 июл. 2024 г.
 *      Author: dimoc
 */

#ifndef INC_DS24B33_H_
#define INC_DS24B33_H_

enum memory_status {
	memory_false = 0,
	memory_true = 1
};

typedef struct DS24B33_Command
{
  uint8_t code;
  uint16_t rxBytesNum;
  uint16_t txBytesNum;
} DS24B33_Command;

extern void WriteScratchpad(UART_HandleTypeDef *huart, uint8_t *data, size_t size);
extern void ReadScratchpad(UART_HandleTypeDef *huart, uint8_t *data);
extern void CopyScratchpad(UART_HandleTypeDef *huart, uint8_t *data);
extern void ReadMemory(UART_HandleTypeDef *huart, uint8_t *data, size_t size);
extern void ReadRom(UART_HandleTypeDef *huart, uint8_t *data);

#endif /* INC_DS24B33_H_ */
