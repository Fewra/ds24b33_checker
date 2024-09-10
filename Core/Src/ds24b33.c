/*
 * ds24b33.c
 *
 *  Created on: 14 июл. 2024 г.
 *      Author: dimoc
 */


/* Includes ----------------------------------------------------------*/
#include "onewire.h"
#include "ds24b33.h"


/* Declarations and definitions ----------------------------------------------*/

static DS24B33_Command writeScratch = {.code = 0x0F, .rxBytesNum = 0};
static DS24B33_Command readScratch = {.code = 0xAA, .rxBytesNum = 35, .txBytesNum = 0};
static DS24B33_Command copyScratch = {.code = 0x55, .rxBytesNum = 0, .txBytesNum = 3};
static DS24B33_Command readMemory = {.code = 0xF0, .txBytesNum = 2};
static DS24B33_Command readRom = {.code = 0x33, .rxBytesNum = 8, .txBytesNum = 0};

static void ExecuteCommand(UART_HandleTypeDef *huart, DS24B33_Command command, uint8_t *data)
{
	OneWire_ProcessByte(huart, command.code);

	for (uint8_t i = 0; i < command.txBytesNum; i++)
	{
		OneWire_ProcessByte(huart, data[i]);
	}

	for (uint8_t i = 0; i < command.rxBytesNum; i++)
	{
		data[i] = OneWire_ProcessByte(huart, 0xFF);
	}


}

void WriteScratchpad(UART_HandleTypeDef *huart, uint8_t *data, size_t size)
{
	writeScratch.txBytesNum = size;
	ExecuteCommand(huart, writeScratch, data);
	OneWire_Reset(huart);
}

void ReadScratchpad(UART_HandleTypeDef *huart, uint8_t *data)
{
	ExecuteCommand(huart, readScratch, data);
	OneWire_Reset(huart);
}

void CopyScratchpad(UART_HandleTypeDef *huart, uint8_t *data)
{
	ExecuteCommand(huart, copyScratch, data);

	uint32_t t = HAL_GetTick();
	while (HAL_GetTick() - t < 7) {}

	OneWire_Reset(huart);
}

void ReadMemory(UART_HandleTypeDef *huart, uint8_t *data, size_t size)
{
	readMemory.rxBytesNum = size;
	ExecuteCommand(huart, readMemory, data);
	OneWire_Reset(huart);
}
void ReadRom(UART_HandleTypeDef *huart, uint8_t *data)
{
	ExecuteCommand(huart, readRom, data);
}
