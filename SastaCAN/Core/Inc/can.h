/*
 * can.h
 *
 *  Created on: 10-Aug-2021
 *      Author: Mayand
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef CAN_Start(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef CAN_SetFilter(CAN_HandleTypeDef *hcan, uint32_t FIFOAssignment, uint32_t filterBank, uint32_t filterScale, uint32_t filterMode, uint32_t maskID_H, uint32_t maskID_L, uint32_t id_l, uint32_t id_h);

HAL_StatusTypeDef CAN_TransmitType2(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint16_t aData[], uint8_t bData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef CAN_TransmitAnalog(CAN_HandleTypeDef *hcan, uint32_t message_id, uint32_t dlc, uint16_t *adata, uint8_t *bdata);

HAL_StatusTypeDef CAN_RecieveType2(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint16_t aData[], uint8_t bData[]);
HAL_StatusTypeDef CAN_Receive(CAN_HandleTypeDef *hcan, uint32_t Fifo, uint32_t *id, uint8_t *data, uint32_t *dlc);
HAL_StatusTypeDef CAN_RecieveAnalog(CAN_HandleTypeDef *hcan, uint32_t RxFifo, uint32_t *id, uint16_t *aData, uint8_t *bData, uint32_t *dlc);


#endif /* INC_CAN_H_ */


