/*
 * can.c
 *
 *  Created on: 10-Aug-2021
 *      Author: Mayand
 */

#include "can.h"

HAL_StatusTypeDef CAN_Start(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
	if(HAL_CAN_ActivateNotification(hcan, ActiveITs) != HAL_OK){
		return HAL_ERROR;
	}
	return HAL_CAN_Start(hcan);
}

HAL_StatusTypeDef CAN_SetFilter(CAN_HandleTypeDef *hcan, uint32_t FIFOAssignment, uint32_t filterBank, uint32_t filterScale, uint32_t filterMode, uint32_t maskID_H, uint32_t maskID_L, uint32_t id_l, uint32_t id_h){
	CAN_FilterTypeDef filter;
	filter.FilterActivation = ENABLE;
	filter.FilterFIFOAssignment = FIFOAssignment;

	filter.FilterBank = filterBank;
	filter.FilterScale = filterScale;
	filter.FilterMode = filterMode;

	filter.FilterMaskIdHigh = maskID_H;
	filter.FilterMaskIdLow  = maskID_L;

	filter.FilterIdLow  = id_l;
	filter.FilterIdHigh = id_h;

	return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t message_id, uint32_t dlc, uint8_t *data){

	uint32_t mailbox;
	CAN_TxHeaderTypeDef Header;
	Header.StdId= message_id;
	Header.IDE= 0;
	Header.RTR= 0;
	Header.DLC= dlc;

	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0) {
	  HAL_CAN_AddTxMessage(hcan, &Header, data , &mailbox);
	}
	if((hcan->Instance->TSR & CAN_TSR_TXOK0) && (hcan->Instance->TSR & CAN_TSR_RQCP0))
		return HAL_OK;
	else
		return HAL_ERROR;
}

HAL_StatusTypeDef CAN_TransmitType2(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint16_t aData[], uint8_t bData[], uint32_t *pTxMailbox)
{
  /*adata max length = 4 (12*4 = 48 bit)*/
  /*bdata max length = 2 (8**2 = 16 bit)*/

  /* Data in Regeister */
  /*
   *  TDHR --- |  12bit  |  12bit  |  8bit  |
   *  TDLR --- |  12bit  |  12bit  |  8bit  |
   */


  uint32_t transmitmailbox;
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);

  /* Check the parameters */
  assert_param(IS_CAN_IDTYPE(pHeader->IDE));
  assert_param(IS_CAN_RTR(pHeader->RTR));
  assert_param(IS_CAN_DLC1(pHeader->DLC1));
  assert_param(IS_CAN_DLC2(pHeader->DLC2));
  if (pHeader->IDE == CAN_ID_STD)
  {
    assert_param(IS_CAN_STDID(pHeader->StdId));
  }
  else
  {
    assert_param(IS_CAN_EXTID(pHeader->ExtId));
  }
  assert_param(IS_FUNCTIONAL_STATE(pHeader->TransmitGlobalTime));

  if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check that all the Tx mailboxes are not full */
    if (((tsr & CAN_TSR_TME0) != 0U) ||
        ((tsr & CAN_TSR_TME1) != 0U) ||
        ((tsr & CAN_TSR_TME2) != 0U))
    {
      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

      /* Check transmit mailbox value */
      if (transmitmailbox > 2U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INTERNAL;

        return HAL_ERROR;
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */
      if (pHeader->IDE == CAN_ID_STD)
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) |
                                                           pHeader->RTR);
      }
      else
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->ExtId << CAN_TI0R_EXID_Pos) |
                                                           pHeader->IDE |
                                                           pHeader->RTR);
      }

      /* Set up the DLC */
      hcan->Instance->sTxMailBox[transmitmailbox].TDTR = (pHeader->DLC);

      /* Set up the Transmit Global Time mode */
      if (pHeader->TransmitGlobalTime == ENABLE)
      {
        SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      }

      /* Set up the data field */
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR, ((uint32_t)aData[3] << 20) | ((uint32_t)aData[2] << 8) | ((uint32_t)bData[1] << 0));
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR, ((uint32_t)aData[1] << 20) | ((uint32_t)aData[0] << 8) | ((uint32_t)bData[0] << 0));

      /* Request transmission */
      SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

      /* Return function status */
      return HAL_OK;
    }
    else
    {
      /* Update error code */
      hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

      return HAL_ERROR;
    }
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}

HAL_StatusTypeDef CAN_TransmitAnalog(CAN_HandleTypeDef *hcan, uint32_t message_id, uint32_t dlc, uint16_t *adata, uint8_t *bdata){

	uint32_t mailbox;
	CAN_TxHeaderTypeDef Header;
	Header.StdId= message_id;
	Header.IDE= 0;
	Header.RTR= 0;
	Header.DLC= dlc;

	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0) {
		CAN_TransmitType2(hcan, &Header, adata, bdata, &mailbox);
	}
	if((hcan->Instance->TSR & CAN_TSR_TXOK0) && (hcan->Instance->TSR & CAN_TSR_RQCP0))
		return HAL_OK;
	else
		return HAL_ERROR;
}

HAL_StatusTypeDef CAN_RecieveType2(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint16_t aData[], uint8_t bData[])
{
  HAL_CAN_StateTypeDef state = hcan->State;

  assert_param(IS_CAN_RX_FIFO(RxFifo));

  if ((state == HAL_CAN_STATE_READY) || (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check the Rx FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Check that the Rx FIFO 0 is not empty */
      if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

        return HAL_ERROR;
      }
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Check that the Rx FIFO 1 is not empty */
      if ((hcan->Instance->RF1R & CAN_RF1R_FMP1) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

        return HAL_ERROR;
      }
    }

    /* Get the header */
    pHeader->IDE = CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[RxFifo].RIR;
    if (pHeader->IDE == CAN_ID_STD)
    {
      pHeader->StdId = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_STID_Pos;
    }
    else
    {
      pHeader->ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
    }
    pHeader->RTR = (CAN_RI0R_RTR & hcan->Instance->sFIFOMailBox[RxFifo].RIR);
    pHeader->DLC = (CAN_RDT0R_DLC & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    pHeader->FilterMatchIndex = (CAN_RDT0R_FMI & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    pHeader->Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    /* Get the data */
    bData[0] = (uint8_t)(((0xFFUL << 0U) & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> 0U);
    aData[0] = (uint16_t)(((0xFFFUL << 8U) & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> 8U);
    aData[1] = (uint16_t)(((0xFFFUL << 20U) & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >>20U);
    bData[0] = (uint8_t)(((0xFFFUL << 0U) & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> 0);
    aData[2] = (uint16_t)(((0xFFFUL << 8U) & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> 8);
    aData[3] = (uint16_t)(((0xFFFUL << 20U) & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> 20);

    /* Release the FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Release RX FIFO 0 */
      SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Release RX FIFO 1 */
      SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}

HAL_StatusTypeDef CAN_Recieve(CAN_HandleTypeDef *hcan, uint32_t Fifo, uint32_t *id, uint8_t *data, uint32_t *dlc)
{
	CAN_RxHeaderTypeDef Header;

	if(HAL_CAN_GetRxMessage(hcan, Fifo, &Header, data) == HAL_OK)
	{
		*id = Header.StdId;
		*dlc = Header.DLC;
		return HAL_OK;
	}
	else return HAL_ERROR;
}

HAL_StatusTypeDef CAN_RecieveAnalog(CAN_HandleTypeDef *hcan, uint32_t RxFifo, uint32_t *id, uint16_t *aData, uint8_t *bData, uint32_t *dlc)
{
	CAN_RxHeaderTypeDef Header;

	if(CAN_RecieveType2(hcan, RxFifo, &Header, aData, bData) == HAL_OK)
	{
		*id = Header.StdId;
		*dlc = Header.DLC;
		return HAL_OK;
	}
	else return HAL_ERROR;
}
