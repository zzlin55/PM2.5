/*
 * CMSIS-USERDEF.c
 *
 *  Created on: Jan 10, 2020
 *      Author: zzlin
 *
 *      This file used to copy to cmsis_os.c
 */


/**
* @brief Overwrite a Message to a Queue.
* @param  queue_id  message queue ID obtained with \ref osMessageCreate.
* @param  info      message information.
* @param  millisec  timeout value or 0 in case of no time-out.
* @retval status code that indicates the execution status of the function.
* @note   MUST REMAIN UNCHANGED: \b osMessagePut shall be consistent in every CMSIS-RTOS.
*/

osStatus osMessageOverwrite (osMessageQId queue_id, uint32_t info)
{
  portBASE_TYPE taskWoken = pdFALSE;

  if (inHandlerMode()) {
    if (xQueueOverwriteFromISR(queue_id, &info, &taskWoken) != pdTRUE) {
      return osErrorOS;
    }
    portEND_SWITCHING_ISR(taskWoken);
  }
  else {
    if (xQueueOverwrite(queue_id, &info) != pdTRUE) {
      return osErrorOS;
    }
  }

  return osOK;
}
