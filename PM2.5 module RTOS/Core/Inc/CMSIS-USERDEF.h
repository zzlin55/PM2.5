/*
 * CMSIS-USERDEF.h
 *
 *  Created on: Jan 10, 2020
 *      Author: zzlin
 */

#ifndef INC_CMSIS_USERDEF_H_
#define INC_CMSIS_USERDEF_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

osStatus osMessageOverwrite (osMessageQId queue_id, uint32_t info);

#endif /* INC_CMSIS_USERDEF_H_ */
