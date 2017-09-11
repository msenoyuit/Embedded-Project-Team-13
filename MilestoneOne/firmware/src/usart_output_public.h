#ifndef _USART_OUTPUT_PUBLIC_H
#define _USART_OUTPUT_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"

#include "usart_output.h"

BaseType_t usartOutputSendMsgToQFromISR(QueueMessage message,
                                        BaseType_t * higherPriorityTaskWoken);

#endif // _USART_OUTPUT_PUBLIC_H
