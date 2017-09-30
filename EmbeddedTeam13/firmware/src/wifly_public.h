#ifndef _WIFLY_PUBLIC_H
#define _WIFLY_PUBLIC_H
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END

#define WIFLY_MAX_MSG_LEN 100

struct StandardQueueMessage;

typedef struct {
    char text[WIFLY_MAX_MSG_LEN];
} WiflyMsg;

BaseType_t wiflySendMsg(struct StandardQueueMessage * message,
                        TickType_t ticksToWait);

#endif // WIFLY_PUBLIC_H

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
