#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "motor_data_types.h"

MotorSignals getMotorSignals();
void setMotorSignals(MotorSignals new_signals);
