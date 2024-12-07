#pragma once

#include <tim.h>
#include "stdbool.h"
#include "main.h"

typedef struct {
  uint32_t start_tick;
  uint32_t delay;
  bool active;
} SoftTimer;

void SoftTimer_Init(void);
void SoftTimer_Start(SoftTimer* timer, uint32_t delay_ms);
bool SoftTimer_Expired(SoftTimer* timer);
