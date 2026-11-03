#pragma once
typedef int SemaphoreHandle_t;
static inline int xSemaphoreCreateMutex(void){return 1;}
static inline void xSemaphoreTake(int l,int n){(void)l;(void)n;}
static inline void xSemaphoreGive(int l){(void)l;}
