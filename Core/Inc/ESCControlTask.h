//
// Created by Administrator on 25-6-20.
//

#ifndef CONTROLTASK_H
#define CONTROLTASK_H
#include <stdint.h>

#define ESC_MOUDLE_ID 2

#define ESC_MIN  1100
#define ESC_MAX  2000

#define BASE_THROTTLE 1200

typedef struct
{
    uint16_t ESCA;
    uint16_t ESCB;
    uint16_t ESCC;
    uint16_t ESCD;
} ESCOutput;

extern ESCOutput ESC_Output;

void ESC_Init();
void ESC_Update();
void ESCControlTask();
#endif //CONTROLTASK_H
