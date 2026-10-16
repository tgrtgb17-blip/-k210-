#ifndef _SG90_H
#define _SG90_H


void SG90_init(void);
void SG90_set_angle(float *angle);
void SG90_Mount(uint8_t color);
static uint16_t SG90_Map(float angle);
void SG90_j5(float angle);
void SG90_j6_start(uint8_t mode);
uint8_t SG90_j6_done(void);
#endif
