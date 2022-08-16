#ifndef OAL_GAUSSION_HPP
#define OAL_GAUSSION_HPP

#include <fstream>
#include <iostream>

#include "arm_neon.h"

bool check_x3_chip_id();
namespace oal {
void gaussion_3x3(uint16_t *input, uint16_t *output, int width, int height);
void gaussion_5x5(uint16_t *input, uint16_t *output, int width, int height);
void gaussion_3x3_int16(int16_t *input, int16_t *output, int width, int height);
void gaussion_5x5_int16(int16_t *input, int16_t *output, int width, int height);
void mean_3x3(uint16_t *input, uint16_t *output, int width, int height);
void mean_5x5(uint16_t *input, uint16_t *output, int width, int height);
void mean_3x3_int16(int16_t *input, int16_t *output, int width, int height);
void mean_5x5_int16(int16_t *input, int16_t *output, int width, int height);
}  // namespace oal

#endif  // OAL_GAUSSION_HPP