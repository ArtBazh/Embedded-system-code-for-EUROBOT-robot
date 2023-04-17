#include "std_def.h"
void driveDC (uint8_t ID, uint8_t dir, uint16_t speed);
void setPosition(uint8_t ID, int32_t angle);
void resetEncoder(uint8_t ID);
void set01Position(int32_t angle0, int32_t angle1);