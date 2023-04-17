/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __flags_H
#define __BYTE_2_WORD_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "std_def.h"


struct flag_s{
    uint8_t *side;
		uint8_t *flag_grub_pile;
    uint8_t *ID_Dyn;
    uint8_t *flag_DC1;
    uint8_t *flag_DC2;
    uint8_t *ID_DC;
    int32_t *angle_DC;

    int32_t *angle_DC0;
    int32_t *angle_DC1;
    int32_t *angle_DC2;

    uint8_t *DC_state;
    uint8_t *sort;

    uint8_t *DC0_state;
    uint8_t *DC1_state;
    uint8_t *DC2_state;

    uint32_t *DC0_current_pos;
    uint32_t *DC1_current_pos;
    uint32_t *DC2_current_pos;

    uint8_t *start_game;
    uint8_t *vacuum_on;w
};

extern struct flag_s flag;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
