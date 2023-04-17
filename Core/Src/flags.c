#include "flags.h"
#include "stm32f4xx_ll_gpio.h"

static uint8_t side = 0;
static uint8_t flag_grub_pile = 0;
static uint8_t ID_Dyn = 0;
static uint8_t flag_DC1 = 0;
static uint8_t flag_DC2 = 0;
static uint8_t ID_DC = 5;
static int32_t angle_DC = 32000;
static int32_t angle_DC0 = 0;
static int32_t angle_DC1 = 0;
static int32_t angle_DC2 = 0;
static uint8_t DC_state = 0;
static uint8_t sort = 0;
static uint8_t DC0_state = 0;
static uint8_t DC1_state = 0;
static uint8_t DC2_state = 0;
static uint32_t DC0_current_pos = 0;
static uint32_t DC1_current_pos = 0;
static uint32_t DC2_current_pos = 0;
static uint8_t start_game = 0;
static uint8_t vacuum_on = 0;

struct flag_s flag = {.side = &side,
                      .flag_grub_pile = &flag_grub_pile,
                      .ID_Dyn = &ID_Dyn,
                      .flag_DC1 = &flag_DC1,
                      .flag_DC2 = &flag_DC2,
                      .ID_DC = &ID_DC,
                      .angle_DC = &angle_DC,
                      .angle_DC0 = &angle_DC0,
                      .angle_DC1 = &angle_DC1,
                      .angle_DC2 = &angle_DC2,
                      .DC_state = &DC_state,
                      .sort = &sort,
                      .DC0_state = &DC0_state,
                      .DC1_state = &DC1_state,
                      .DC2_state = &DC2_state,
                      .DC0_current_pos = &DC0_current_pos,
                      .DC1_current_pos = &DC1_current_pos,
                      .DC2_current_pos = &DC2_current_pos,
                      .start_game = &start_game,
                      .vacuum_on = &vacuum_on};