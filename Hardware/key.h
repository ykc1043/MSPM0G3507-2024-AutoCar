#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"

#define KEY_N1_READ() 	( ((DL_GPIO_readPins(KEY_N3_PORT,KEY_N3_PIN) & KEY_N3_PIN) > 0) ? 1 : 0 )
#define KEY_N2_READ()	( ((DL_GPIO_readPins(KEY_N2_PORT,KEY_N2_PIN) & KEY_N2_PIN) > 0) ? 1 : 0 )
#define KEY_N3_READ()	( ((DL_GPIO_readPins(KEY_N1_PORT,KEY_N1_PIN) & KEY_N1_PIN) > 0) ? 1 : 0 )


//uint8_t click(void);

#endif
