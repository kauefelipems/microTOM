/*
 * state_machine.h
 *
 *  Created on: 22 de jun de 2021
 *      Author: DELL
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_
#include <stdio.h>

// Parameter Definitions
#define N_STATES 9
#define N_ACTIONS 10

// TypeDef Definitions
typedef enum{wait_code, g_sel, ch_sel, measuring, error_msg, diagnostics, excitation, prot_store, run_prot
} States_TypeDef;

typedef enum{go_g, go_ch, go_meas, ok, repeat, fail, go_diag, go_ex, go_prot, go_run
} Actions_TypeDef;


States_TypeDef ST_nextstate(States_TypeDef state, Actions_TypeDef action);


#endif /* INC_STATE_MACHINE_H_ */
