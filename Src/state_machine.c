/*
 * state_machine.c
 *
 *  Created on: 22 de jun de 2021
 *      Author: Kaue
 */

#include "state_machine.h"
/* Function Declaration*/

States_TypeDef ST_nextstate(States_TypeDef state, Actions_TypeDef action){

	States_TypeDef state_transition[N_STATES][N_ACTIONS] = { //possible transitions of the SM
		 /* [current state] -> action {go_g, 	  go_ch,     go_meas,   OK,        repeat,    	fail,	   go_diag,     go_ex,      go_prot,    go_run}*/
			[wait_code] 	   = 	  {g_sel,     ch_sel,    measuring, wait_code, wait_code, 	error_msg, diagnostics, excitation, prot_store, run_prot},
			[g_sel] 	   	   = 	  {error_msg, error_msg, error_msg, wait_code, g_sel,     	error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[ch_sel] 	       = 	  {error_msg, error_msg, error_msg, wait_code, ch_sel, 		error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[measuring] 	   = 	  {error_msg, error_msg, error_msg, wait_code, measuring, 	error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[error_msg] 	   = 	  {error_msg, error_msg, error_msg, wait_code, error_msg, 	error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[diagnostics] 	   = 	  {error_msg, error_msg, error_msg, wait_code, diagnostics, error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[excitation]       =      {error_msg, error_msg, error_msg, wait_code, go_ex,       error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[prot_store]       =      {error_msg, error_msg, error_msg, wait_code, prot_store,  error_msg, error_msg,   error_msg,  error_msg,  error_msg},
			[run_prot]         =      {error_msg, error_msg, error_msg, wait_code, run_prot,    error_msg, error_msg,   error_msg,  error_msg,  error_msg}
	};

	return state_transition[state][action];
}

