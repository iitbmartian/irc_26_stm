/*
 * enc_poll.h
 *
 *  Created on: Nov 27, 2025
 *      Author: Siddhant Kaul
 */

#ifndef SRC_ENC_POLL_H_
#define SRC_ENC_POLL_H_

#include "main.h"

void timer_quad_poll(void);
void timer_update_TX(void);
void drill_quad_poll(void);
void drill_update_TX(void);

#endif /* SRC_ENC_POLL_H_ */
