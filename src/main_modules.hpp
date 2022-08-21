/*
 * main_modules.hpp
 *
 *  Created on: Aug 16, 2022
 *      Author: kosa
 */

#ifndef MAIN_MODULES_HPP_
#define MAIN_MODULES_HPP_

#include "tlay2.hpp"
#include "display.hpp"

#include <stdint.h>
#include <zephyr/kernel.h>

struct Reg_data {
	uint16_t b_soc;
	uint16_t b_v;
	int16_t  b_a;
	uint16_t l_v;
	uint16_t l_a;
	uint16_t s_v;
	uint16_t s_a;
	k_mutex mut;
};

extern struct Reg_data reg_data;

extern int client_iface;

extern Tlay2<128> tlay2;

extern Display display;




#endif /* MAIN_MODULES_HPP_ */
