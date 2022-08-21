/*
 * display.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: kosa
 */

#ifndef DISPLAY_HPP_
#define DISPLAY_HPP_

#include <zephyr/zephyr.h>

class Display {
	const struct device *dev;
	k_mutex mut;
public:
	int Init();
	void blank_off();
	void print_chr(int x, int y, char c);
	void print_str(int x, int y, const char* str);
};






#endif /* DISPLAY_HPP_ */
