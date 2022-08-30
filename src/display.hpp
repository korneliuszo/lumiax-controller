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
	uint8_t buff[128*296/8];
public:
	class Display_lock
	{
		k_mutex *m;
	public:
		Display_lock(k_mutex *_m)
		: m(_m)
		{
			k_mutex_lock(m, K_FOREVER);
		}
		~Display_lock()
		{
			k_mutex_unlock(m);
		}
	};
	int Init();
	Display_lock lock()
	{
		return Display_lock(&mut);
	}
	void update();
	void print_chr(int x, int y, char c);
	void print_str(int x, int y, const char* str);
};






#endif /* DISPLAY_HPP_ */
