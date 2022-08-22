/*
 * display.cpp
 *
 *  Created on: Aug 21, 2022
 *      Author: kosa
 */

#include <zephyr/drivers/display.h>

#include "display.hpp"
#include "printt.h"
#include "font.h"

int Display::Init()
{
	k_mutex_init(&mut);
	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(dev)) {
		printt("Device %s not ready", dev->name);
		return 1;
	}

	if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO10) != 0) {
		printt("Failed to set required pixel format");
		return 1;
	}
	printt("Initialized %s", dev->name);
	return 0;
}

void Display::blank_off()
{
	display_blanking_off(dev);
}

void Display::blank_on()
{
	display_blanking_on(dev);
}

void Display::print_chr(int x, int y, char c)
{
	int idx;
	for(idx=0;my_font_map_[idx];idx++)
	{
		if(my_font_map_[idx] == c)
			break;
	}
	if(!my_font_map_[idx])
		idx=0;

	struct display_buffer_descriptor desc={
			.buf_size=16,
			.width=8,
			.height=16,
			.pitch=8
	};
	display_write(dev,x,y,&desc,my_font_[idx]);
}

void Display::print_str(int x, int y, const char* str)
{
	while(*str)
	{
		print_chr(x,y,*str++);
		x+=8;
	}
}

