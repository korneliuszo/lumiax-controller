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
#include <string.h>

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

	memset(buff,0xff,sizeof(buff));

	printt("Initialized %s", dev->name);
	return 0;
}

void Display::update()
{
	display_blanking_off(dev);
	struct display_buffer_descriptor desc={
			.buf_size=128*296/8,
			.width=296,
			.height=128,
			.pitch=296
	};
	display_write(dev,0,0,&desc,buff);
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
	for(int yp=0;yp<16;yp++)
	{
		buff[296/8*8*((yp+y)>>3)+(yp+y)%8+x] = my_font_[idx][yp];
	}
}

void Display::print_str(int x, int y, const char* str)
{
	while(*str)
	{
		print_chr(x,y,*str++);
		x+=8;
	}
}

