/* 
  Ticker.cpp - esp8266 library that calls functions periodically

  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#if defined(ESP8266)
#include <stddef.h>
#include <stdint.h>

#include "c_types.h"
#include "eagle_soc.h"
#include "ets_sys.h"
#include "osapi.h"

static const int ONCE   = 0;
static const int REPEAT = 1;

#include "Ticker.h"

Ticker::Ticker()
: _timer(nullptr)
{
}

Ticker::~Ticker()
{
	detach();
}

void Ticker::_attach_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg)
{
	if (_timer)
	{
		os_timer_disarm(_timer);
	}
	else
	{
		_timer = new ETSTimer;
	}

	os_timer_setfn(_timer, reinterpret_cast<ETSTimerFunc*>(callback), reinterpret_cast<void*>(arg));
	os_timer_arm(_timer, milliseconds, (repeat)?REPEAT:ONCE);
}

void Ticker::detach()
{
	if (!_timer)
		return;

	os_timer_disarm(_timer);
	delete _timer;
	_timer = nullptr;
	_callback_function = nullptr;
}

bool Ticker::active() const
{
	return (bool)_timer;
}

void Ticker::_static_callback(void* arg)
{
	Ticker* _this = (Ticker*)arg;
	if (_this == nullptr)
	{
		return;
	}
	if (_this->_callback_function)
	{
		_this->_callback_function();
	}
}
#endif

#if defined(ESP32)
/* 
  Ticker.cpp - esp32 library that calls functions periodically
  Copyright (c) 2017 Bert Melis. All rights reserved.
  
  Based on the original work of:
  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
  The original version is part of the esp8266 core for Arduino environment.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Ticker.h"

Ticker::Ticker() :
  _timer(nullptr) {}

Ticker::~Ticker() {
  detach();
}

void Ticker::_attach_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg) {
  esp_timer_create_args_t _timerConfig;
  _timerConfig.arg = reinterpret_cast<void*>(arg);
  _timerConfig.callback = callback;
  _timerConfig.dispatch_method = ESP_TIMER_TASK;
  _timerConfig.name = "Ticker";
  if (_timer) {
    esp_timer_stop(_timer);
    esp_timer_delete(_timer);
  }
  esp_timer_create(&_timerConfig, &_timer);
  if (repeat) {
    esp_timer_start_periodic(_timer, milliseconds * 1000ULL);
  } else {
    esp_timer_start_once(_timer, milliseconds * 1000ULL);
  }
}

void Ticker::detach() {
  if (_timer) {
    esp_timer_stop(_timer);
    esp_timer_delete(_timer);
    _timer = nullptr;
  }
}
#endif
