// 
// Copyright (c) 2018 Stuart B. WIlkins
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "timer.h"

Timer::Timer(void)
{
  _start_hrs = 0;
  _start_mins = 0;
  _stop_hrs = 0;
  _stop_mins = 0;
  _start_days = 0;
  _stop_days = 0;

  _active = false;

  _start_callback = NULL;
  _stop_callback = NULL;
}

Timer::Timer(uint8_t start_days, int8_t start_hrs, int8_t start_mins, 
             uint8_t stop_days, int8_t stop_hrs, int8_t stop_mins) 
    : Timer()
{
  this->_start_days = start_days;
  this->_start_hrs = start_hrs;
  this->_start_mins = start_mins;
  this->_stop_days = stop_days;
  this->_stop_hrs = stop_hrs;
  this->_stop_mins = stop_mins;
}

bool Timer::isPastStart(DateTime now)
{
    uint8_t day = 0x01 << now.dayOfTheWeek();
    if(!(day & _start_days))
    {
        return false;
    }

    if(_start_hrs < now.hour())
    {
        return false;
    }

    if(_start_mins < now.minute())
    {
        return false;
    }

    return true;
} 

bool Timer::isNotPastStop(DateTime now)
{
    uint8_t day = 0x01 << now.dayOfTheWeek();
    if(!(day & _stop_days))
    {
        return false;
    }

    if(_stop_hrs < now.hour())
    {
        return false;
    }

    if(_stop_mins < now.minute())
    {
        return false;
    }

    return true;
} 

bool Timer::isActive(DateTime now)
{
    return isPastStart(now) && isNotPastStop(now);
}

void Timer::setStartCallback(bool (*start_callback)(void *))
{
    _start_callback = start_callback;
}

void Timer::setStopCallback(bool (*stop_callback)(void *))
{
    _stop_callback = stop_callback;
}

bool Timer::loop(DateTime now)
{
    // Lets check if we are past the start
    if(isPastStart(now) && !_active)
    {
        // Fire the callbacks
        if(_start_callback(_callback_data))
        {
            _active = true;
            return true;
        }
    }
    if(!isNotPastStop(now) && _active)
    {
        // Fire the callbacks
        if(_stop_callback(_callback_data))
        {
            _active = false;
            return true;
        }
    }

    return false;
}
