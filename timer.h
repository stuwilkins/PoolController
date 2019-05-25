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

#ifndef _TIMER_H_
#define _TIMER_H_

#include <RTClib.h>

#define ALLDAYS 0x7F

class Timer
{
  public:
    Timer(void);
    Timer(uint8_t start_days, int8_t start_hrs, int8_t start_mins, 
          uint8_t stop_days, int8_t stop_hrs, int8_t stop_mins);
    bool isPastStart(DateTime now);
    bool isNotPastStop(DateTime now);
    bool isActive(DateTime now);
    void setStartCallback(bool (*start_callback)(void *));
    void setStopCallback(bool (*start_callback)(void *));
    bool loop(DateTime now);

  private:
    int8_t _start_hrs;
    int8_t _start_mins;
    int8_t _stop_hrs;
    int8_t _stop_mins;
    uint8_t _start_days;
    uint8_t _stop_days;
    bool _active;
    void *_callback_data;
    bool (*_start_callback)(void *);
    bool (*_stop_callback)(void *);
};

#endif
