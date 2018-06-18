/*
 * =====================================================================================
 *
 *       Filename:  timer.cpp
 *
 *    Description:  Timer for Pool, based on RTC
 *
 *        Version:  1.0
 *        Created:  06/17/2018 17:09:21
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuwilkins@mac.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "timer.h"

PoolTimer::PoolTimer(void)
{
  start_hrs = 0;
  start_mins = 0;
  stop_hrs = 0;
  stop_mins = 0;
  days = 0;
}
