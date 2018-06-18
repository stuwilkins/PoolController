/*
 * =====================================================================================
 *
 *       Filename:  timer.h
 *
 *    Description:  Header File for Timer
 *
 *        Version:  1.0
 *        Created:  06/17/2018 17:09:44
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuwilkins@mac.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include <RTClib.h>

#define POOLTIMER_ALLDAYS 0x7F

class PoolTimer
{
  public:
    PoolTimer(void);
    bool isActive(DateTime now);
    void setStart(int hrs, int mins);
    void setStop(int hrs, int mins);
    void setDays(int days);

  private:
    int start_hrs;
    int start_mins;
    int stop_hrs;
    int stop_mins;
    int days;

};

