/* Copyright (C) 
* 2015 - Gowtham Garimella
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
* 
*/

#ifndef COMMON_H
#define COMMON_H
#include <sys/time.h>
#include <string>

/**
* @brief This file provides common mapping functions and any small math functions needed for the UI and parser. This is done to keep the libraries independent of small libraries that need to be used 
*/
namespace common{
/**
* @brief Add time to the input string
*
* @param inmsg  Input string
*
* @return   Output String with date and time attached
*/
inline std::string addtimestring( std::string inmsg)
{
# define TIME_SIZE 80
  const struct tm *tm;
  size_t len;
  time_t now;
  char *s;

  now = time ( NULL );
  tm = localtime ( &now );

  s = (char *)malloc ( TIME_SIZE * sizeof ( char ) );

  len = strftime ( s, TIME_SIZE, "%d_%B_%Y_%I_%M_%S_%p", tm );
  return (inmsg+std::string(s));
# undef TIME_SIZE
}

/**
  * Maps input value linearly between inpmin and inpmax to resmin and resmax
  * @inp        Input
  * @inpmin     Minimum for Input
  * @inpmax     Maximum for Input
  * @resmin     Minimum for result
  * @resmax     Maximum for result
  * @return     Output
  */
inline double map(double inp, double inpmin, double inpmax,  double resmin, double resmax)
{
  assert(resmin <= resmax);
  assert(inpmin <= inpmax);
  if(inpmin == inpmax)
    return resmin;
  if(inp >= inpmax)
    return resmax;
  else if(inp <= inpmin)
    return resmin;
  //In all other cases
  return (resmin + ((inp-inpmin)*(resmax-resmin))/(inpmax-inpmin));
}

/**
* @brief Convert angles between 0 to 2*pi to -pi to pi
*
* @param inpangle Input angle between 0 to 2*M_PI
*
* @return  output angle between -M_PI to M_PI
*/
inline double map_angle(double inpangle)
{
  assert(inpangle <= 2*M_PI);
  assert(inpangle >= -2*M_PI);

  if(inpangle > M_PI)
    inpangle = inpangle - 2*M_PI;
  else if(inpangle < -M_PI)
    inpangle = inpangle + 2*M_PI;
  return inpangle;
}

/**
* @brief Get current time in microseconds
*
* @return Time in microseconds
*/
inline long timeMicroseconds()
{
  //Get current time:
  struct timeval now;
  gettimeofday(&now, 0);
  return (now.tv_sec*1000000 + now.tv_usec);
}

}
#endif//COMMON_H
