/* -*- c++ -*- */
/* 
 * Copyright 2018 Gereon Such.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef FDC_TIME_H
#define FDC_TIME_H

#include <string>
//#include <ctime>

std::string get_time(const char* format="%Y%m%d%H%M%S"){
    /*
    std::time_t t=std::time(0);
    char puffer[100];
    std::strftime(puffer, sizeof(puffer), format, localtime(&t));

    return std::string( puffer );*/

    return std::string("notime");
}

#endif /* FDC_TIME_H */

