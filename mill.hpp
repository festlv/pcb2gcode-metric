
/*
 * This file is part of pcb2gcode.
 * 
 * Copyright (C) 2009, 2010 Patrick Birnzain <pbirnzain@users.sourceforge.net>
 * 
 * pcb2gcode is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * pcb2gcode is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with pcb2gcode.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MILL_H
#define MILL_H

#include <stdint.h>

#define METRIC_OUTPUT


#ifdef METRIC_OUTPUT
    #define CONVERT_UNITS(in) ((in)*25.4)
#else
    #define CONVERT_UNITS(in) (in)
#endif


class Mill
{
public:
	virtual ~Mill() {};

	double feed;
	int    speed;

	double zchange;
	double zsafe;
	double zwork;
};

class RoutingMill : public Mill
{
public:
	double tool_diameter;
};

class Isolator : public RoutingMill
{
public:
        int extra_passes;
};

class Cutter : public RoutingMill
{
public:
	bool do_steps;
	double stepsize;
};

class Driller : public Mill
{
};

#endif // MILL_H
