
/*
 * This file is part of pcb2gcode.
 * 
 * Copyright (C) 2012 Thomas Fritz <frithomas@gmail.com>
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

#ifndef SMOOTHNGCEXPORTER_H
#define SMOOTHNGCEXPORTER_H

#include <vector>
using std::vector;

#include <string>
using std::string;
using std::pair;

#include <fstream>
using std::ofstream;

#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

#include <boost/program_options.hpp>

#include "coord.hpp"
#include "mill.hpp"
#include "exporter.hpp"
#include "ngc_exporter.hpp"

class SNGC_Exporter : public NGC_Exporter
{
public:
	SNGC_Exporter( shared_ptr<Board> board );

protected:
	void export_layer( shared_ptr<Layer> layer, string of_name );
};

#endif // SMOOTHNGCEXPORTER_H
