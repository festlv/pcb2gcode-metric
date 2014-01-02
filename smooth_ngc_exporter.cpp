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

#include "smooth_ngc_exporter.hpp"
#include "douglas_peucker.hpp"

#include <boost/foreach.hpp>

#include <iostream>
#include <iomanip>
using namespace std;

SNGC_Exporter::SNGC_Exporter( shared_ptr<Board> board ) : NGC_Exporter(board)
{
	this->board = board;
	bDoSVG = false;
}

void
SNGC_Exporter::export_layer( shared_ptr<Layer> layer, string of_name )
{
	string layername = layer->get_name();
	shared_ptr<RoutingMill> mill = layer->get_manufacturer();

	bool bSvgOnce = TRUE;
	
	// open output file
	std::ofstream of; of.open( of_name.c_str() );

    // create Gcode D-P filter
    Gcode gc(mill->zchange, mill->zsafe, get_tolerance(), mill->speed, "G20", of);

	// write header to .ngc file
        BOOST_FOREACH( string s, header )
        {
                of << "( " << s << " )" << endl;
        }
        of << endl;

        of.setf( ios_base::fixed );
        of.precision(5);
	of << setw(7);

	// preamble
	of << "G94     ( Inches per minute feed rate. )\n"
	   << "G20     ( Units == INCHES.             )\n"
	   << "G90     ( Absolute coordinates.        )\n"
	   << "S" << left << mill->speed << "  ( RPM spindle speed.           )\n"
	   << "M3      ( Spindle on clockwise.        )\n"
	   << endl;

	of << "G64 P" << get_tolerance() << " ( set maximum deviation from commanded toolpath )\n"
	   << endl;

	
	//SVG EXPORTER
	if (bDoSVG) {
		//choose a color
		svgexpo->set_rand_color();
	}
	
	
	// contours
 	BOOST_FOREACH( shared_ptr<icoords> path, layer->get_toolpaths() )
        {
		// retract, move to the starting point of the next contour
//		of << "G04 P0 ( dwell for no time -- G64 should not smooth over this point )\n";
        gc.safety();
        gc.rapid(Move().X(path->begin()->first).Y(path->begin()->second));
			
		//SVG EXPORTER
		if (bDoSVG) {						
			svgexpo->move_to(path->begin()->first, path->begin()->second);
			bSvgOnce = TRUE;
		}
			
		/** if we're cutting, perhaps do it in multiple steps, but do isolations just once.
		 *  i know this is partially repetitive, but this way it's easier to read
		 */
		shared_ptr<Cutter> cutter = boost::dynamic_pointer_cast<Cutter>( mill );
		if( cutter && cutter->do_steps ) {
			// cutting
			double z_step = cutter->stepsize;
			double z = mill->zwork + z_step * abs( int( mill->zwork / z_step ) );

			while( z >= mill->zwork ) {
                gc.set_feed(mill->feed);
                gc.cut(Move().Z(z));
//				of << "G04 P0 ( dwell for no time -- G64 should not smooth over this point )\n";

				icoords::iterator iter = path->begin();
				icoords::iterator last = path->end(); // initializing to quick & dirty sentinel value
				icoords::iterator peek;
				while( iter != path->end() ) {
					peek = iter + 1;
					if( /* it's necessary to write the coordinates if... */
							last == path->end() || /* it's the beginning */
							peek == path->end() || /* it's the end */
							!( /* or if neither of the axis align */
								( last->first == iter->first && iter->first == peek->first ) || /* x axis aligns */
								( last->second == iter->second && iter->second == peek->second ) /* y axis aligns */
							)
							/* no need to check for "they are on one axis but iter is outside of last and peek" becaus that's impossible from how they are generated */
					  ) {
                        gc.cut(Move().X(iter->first).Y(iter->second));
						
						//SVG EXPORTER
						if (bDoSVG) {
							if (bSvgOnce) svgexpo->line_to(iter->first, iter->second);
						}
					}
					last = iter;
					++iter;
				}
				//SVG EXPORTER
				if (bDoSVG) {
					svgexpo->close_path();
					bSvgOnce = FALSE;
				}
			
				z -= z_step;
			}
		} else {
			// isolating
            gc.set_feed(mill->feed);
            gc.cut(Move().Z(mill->zwork));
//			of << "G04 P0 ( dwell for no time -- G64 should not smooth over this point )\n";

			icoords::iterator iter = path->begin();
			icoords::iterator last = path->end(); // initializing to quick & dirty sentinel value
			icoords::iterator peek;
			while( iter != path->end() ) {
				peek = iter + 1;
				if( /* it's necessary to write the coordinates if... */
						last == path->end() || /* it's the beginning */
						peek == path->end() || /* it's the end */
						!( /* or if neither of the axis align */
							( last->first == iter->first && iter->first == peek->first ) || /* x axis aligns */
							( last->second == iter->second && iter->second == peek->second ) /* y axis aligns */
						)
						/* no need to check for "they are on one axis but iter is outside of last and peek" becaus that's impossible from how they are generated */
				  ) {
                    gc.cut(Move().X(iter->first).Y(iter->second));
					
					//SVG EXPORTER
					if (bDoSVG) if (bSvgOnce) svgexpo->line_to(iter->first, iter->second);

				}
				last = iter;
				++iter;
			}
			//SVG EXPORTER
			if (bDoSVG) {
				svgexpo->close_path();
				bSvgOnce = FALSE;
			}

		}


		
        }

        of << endl;

	// retract, end
//	of << "G04 P0 ( dwell for no time -- G64 should not smooth over this point )\n";
//	of << "G00 Z" << mill->zchange << " ( retract )\n" << endl;
    gc.safety();
    gc.end();

//	of << "M9 ( Coolant off. )\n";
//	of << "M2 ( Program end. )\n\n";

	of.close();
	
	//SVG EXPORTER
	if (bDoSVG) {
		svgexpo->stroke();
	}
}
