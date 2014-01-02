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

/*
 * =====================================================================================
 *
 *       Filename:  douglase_peucker.cpp
 *
 *    Description:  Based on the python lib 'rs274.author' which is part of linuxcnc
 *    (http://linuxcnc.org), after I hacked and improved the vastly non-pythonic code
 *    they had there.  Implements a variant of the Douglas-Peucker algorithm for
 *    simplifying polylines, with additional support for creating arcs.
 *
 *    Known limitations:
 *      - The polyline must consist of linear (G00/G01) moves.  Arcs are not processed.
 *      - The polyline must not be closed (endpoints must not be equal).  If closed,
 *          the polyline will get simplified out of existance.
 *      - Better results are achieved at higher dpi rendering.  1000 dpi is a little
 *          rough, but 5000 dpi works very well.
 *
 *        Version:  1.0
 *        Created:  05/20/2012 12:16:52 AM
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include <set>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <utility>
#include "douglas_peucker.hpp"

using namespace std;

float dist_lseg(Point3f& l1, Point3f& l2, Point3f& p) {
// Compute the 3D distance from the line segment l1..l2 to the point p
    float dx,dy,dz,d2,t,dist2;
    dx = l2.x-l1.x; dy = l2.y-l1.y; dz = l2.z-l1.z;
    d2 = dx*dx + dy*dy + dz*dz;
    if (d2 == 0) {
        return 0.0;
    }
    t = (dx * (p.x-l1.x) + dy * (p.y-l1.y) + dz * (p.z-l1.z)) / d2;
    if (t < 0.0) { t = 0.0; }
    if (t > 1.0) { t = 1.0; }
    dist2 = pow((p.x - l1.x - t*dx), 2) + \
            pow((p.y - l1.y - t*dy), 2) + \
            pow((p.z - l1.z - t*dz), 2);
    return pow(dist2, 0.5);
//    return sqrt(dist2);
}

float rad1(float x1, float y1, float x2, float y2, float x3, float y3) {
    float x12,y12,x23,y23,x31,y31,den;
    x12 = x1-x2;
    y12 = y1-y2;
    x23 = x2-x3;
    y23 = y2-y3;
    x31 = x3-x1;
    y31 = y3-y1;
    den = abs(x12 * y23 - x23 * y12);
    if (abs(den) < FLT_EPSILON) {
        return FLT_MAX;
    }
    return hypot(x12, y12) * hypot(x23, y23) * hypot(x31, y31) / 2 / den;
}

Point2f cent1(float x1, float y1, float x2, float y2, float x3, float y3) {
    float den, alpha, beta, gamma;
    Point2f p1(x1,y1), p2(x2,y2), p3(x3,y3);
    den = abs((p1 - p2).cross(p2 - p3));
    if (abs(den) < FLT_EPSILON) {
        return Point2f(FLT_MAX, FLT_MAX); // XXX should this be FLT_MAX?
    }
    alpha = (p2 - p3).mag2() * (p1 - p2).dot(p1 - p3) / 2 / den / den;
    beta =  (p1 - p3).mag2() * (p2 - p1).dot(p2 - p3) / 2 / den / den;
    gamma = (p1 - p2).mag2() * (p3 - p1).dot(p3 - p2) / 2 / den / den;
    Point2f Pc = (p1 * alpha) + (p2 * beta) + (p3 * gamma);
    return Pc;
}

Point2f arc_center(int plane, Point3f& p1, Point3f p2, Point3f p3) {
    switch (plane) {
    case 17:
        return cent1(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
        break;
    case 18:
        return cent1(p1.x, p1.z, p2.x, p2.z, p3.x, p3.z);
        break;
    case 19:
        return cent1(p1.y, p1.z, p2.y, p2.z, p3.y, p3.z);
        break;
    }
//    return Point2f(FLT_MAX, FLT_MAX);
}

float arc_rad(int plane, Point3f& p1, Point3f p2, Point3f p3) {
    switch (plane) {
    case 17:
        return rad1(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
    case 18:
        return rad1(p1.x, p1.z, p2.x, p2.z, p3.x, p3.z);
    case 19:
        return rad1(p1.y, p1.z, p2.y, p2.z, p3.y, p3.z);
    }
//    return FLT_MAX;
}

Point2f get_pts(int plane, Point3f& p) {
    switch (plane) {
    case 17:
        return Point2f(p.x, p.y);
    case 18:
        return Point2f(p.x, p.z);
    case 19:
        return Point2f(p.y, p.z);
    }
//    return Point2f(p.x, p.y);
}

int sign(float i) {
//    return (i > 0) - (i < 0); // doesn't quite do the behavior needed
    if (abs(i) < FLT_EPSILON) { return 0; }
    if (i < 0) { return -1; }
    return 1;
}

bool one_quadrant(int plane, Point2f& c, Point3f& p1, Point3f& p2, Point3f& p3) {
    float xc,yc,x1,y1,x2,y2,x3,y3;
    Point2f tmp;
    xc = c.x; yc = c.y;
    tmp = get_pts(plane, p1);
    x1 = tmp.x; y1 = tmp.y;
    tmp = get_pts(plane, p2);
    x2 = tmp.x; y2 = tmp.y;
    tmp = get_pts(plane, p3);
    x3 = tmp.x; y3 = tmp.y;
    
    cerr << "one_quadrant: plane " << plane << "center(" << xc << "," << yc << ")" << endl;
    cerr << "one_quadrant: p1(" << x1 << "," << y1 << "," << ")" << endl;
    cerr << "one_quadrant: p2(" << x2 << "," << y2 << "," << ")" << endl;
    cerr << "one_quadrant: p3(" << x3 << "," << y3 << "," << ")" << endl;
    typedef pair<int,int> quadrant;
    set<quadrant> signs;
    signs.insert(quadrant(sign(x1-xc), sign(y1-yc)));
    signs.insert(quadrant(sign(x2-xc), sign(y2-yc)));
    signs.insert(quadrant(sign(x3-xc), sign(y3-yc)));
    signs.insert(quadrant(sign(x1-xc), sign(y1-yc)));
    signs.insert(quadrant(sign(x2-xc), sign(y2-yc)));
    signs.insert(quadrant(sign(x3-xc), sign(y3-yc)));

    if (signs.size() == 1) {
        cerr << "one_quadrant: result=true1" << endl;
        return true;
    }

    if (signs.count(quadrant(1,1))) {
        signs.erase(quadrant(1,0));
        signs.erase(quadrant(0,1));
    }
    if (signs.count(quadrant(1,-1))) {
        signs.erase(quadrant(1,0));
        signs.erase(quadrant(0,-1));
    }
    if (signs.count(quadrant(-1,1))) {
        signs.erase(quadrant(-1,0));
        signs.erase(quadrant(0,1));
    }
    if (signs.count(quadrant(-1,-1))) {
        signs.erase(quadrant(-1,0));
        signs.erase(quadrant(0,-1));
    }
    
    if (signs.size() == 1) {
        cerr << "one_quadrant: result=true2" << endl;
        return true;
    }
    cerr << "one_quadrant: result=false" << endl;
    return false;
}

#define PI 3.141592654
bool arc_dir(int plane, Point2f& c, Point3f& p1, Point3f& p2, Point3f& p3) {
    float xc,yc,x1,y1,x2,y2,x3,y3,theta_start,theta_mid,theta_end;
    Point2f tmp;
    xc = c.x; yc = c.y;
    tmp = get_pts(plane, p1);
    x1 = tmp.x; y1 = tmp.y;
    tmp = get_pts(plane, p2);
    x2 = tmp.x; y2 = tmp.y;
    tmp = get_pts(plane, p3);
    x3 = tmp.x; y3 = tmp.y;

    theta_start = atan2(y1-yc, x1-xc);
    theta_mid = atan2(y2-yc, x2-xc);
    theta_end = atan2(y3-yc, x3-xc);

    if (theta_mid < theta_start) {
        theta_mid += 2*PI;
    }
    while (theta_end < theta_mid) {
        theta_end += 2*PI;
    }

    return theta_end < 2*PI;
}

float arc_dist(int plane, Point2f& cr, Point3f& p, float radius) {
    switch (plane) {
        case 17:
            return abs(hypot(cr.x - p.x, cr.y - p.y) - radius);
        case 18:
            return abs(hypot(cr.x - p.x, cr.y - p.z) - radius);
        case 19:
            return abs(hypot(cr.x - p.y, cr.y - p.z) - radius);
    }
}

string arc_fmt(int plane, Point2f& cr, Point3f& p) {
    stringstream result;

    result << setiosflags(ios::fixed) << setprecision(6);
    switch (plane) {
        case 17:
            result << " I" << (cr.x-p.x) << " J" << (cr.y-p.y);
            break;
        case 18:
            result << " I" << (cr.x-p.x) << " K" << (cr.y-p.z);
            break;
        case 19:
            result << " J" << (cr.x-p.y) << " K" << (cr.y-p.z);
            break;
    }
    return result.str();
}

//Perform Douglas-Peucker simplification on the path 'st' with the specified
//tolerance.  The 'index' and 'first' argument is for internal use only.
//
//The Douglas-Peucker simplification algorithm finds a subset of the input points
//whose path is never more than 'tolerance' away from the original input path.
//
//If 'plane' is specified as 17, 18, or 19, it may find helical arcs in the given
//plane in addition to lines.  Note that if there is movement in the plane
//perpendicular to the arc, it will be distorted, so 'plane' should usually
//be specified only when there is only movement on 2 axes
MovesVector_t*
douglas( 
    float tolerance, \
    int plane, \
    Point3fList::iterator begin,
    Point3fList::iterator end,
    bool first = true) {

    MovesVector_t* result = new MovesVector_t;

    if (distance(begin, end) == 1) {
       result->push_back(Move(*begin));
       return result;
    }

    Point3f ps = *begin;
    Point3f pe = *(end-1);
    if(ps == pe) { cerr << "DP: Endpoints are equal!" << endl; }

    vector<float> d_v, r_v;
    for (Point3fList::iterator p = begin; p != end; ++p) {
        d_v.push_back(dist_lseg(ps, pe, *p));
        r_v.push_back(arc_rad(plane, ps, *p, pe));
    }
    int worst_dist_i = distance(d_v.begin(), max_element(d_v.begin(), d_v.end()));
    float worst_dist = d_v.at(worst_dist_i);
    float min_radius = min(FLT_MAX, *min_element(r_v.begin(), r_v.end()));
    int arc_i = min_radius < FLT_MAX ? \
        distance(r_v.begin(), min_element(r_v.begin(), r_v.end()))-1 : \
        distance(begin, end)-1;

    float worst_arc_dist = FLT_MAX;
    Point2f cr = arc_center(plane, ps, begin[arc_i], pe);
    if (min_radius < FLT_MAX) {
        if (one_quadrant(plane, cr, ps, begin[arc_i], pe)) {
            vector<float> arcdists;
            for (Point3fList::iterator p = begin; p != end; ++p) {
                arcdists.push_back(arc_dist(plane, cr, *p, min_radius));
            }
            worst_arc_dist = *max_element(arcdists.begin(), arcdists.end());
        }
    }

    bool ccw;
    if (worst_arc_dist < tolerance and worst_arc_dist < worst_dist) {
        ccw = arc_dir(plane, cr, ps, begin[arc_i], pe);
        if (plane == 18) { ccw = not ccw; } // wtf?
        result->push_back(Move(ps));
        if (ccw) {
            result->push_back(Move(pe).GC("G03").Center(arc_fmt(plane, cr, ps)));
        } else {
            result->push_back(Move(pe).GC("G02").Center(arc_fmt(plane, cr, ps)));
        }
    } else if (worst_dist > tolerance) {
        if (first) { result->push_back(Move(ps)); }
        MovesVector_t* tmp;
        tmp = douglas(tolerance, plane, begin, begin+worst_dist_i, false);
        result->reserve(result->size() + tmp->size());
        result->insert(result->end(), tmp->begin(), tmp->end());
        tmp->clear();
        result->push_back(Move(begin[worst_dist_i]));
        tmp = douglas(tolerance, plane, begin+worst_dist_i, end, false);
        result->reserve(result->size() + tmp->size());
        result->insert(result->end(), tmp->begin(), tmp->end());
        tmp->clear();
        delete tmp;
        if (first) { result->push_back(Move(pe)); }
    } else {
        if (first) {
            result->push_back(Move(ps));
            result->push_back(Move(pe));
        }
    }
   return result;
}

Gcode::Gcode(float homeheight, \
        float safetyheight, \
        float tolerance, \
        float spindle_speed, \
        string units, \
        ostream& of) : \
    m_homeheight(homeheight), \
    m_safetyheight(safetyheight), \
    m_tolerance(tolerance), \
    m_speed(spindle_speed), \
    m_units(units) {
        m_of = &of;
        plane = 17;
//        cerr << "Gcode: We were given a tolerance of " << m_tolerance << endl;
}

void Gcode::set_plane(int p) {
    if (p != plane) {
        plane = p;
        *m_of << "G" << p << endl;
    }
}

void Gcode::set_feed(float f) {
    this->flush();
    *m_of << "F" << f << endl;
}

void Gcode::begin() {
    *m_of << m_units << endl;
    *m_of << "G00 Z" << m_safetyheight << endl;
    *m_of << "G17 G40" << endl;
    *m_of << "G80 G90 G94" << endl;
    *m_of << "S" << m_speed << " M3" << endl;
    *m_of << "G04 P3" << endl;
}

void Gcode::flush() {
    cerr << "flush: flushing " << cuts.size() << " cuts" << endl;
    *m_of << setiosflags(ios::fixed) << setprecision(6);
    MovesVector_t *moves;
    if (cuts.size()) { // no moves, do nothing
        Point3f ps = cuts.front();
        Point3f pe = cuts.back();
        if (ps == pe and cuts.size() > 1) { // endpoints are equal and we have multiple moves
            // so let's split
            cerr << "flush: Same endpoints, splitting vector length of " << cuts.size() << endl;
            int half = cuts.size() >> 1;
            cerr << "flush: half = " << half << endl;
            moves = douglas(m_tolerance, plane, cuts.begin(), cuts.begin()+half);
            MovesVector_t *tmp = douglas(m_tolerance, plane, cuts.begin()+half, cuts.end());
            moves->insert(moves->end(), tmp->begin(), tmp->end());
        } else {
            moves = douglas(m_tolerance, plane, cuts.begin(), cuts.end());
        }
        for (MovesVector_t::iterator m = moves->begin(); m != moves->end(); ++m) {
            Move t = *m;
            if (t.center.size()) {
                *m_of << t.gc << " X" << t.x << " Y" << t.y << " Z" << t.z << t.center << endl;
                m_lastgc = "";
                m_lastx = t.x;
                m_lasty = t.y;
                m_lastz = t.z;
            } else {
                this->move_common(t, "G01");
            }
        }
        moves->clear();
        delete moves;
    }
    cuts.clear();
}

void Gcode::end() {
    flush();
    safety();
    *m_of << "M2" << endl;
}

void Gcode::exactpath() {
    *m_of << "G61" << endl;
}

void Gcode::continuous(float t) {
    if (t > 0.0) {
        *m_of << "G64 P" << t << endl;
    } else {
        *m_of << "G64" << endl;
    }
}

void Gcode::rapid(Move& move) {
    flush();
    move_common(move, "G00");
}

void Gcode::move_common(Move& move, string gc) {
    float x,y,z;
    stringstream ss;
    ss << setiosflags(ios::fixed) << setprecision(6);
    string s;
    x = move.nx ? move.x : m_lastx;
    y = move.ny ? move.y : m_lasty;
    z = move.nz ? move.z : m_lastz;
//    if (isnan(x) or isnan(y) or isnan(z)) { cerr << "Gcode::move_common: NaN detected." << endl; }
    if (!isnan(x) and !isnan(m_lastx) and x != m_lastx) { ss << " X" << x; }
    if (!isnan(y) and !isnan(m_lasty) and y != m_lasty) { ss << " Y" << y; }
    if (!isnan(z) and !isnan(m_lastz) and z != m_lastz) { ss << " Z" << z; }
    if (!isnan(x) and x != m_lastx) { m_lastx = x; }
    if (!isnan(y) and y != m_lasty) { m_lasty = y; }
    if (!isnan(z) and z != m_lastz) { m_lastz = z; }
    s = ss.str();
    if (s.size()) {
        if (gc != m_lastgc) {
            *m_of << gc;
            m_lastgc = gc;
        }
        *m_of << s << endl;
    }
}

void Gcode::cut(Move& move) {
    float x,y,z,lx,ly,lz,dx,dy,dz;
    if (cuts.size()) {
        Point3f t = cuts.back();
        lx = t.x; ly = t.y; lz = t.z;
    } else {
        lx = m_lastx; ly = m_lasty; lz = m_lastz;
    }
    x = move.nx ? move.x : lx;
    y = move.ny ? move.y : ly;
    z = move.nz ? move.z : lz;
    dx = abs(lx - x);
    dy = abs(ly - y);
    dz = abs(lz - z);
    // if this move is greater than the tolerance size in any direction
    // flush out the previous moves and then add it to the queue.
    if (dx > m_tolerance or dy > m_tolerance or dz > m_tolerance) {
        flush();
    }
    cuts.push_back(Point3f(x,y,z));
}

void Gcode::home() {
    flush();
    rapid(Move().Z(m_homeheight).Center(" (home height)"));
}

void Gcode::safety() {
    flush();
    rapid(Move().Z(m_safetyheight).Center(" (safety height)"));
}
