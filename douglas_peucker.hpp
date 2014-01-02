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

#ifndef DOUGLAS_PEUCKER_8Z613VV3

#define DOUGLAS_PEUCKER_8Z613VV3

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <float.h>

using namespace std;

class Point2f {
public:
    Point2f () {}
    Point2f (float a, float b): x(a), y(b) {}
//    virtual ~Point2f ();

    float x;
    float y;
    Point2f operator-(const Point2f &rhs) const { return Point2f(x - rhs.x, y - rhs.y); }
    Point2f operator+(const Point2f &rhs) const { return Point2f(x + rhs.x, y + rhs.y); }
    Point2f operator*(const float &rhs) const { return Point2f(x * rhs, y * rhs); }
    float cross(const Point2f &other) const { return x * other.y + y * other.x; }
    float dot(const Point2f &other) const { return x * other.x + y * other.y; }
    float mag() { return hypot(x, y); }
    float mag2() { return pow(x, 2) + pow(y, 2); }

};

struct Point3f {
    float x, y, z;
    Point3f(float a, float b, float c) : x(a), y(b), z(c) {}
    bool operator==(const Point3f &rhs) const { return x==rhs.x and y==rhs.y and z==rhs.z; }
};



class Move { // used as both Named Parameter Idiom for moves, and for getting data back from douglas()
public:
    Move (): nx(false), ny(false), nz(false) {}
    Move (Point3f& p): nx(true), ny(true), nz(true), \
        x(p.x), y(p.y), z(p.z) {}
    virtual ~Move () {}

    float x, y, z, i, j, k;
    bool nx, ny, nz, ni, nj, nk;
    string gc, center;

    Move& X(float value) { x=value; nx=true; return *this;}
    Move& Y(float value) { y=value; ny=true; return *this;}
    Move& Z(float value) { z=value; nz=true; return *this;}
    Move& Center(string value) { center=value; return *this;}
    Move& GC(string value) { gc=value; return *this;}
};

// all points and moves stored in vectors are implicitly G01/2/3
// rapids are flushed to output immediately and thus not stored
typedef vector<Move> MovesVector_t;
typedef vector<Point3f> Point3fList;

class Gcode {
public:
    Gcode (float homeheight = 1.5, \
            float safetyheight = 0.04, \
            float tolerance = 0.001, \
            float spindle_speed = 1000, \
            string units = "G20", \
            ostream& of = cout);
            
    virtual ~Gcode () {
        cuts.clear();
    }

    void set_plane(int p);
    void begin();
    void flush();
    void end();
    void exactpath();
    void continuous(float tolerance);
    void rapid(Move& move);
    void set_feed(float);
    void cut(Move& move);
    void home();
    void safety();
private:
    /* data */
    float m_lastx;
    float m_lasty;
    float m_lastz;
    string m_lastgc;
    float m_homeheight;
    float m_safetyheight;
    float m_tolerance;
    float m_speed;
    int plane;
    ostream* m_of;
    string m_units;
    Point3fList cuts;

    void move_common(Move& move, string gcode);
};

#endif /* end of include guard: DOUGLAS_PEUCKER_8Z613VV3 */
