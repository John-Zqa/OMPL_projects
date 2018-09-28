#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Definition of Point.
struct Point
{
    double x, y;
};

// Check whether the robot contains the whole obstacle.
bool robotContain(const Point *r1, const Point *r2, const Point *r3, const Point *o);

// Generate 4 points for rectangle cur.
void recPoints(const Rectangle *cur, Point *oa, Point *ob, Point *oc, Point *od);

// Generate vector p1p2 by two points p1, p2.
void lineVector(Point *p1p2, const Point *p1, const Point *p2);

// Cross Sign: sign(||p1 cross p2||).
int crossSign(const Point *p1, const Point *p2);

// Cross Sign: sign(||(x1, y1) cross (x2, y2)||).
int crossSign(double x1, double y1, double x2, double y2);

// Check if line a1b1 intersects line a2b2
bool intersect(const Point *a1, const Point *b1, const Point *a2, const Point *b2);

// Check if point p is in rectangle cur.
bool inRectangle(const Point *p, const Rectangle *cur);

// Check if point p is in rectangle cur.
bool inRectangle(double x, double y, const Rectangle *cur);

// Translate point p by (x, y).
void translate(Point *p, double x, double y);

// Rotate point p by theta (radian) counterclockwise.
void rotate(Point *p, double theta);

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);

#endif
