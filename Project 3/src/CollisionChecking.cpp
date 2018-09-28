///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Qiao Zhu
//////////////////////////////////////

#include "CollisionChecking.h"

// TODO: Copy your implementation of Project 2!

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // Iterate over all obstacles to check whether the point is collided with each obstacle.
    for (unsigned int i = 0; i < obstacles.size(); ++i)
    {
        Rectangle cur = obstacles[i]; // Pick up current obstacle for following checking.
        // Is point (x, y) in the current obstacle?
        // It is true when (xmin <= x <= xmax) and (ymin <= y <= ymax).
        if (x >= cur.x && x - cur.width <= cur.x && y >= cur.y && y - cur.height <= cur.y)
        {
            return false; // One collision means invalid.
        }
    }

    // It is true only if there is no collosion.
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // Iterate over all obstacles to check whether the point is collided with each obstacle.
    for(unsigned int i = 0; i < obstacles.size(); ++i)
    {
        Rectangle cur = obstacles[i]; // Pick up current obstacle for following checking.

        // Is circle (x, y, r) in the current obstacle?
        // Case 1: It is true when (xmin - r <= x <= xmax + r) and (ymin <= y <= ymax).
        if (x >= cur.x - radius && x - cur.width - radius <= cur.x && y >= cur.y && y - cur.height <= cur.y)
        {
            return false;
        }
        // Case 2: It is true when (xmin <= x <= xmax) and (ymin - r <= y <= ymax + r).
        if (x >= cur.x && x - cur.width <= cur.x && y >= cur.y - radius && y - cur.height - radius <= cur.y)
        {
            return false;
        }
        // Case 3: It is true when ||(x, y) - (x_e, y_e)|| <= r, that e =(x_e, y_e) is a vertex of rectangle.
        if ((x - cur.x) * (x - cur.x) + (y - cur.y) * (y - cur.y) <= radius * radius ||
            (x - cur.x - cur.width) * (x - cur.x - cur.width) + (y - cur.y) * (y - cur.y) <= radius * radius ||
            (x - cur.x) * (x - cur.x) + (y - cur.y - cur.height) * (y - cur.y - cur.height) <= radius * radius ||
            (x - cur.x - cur.width) * (x - cur.x - cur.width) + (y - cur.y - cur.height) * (y - cur.y - cur.height) <= radius * radius)
        {
            return false;
        }
    }
    // It is true only if there is no collosion.
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
	// Define the 4 points of the robot.
	Point a, b, c, d;
	double half = sideLength / 2;
	a.x = a.y = b.y = d.x = -half;
	b.x = c.x = c.y = d.y = half;
	// Rotate each point by theta (radian) counterclockwise.
	rotate(&a, theta);
	rotate(&b, theta);
	rotate(&c, theta);
	rotate(&d, theta);
	// Translate each point by (x, y).
	translate(&a, x, y);
	translate(&b, x, y);
	translate(&c, x, y);
	translate(&d, x, y);

    // Iterate over all obstacles to check whether the point is collided with each obstacle.
    for (unsigned int i = 0; i < obstacles.size(); ++i)
    {
        Rectangle cur = obstacles[i]; // Pick up current obstacle for following checking.

        // True if at least one of points a, b, c, d is in rectangle cur.
        if (inRectangle(&a, &cur) || inRectangle(&b, &cur) || inRectangle(&c, &cur) || inRectangle(&d, &cur))
        {
        	return false; // One collision means invalid.
        }

        // Generate 4 points for rectangle cur.
        Point oa, ob, oc, od;
        recPoints(&cur, &oa, &ob, &oc, &od);

        // Check whether the robot contains the whole obstacle.
        if ((robotContain(&a, &b, &c, &oa) && robotContain(&b, &c, &d, &oa) && robotContain(&c, &d, &a, &oa) && robotContain(&d, &a, &b, &oa)) ||
        	(robotContain(&a, &b, &c, &ob) && robotContain(&b, &c, &d, &ob) && robotContain(&c, &d, &a, &ob) && robotContain(&d, &a, &b, &ob)) ||
			(robotContain(&a, &b, &c, &oc) && robotContain(&b, &c, &d, &oc) && robotContain(&c, &d, &a, &oc) && robotContain(&d, &a, &b, &oc)) ||
			(robotContain(&a, &b, &c, &od) && robotContain(&b, &c, &d, &od) && robotContain(&c, &d, &a, &od) && robotContain(&d, &a, &b, &od)))
        {
        	return false; // One collision means invalid.
        }

        // Check intersection.
        if (intersect(&a, &b, &oa, &ob) || intersect(&a, &b, &ob, &oc) || intersect(&a, &b, &oc, &od) || intersect(&a, &b, &od, &oa) ||
        	intersect(&b, &c, &oa, &ob) || intersect(&b, &c, &ob, &oc) || intersect(&b, &c, &oc, &od) || intersect(&b, &c, &od, &oa) ||
			intersect(&c, &d, &oa, &ob) || intersect(&c, &d, &ob, &oc) || intersect(&c, &d, &oc, &od) || intersect(&c, &d, &od, &oa) ||
			intersect(&d, &a, &oa, &ob) || intersect(&d, &a, &ob, &oc) || intersect(&d, &a, &oc, &od) || intersect(&d, &a, &od, &oa)) {
			return false; // One collision means invalid.
		}

    }
    // It is true only if there is no collosion.
    return true;
}

// Check whether the robot contains the whole obstacle.
bool robotContain(const Point *r1, const Point *r2, const Point *r3, const Point *o)
{
	Point l, m, r;
	lineVector(&l, r2, r1);
	lineVector(&m, r2, o);
	lineVector(&r, r2, r3);

	if (crossSign(&l, &m) * crossSign(&r, &m) == -1)
	{
		return true;
	}
	return false;
}

// Generate 4 points for rectangle cur.
void recPoints(const Rectangle *cur,
			   Point *oa, Point *ob, Point *oc, Point *od)
{
	oa->x = od->x = cur->x;
	oa->y = ob->y = cur->y;
	ob->x = oc->x = (cur->x + cur->width);
	oc->y = od->y = (cur->y + cur->height);
}

// Check if line a1b1 intersects line a2b2
bool intersect(const Point *a1, const Point *b1, const Point *a2, const Point *b2)
{
	// Check 1: Roughly detect by range first.
	// Calculate min and max for x and y of each line.
	double minx1 = 0, minx2 = 0, miny1 = 0, miny2 = 0, maxx1 = 0, maxx2 = 0, maxy1 = 0, maxy2 = 0;
	// for x of a1b1: min and max
	if (a1->x < b1->x)
	{
		minx1 = a1->x;
		maxx1 = b1->x;
	}
	else
	{
		minx1 = b1->x;
		maxx1 = a1->x;
	}
	// for y of a1b1: min and max
	if (a1->y < b1->y)
	{
		miny1 = a1->y;
		maxy1 = b1->y;
	}
	else
	{
		miny1 = b1->y;
		maxy1 = a1->y;
	}
	// for x of a2b2: min and max
	if (a2->x < b2->x)
	{
		minx2 = a2->x;
		maxx2 = b2->x;
	}
	else
	{
		minx2 = b2->x;
		maxx2 = a2->x;
	}
	// for y of a2b2: min and max
	if (a2->y < b2->y)
	{
		miny2 = a2->y;
		maxy2 = b2->y;
	}
	else
	{
		miny2 = b2->y;
		maxy2 = a2->y;
	}

	// Check if max < min
	if (minx1 > maxx2 || miny1 > maxy2 || minx2 > maxx1 || miny2 > maxy1)
	{
		return false;
	}


	// Case 2: Spanning test
	Point a2a1, a2b2, a2b1;
	lineVector(&a2a1, a2, a1);
	lineVector(&a2b2, a2, b2);
	lineVector(&a2b1, a2, b1);
	// Test whether a2b2 spans across a1b1.
	if (crossSign(&a2a1, &a2b2) * crossSign(&a2b1, &a2b2) > 0)
	{
		return false; // false if not spans across
	}
	Point a1a2, a1b1, a1b2;
	lineVector(&a1a2, a1, a2);
	lineVector(&a1b1, a1, b1);
	lineVector(&a1b2, a1, b2);
	// Test whether a1b1 spans across a2b2.
	if (crossSign(&a1a2, &a1b1) * crossSign(&a1b2, &a1b1) > 0)
	{
		return false; // false if not spans across
	}

	return true;
}

// Generate vector p1p2 by two points p1, p2.
void lineVector(Point *p1p2, const Point *p1, const Point *p2)
{
	p1p2->x = p2->x - p1->x;
	p1p2->y = p2->y - p1->y;
}

// Cross Sign: sign(||p1 cross p2||).
int crossSign(const Point *p1, const Point *p2)
{
	double flag = p1->x * p2->y - p1->y * p2->x;
	if (flag == 0)
	{
		return 0;
	}
	if (flag < 0)
	{
		return -1;
	}
	return 1;
}

// Cross Sign: sign(||(x1, y1) cross (x2, y2)||).
int crossSign(double x1, double y1, double x2, double y2)
{
	double flag = x1 * y2 - y1 * x2;
	if (flag == 0)
	{
		return 0;
	}
	if (flag < 0)
	{
		return -1;
	}
	return 1;
}

// Check if point p is in rectangle cur.
bool inRectangle(const Point *p, const Rectangle *cur)
{
	if (p->x >= cur->x && p->x - cur->width <= cur->x &&
		p->y >= cur->y && p->y - cur->height <= cur->y)
	{
		return true; // One collision means invalid.
	}
	return false;
}

// Check if point (x, y) is in rectangle cur.
bool inRectangle(double x, double y, const Rectangle *cur)
{
	if (x >= cur->x && x - cur->width <= cur->x &&
		y >= cur->y && y - cur->height <= cur->y)
	{
		return true; // One collision means invalid.
	}
	return false;
}

// Translate point p by (x, y).
void translate(Point *p, double x, double y)
{
	p->x += x;
	p->y += y;
}

// Rotate point p by theta (radian) counterclockwise.
void rotate(Point *p, double theta)
{
	double x_temp = p->x * cos(theta) - p->y * sin(theta);
	p->y = p->x * sin(theta) + p->y * cos(theta);
	p->x = x_temp;
}
