#ifndef __DEF_H
#define __DEF_H

#include <cmath>
#include <iostream>

const double PIOVER180 = 0.017453292519943295769236907684886;

struct Triangle {
	int indices[3];
};

struct Point {
	double x, y, z;
};

struct Vector {
	double x, y, z;

    Vector& operator += (const Vector &v2){
	    this->x += v2.x;
        this->y += v2.y;
        this->z += v2.z;
	    return *this;
    }

	inline double norm() const {
		return sqrt( x * x + y * y + z * z);
	}

	inline bool normalize() {
		double n = norm();

		if( fabs(n) <= 1e-12 )
			return false;

		x /= n;
		y /= n;
		z /= n;

		return true;
	}

};

inline Point operator + (const Point&p, const Vector &v){
	Point p2={p.x + v.x, p.y + v.y, p.z + v.z };
	return p2;
}

inline Point operator - (const Point&p, const Vector &v){
	Point p2={p.x - v.x, p.y - v.y, p.z - v.z };
	return p2;
}

inline Point operator * (const Point&p, const float c){
	Point p2={p.x * c, p.y * c, p.z * c };
	return p2;
}

inline Point operator * (const float c, const Point&p){
	Point p2={p.x * c, p.y * c, p.z * c };
	return p2;
}

inline Point operator + (const Point &p, const Point &v){
	Point p2={p.x + v.x, p.y + v.y, p.z + v.z };
	return p2;
}


inline Vector operator + (const Vector&v1, const Vector &v2){
	Vector v={v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	return v;
}

inline Vector operator - (const Point&p1, const Point &p2){
	Vector v={p1.x - p2.x, p1.y - p2.y, p1.z - p2.z };
	return v;
}

inline Vector operator * (float c, const Vector &v)
{
	Vector v2={v.x *c, v.y * c, v.z * c };
	return v2;
}

inline Vector operator * (const Vector &v, float c)
{
	Vector v2={v.x *c, v.y * c, v.z * c };
	return v2;
}

inline Vector operator / (const Vector &v, double c)
{
	Vector v2 = { v.x / c, v.y / c, v.z / c };
	return v2;
}

inline Vector operator - (const Vector&v1, const Vector &v2){
	Vector v={v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
	return v;
}

inline double operator * (const Vector&v1, const Vector &v2 ) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline Vector operator % (const Vector&v1, const Vector &v2 ) {
	Vector v;
	v.x = v1.y * v2.z - v1.z * v2.y;
	v.y = v1.z * v2.x - v1.x * v2.z;
	v.z = v1.x * v2.y - v1.y * v2.x;
	return v;
}

struct color {
    enum OFFSET 
    {
        OFFSET_RED = 0,
        OFFSET_GREEN = 1,
        OFFSET_BLUE = 2,
        OFFSET_MAX  = 3
    };
    float red, green, blue;

    inline color & operator += (const color &c2 ) {
	    this->red +=  c2.red;
        this->green += c2.green;
        this->blue += c2.blue;
	    return *this;
    }

    inline float & getChannel(OFFSET offset )
    {
        return reinterpret_cast<float*>(this)[offset];
    }

    inline float getChannel(OFFSET offset ) const
    {
        return reinterpret_cast<const float*>(this)[offset];
    }
};

inline color operator * (const color&c1, const color &c2 ) {
	color c = {c1.red * c2.red, c1.green * c2.green, c1.blue * c2.blue};
	return c;
}

inline color operator + (const color&c1, const color &c2 ) {
	color c = {c1.red + c2.red, c1.green + c2.green, c1.blue + c2.blue};
	return c;
}

inline color operator * (float coef, const color &c ) {
	color c2 = {c.red * coef, c.green * coef, c.blue * coef};
	return c2;
}

struct Bounding_Box 
{
	double min_x, min_y, min_z;
	double max_x, max_y, max_z;
	double wx, wy, wz;
	double mid_x, mid_y, mid_z;

	Bounding_Box()
	{
		min_x = min_y = min_z = 100000000;
		max_x = max_y = max_z = -100000000;
	}

	void update(Point p)
	{
		if (p.x < min_x)
			min_x = p.x;

		if (p.y < min_y)
			min_y = p.y;

		if (p.z < min_z)
			min_z = p.z;

		if (p.x > max_x)
			max_x = p.x;

		if (p.y > max_y)
			max_y = p.y;

		if (p.z > max_z)
			max_z = p.z;

		wx = max_x - min_x;
		wy = max_y - min_y;
		wz = max_z - min_z;

		mid_x = min_x + wx / 2.0;
		mid_y = min_y + wy / 2.0;
		mid_z = min_z + wz / 2.0;
	}

	void display()
	{
		std::cout << "min: " << min_x << " " << min_y << " " << min_z << std::endl;
		std::cout << "max: " << max_x << " " << max_y << " " << max_z << std::endl;
	}
};

#endif //__DEF_H
