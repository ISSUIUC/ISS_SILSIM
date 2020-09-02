
#ifndef _VECTOR3_H_
#define _VECTOR3_H_


class Vector3 {
public:
    double x;
    double y;
    double z;

	Vector3(): x(0), y(0), z(0) {};
	Vector3(double _x, double _y, double _z): x(_x), y(_y), z(_z) {};
	Vector3(const Vector3& rhs);

	Vector3 operator+(const Vector3& rhs) const;
	Vector3 operator-(const Vector3& rhs) const;
	Vector3 operator-() const;
	Vector3 operator*(double scalar) const;
	Vector3 operator/(double scalar) const;
	Vector3& operator+=(const Vector3& rhs);
	Vector3& operator-=(const Vector3& rhs);
	Vector3& operator*=(double scalar);
	Vector3& operator/=(double scalar);
	Vector3& operator=(const Vector3& rhs);

	Vector3 cross (const Vector3& rhs) const;
	double dot (const Vector3& rhs) const;

	double magnitude() const;
	double magnitude2() const;
	void normalize();
	Vector3 normalized();

};

Vector3 operator*(double scalar, const Vector3& rhs);

#endif
