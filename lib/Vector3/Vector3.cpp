#include "Vector3.h"

#include <math.h>

Vector3::Vector3(const Vector3& rhs) {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
}

Vector3 Vector3::operator+(const Vector3& rhs) const {
    return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
}

Vector3 Vector3::operator-(const Vector3& rhs) const {
    return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
}

Vector3 Vector3::operator-() const { return Vector3(-x, -y, -z); }

Vector3 Vector3::operator*(double scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
}

Vector3 operator*(double scalar, const Vector3& rhs) {
    return Vector3(rhs.x * scalar, rhs.y * scalar, rhs.z * scalar);
}

Vector3 Vector3::operator/(double scalar) const {
    return Vector3(x / scalar, y / scalar, z / scalar);
}

Vector3& Vector3::operator+=(const Vector3& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

Vector3& Vector3::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector3& Vector3::operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

Vector3& Vector3::operator=(const Vector3& rhs) {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    return *this;
}

Vector3 Vector3::cross(const Vector3& rhs) const {
    return Vector3(y * rhs.z - z * rhs.y, -(x * rhs.z - z * rhs.x),
                   x * rhs.y - y * rhs.x);
}

double Vector3::dot(const Vector3& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
}

double Vector3::magnitude() const {
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

double Vector3::magnitude2() const { return pow(x, 2) + pow(y, 2) + pow(z, 2); }

void Vector3::normalize() {
    double mag = magnitude();
    if (mag > 0.0) {
        (*this) /= magnitude();
    }
}

Vector3 Vector3::normalized() {
    double mag = magnitude();
    if (mag > 0.0) {
        return Vector3(x / mag, y / mag, z / mag);
    } else {
        return Vector3();
    }
}

void Vector3::randomize(std::default_random_engine& generator,
                        std::normal_distribution<double>& dist) {
    x = dist(generator);
    y = dist(generator);
    z = dist(generator);
}
