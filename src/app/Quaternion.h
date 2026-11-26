#pragma once

#include <algorithm>
#include <cmath>

// Minimal unit quaternion q = w + xi + yj + zk for camera rotation.
//   rotate v:   v' = q v q*
//   compose:    (a * b) v = a (b v)   (apply b first, then a)
struct Quat
{
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    static Quat identity() { return {}; }

    // rotation by 'angle' [rad] about the unit axis a
    static Quat axisAngle(const double a[3], double angle)
    {
        const double s = std::sin(0.5 * angle);
        return { std::cos(0.5 * angle), a[0] * s, a[1] * s, a[2] * s };
    }

    static Quat aboutZ(double angle)
    {
        const double zAxis[3] = { 0.0, 0.0, 1.0 };
        return axisAngle(zAxis, angle);
    }

    // Hamilton product
    Quat operator*(const Quat& q) const
    {
        return { w * q.w - x * q.x - y * q.y - z * q.z,
                 w * q.x + x * q.w + y * q.z - z * q.y,
                 w * q.y - x * q.z + y * q.w + z * q.x,
                 w * q.z + x * q.y - y * q.x + z * q.w };
    }

    Quat conj() const { return { w, -x, -y, -z }; }   // inverse of a unit quaternion

    double dot(const Quat& q) const { return w * q.w + x * q.x + y * q.y + z * q.z; }

    Quat normalized() const
    {
        const double n = std::sqrt(dot(*this));
        return n > 0.0 ? Quat{ w / n, x / n, y / n, z / n } : Quat{};
    }

    // v' = q v q*  (expanded form: v + 2w (u x v) + 2 u x (u x v), u = (x, y, z))
    void rotate(const double v[3], double out[3]) const
    {
        const double u[3] = { x, y, z };
        const double c1[3] = { u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0] };
        const double c2[3] = { u[1] * c1[2] - u[2] * c1[1], u[2] * c1[0] - u[0] * c1[2], u[0] * c1[1] - u[1] * c1[0] };
        for (int i = 0; i < 3; ++i)
            out[i] = v[i] + 2.0 * (w * c1[i] + c2[i]);
    }

    // shortest-arc rotation taking unit vector a onto unit vector b
    static Quat fromTo(const double a[3], const double b[3])
    {
        const double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        const double c[3] = { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        // q = (1 + d, a x b) normalized == rotation by acos(d) about a x b
        return Quat{ 1.0 + d, c[0], c[1], c[2] }.normalized();
    }

    // spherical linear interpolation, constant angular speed, shortest path
    static Quat slerp(const Quat& a, Quat b, double t)
    {
        double d = a.dot(b);
        if (d < 0.0)   // q and -q are the same rotation: take the short way
        {
            b = { -b.w, -b.x, -b.y, -b.z };
            d = -d;
        }
        if (d > 0.9995)   // nearly identical: linear interpolation is fine
            return Quat{ a.w + t * (b.w - a.w), a.x + t * (b.x - a.x),
                         a.y + t * (b.y - a.y), a.z + t * (b.z - a.z) }.normalized();
        const double theta = std::acos(std::clamp(d, -1.0, 1.0));
        const double s = std::sin(theta);
        const double ka = std::sin((1.0 - t) * theta) / s;
        const double kb = std::sin(t * theta) / s;
        return { ka * a.w + kb * b.w, ka * a.x + kb * b.x, ka * a.y + kb * b.y, ka * a.z + kb * b.z };
    }
};

// Shoemake arcball: map a point in normalized screen coords ([-1, 1], y up)
// onto the unit sphere (inside) or its silhouette circle (outside).
inline void arcballVector(double sx, double sy, double out[3])
{
    const double r2 = sx * sx + sy * sy;
    if (r2 <= 1.0)
    {
        out[0] = sx; out[1] = sy; out[2] = std::sqrt(1.0 - r2);
    }
    else
    {
        const double r = std::sqrt(r2);
        out[0] = sx / r; out[1] = sy / r; out[2] = 0.0;
    }
}
