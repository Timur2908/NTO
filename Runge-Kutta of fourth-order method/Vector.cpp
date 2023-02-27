#include "Vector.h"
#include<cstdio>

Vector::Vector() {
    theta = 0.0;
    omega = 0.0;
    I = 0.0;
}
Vector::Vector(double theta_, double omega_, double I_) {
    theta = theta_;
    omega = omega_;
    I = I_;
}
void Vector::out() {
    printf("Theta = %.6Lf ", theta);
    printf("Omega = %.6Lf ", omega);
    printf("I = %.6Lf\n", I);
}

Vector operator +(Vector a, Vector b) {
    Vector c;
    c.theta = a.theta + b.theta;
    c.omega = a.omega + b.omega;
    c.I = a.I + b.I;
    return c;
}
Vector operator -(Vector a, Vector b) {
    Vector c;
    c.theta = a.theta - b.theta;
    c.omega = a.omega - b.omega;
    c.I = a.I - b.I;
    return c;
}
Vector operator *(double a, Vector b) {
    Vector c;
    c.theta = a * b.theta;
    c.omega = a * b.omega;
    c.I = a * b.I;
    return c;
}
Vector operator *(Vector a, Vector b) {
    Vector c;
    c.theta = a.theta * b.theta;
    c.omega = a.omega * b.omega;
    c.I = a.I * b.I;
    return c;
}