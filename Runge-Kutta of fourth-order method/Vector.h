#pragma once
/*
����� ������
����� ����� ���������� ������� ���� ������ ��������� � ����� ������
������������ �������� ���������, �������� �� ���������
*/
class Vector {
public:
    double theta, omega, I;
    //����������� ������� ������
    Vector();
    //����������� � �����������
    Vector(double theta_, double omega_, double I_);
    //����� �������
    void out();
};

Vector operator +(Vector a, Vector b);
Vector operator -(Vector a, Vector b);
Vector operator *(double a, Vector b);
Vector operator *(Vector a, Vector b);