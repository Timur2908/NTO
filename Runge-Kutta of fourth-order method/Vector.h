#pragma once
/*
Класс ветора
Имеет набор параметров который можн задать массивать в общем случае
Поддерживает операции умножения, сложения ит вычитания
*/
class Vector {
public:
    double theta, omega, I;
    //Конструктор пустого класса
    Vector();
    //Конструктор с параметрами
    Vector(double theta_, double omega_, double I_);
    //Вывод вектора
    void out();
};

Vector operator +(Vector a, Vector b);
Vector operator -(Vector a, Vector b);
Vector operator *(double a, Vector b);
Vector operator *(Vector a, Vector b);