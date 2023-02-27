#include <cstdio>
#include <vector>
#include <iostream>
#include "Vector.h"
using namespace std;

//Количтво шагов моделирования
const int SIZE = 1000;
//Основный параметры двигателя
const double R = 0.960490;      //Сопротивление Ом
const double L = 0.600528;      //Индуктивность
const double k = 0.572761;      //Конструктивная постоянная
const double k_f = 0.630305;    //Коэффициент трения
const double J = 0.917572;      //Момент инерции

//Функция пересчёта - сама система дифференциальных уравнений
//В данном сучае модель двигателя постоянниго тока с нагрузкой
Vector F(Vector U, double u, double taught, double t) {
    Vector res;
    res.theta = U.omega;
    res.omega = (k * U.I - k_f * U.omega - taught) / J;
    res.I = (-R * U.I - k * U.omega + u) / L;
    return res;
}

int main() {
    double taught = 0.0;    //Момент нагрузки Н\м

    double t = 0.0;         //Время
    double dt = 0.01;       //Шаг по времени в секундах

    double u = 5.0;           //Управляющее напряжения в вольтах

    int n = SIZE;           //Количество шагов моделирования

    // Начальный условия
    double theta0 = 0.0;
    double omega0 = 0.0;
    double I0 = 0.0;
    
    Vector U(theta0, omega0, I0);   //Основной вектор управнения

    Vector k1, k2, k3, k4;          //Коэффициенты Рунге_Кутты

    //Вектор с выходными данными
    vector<Vector> ans;
    ans.push_back(U);

    for (int i = 1; i < n; i++) {
        //Выполняем решение системы уравнений
        k1 = dt * F(U, u, taught, t);
        k2 = dt * F(U + 0.5 * k1, u, taught, t + 0.5 * dt);
        k3 = dt * F(U + 0.5 * k2, u, taught, t + 0.5 * dt);
        k4 = dt * F(U + k3, u, taught, t + dt);
        U = U + 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        //Увеличиваем время
        t += dt;
        //Добаляем состояние к ответу
        ans.push_back(U);
    }
    //Выводим все состояния двигателя за перид моделирования
    for (auto i : ans) i.out();
    
    return 0;
}