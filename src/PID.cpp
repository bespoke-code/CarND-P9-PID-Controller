#include <cstdio>
#include <vector>
#include "PID.h"

using namespace std;

/*
* Complete the PID class.
*/

PID::PID() {
    p_error = 0;
    d_error = 0;
    i_error = 0;
    previous_d_error = 0;

    Kp = 0.1;
    Kd = 1;
    Ki = 0.01;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    p_error = cte;
    i_error += cte;
    d_error = cte - previous_d_error;
    previous_d_error = cte;
}

double PID::TotalError() {
    return -Kp*p_error - Kd*d_error - Ki*i_error;
}
