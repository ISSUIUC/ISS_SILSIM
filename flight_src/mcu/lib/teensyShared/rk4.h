#include <iostream>
#include <array>
#include <chrono>

using std::array;

class rk4 {
    public:
    
    array<float, 2> accel(array<float, 2> u, float rho);

    array<float, 2> rk4_step(array<float, 2> state, float dt, float rho);

    array<float, 2> sim_apogee(array<float, 2> state, float dt);

    private:

    array<float, 2> y1{0, 0};
    array<float, 2> y2{0, 0};
    array<float, 2> y3{0, 0};
    array<float, 2> y4{0, 0};
    array<float, 2> rk4_kp1{0, 0};
};