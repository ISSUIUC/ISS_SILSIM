//
// Created by 16182 on 1/31/2022.
//

#ifndef SILSIM_PARACHUTE_H
#define SILSIM_PARACHUTE_H

#include <Eigen/Dense>

class DrogueParachute {
   public:
    DrogueParachute(double drag_coefficient, double area):
          drag_coefficient_(drag_coefficient), area_(area), is_open_(false){}

    void update(Eigen::Vector3d position, Eigen::Vector3d velocity);

    bool is_open() const {
        return is_open_;
    }
    double drag_coefficient() const {
        return drag_coefficient_;
    }
    double area() const {
        return area_;
    }

   private:
    double drag_coefficient_;
    double area_;
    bool is_open_;
};


class MainParachute {
   public:
    MainParachute(double drag_coefficient, double area, double deploy_altitude):
    deploy_altitude_(deploy_altitude),
    drag_coefficient_(drag_coefficient), area_(area), is_open_(false){}

    void update(Eigen::Vector3d position, Eigen::Vector3d velocity);

    bool is_open() const {
        return is_open_;
    }
    double drag_coefficient() const {
        return drag_coefficient_;
    }
    double area() const {
        return area_;
    }

   private:
    double deploy_altitude_;
    double drag_coefficient_;
    double area_;
    bool is_open_;
};

#endif  // SILSIM_PARACHUTE_H
