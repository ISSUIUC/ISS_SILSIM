#include <string>
#include <vector>
#include <iostream>

#include "Rocket.h"
#include "Sensor.h"
#include "quaternion.h"
#include "Vector3.h"

Rocket::Rocket() {

	std::cout << "Rocket custom constructor" << std::endl;

	_r_vect = Vector3();
	_r_dot = Vector3();
	_r_ddot = Vector3();

	_q_ornt = Quaternion<double>(1, 0, 0, 0);

	_w_vect = Vector3();
	_w_dot = Vector3();

	_f_net = Vector3();
	_t_net = Vector3();
}

void Rocket::get_r_vect(Vector3& vector) const {
	vector = _r_vect;
}

void Rocket::set_r_vect(Vector3& vector) {
	_r_vect = vector;
}

void Rocket::get_r_dot(Vector3& vector) const {
	vector = _r_dot;
}

void Rocket::set_r_dot(Vector3& vector) {
	_r_dot = vector;
}

void Rocket::get_r_ddot(Vector3& vector) const {
	vector = _r_ddot;
}

void Rocket::set_r_ddot(Vector3& vector) {
	_r_ddot = vector;
}

void Rocket::get_q_ornt(Quaternion<double>& quatrn) const {
	quatrn = _q_ornt;
}

void Rocket::set_q_ornt(Quaternion<double>& quatrn) {
	_q_ornt = quatrn;
}

void Rocket::get_w_vect(Vector3& vector) const {
	vector = _w_vect;
}

void Rocket::set_w_vect(Vector3& vector) {
	_w_vect = vector;
}

void Rocket::get_w_dot(Vector3& vector) const {
	vector = _w_dot;
}

void Rocket::set_w_dot(Vector3& vector) {
	_w_dot = vector;
}

void Rocket::get_f_net(Vector3& vector) const {
	vector = _f_net;
}

void Rocket::set_f_net(Vector3& vector) {
	_f_net = vector;
}

void Rocket::get_t_net(Vector3& vector) const {
	vector = _t_net;
}

void Rocket::set_t_net(Vector3& vector) {
	_t_net = vector;
}

void Rocket::get_mass(double& mass) const {
	mass = _mass;
}

void Rocket::set_mass(double& mass) {
	_mass = mass;
}

void Rocket::get_Cd(double& Cd) const {
	Cd = _Cd;
}

void Rocket::set_Cd(double& Cd) {
	_Cd = Cd;
}

void Rocket::inertial2rocket(Vector3& vector) {
	Quaternion<double> p(0, vector.x, vector.y, vector.z);
	p = (_q_ornt.conj() * p) * _q_ornt;
	vector.x = p.Getx();
	vector.y = p.Gety();
	vector.z = p.Getz();
}

void Rocket::rocket2inertial(Vector3& vector) {
	Quaternion<double> p(0, vector.x, vector.y, vector.z);
	p = (_q_ornt * p) * _q_ornt.conj();
	vector.x = p.Getx();
	vector.y = p.Gety();
	vector.z = p.Getz();
}
