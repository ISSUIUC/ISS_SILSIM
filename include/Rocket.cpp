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

	_Cp_vect = Vector3(0, 0, -(_nose_to_cp - _nose_to_cg));

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

void Rocket::get_d_ref(double& d_ref) const {
	d_ref = _d_ref;
}

void Rocket::set_d_ref(double& d_ref) {
	_d_ref = d_ref;
}

void Rocket::get_A_ref(double& A_ref) const {
	A_ref = _A_ref;
}

void Rocket::set_A_ref(double& A_ref) {
	_A_ref = A_ref;
}

void Rocket::get_Cna(double& Cna) const {
	Cna = _Cna;
}

void Rocket::set_Cna(double& Cna) {
	_Cna = Cna;
}

void Rocket::get_Cd(double& Cd) const {
	Cd = _Cd;
}

void Rocket::set_Cd(double& Cd) {
	_Cd = Cd;
}

void Rocket::get_nose_to_cg(double& nose_to_cg) const {
	nose_to_cg = _nose_to_cg;
}

void Rocket::set_nose_to_cg(double& nose_to_cg) {
	_nose_to_cg = nose_to_cg;
	_Cp_vect.x = 0;
	_Cp_vect.y = 0;
	_Cp_vect.z = -(_nose_to_cp - _nose_to_cg);
}

void Rocket::get_nose_to_cp(double& nose_to_cp) const {
	nose_to_cp = _nose_to_cp;
}

void Rocket::set_nose_to_cp(double& nose_to_cp) {
	_nose_to_cp = nose_to_cp;
	_Cp_vect.x = 0;
	_Cp_vect.y = 0;
	_Cp_vect.z = -(_nose_to_cp - _nose_to_cg);
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
