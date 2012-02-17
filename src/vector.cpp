#include <math.h>
#include <assert.h>
#include "vector.hpp"

vector2d_t::vector2d_t(real_t x, real_t y) {
	this->x = x;
	this->y = y;
}

real_t vector2d_t::dot(vector2d_t& v) {
	return this->x * v.x + this->y * v.y;
}

real_t vector2d_t::norm(void) {
	return sqrt(this->x * this->x + this->y * this->y);
}

bool vector2d_t::normalize(void) {
	real_t norm = this->norm();
	if (norm != 0) {
		this->x /= norm;
		this->y /= norm;
		return true;
	}
	return false;
}

vector2d_t& vector2d_t::operator + (vector2d_t& v) {
	vector2d_t& r = *new vector2d_t();
	r.x = this->x + v.x;
	r.y = this->y + v.y;
	return r;
}

vector2d_t& vector2d_t::operator - (vector2d_t& v) {
	vector2d_t& r = *new vector2d_t();
	r.x = this->x - v.x;
	r.y = this->y - v.y;
	return r;
}

vector2d_t& vector2d_t::operator / (real_t s) {
	vector2d_t& r = *new vector2d_t();
	r.x = this->x / s;
	r.y = this->y / s;
	return r;
}

vector2d_t& vector2d_t::operator * (real_t s) {
	vector2d_t& r = *new vector2d_t();
	r.x = this->x * s;
	r.y = this->y * s;
	return r;
}

vector2d_t& vector2d_t::operator = (vector2d_t& v) {
	this->x = v.x;
	this->y = v.y;
	return *this;
}

vector2d_t& vector2d_t::operator - (void) {
	return (*this) * -1;
}

vector2d_t& vector2d_t::operator -= (vector2d_t& v) {
	(*this) = (*this) - v;
	return *this;
}

vector2d_t& vector2d_t::operator += (vector2d_t& v) {
	(*this) = (*this) + v;
	return *this;
}

vector2d_t& vector2d_t::operator *= (real_t s) {
	(*this) = (*this) * s;
	return *this;
}

vector2d_t& vector2d_t::operator /= (real_t s) {
	(*this) = (*this) / s;
	return *this;
}

bool vector2d_t::operator == (vector2d_t& v) {
	return v.x == this->x && v.y == this->y;
}

vector2d_t angle_to_dir(real_t angle) {
	vector2d_t dir;
	dir.x = cos(angle);
	dir.y = sin(angle);
	return dir;
}

/* requires dir is a unit vector */
real_t dir_to_angle(vector2d_t& dir) {
	real_t ang = atan2(dir.y, dir.x);
	if (ang < 0)
		ang += 2 * M_PI;

	return ang;
}

void average_dir(vector2d_t& dir1, vector2d_t& dir2, vector2d_t *buf) {
	assert(buf);

	vector2d_t d1 = dir1;
	vector2d_t d2 = dir2;
	bool normed = d1.normalize();
	assert(normed);
	normed = d2.normalize();
	assert(normed);

	real_t ang1 = dir_to_angle(d1);
	real_t ang2 = dir_to_angle(d2);
	real_t avg_ang = (ang1 + ang2) / 2;
	vector2d_t avg_dir = angle_to_dir(avg_ang);
	*buf = avg_dir;
}
