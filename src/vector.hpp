#ifndef NB_VECTOR_H
#define NB_VECTOR_H

typedef float real_t;

class vector2d_t {
	public:
		real_t x, y;
		vector2d_t(real_t x=0.0, real_t y=0.0);
		vector2d_t& operator + (vector2d_t& v);
		vector2d_t& operator - (vector2d_t& v);
		vector2d_t& operator * (real_t s);
		vector2d_t& operator / (real_t s);
		vector2d_t& operator = (vector2d_t& v);
		vector2d_t& operator - (void);
		vector2d_t& operator *= (real_t s);
		vector2d_t& operator /= (real_t s);
		vector2d_t& operator += (vector2d_t& v);
		vector2d_t& operator -= (vector2d_t& v);
		bool operator == (vector2d_t& v);
		real_t norm(void);
		bool normalize(void);
		real_t dot(vector2d_t& v);
};

static vector2d_t rightvec = vector2d_t(1.0, 0.0);
static vector2d_t leftvec = -rightvec;
static vector2d_t downvec = vector2d_t(0.0, 1.0);
static vector2d_t upvec = -downvec;

vector2d_t angle_to_dir(real_t angle);
real_t dir_to_angle(vector2d_t& dir);

#endif
