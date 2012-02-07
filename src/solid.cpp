#include <math.h>
#include "solid.hpp"
#include "test_utils.hpp"

collision_function coll_funcs[NB_NUM_COLL_FUNCS];

/* Absolute parallel velocity threshold such that the mover
 * starts being "on" the solid it's colliding with. */
#define NB_COLL_V_THRESH  (7.0)
/* Distance to move an object towards something it's on in order
 * to check for a collision and decide if it's really still on. */
#define NB_ONCHECK_P_DIST (1.5)

solid::solid(real_t x, real_t y, real_t e) {
	this->x = x;
	this->y = y;
	this->elasticity = e;

	this->onbases = new list<onbase_data>();
	this->on_backrefs = new list<on_backref_data>();
	//TODO make abstract?
	this->solid_type = NB_NUM_SLD_TYPES;
	this->solid_data = new solid_data_t;
}

unsigned solid::get_solid_type(void) {
	return this->solid_type;
}

real_t solid::left_edge(void) {
	return this->x;
}

real_t solid::top_edge(void) {
	return this->y;
}

#include <stdio.h>
/* rectangle intersection */
bool rect_rect_coll (solid& s1, solid& s2, vector2d_t *dir) {
	assert(s1.get_solid_type() == NB_SLD_RECT);
	assert(s2.get_solid_type() == NB_SLD_RECT);
	struct rect_data_t& rd1 = s1.solid_data->rect_data;
	struct rect_data_t& rd2 = s2.solid_data->rect_data;
	real_t top1 = s1.y, top2 = s2.y, left1 = s1.x, left2 = s2.x,
		bot1 = s1.y + rd1.h, bot2 = s2.y + rd2.h,
		right1 = s1.x + rd1.w, right2 = s2.x + rd2.w;
	if (left1 > right2)
		return false;
	if (top1 > bot2)
		return false;
	if (left2 > right1)
		return false;
	if (top2 > bot1)
		return false;
	
	if (dir) {
		/* edge contact heuristic: the dimension with a greater overlap
		 * is the one where the collision "really" happens */
		/*
		real_t xmin = min(left1, left2);
		real_t xmax = max(right1, right2);
		real_t x_overlap = xmax - xmin - fabs(left1 - left2) - fabs(right1 - right2);
		*/
		real_t x_overlap = SINGLE_DIM_OVERLAP(left1, right1,
				left2, right2);
		/*
		real_t ymin = min(top1, top2);
		real_t ymax = max(bot1, bot2);
		real_t y_overlap = ymax - ymin - fabs(top1 - top2) - fabs(bot1 - bot2);
		*/
		real_t y_overlap = SINGLE_DIM_OVERLAP(top1, bot1, top2, bot2);

		if (x_overlap > y_overlap) {
			if (s1.y > s2.y)
				*dir = upvec;
			else
				*dir = downvec;
		}
		else {
			if (s1.x > s2.x)
				*dir = leftvec;
			else
				*dir = rightvec;
		}
	}

	return true;
}

bool ball_ball_coll (solid& s1, solid& s2, vector2d_t *dir) {
	assert(s1.get_solid_type() == NB_SLD_BALL);
	assert(s2.get_solid_type() == NB_SLD_BALL);
	vector2d_t c_axis;
	c_axis.x = s2.x - s1.x;
	c_axis.y = s2.y - s1.y;
	real_t r1 = s1.solid_data->ball_data.r;
	real_t r2 = s2.solid_data->ball_data.r;

	if (c_axis.dot(c_axis) > (r1 + r2) * (r1 + r2))
		return false;
	
	if (dir)
		*dir = c_axis;

	return true;
}

/* requires s1 is the rect and s2 is the ball 
 *
 * This function assumes an EDGE collision; one solid well inside the
 * other may fail to be detected. */
bool rect_ball_coll (solid& s1, solid& s2, vector2d_t *dir) {
	assert(s1.get_solid_type() == NB_SLD_RECT);
	assert(s2.get_solid_type() == NB_SLD_BALL);
	struct rect_data_t& rd = s1.solid_data->rect_data;
	real_t r = s2.solid_data->ball_data.r,
	       bx = s2.x, by = s2.y;
	real_t top = s1.y, left = s1.x,
	       bot = s1.y + rd.h, right = s1.x + rd.w;
	
	if ((left < bx) && (bx < right)) {
		if ((r > fabs(by - top)) || (r > fabs(by - bot))) {
			if (dir) {
				if (s1.y > s2.y)
					*dir = upvec;
				else
					*dir = downvec;
			}
			return true;
		}

		return false;
	}
	if ((top < by) && (by < bot)) {
		if ((r > fabs(bx - left)) || (r > fabs(bx - right))) {
			if (dir) {
				if (s1.x > s2.x)
					*dir = leftvec;
				else
					*dir = rightvec;
			}
			return true;
		}

		return false;
	}

	real_t rsq = r * r;
	vector2d_t d_vec;

	//TODO move somewhere better ?
#define CHECK_DVEC do {\
	if (rsq > d_vec.x * d_vec.x + d_vec.y * d_vec.y) {\
		if (dir) {\
			d_vec.normalize();\
			*dir = d_vec;\
		}\
		return true;\
	}\
} while (0)

	d_vec.x = bx - left;
	d_vec.y = by - top;
	CHECK_DVEC;
	d_vec.x = bx - right;
	d_vec.y = by - top;
	CHECK_DVEC;
	d_vec.x = bx - left;
	d_vec.y = by - bot;
	CHECK_DVEC;
	d_vec.x = bx - right;
	d_vec.y = by - bot;
	CHECK_DVEC;

	return false;
}

/* requires s1 is the ball and s2 is the segment */
/* requires the seg_data obeys the x <= x2 convention and has nonzero length */
bool ball_seg_coll (solid& s1, solid& s2, vector2d_t *dir) {
	struct ball_data_t& bd = s1.solid_data->ball_data;
	struct seg_data_t& sd = s1.solid_data->seg_data;
	vector2d_t cv(s1.x - s2.x, s1.y - s2.y);
	vector2d_t lv = (sd.x2 - s2.x, sd.y2 - s2.y);
	real_t seg_length = lv.norm();

	real_t proj = cv.dot(lv) / seg_length;
	if (proj < 0 || proj > seg_length) {
		/* center does not project onto segment;
		 * check distance from endpoints */
		//TODO norm = 0 ??
		real_t norm = cv.norm();
		if (cv.norm() < bd.r) {
			if (dir) {
				/* dir must go FROM s1 INTO s2 */
				*dir = cv / (-norm);		
			}
			return true;
		}

		vector2d_t ep2v(sd.x2 - s1.x, sd.y2 - s1.y);
		norm = ep2v.norm();
		if (norm < bd.r) {
			if (dir) {
				*dir = ep2v / norm;
			}
			return true;
		}

		return false;
	}
	else {
		/* center projects directly onto segment;
		 * check perpendicular distance */
		vector2d_t para = lv * (proj / seg_length);
		vector2d_t perp = cv - para;
		real_t perp_dist = perp.norm();
		if (perp_dist < bd.r) {
			if (dir) {
				*dir = para - cv;
				if (!dir->normalize()) {
					/* ball centered on segment! */
					real_t tmp = lv.x;
					lv.x = lv.y;
					lv.y = tmp;
					lv.normalize();
					//TODO in which direction should this go?
					*dir = lv;
					printf("ball centered on segment!! LOOK OUT!!\n");
				}
			}
			return true;
		}

		return false;
	}
}

/* requires that s1 and s2 are segments that obey the x <= x2 convention and
 * have nonzero length */
bool seg_seg_coll (solid& s1, solid& s2, vector2d_t *dir) {
	seg_data_t sd1 = s1.solid_data->seg_data;
	seg_data_t sd2 = s2.solid_data->seg_data;
	real_t Ax1 = s1.x, Ay1 = s1.y, Bx1 = s2.x, By1 = s2.y,
	       Ax2 = sd1.x2, Ay2 = sd1.y2, Bx2 = sd2.x2, By2 = sd2.y2;
	assert(Ax1 <= Ax2);
	assert(Bx1 <= Bx2);
	
	real_t Am = (Ay1 - Ay2) / (Ax1 - Ax2);
	real_t Bm = (By1 - By2) / (Bx1 - Bx2);
	/* these aliases help keep names straight if arbitary
	 * points are needed */
	real_t Ax = Ax1, Ay = Ay1, Bx = Bx1, By = By1;
	
	/* vertical segment logic */
	if (Ax1 == Ax2) {
		if (Bx1 == Bx2) {
			/* parallel verticals; check for y intersection */
			real_t Aymin = min(Ay1, Ay2);
			real_t Bymin = min(By1, By2);
			real_t Aymax = max(Ay1, Ay2);
			real_t Bymax = max(By1, By2);
			real_t y_overlap = SINGLE_DIM_OVERLAP(Aymin, Aymax,
					Bymin, Bymax);
			/*
			real_t ymin = min(Aymin, Bymin);
			real_t ymax = max(Aymax, Bymax);
			real_t y_overlap = ymax - ymin -
				fabs(Aymax - Bymax) - fabs(Aymin - Bymin);
			 */
			if (y_overlap >= 0) {
				//TODO dir

				return true;
			}

			return false;
		}
		else {
			//TODO factor out this logic
			real_t Bx_int = Ax1;
			real_t By_int = (Bx_int - Bx) * Bm + By;
			if ((By1 >= By_int && By_int >= By2) ||
					(By1 <= By_int && By_int <= By2)) {
				//TODO dir
				return true;
			}

			return false;
		}
	}
	/* else, A is NOT vertical */
	if (Bx1 == Bx2) {
		real_t Ax_int = Bx1;
		real_t Ay_int = (Ax_int - Ax) * Am + Ay;
		if ((Ay1 >= Ay_int && Ay_int >= Ay2) ||
				(Ay1 <= Ay_int && Ay_int <= Ay2)) {
			//TODO dir
			return true;
		}

		return false;
	}
	/* else, no verticals. GOOD */
	
	if (Am == Bm) { /* parallels */
		real_t overlap = SINGLE_DIM_OVERLAP(Ax1, Ax2, Bx1, Bx2);
		if (overlap >= 0) {
			//TODO dir
			return true;
		}

		return false;
	}

	/* find intersection of lines, see if that's included in the
	 * segments */
	real_t X_int = (Ax * Am - Bx * Bm + By - Ay) / (Am - Bm);

	/* algebra sanity check */
	real_t Y_int = (X_int - Ax) * Am + Ay;
	real_t Y_int_alt = (X_int - Bx) * Bm + By;
	assert(fabs(Y_int - Y_int_alt) <= 0.01);

	if (Bx1 <= X_int && X_int <= Bx2 && Ax1 <= X_int && X_int <= Ax2) {
		//TODO dir
		return true;
	}

	return false;
}

/* Sets up record keeping so a solid knows which normal forces are exerted
 * on it. Also notifies the onbase of this.
 * requires onbase be non-null */
list<solid::onbase_data>::iterator solid::become_on(solid *onbase,
		vector2d_t& normal) {
	assert(onbase);
	onbase_data data;
	data.onbase = onbase;
	data.normal = normal;
	this->onbases->push_front(data);
	printf("a solid is now on, with normal %g,%g\n",normal.x,normal.y);

	onbase->become_onbase(this, normal, this->onbases->begin());

	return this->onbases->begin();
}

/* Installs a backreference in the onbase when something becomes on.
 * Only meant to be called from become_on(). If called from elsewhere,
 * this is likely an ERROR! */
void solid::become_onbase(solid *backref, vector2d_t& normal,
		list<onbase_data>::iterator I) {
	on_backref_data data;
	data.backref = backref;
	data.normal = normal;
	data.I = I;
	this->on_backrefs->push_back(data);
}

list<solid::onbase_data>::iterator
solid::stop_being_on(list<solid::onbase_data>::iterator I) {
	vector2d_t& normal = (*I).normal;
	list<solid::onbase_data>::iterator ret = this->onbases->erase(I);

	bool normal_still_applies = false;
	for (list<on_backref_data>::iterator J = this->on_backrefs->begin();
			J != this->on_backrefs->end(); J++) {
		if ((*J).normal == normal) {
			normal_still_applies = true;
			break;
		}
	}

	if (!normal_still_applies) {
		/* any solid that uses this as an onbase with this normal
		 * is no longer on; notify them */

		list<on_backref_data>::iterator J =
			this->on_backrefs->begin();
		while (J != this->on_backrefs->end()) {
			if ((*J).normal == normal) {
				(*J).backref->stop_being_on((*J).I);
				J = this->on_backrefs->erase(J);
			}
			else {
				J++;
			}
		}
	}

	return ret;
}

/* MUST be called before collisions are used! */
void init_coll_funcs(void) {
	//XXX make sure to put the types in ascending order!
	coll_funcs[get_coll_func_ind(NB_SLD_RECT, NB_SLD_RECT)] =
		rect_rect_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_BALL, NB_SLD_BALL)] =
		ball_ball_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_RECT, NB_SLD_BALL)] =
		rect_ball_coll;
}

/* Requires dir is a direction and not a 0 vector, pointing FROM s1 INTO s2. */
void resolve_collision(solid& s1, solid& s2, physics_t *p1, physics_t *p2,
		vector2d_t& dir) {
	/*TODO This artificial immobility is fucked! If the onbase is
	 * actually moving in the direction of its normal, then that
	 * collision should be treated as a mobile one. */
	/* break velocities into parallel and perpendicular components */
	vector2d_t para_c1, para_c2, perp_c1, perp_c2;
	real_t c1, c2, new_c1, new_c2;

	real_t norm_d = dir.norm();
	assert(norm_d > 0.0);
	dir = dir / norm_d;
	
	vector2d_t& v1 = p1->velocity;
	c1 = 0.0;
	if (v1.x != 0 || v1.y != 0)
		c1 = v1.dot(dir);
	vector2d_t& v2 = p2->velocity;
	c2 = 0.0;
	if (v2.x != 1 || v1.y != 0)
		c2 = v2.dot(dir);

	bool immobile1 = p1->immobile;
	if ((!immobile1) && c1 <= 0) {
		vector2d_t& normal = dir;
		for (list<solid::onbase_data>::iterator I = s1.onbases->begin();
				I != s1.onbases->end(); I++) {
			if ((*I).normal == normal) {
				immobile1 = true;
				break;
			}
		}
	}
	bool immobile2 = p2->immobile;
	if (!immobile2 && c2 >= 0) {
		vector2d_t& normal = -dir; /* from s2 into s1 */
		for (list<solid::onbase_data>::iterator I = s2.onbases->begin();
				I != s2.onbases->end(); I++) {
			if ((*I).normal == normal) {
				immobile2 = true;
				break;
			}
		}
	}

	/* use parallels to calculate new parallels */
	if ((!immobile1) && (!immobile2)) {
		mass_t total_mass = p1->mass + p2->mass;
		new_c1 = (c1 * (p1->mass - p2->mass) + 2 * p2->mass * c2)
			/ total_mass;
		new_c2 = (c2 * (p2->mass - p1->mass) + 2 * p1->mass * c1)
			/ total_mass;
	}
	else {
		if (!immobile1)
			new_c1 = -c1;
		if (!immobile2)
			new_c2 = -c2;
	}

	new_c1 *= s1.elasticity;
	new_c2 *= s2.elasticity;

	if (immobile2 && fabs(new_c1) < NB_COLL_V_THRESH) {
		vector2d_t normal = -dir; /* from s2 into s1 */

		if (!p2->immobile)
			printf("mobile onbase!\n");
		printf("s1 is on (%g); dir = %g,%g\n",new_c1,dir.x,dir.y);
		s1.become_on(&s2, normal); //TODO double onness??
		new_c1 = 0;
	}
	if (immobile1 && fabs(new_c2) < NB_COLL_V_THRESH) {
		vector2d_t normal = dir;

		if (!p1->immobile)
			printf("mobile onbase!\n");
		printf("s2 is on; dir = %g,%g\n",dir.x,dir.y);
		s2.become_on(&s1, normal); //TODO double onness??
		new_c2 = 0;
	}

	para_c1 = dir * new_c1;
	para_c2 = dir * new_c2;

	/* use new parallels to calculate new velocities */
	if (!immobile1) {
		//printf("v1,1 = %g,%g\n",p1->velocity.x,p1->velocity.y);
		//printf("para1,1 = %g,%g[%g]\n",(dir*c1).x,(dir*c1).y,c1);
		//printf("para1,2 = %g,%g[%g]\n",para_c1.x,para_c1.y,new_c1);
		perp_c1 = v1 - (dir * c1);
		//printf("perp1 = %g,%g\n",perp_c1.x,perp_c1.y);
		p1->velocity = para_c1 + perp_c1;
		//printf("v1,2 = %g,%g\n",p1->velocity.x,p1->velocity.y);
	}
	if (!immobile2) {
		//printf("v2,1 = %g,%g\n",p2->velocity.x,p2->velocity.y);
		//printf("para2 = %g,%g\n",para_c2.x,para_c2.y);
		perp_c2 = v2 - (dir * c2);
		p2->velocity = para_c2 + perp_c2;
		//printf("v2,2 = %g,%g\n",p2->velocity.x,p2->velocity.y);
	}
}

/* requires s non-null */
/* reqires that s is on the solid referred to in data */
bool is_still_on(solid::onbase_data& data, solid *s) {
	assert(s);

	/* Advance s's position by a constant along the onbase normal
	 * and check for collision. s is still on iff there is a collision.
	 */
	real_t old_x = s->x, old_y = s->y;
	vector2d_t& normal = data.normal;
	vector2d_t dp = (-normal) * NB_ONCHECK_P_DIST;

	vector2d_t p_vec;
	p_vec.x = s->x;
	p_vec.y = s->y;
	p_vec += dp;
	s->x = p_vec.x;
	s->y = p_vec.y;

	bool ret = solids_collide(*s, *data.onbase, NULL);
	if (!ret)
		printf("adjust pos by %g,%g and the object does not collide with onbase\n", dp.x,dp.y);

	s->x = old_x;
	s->y = old_y;
	return ret;
}
