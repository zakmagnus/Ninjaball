#include <math.h>
#include "solid.hpp"
#include "seg.hpp"
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
/* requires the seg has nonzero length */
bool ball_seg_coll (solid& s1, solid& s2, vector2d_t *dir) {
	struct ball_data_t& bd = s1.solid_data->ball_data;
	struct seg_data_t& sd = s2.solid_data->seg_data;
	vector2d_t cv(s1.x - s2.x, s1.y - s2.y);
	real_t seg_length = sd.dir->norm();
	assert(seg_length > 0);

	real_t proj = cv.dot(*(sd.dir)) / seg_length;
	if (proj < 0 || proj > seg_length) {
		/* center does not project onto segment;
		 * check distance from endpoints */
		//TODO norm = 0 ??
		real_t norm = cv.norm();
		if (cv.norm() < bd.r) {
			/*
			printf("ball's %g,%g is only %g away from %g,%g\n",s1.x,s1.y,norm,
					s2.x,s2.y);
			*/
			if (dir) {
				/* dir must go FROM s1 INTO s2 */
				*dir = cv / (-norm);		
				//printf("dir = %g,%g\n",dir->x,dir->y);
			}
			return true;
		}

		vector2d_t ep2v(sd.dir->x + s2.x - s1.x, sd.dir->y + s2.y - s1.y);
		//TODO norm = 0 ??
		norm = ep2v.norm();
		if (norm < bd.r) {
			/*
			printf("ball's %g,%g is only %g away from %g,%g\n",s1.x,s1.y,norm,
					s2.x+sd.dir->x,s2.y+sd.dir->y);
			*/
			if (dir) {
				*dir = ep2v / norm;
				//printf("dir = %g,%g\n",dir->x,dir->y);
			}
			return true;
		}

		return false;
	}
	else {
		/* center projects directly onto segment;
		 * check perpendicular distance */
		vector2d_t para = (*(sd.dir)) * (proj / seg_length);
		vector2d_t perp = cv - para;
		real_t perp_dist = perp.norm();
		if (perp_dist < bd.r) {
			/*
			printf("ball at %g,%g projects onto %g,%g with %g which is %g away\n",
				s1.x,s1.y,sd.dir->x,sd.dir->y,proj,perp_dist);
			*/
			if (dir) {
				*dir = -perp; /* FROM ball INTO segment */
				if (!dir->normalize()) {
					/* ball centered on segment! */
					/* This is a -pi/2 rotation. If the
					 * seg is directed, that's the right
					 * way (INTO the segment); if it's
					 * not, then I don't know what to
					 * make the direction, so this works
					 * as well as anything.
					 */
					dir->x = sd.dir->y;
					dir->y = -sd.dir->x;
					dir->normalize();
				}
				//printf("dir = %g,%g\n",dir->x,dir->y);
			}
			return true;
		}

		return false;
	}
}

/* Finds the normal vector FROM s1 INTO s2.
 * requires that s1 and s2 collide */
static void seg_seg_coll_dir(solid& s1, solid& s2, vector2d_t *dir) {
	if (!dir)
		return;

	seg_data_t& sd1 = s1.solid_data->seg_data;
	seg_data_t& sd2 = s2.solid_data->seg_data;

	vector2d_t d1 = *sd1.dir;
	vector2d_t d2 = *sd2.dir;
	printf("figuring out collision dir between %g,%g and %g,%g\n",
			d1.x,d1.y,d2.x,d2.y);
	d1.normalize();
	d2.normalize();
	/* these are lines, not vectors; establish
	 * minimum separation between them */
	bool d1_negated = false;
	if (d1.dot(d2) < 0) {
		d1 = -d1;
		d1_negated = true;
	}

	real_t ang1 = dir_to_angle(d1);
	real_t ang2 = dir_to_angle(d2);
	real_t avg_ang = (ang1 + ang2) / 2;
	real_t rotation = -M_PI / 2;
	real_t diff = 0;
	if (sd1.directed) {
		diff = avg_ang - ang1;

		if (diff == 0) {
			rotation = M_PI / 2; /* positive = away from d1 */
		}
		if (d1_negated)
			rotation = -rotation;
	}
	else if (sd2.directed) {
		diff = ang2 - avg_ang;
		if (diff == 0) {
			rotation = -M_PI / 2; /* negative = into d2 */
		}
	}
	if (diff != 0) {
		if (between_ord(0, diff, M_PI) ||
				between_ord(-2 * M_PI, diff, -M_PI)) {
			rotation = M_PI / 2;
		}
	}

	avg_ang += rotation;
	if (avg_ang > 2 * M_PI)
		avg_ang -= 2 * M_PI;
	else if (avg_ang < 0)
		avg_ang += 2 * M_PI;

	vector2d_t normal = angle_to_dir(avg_ang);
	*dir = normal;
}

/* requires that s1 and s2 are segments that have nonzero length */
bool seg_seg_coll (solid& s1, solid& s2, vector2d_t *dir) {
	seg_data_t& sd1 = s1.solid_data->seg_data;
	seg_data_t& sd2 = s2.solid_data->seg_data;
	real_t Ax1 = s1.x, Ay1 = s1.y, Bx1 = s2.x, By1 = s2.y;
	real_t Ax2 = Ax1 + sd1.dir->x, Ay2 = Ay1 + sd1.dir->y,
	       Bx2 = Bx1 + sd2.dir->x, By2 = By1 + sd2.dir->y;

	real_t Am = (Ay1 - Ay2) / (Ax1 - Ax2);
	real_t Bm = (By1 - By2) / (Bx1 - Bx2);

	/* these aliases help keep names straight when arbitrary
	 * points are needed */
	real_t Ax = Ax1, Ay = Ay1, Bx = Bx1, By = By1;

	/* vertical segment logic */
	if (Ax1 == Ax2) {
		if (Bx1 == Bx2) {
			/* parallel verticals; check for y intersection */
			if (Ax1 != Bx1)
				return false;
			real_t Aymin = min(Ay1, Ay2);
			real_t Bymin = min(By1, By2);
			real_t Aymax = max(Ay1, Ay2);
			real_t Bymax = max(By1, By2);
			real_t y_overlap = SINGLE_DIM_OVERLAP(Aymin, Aymax,
					Bymin, Bymax);
			if (y_overlap >= 0) {
				seg_seg_coll_dir(s1, s2, dir);
				printf("%g,%g->%g,%g collides with %g,%g->%g,%g at %g,??\n",
					Ax1, Ay1, Ax2, Ay2, Bx1, By1, Bx2, By2,
					Ax1);
				return true;
			}

			return false;
		}
#define VERT_SLOPE_COLL(vx,vy1,vy2,sx1,sx2,sy1,sy2,sm) do {\
	real_t x_int = vx;\
	if (!between((sx1), x_int, (sx2)))\
		return false;\
	real_t y_int = LINE_Y_COORD(x_int, sm, sx1, sy1);\
	if (between(vy1, y_int, vy2)) {\
		printf("%g,%g->%g,%g collides with %g,%g->%g,%g at %g,%g\n",\
				vx, vy1, vx, vy2, sx1, sy1, sx2, sy2,\
				x_int, y_int);\
		seg_seg_coll_dir(s1, s2, dir);\
		return true;\
	}\
	return false;\
} while (0)
		else {
			/* only A is vertical */
			VERT_SLOPE_COLL(Ax1, Ay1, Ay2, Bx1, Bx2, By1, By2, Bm);
		}
	}
	/* else, A is NOT vertical */
	if (Bx1 == Bx2) {
		VERT_SLOPE_COLL(Bx1, By1, By2, Ax1, Ax2, Ay1, Ay2, Am);
	}

	/* else, no verticals. GOOD */

	if (Am == Bm) { /* parallels */
		real_t Axmin = min(Ax1, Ax2);
		real_t Bxmin = min(Bx1, Bx2);
		//TODO huh? does this make sense??
		real_t y1 = LINE_Y_COORD(Axmin, Bm, Bx, By);
		real_t y2 = LINE_Y_COORD(Axmin, Am, Ax, Ay);
		real_t y3 = LINE_Y_COORD(Bxmin, Bm, Bx, By);
		real_t y4 = LINE_Y_COORD(Bxmin, Am, Ax, Ay);
		//TODO standardize this real_t closeness
		if (fabs(y1 - y2) <= 0.001 || fabs(y3 - y4) <= 0.001) {
			printf("%g,%g->%g,%g collides with %g,%g->%g,%g at %g,%g or %g,%g\n",
					Ax1, Ay1, Ax2, Ay2, Bx1, By1, Bx2, By2,
					Axmin, y1, Bxmin, y3);
			seg_seg_coll_dir(s1, s2, dir);
			return true;
		}

		return false;
	}

	/* find intersection of lines, see if that's included in the
	 * segments */
	real_t X_int = (Ax * Am - Bx * Bm + By - Ay) / (Am - Bm);

	/* algebra sanity check */
	real_t Y_int = LINE_Y_COORD(X_int, Am, Ax, Ay);
	real_t Y_int_alt = LINE_Y_COORD(X_int, Bm, Bx, By);
	assert(fabs(Y_int - Y_int_alt) <= 0.01);

	if (between(Bx1, X_int, Bx2) && between(Ax1, X_int, Ax2)) {
		printf("%g,%g->%g,%g collides with %g,%g->%g,%g at %g,%g\n",
				Ax1, Ay1, Ax2, Ay2, Bx1, By1, Bx2, By2,
				X_int, Y_int);
		seg_seg_coll_dir(s1, s2, dir);
		return true;
	}

	return false;
}

/* requires s1 is the ball and s2 is a sane poly (no nulls) */
bool ball_poly_coll (solid& s1, solid& s2, vector2d_t *dir) {
	assert(s1.get_solid_type() == NB_SLD_BALL);
	assert(s2.get_solid_type() == NB_SLD_POLY);

	const struct poly_data_t& pd = s2.solid_data->poly_data;

#define SEG_PTR(i) ((seg *)(pd.segs[(i)]))
#define GET_GLOBAL_SEG(i) do {\
	assert(SEG_PTR((i)));\
	global_seg = *SEG_PTR((i));\
	global_seg.x += s2.x;\
	global_seg.y += s2.y;\
} while (0)
	seg global_seg; /* global coordinates, not poly-centric ones */
	for (int i = 0; i < pd.num_segs; i++) {
		assert(pd.segs);
		GET_GLOBAL_SEG(i);
		if (solids_collide(s1, global_seg, dir)) {
			return true;
		}
	}

	return false;
}

/* requires s1 is the seg and s2 is the poly */
bool seg_poly_coll (solid& s1, solid& s2, vector2d_t *dir) {
	assert(s1.get_solid_type() == NB_SLD_SEG);
	assert(s2.get_solid_type() == NB_SLD_POLY);

	const struct seg_data_t& sd = s1.solid_data->seg_data;
	const struct poly_data_t& pd = s2.solid_data->poly_data;

	//TODO duplicated macro? ok something is wrong here
#define SEG_PTR(i) ((seg *)(pd.segs[(i)]))
#define GET_GLOBAL_SEG(i) do {\
	assert(SEG_PTR((i)));\
	global_seg = *SEG_PTR((i));\
	global_seg.x += s2.x;\
	global_seg.y += s2.y;\
} while (0)
	for (int i = 0; i < pd.num_segs; i++) {
		seg global_seg;
		GET_GLOBAL_SEG(i);
		if (solids_collide(s1, global_seg, dir))
			return true;
	}

	return false;
}

/* Checks whether x, y is inside the polygon given by sd.
 * Requires that sd.segs is non-null, contains no nulls,
 * and is generally a sane polygon. */
static bool is_in_poly(real_t x, real_t y, solid& poly) {
	const poly_data_t sd = poly.solid_data->poly_data;
	assert(sd.segs);
	assert(*sd.segs);
	assert(((seg *)sd.segs[0])->solid_data->seg_data.directed);

	vector2d_t rot = *((seg *)sd.segs[0])->solid_data->seg_data.dir;
	/* pi/2 rotation; should be AWAY from the polygon */
	real_t tmp = rot.x;
	rot.x = -rot.y;
	rot.y = tmp;
	real_t cx = ((seg *)sd.segs[0])->x + rot.x;
	real_t cy = ((seg *)sd.segs[0])->y + rot.y;
	/* translate x,y into "polyspace" */
	real_t poly_x = x - poly.x;
	real_t poly_y = y - poly.y;
	seg tester(cx, cy, poly_x - cx, poly_y - cy);
	printf("tester segment is %g,%g (%g,%g)\n",tester.x,tester.y,
			tester.solid_data->seg_data.dir->x,
			tester.solid_data->seg_data.dir->y);

	int num_hit = 0;
	int num_corners = 0;
	for (int i = 0; i < sd.num_segs; i++) {
		assert(sd.segs[i]);
		if (point_on_segment(poly_x, poly_y, *(seg *)sd.segs[i]))
			return true;
		if (solids_collide(*(seg *)sd.segs[i], tester, NULL)) {
			if (point_on_segment(((seg *)sd.segs[i])->x,
						((seg *)sd.segs[i])->y, tester))
				num_corners++;
			/* check if the segments are parallel; if so, the
			 * tester will pass through a corner or two and
			 * this will give the correct hit count */
			/*
			vector2d_t v1 = *tester.solid_data->seg_data.dir;
			vector2d_t v2 = *((seg *)sd.segs[i])->
				solid_data->seg_data.dir;
			bool normed = v1.normalize();
			assert(normed);
			normed = v2.normalize();
			assert(normed);
			real_t dot = fabs(v1.dot(v2));
			if (SIMILAR_REALS(dot, 1))
				continue;
				*/

			num_hit++;
		}
	}
	printf("%d hits, %d corners\n", num_hit, num_corners);
	if (num_corners == 2) /* came in and went out */
		return false;
	if (num_corners == 1) /* count the corner as a hit */
		num_hit--; /* it has two segments so it's double-counted */

	return (num_hit % 2) == 1;
}

static void project_poly(solid& poly, vector2d_t& axis, real_t& min,
		real_t& max) {
	const struct poly_data_t& pd = poly.solid_data->poly_data;
	for (int i = 0; i < pd.num_segs; i++) {
		vector2d_t point(((seg *)pd.segs[i])->x + poly.x,
				((seg *)pd.segs[i])->y + poly.y);
		real_t proj = point.dot(axis);
		if ((i == 0) || proj < min)
			min = proj;
		if ((i == 0) || proj > max)
			max = proj;
	}
}

/* requires s1 and s2 are sane polygons (no nulls) */
bool poly_poly_coll (solid& s1, solid& s2, vector2d_t *dir) {
	assert(s1.get_solid_type() == NB_SLD_POLY);
	assert(s1.get_solid_type() == NB_SLD_POLY);

	const struct poly_data_t& pd1 = s1.solid_data->poly_data;
	const struct poly_data_t& pd2 = s2.solid_data->poly_data;

	assert(pd1.segs);
	assert(pd2.segs);
	
	real_t min_overlap;
	vector2d_t min_axis;
	for (int i = 0; i < pd1.num_segs; i++) {
		vector2d_t axis = *((seg *)pd1.segs[i])->solid_data->seg_data.dir;
		//TODO factor this out!
		real_t tmp = axis.x;
		axis.x = -axis.y;
		axis.y = tmp;
		axis.normalize();

		real_t s1min, s1max, s2min, s2max;
		project_poly(s1, axis, s1min, s1max);
		project_poly(s2, axis, s2min, s2max);
		/*
		printf("s1 projects onto %g,%g as %g,%g\n",axis.x,axis.y,s1min,s1max);
		printf("s2 projects onto %g,%g as %g,%g\n",axis.x,axis.y,s2min,s2max);
		*/
		real_t overlap = SINGLE_DIM_OVERLAP(s1min, s1max, s2min, s2max);
		if (overlap <= 0) { /* found separating axis; no collision */
			//printf("separate!\n");
			return false;
		}

		if ((i == 0) || overlap < min_overlap) {
			min_overlap = overlap;
			min_axis = axis;
			if (s2min < s1min) /* pointing the wrong way */
				min_axis = -min_axis;
		}
	}
	//TODO factor out obviously
	for (int i = 0; i < pd2.num_segs; i++) {
		vector2d_t axis = *((seg *)pd2.segs[i])->solid_data->seg_data.dir;
		real_t tmp = axis.x;
		axis.x = -axis.y;
		axis.y = tmp;
		axis.normalize();

		real_t s1min, s1max, s2min, s2max;
		project_poly(s1, axis, s1min, s1max);
		project_poly(s2, axis, s2min, s2max);
		/*
		printf("s1 projects onto %g,%g as %g,%g\n",axis.x,axis.y,s1min,s1max);
		printf("s2 projects onto %g,%g as %g,%g\n",axis.x,axis.y,s2min,s2max);
		*/
		real_t overlap = SINGLE_DIM_OVERLAP(s1min, s1max, s2min, s2max);
		if (overlap <= 0) { /* found separating axis; no collision */
			//printf("separate!\n");
			return false;
		}

		if (overlap < min_overlap) {
			min_overlap = overlap;
			min_axis = axis;
			if (s2min < s1min) /* pointing the wrong way */
				min_axis = -min_axis;
		}
	}
	//printf("min overlap is %g on %g,%g\n",min_overlap,min_axis.x,min_axis.y);

	if (dir)
		*dir = min_axis;
	
	return true;
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
	coll_funcs[get_coll_func_ind(NB_SLD_BALL, NB_SLD_SEG)] =
		ball_seg_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_BALL, NB_SLD_POLY)] =
		ball_poly_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_SEG, NB_SLD_SEG)] =
		seg_seg_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_SEG, NB_SLD_POLY)] =
		seg_poly_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_POLY, NB_SLD_POLY)] =
		poly_poly_coll;
	//TODO segs, polys
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
		if (new_c1 - c1 > 0)
			return;
		//printf("delta-c1: %g\n",new_c1-c1);
		//printf("v1,1 = %g,%g\n",p1->velocity.x,p1->velocity.y);
		//printf("para1,1 = %g,%g[%g]\n",(dir*c1).x,(dir*c1).y,c1);
		//printf("para1,2 = %g,%g[%g]\n",para_c1.x,para_c1.y,new_c1);
		perp_c1 = v1 - (dir * c1);
		//printf("perp1 = %g,%g\n",perp_c1.x,perp_c1.y);
		p1->velocity = para_c1 + perp_c1;
		//printf("v1,2 = %g,%g\n",p1->velocity.x,p1->velocity.y);
	}
	if (!immobile2) {
		if (new_c2 - c2 < 0)
			return;
		//printf("delta-c2: %g\n",new_c2-c2);
		//printf("v2,1 = %g,%g\n",p2->velocity.x,p2->velocity.y);
		//printf("para2,1 = %g,%g[%g]\n",(dir*c2).x,(dir*c2).y,c2);
		//printf("para2,2 = %g,%g[%g]\n",para_c2.x,para_c2.y,new_c2);
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
