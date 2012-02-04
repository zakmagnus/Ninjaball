#include "wall.hpp"
#include "test_utils.hpp"

wall::wall(SDL_Surface *img, real_t x, real_t y, real_t h, real_t w)
	: rect(x, y, h, w, 0.0) {
	physics = new physics_t((mass_t)1, true);
	this->img = img;
}

void wall::show(SDL_Surface *screen, SDL_Rect *camera) {
	if (this->img)
		apply_surface(this->img, screen, this->x, this->y, camera);
}
