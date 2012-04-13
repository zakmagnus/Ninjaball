#ifndef NB_UI_HPP
#define NB_UI_HPP

#include "SDL/SDL_ttf.h"

#define NB_DEFAULT_SHOW_DUR (1500) /* ms */

extern TTF_Font *font;

int init_sdl (void);
void teardown_sdl (void);
int show_screen (char **message, int dur=NB_DEFAULT_SHOW_DUR);

#endif
