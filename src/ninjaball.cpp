#include "level.hpp"
#include "ui.hpp"

char *intro[] = {"NINJABALL",
	"",
	"A: move left",
	"D: move right",
	"Left click: fire rope at pointer",
	"Release left click: release rope",
	"W: retract rope",
	"S: extend rope",
	"",
	"Press any key to begin.",
	NULL};
int main (int argc, char **argv) {
	init_sdl();

	SDL_WM_SetCaption("Ninjaball", NULL);
	show_screen(intro);

	char *dead_msg[] = {"YOU DIE!", "", "Press any key to restart.", NULL};
	while (run_level() < 0) {
		SDL_WM_SetCaption("x_x", NULL);
		show_screen(dead_msg);
	}

	teardown_sdl();

	return 0;
}
