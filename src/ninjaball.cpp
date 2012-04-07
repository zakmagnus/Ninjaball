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
	show_screen(intro);

	run_level();

	teardown_sdl();

	return 0;
}
