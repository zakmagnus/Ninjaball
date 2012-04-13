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
char *dead_msg[] = {"YOU DIE!", "", "Press any key to restart.", NULL};
char *yay_msg[] = {"Level complete.", "", "Press any key to proceed.", NULL};
int main (int argc, char **argv) {
	init_sdl();
	init_level_funcs();

	SDL_WM_SetCaption("Ninjaball", NULL);
	if (show_screen(intro) < 0)
		goto exit_func;

	//TODO this logic makes me want to throw up
	for (int i = 0; i < NB_NUM_LEVELS; i++) {
		int ret;
		do {
			ret = run_level(level_funcs[i]);
			switch (ret) {
			case NB_LEVEL_SUCCESS:
				SDL_WM_SetCaption("^_^", NULL);
				if (show_screen(yay_msg) < 0)
					goto exit_func;
				break;
			case -NB_LEVEL_DEAD:
				SDL_WM_SetCaption("x_x", NULL);
				if (show_screen(dead_msg, 1000) < 0)
					goto exit_func;
				break;
			case -NB_LEVEL_ERROR:
				printf("ERROR!\n");
			case -NB_LEVEL_QUIT:
			default:
				goto exit_func;
				break;
			}
		} while (ret == -NB_LEVEL_DEAD);
	}

exit_func:
	teardown_sdl();

	return 0;
}
