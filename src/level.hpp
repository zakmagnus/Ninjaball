#ifndef NB_LEVEL_HPP
#define NB_LEVEL_HPP

typedef void (*level_func)(void);

#define NB_NUM_LEVELS 3
extern level_func level_funcs[NB_NUM_LEVELS];

#define NB_LEVEL_SUCCESS 1
enum {
	NB_LEVEL_DEAD = 1,
	NB_LEVEL_QUIT,
	NB_LEVEL_ERROR
};
int run_level (level_func level_init);
void init_level_funcs (void);
void load_tutorial (void);
void load_level_one (void);
void load_level_two (void);

#endif
