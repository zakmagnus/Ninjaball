#ifndef NB_LEVEL_HPP
#define NB_LEVEL_HPP

typedef void (*level_func)(void);

#define NB_NUM_LEVELS 1
extern level_func level_funcs[NB_NUM_LEVELS];

int run_level (level_func level_init);

void init_level_funcs (void);

void load_level_one (void);

#endif
