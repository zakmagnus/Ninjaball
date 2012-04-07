CC=g++
LFLAGS=-lSDL -lSDL_image -lSDL_gfx -lSDL_ttf
CFLAGS=$(LFLAGS)
SRCS=level.cpp level_utils.cpp ui.cpp vector.cpp solid.cpp physics.cpp Moveable.cpp player.cpp visible.cpp
OBJS=$(SRCS:.cpp=.o)
HEADERS=$(SRCS:.cpp=.hpp)

ninjaball: $(OBJS)
	$(CC) -o ninjaball $(LFLAGS) ninjaball.cpp $(OBJS)

.cpp.o: $(HEADERS)
	$(CC) $(CFLAGS) $< -c

clean:
	rm *.o ninjaball
