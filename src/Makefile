CC=g++
LFLAGS=-lSDL -lSDL_image -lSDL_gfx
CFLAGS=$(LFLAGS)
SRCS=test.cpp vector.cpp solid.cpp physics.cpp Moveable.cpp player.cpp visible.cpp test_utils.cpp
OBJS=$(SRCS:.cpp=.o)
HEADERS=$(SRCS:.cpp=.hpp)

test: $(OBJS)
	$(CC) -o test $(LFLAGS) $(OBJS)

.cpp.o: $(HEADERS)
	$(CC) $(CFLAGS) $< -c

clean:
	rm *.o test
