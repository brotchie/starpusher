CFLAGS = -I../main

VIZ_SOURCES = $(wildcard ../main/vizualizations/*.c)

viz-sdl: viz-sdl.c ../main/color.c ../main/buttons.c ../main/color.h ../main/vizualizations/viz.h $(VIZ_SOURCES)
	gcc viz-sdl.c ../main/color.c ../main/buttons.c $(VIZ_SOURCES) -I../main -I../main/vizualizations -lm $(shell sdl2-config --cflags --libs) -o viz-sdl

viz: viz.c ../main/color.c ../main/color.h ../main/vizualizations/viz.h $(VIZ_SOURCES)
	gcc viz.c ../main/color.c $(VIZ_SOURCES) -I../main -I../main/vizualizations -lm -o viz

all: viz

clean:
	rm viz
