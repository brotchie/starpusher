CFLAGS = -I../main

VIZ_SOURCES = $(wildcard ../main/vizualizations/*.c)

viz: viz.c ../main/color.c ../main/color.h ../main/vizualizations/viz.h $(VIZ_SOURCES)
	gcc viz.c ../main/color.c $(VIZ_SOURCES) -I../main -I../main/vizualizations -lm -o viz

all: viz

clean:
	rm viz
