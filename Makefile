
th02: main.c th02.c th02.h
	gcc -Os -lm -Wall -Wextra -o $@ $^

clean:
	rm -f th02
