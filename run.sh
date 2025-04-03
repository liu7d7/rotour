cp main.c main.c.bak && gcc -Ofast -g -o main main.c -std=gnu2x -lpigpio -lm && sudo ./main
