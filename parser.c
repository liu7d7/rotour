#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tgmath.h>

#define MAX_POINTS 4096

typedef struct {
    float x;
    float y;
} Point;

static float dist(Point a, Point b) {
  return hypot(a.x - b.x, a.y - b.y);
}

Point points[MAX_POINTS];
int point_count;
float max_time;

static void read_points() {
    point_count = 0;

    FILE *file = fopen("points.txt", "r");
    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    char line[256];

    float px = 0, py = 0;

    fscanf(file, "%f\n", &max_time);

    while (fgets(line, sizeof(line), file) != NULL) {
        // Skip empty lines and comments (lines starting with #)
        if (line[0] == '\n' || line[0] == '#') {
            continue;
        }

        // Remove trailing newline
        line[strcspn(line, "\n")] = 0;

        float x, y;
        //parse doubles separated by space
        if (sscanf(line, "%f %f", &x, &y) == 2) {
            if (point_count < MAX_POINTS) {
		float dx = x - px;
		float dy = y - py;
		float len = hypot(px - x, py - y);

                if (point_count > 0 && len > 0.75) {
		    float slices = ceil(len / 0.5);
		    for (int s = 0; s < (int)slices; s++) {
                        points[point_count].x = px + dx * (s + 1) / slices;
                        points[point_count].y = py + dy * (s + 1) / slices;
                        point_count++;
		    }
                } else {
		  points[point_count].x = x;
		  points[point_count].y = y;
		  point_count++;
		}
            } 
            else {
                fprintf(stderr, "Maximum number of points (%d) reached. Skipping the rest.\n", MAX_POINTS);
                break;
            }

	    px = x, py = y;
        } 
        else {
            fprintf(stderr, "Could not parse line: %s\n", line);
        }
    }

    for (int i = 0; i < point_count; i++) {
        printf("Point %d: x = %f, y = %f\n", i+1, points[i].x, points[i].y);
    }

    fclose(file);
}
