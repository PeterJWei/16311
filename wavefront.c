#define HEIGHT 8
#define WIDTH 16
typedef struct node {
	int startx;
	int starty;
	int endx;
	int endy;
};

int grid[HEIGHT][WIDTH] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},
	{0,0,1,1,0,1,1,0,0,1,1,0,0,0,0,0},
	{1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0},
	{1,1,1,1,1,1,1,1,1,1,0,0,1,1,0,0},
	{0,0,0,0,1,1,0,0,1,0,0,0,1,1,0,0},
	{0,0,0,0,0,0,0,0,0,0,1,1,0,1,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0}
};

void printgrid() {
	int i,j;
	writeDebugStream ("[\n");
	for (i=HEIGHT-1; i >= 0; i--) {
		for (j = 0;j < WIDTH; j++) {
			writeDebugStream("%d ", grid[i][j]);
		}
		writeDebugStream("\n");
	}
	writeDebugStream("]\n");
}

task main()
{
	int startx;
	int starty;
	int goalx;
	int goaly;
	int i,j;
	int n = 2;
	grid[goaly][goalx] = 2;
	while (grid[startx][starty] == 0) {
		for (i = 0; i < HEIGHT; i++) {
			for (j = 0; j < WIDTH; j++) {
				if (grid[i][j] == 0) {//unexplored
					if (i > 0 && i < HEIGHT-1 && j > 0 && j < WIDTH-1) {
						if (i > 0 && grid[i-1][j] == n) {
							grid[i][j] = n+1;
							continue;
				    }
						if (i < HEIGHT-1 && grid[i+1][j] == n) {
							grid[i][j] = n+1;
							continue;
						}
						if (j > 1 && grid[i][j-1] == n) {
							grid[i][j] = n+1;
							continue;
						}
						if (j < WIDTH-1 && grid[i][j+1] == n) {
							grid[i][j] = n+1;
							continue;
						}
					}
				}
			}
		}
	}
}
