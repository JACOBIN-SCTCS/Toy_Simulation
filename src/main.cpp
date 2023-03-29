#include <iostream>
#include <time.h>
#include "framework.h"

class Robot
{	
	public:
		Robot(int size) : grid_size(size) , fw(size,size)
		{
			grid.resize(grid_size,std::vector<int>(grid_size,1));

			for(int i=100;i<150;++i)
				for(int j=100;j<150;++j)
					grid[i][j] = 0;
			
			fw.render_screen(grid);
		}

	private:
		int grid_size;
		std::vector<std::vector<int>> grid;
		Framework fw;

};

int main(int argc, char *argv[])
{
	srand (time(NULL));

	Robot robot(600);

	SDL_Event event;
	while (!(event.type == SDL_QUIT))
	{
		SDL_Delay(10);
		SDL_PollEvent(&event);
	}
	return 0;
}