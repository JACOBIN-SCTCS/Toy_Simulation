#include <SDL2/SDL.h>
#include <vector>
#include <iostream>

class Framework
{
	public:
		Framework(int h ,  int w , double s) : height(h), width(w) ,  scale(s)
		{
			SDL_Init(SDL_INIT_VIDEO);
			SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);		
			SDL_RenderSetScale( renderer, scale, scale );

            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
			SDL_RenderClear(renderer);
			SDL_RenderPresent(renderer);
		}
		~Framework()
		{
			SDL_DestroyRenderer(renderer);
			SDL_DestroyWindow(window);
			SDL_Quit();
    	}

		void draw_point(int center_x, int center_y)
		{
			SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
			SDL_RenderDrawPoint(renderer,center_x,center_y);
			SDL_RenderPresent(renderer);

		}

		void render_screen(std::vector<std::vector<int>> grid)
		{
			SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
			SDL_RenderClear(renderer);
			for(int i=0;i<grid.size();++i)
			{
				for(int j=0;j<grid[0].size();++j)
				{
					if(grid[i][j]==-1)
					{
						SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
						SDL_RenderDrawPoint(renderer,i,j);
					}
					else if(grid[i][j]==0)
					{
						SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
						SDL_RenderDrawPoint(renderer,i,j);
					}
					else if(grid[i][j]==1)
					{
						SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
						SDL_RenderDrawPoint(renderer,i,j);
					}
					else
					{
						SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
						SDL_RenderDrawPoint(renderer,i,j);	
					}
				}
			}
			SDL_RenderPresent(renderer);
		}

	

	private:
		int height;
		int width;
		SDL_Window *window = NULL;
		SDL_Renderer *renderer = NULL;
        SDL_Texture* texture = NULL;
        uint32_t* textureBuffer = NULL;
		double scale = 10;

};
