#include <SDL2/SDL.h>
#include <vector>
#include <iostream>

class Framework
{
	public:
		Framework(int h ,  int w) : height(h), width(w)
		{
			SDL_Init(SDL_INIT_VIDEO);
			SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);		
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
			SDL_RenderSetScale( renderer, scale, scale );
			SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
			SDL_RenderDrawPoint(renderer,center_x/scale,center_y/scale);
		}

		void render_screen(std::vector<std::vector<int>> grid)
		{
            SDL_RenderClear(renderer);
			SDL_RenderSetScale( renderer, scale, scale );
			for(int i=0;i<grid.size();++i)
			{
				for(int j=0;j<grid.size();++j)
				{
					if(grid[i][j] == 1)
					{
						SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
						SDL_RenderDrawPoint(renderer,i/scale,j/scale);
					}
					else
					{
                        //std::cout<<"Entered here"<<std::endl;
						SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
						SDL_RenderDrawPoint(renderer,i/scale,j/scale);
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
		double scale = 10.0;

};
