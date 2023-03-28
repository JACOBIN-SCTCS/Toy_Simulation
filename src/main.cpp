#include <iostream>
#include <SDL2/SDL.h>

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

	private:
		int height;
		int width;
		SDL_Window *window = NULL;
		SDL_Renderer *renderer = NULL;
};

int main(int argc, char *argv[])
{
	Framework fw(600, 600);
	SDL_Event event;

	while (!(event.type == SDL_QUIT))
	{
		SDL_Delay(10);
		SDL_PollEvent(&event);
	}
	return 0;
}