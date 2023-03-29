#include <vector>
#include <iostream>
#include <utility>
#include <queue>

class FrontierExplore
{
    public:
        struct Frontier 
        {
            int x;
            int y;
            std::vector<std::pair<int,int>> cells;
        };

        FrontierExplore(std::vector<std::vector<int>> *grid) : grid(grid) 
        {
            ;
        }

        std::vector<std::pair<int,int>> getNeighbours(int x ,  int y , int max_x , int max_y)
        {
            std::vector<std::pair<int,int>> neighbours;
            int x_dir[] = {0,0,1,-1,1,1,-1,-1};
            int y_dir[] = {1,-1,0,0,1,-1,1,-1};
            for(int i=0;i<8;++i)
            {
                int x_new = x + x_dir[i];
                int y_new = y + y_dir[i];
                if(x_new < 0 || x_new >= max_x || y_new < 0 || y_new >= max_y)
                    continue;
                else
                    neighbours.push_back({x_new,y_new});

            }
            return neighbours;
        }

        void findFrontiers(int x, int y)
        {
            frontiers.clear();
            std::vector<std::vector<int>>& grid_ref = *grid;
            bool visited[grid_ref.size()][grid_ref[0].size()];
            bool frontier_tag[grid_ref.size()][grid_ref[0].size()];
            std::queue<std::pair<int,int>> q;
            q.push({x,y});
            visited[x][y] = true;
            while(!q.empty())
            {
                std::pair<int,int> curr  = q.front();
                q.pop();
                int i = curr.first;
                int j = curr.second;
                std::vector<std::pair<int,int>> neighbours = getNeighbours(i,j,grid_ref.size(),grid_ref[0].size());
                for(int k=0;k<neighbours.size();++k)
                {
                    int x_new = neighbours[k].first;
                    int y_new = neighbours[k].second;
                    if(visited[x_new][y_new] == true || frontier_tag[x_new][y_new]==true)
                        continue;
                    visited[x_new][y_new] = true;
                    
                    if(grid_ref[x_new][y_new]==0)
                        continue;
                    
                    else if(grid_ref[x_new][y_new]==-1)
                    {
                        std::vector<std::pair<int,int>> cell_neighbours = getNeighbours(x_new,y_new,grid_ref.size(),grid_ref[0].size());
                        for(int j =0;j<4;++j)
                        {
                            int x_prime_new = cell_neighbours[j].first;
                            int y_prime_new = cell_neighbours[j].second;
                            if(grid_ref[x_prime_new][y_prime_new]==1)
                            {
                                frontier_tag[x_new][y_new] = true;
                                break;
                            }
                        }
                        if(frontier_tag[x_new][y_new]==true)
                        {
                            std::queue<std::pair<int,int>> new_q;
                            new_q.push({x_new,y_new});
                            std::vector<std::pair<int,int>> frontier_cells;
                            frontier_cells.push_back({x_new,y_new});
                            while(!new_q.empty())
                            {
                                std::vector<std::pair<int,int>> cell_neighbors = getNeighbours(new_q.front().first,new_q.front().second,grid_ref.size(),grid_ref[0].size());
                                new_q.pop();
                                for(int j=0;j<cell_neighbors.size();++j)
                                {
                                    int x_prime_new = cell_neighbors[j].first;
                                    int y_prime_new = cell_neighbors[j].second;
                                    if(visited[x_prime_new][y_prime_new]==true || frontier_tag[x_prime_new][y_prime_new]==true)
                                        continue;
                                    visited[x_prime_new][y_prime_new] = true;
                                
                                    if(grid_ref[x_prime_new][y_prime_new]==-1)
                                    {
                                        std::vector<std::pair<int,int>> frontier_cell_neighbours = getNeighbours(x_prime_new,y_prime_new,grid_ref.size(),grid_ref[0].size());  
                                        for(int k=0;k<4;++k)
                                        {
                                            int x_prime_prime_new = frontier_cell_neighbours[k].first;
                                            int y_prime_prime_new = frontier_cell_neighbours[k].second;
                                            if(grid_ref[x_prime_prime_new][y_prime_prime_new]==1)
                                            {
                                                frontier_tag[x_prime_new][y_prime_new] = true;
                                                break;
                                            }
                                        }
                                        if(frontier_tag[x_prime_new][y_prime_new]==true)
                                        {
                                            new_q.push({x_prime_new,y_prime_new});
                                            frontier_cells.push_back({x_prime_new,y_prime_new});
                                        }                                      
                                    }
                                }
                            } 
                            Frontier frontier;
                            frontier.x = x_new;
                            frontier.y = y_new;
                            frontier.cells = frontier_cells;
                            frontiers.push_back(frontier);
                        }
                    }
                    else
                    {
                        q.push({x_new,y_new});
                   
                    }
                }

            }
        }

        void printElement()
        {
            std::vector<std::vector<int>>& grid_ref = *grid;
            int ele = grid_ref[0][0];
            std::cout<<ele<<std::endl;
        }

        std::vector<std::vector<int>>* grid;
        std::vector<Frontier> frontiers;
};