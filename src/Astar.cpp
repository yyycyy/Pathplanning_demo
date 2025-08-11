#include <SFML/Graphics.hpp>
#include <iostream>
#include <list>
#include <queue>
#include <vector>
static const int GRID_SIZE = 20;
static const int CELL_SIZE = 40;

struct Location {
  int x, y;
  bool operator==(const Location& other) const noexcept { return x == other.x && y == other.y; }
  bool operator!=(const Location& other) const noexcept { return x != other.x || y != other.y; }
};
struct LocationCost
{
   Location location;
   float cost;
};

namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <> struct hash<Location> {
  std::size_t operator()(const Location& id) const noexcept {
    // I wish built-in std::hash worked on pair and tuple
    return std::hash<int>()(id.x ^ (id.y << 16));
  }
};
}

struct CompareCost {
    bool operator()(const LocationCost& a, const LocationCost& b) const {
        return a.cost > b.cost;   // 小顶堆
    }
};

class AstarPlanner
{
private:
    std::array<Location,4> DIRS1={
        Location{1,0},Location{0,1},Location{-1,0},Location{0,-1}};
    std::array<Location,4> DIRS2={
        Location{1,1},Location{-1,1},Location{1,-1},Location{-1,-1}};

    std::vector<LocationCost> neighbors (std::vector<std::vector<sf::Color>> clockMap,Location current)const;
    float ManhattanDistance(Location current,Location goal);
    void get_InitialDate(std::vector<std::vector<sf::Color>> ColorMap,Location& start,Location& goal);
    void search(std::vector<std::vector<sf::Color>> ColorMap,
                    Location satar,Location goal,
                    std::unordered_map<Location,Location>& came_from,
                    std::unordered_map<Location,float>& cost_so_far);
public:

    std::vector<Location> reconstruct_path(Location start,Location goal,std::unordered_map<Location,Location>& came_from);
    void Astar_search(std::vector<std::vector<sf::Color>>& ColorMap);

    AstarPlanner();
    ~AstarPlanner();


};
AstarPlanner::AstarPlanner()
{  

}

AstarPlanner::~AstarPlanner()
{
}


void AstarPlanner::get_InitialDate(std::vector<std::vector<sf::Color>> ColorMap,Location& start,Location& goal)
{
    for (int x = 0; x < GRID_SIZE; x++){
       for (int y = 0; y < GRID_SIZE; y++){
            if(ColorMap[y][x]==sf::Color::Blue){
                start.x=x;
                start.y=y;
            }else if(ColorMap[y][x]==sf::Color::Red){
                goal.x=x;
                goal.y=y;
            }
       }
    }

    std::cout<<"start:x:"<<start.x<<"y:"<<start.y<<std::endl;
    std::cout<<"goal:x:"<<goal.x<<"y:"<<goal.y<<std::endl;

}


void AstarPlanner::Astar_search(std::vector<std::vector<sf::Color>>& ColorMap)
{

    Location start={0,0},goal={0,0};
    get_InitialDate(ColorMap,start,goal);
    std::unordered_map<Location,Location> came_from;
    std::unordered_map<Location,float> cost_so_far;
    search(ColorMap,start,goal,came_from,cost_so_far);
    std::vector<Location> path = reconstruct_path(start,goal,came_from);

    for (Location current:path)
    {
        ColorMap[current.y][current.x]=sf::Color::Green;
    }

    ColorMap[goal.y][goal.x]=sf::Color::Red;

}

 std::vector<Location> AstarPlanner::reconstruct_path(Location start,Location goal,std::unordered_map<Location,Location>& came_from)
 {
    std::vector<Location> path;
    if(came_from.find(goal)==came_from.end()){ 
        return std::vector<Location>{};
    }

    Location current = goal;
    while (current!=start)
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
    return path;
 }

float AstarPlanner::ManhattanDistance(Location current,Location goal)
{
    return float(abs(current.x-goal.x)+abs(current.y-goal.x));
}

std::vector<LocationCost> AstarPlanner::neighbors (std::vector<std::vector<sf::Color>> colorMap,Location current)const
{
    std::vector<LocationCost> results;
    for(Location id:DIRS1)
    {
        LocationCost next={{id.x+current.x,id.y+current.y},999};
        if(next.location.x>=0&&next.location.y>=0&&next.location.x<GRID_SIZE && next.location.y<GRID_SIZE)
        {
            if(colorMap[next.location.y][next.location.x]==sf::Color::Black)
            {

                continue;
            }else if( colorMap[next.location.y][next.location.x]==sf::Color::White||
                      colorMap[next.location.y][next.location.x]==sf::Color::Green||
                      colorMap[next.location.y][next.location.x]==sf::Color::Red){
                next.cost=1;
                results.push_back(next);
                continue;
            }else if(colorMap[next.location.y][next.location.x]==sf::Color::Cyan){
                next.cost=5;
                results.push_back(next);
                continue;
            }
        }
    }
    for (Location id:DIRS2)
    {
        LocationCost next={{id.x+current.x,id.y+current.y},999};
        if(next.location.x>=0&&next.location.y>=0 && next.location.x<GRID_SIZE && next.location.y<GRID_SIZE)
        {
            if(colorMap[next.location.y][next.location.x]==sf::Color::Black)
            {
                continue;
            }else if(colorMap[next.location.y][next.location.x]==sf::Color::White||
                     colorMap[next.location.y][next.location.x]==sf::Color::Red||
                     colorMap[next.location.y][next.location.x]==sf::Color::Green){
                next.cost=1.4f;
                results.push_back(next);
                continue;
            }else if(colorMap[next.location.y][next.location.x]==sf::Color::Cyan){
                next.cost=7.1f;
                results.push_back(next);
                continue;
            }
        }

    }
    return results;
}

void AstarPlanner::search(std::vector<std::vector<sf::Color>> ColorMap,
                        Location start,Location goal,
                        std::unordered_map<Location,Location>& came_from,
                        std::unordered_map<Location,float>& cost_so_far)
{
    std::priority_queue<LocationCost,std::vector<LocationCost>,CompareCost> frontier;

    LocationCost Start = {start,0};
    frontier.push(Start);
    came_from[start]=start;
    cost_so_far[start]=0;
    while ( !frontier.empty() )
    {
        //读取代价最小单元格
        Location current = frontier.top().location;
        //读取后删除
        frontier.pop();
        //到达目标退出
        if(current == goal){ break;}
        //寻找相邻代价最单元格 
        for(LocationCost next:neighbors(ColorMap,current))
        {
            float newcost = cost_so_far[current]+next.cost;
            if(cost_so_far.find(next.location)==cost_so_far.end()||newcost<cost_so_far[next.location])
            {
                cost_so_far[next.location]=newcost;
                LocationCost priority={next.location,newcost+ManhattanDistance(next.location,goal)};
                came_from[next.location]=current;
                frontier.push(priority);
            }
        }
    }
}

class MAPGenerator
{
private:
    /* data */
    sf::RenderWindow window;
public:

    std::vector<std::vector<sf::Color>> grid;
    bool is_ok;

    void draw(std::vector<std::vector<sf::Color>> grid);
    MAPGenerator(/* args */);
    ~MAPGenerator(){};
    void upDate();
};

MAPGenerator::MAPGenerator(/* args */)
{
    is_ok=0;
    //初始化网格
    window.create(sf::VideoMode(GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE), "AstarText");
    grid.resize(GRID_SIZE, std::vector<sf::Color>(GRID_SIZE, sf::Color::White));
    // 绘制网格
    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
            cell.setPosition(x * CELL_SIZE, y * CELL_SIZE);
            cell.setFillColor(grid[y][x]);
            window.draw(cell);
        }
    }
    window.display();
}


void MAPGenerator::upDate()
{
    if(window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
               window.close();
                //鼠标点击事件
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    int x = event.mouseButton.x / CELL_SIZE;
                    int y = event.mouseButton.y / CELL_SIZE;
                    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
                        // 切换颜色：白 -> 红 -> 绿 -> 蓝 -> 白
                        if (grid[y][x] == sf::Color::White) grid[y][x] = sf::Color::Red;
                        else if (grid[y][x] == sf::Color::Red) grid[y][x] = sf::Color::Green;
                        else if (grid[y][x] == sf::Color::Green) grid[y][x] = sf::Color::Blue;
                        else grid[y][x] = sf::Color::White;
                    }
                }
                if(event.mouseButton.button ==sf::Mouse::Right){
                    int x = event.mouseButton.x / CELL_SIZE;
                    int y = event.mouseButton.y / CELL_SIZE;
                    if (x>=0&&x<GRID_SIZE&&y>=0&&y<GRID_SIZE){
                        if (grid[y][x] == sf::Color::Black)  grid[y][x] = sf::Color::Cyan;
                            else grid[y][x] = sf::Color::Black;
                    }
                }
            }
        }
        window.clear();
        // 绘制网格
        for (int y = 0; y < GRID_SIZE; ++y) {
            for (int x = 0; x < GRID_SIZE; ++x) {
                sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
                cell.setPosition(x * CELL_SIZE, y * CELL_SIZE);
                cell.setFillColor(grid[y][x]);
                window.draw(cell);
            }
        }
        if(grid[0][0]==sf::Color::Blue) {
            is_ok=1;
            grid[0][0]=sf::Color::White;
        }
        window.display();  
    }
}


int main() {
    MAPGenerator mm;
    AstarPlanner astar;
    while (1)
    {
        mm.upDate();
        if(mm.is_ok)
        {
            astar.Astar_search(mm.grid);
            mm.is_ok=0; 
        } 
    }
    return 0;
}
