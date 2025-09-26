#include <opencv2/opencv.hpp>
#include <random>
#include <vector>
#include <iostream>
#include <random>
#include <cmath>

class RandomMapGenerator
{
public:
  RandomMapGenerator(int width = 800, int height = 600, int num_obstacles = 10, int min_size = 30, int max_size = 80)
      : width(width), height(height), num_obstacles(num_obstacles),
        min_size(min_size), max_size(max_size)
  {
    map = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255)); // 白色背景
  }
  cv::Mat generateMap()
  {
    map.setTo(cv::Scalar(255, 255, 255)); // 重置为白色
    obstacles.clear();
    int attempts = 0;
    while ((int)obstacles.size() < num_obstacles && attempts < num_obstacles * 50)
    {
      cv::Mat shape_mask = cv::Mat::zeros(height, width, CV_8UC1);
      drawRandomShape(shape_mask);

      if (!isOverlap(shape_mask))
      {
        obstacles.push_back(shape_mask);
        map.setTo(cv::Scalar(0, 0, 0), shape_mask); // 将障碍物绘制为黑色
      }
      attempts++;
    }
    return map;
  }
  void showMap(cv::Mat map, int wait, const std::string &window_name = "Random Map")
  {
    cv::imshow(window_name, map);
    cv::waitKey(wait);
    // cv::destroyAllWindows();
  }
  cv::Mat map;

private:
  int width, height;
  int num_obstacles;
  int min_size, max_size;
  std::vector<cv::Mat> obstacles;
  const int exclude_margin_x = 150;
  const int exclude_margin_y = 150;
  bool isOverlap(const cv::Mat &mask)
  {
    for (auto &obs : obstacles)
    {
      cv::Mat overlap;
      cv::bitwise_and(obs, mask, overlap);
      if (cv::countNonZero(overlap) > 0)
        return true;
    }
    return false;
  }

  void drawRandomShape(cv::Mat &mask)
  {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // 障碍物中心x坐标区间（避开左上和右下x区域）
    std::vector<std::pair<int, int>> allowed_x_ranges = {
        {50 + max_size, exclude_margin_x},           // 左上角右侧安全区间
        {exclude_margin_x, width - exclude_margin_x} // 中间大区域
    };
    // 障碍物中心y坐标区间（避开左上和右下y区域）
    std::vector<std::pair<int, int>> allowed_y_ranges = {
        {50 + max_size, exclude_margin_y},
        {exclude_margin_y, height - exclude_margin_y}};

    // 生成cx时，先随机选一个区间
    std::uniform_int_distribution<> distRangeX(0, (int)allowed_x_ranges.size() - 1);
    int x_range_index = distRangeX(gen);
    std::uniform_int_distribution<> distX(allowed_x_ranges[x_range_index].first, allowed_x_ranges[x_range_index].second);
    int cx = distX(gen);

    // 生成cy同理
    std::uniform_int_distribution<> distRangeY(0, (int)allowed_y_ranges.size() - 1);
    int y_range_index = distRangeY(gen);
    std::uniform_int_distribution<> distY(allowed_y_ranges[y_range_index].first, allowed_y_ranges[y_range_index].second);
    int cy = distY(gen);

    std::uniform_int_distribution<> distSize(min_size, max_size);
    std::uniform_int_distribution<> distShape(0, 2);

    int size = distSize(gen);
    int shape_type = distShape(gen);

    if (shape_type == 0)
    { // 矩形
      cv::Rect rect(cx - size, cy - size, size * 2, size * 2);
      cv::rectangle(mask, rect, cv::Scalar(255), -1);
    }
    else if (shape_type == 1)
    { // 圆形
      cv::circle(mask, cv::Point(cx, cy), size / 2, cv::Scalar(255), -1);
    }
    else
    { // 三角形
      std::vector<cv::Point> pts = {
          cv::Point(cx, cy - size),
          cv::Point(cx - size, cy + size),
          cv::Point(cx + size, cy + size)};
      cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{pts}, cv::Scalar(255));
    }
  }
};

struct Location
{
  int y, x;
  bool operator==(const Location &other) const noexcept { return x == other.x && y == other.y; }
  bool operator!=(const Location &other) const noexcept { return x != other.x || y != other.y; }
};

struct LocationCost
{
  Location location;
  double cost;
};
struct CompareCost
{
  bool operator()(const LocationCost &a, const LocationCost &b) const
  {
    return a.cost > b.cost; // 小顶堆
  }
};

namespace std
{
  /* implement hash function so we can put GridLocation into an unordered_set */
  template <>
  struct hash<Location>
  {
    std::size_t operator()(const Location &id) const noexcept
    {
      // I wish built-in std::hash worked on pair and tuple
      return std::hash<int>()(id.x ^ (id.y << 16));
    }
  };
}

/*********************
    auto nearest_it = std::min_element(VERTEX.begin(), VERTEX.end(),
        [&loc](const Location &a, const Location &b) {
        double dist_a = std::hypot(a.x - loc.x, a.y - loc.y);
        double dist_b = std::hypot(b.x - loc.x, b.y - loc.y);
        return dist_a < dist_b; });
    if (nearest_it != VERTEX.end()) {
        Location nearest = *nearest_it; // nearest 就是距离 loc 最近的节点
    }
**********************/
class RRT
{
private:
  /* data */
public:
  RandomMapGenerator generator;
  RRT() : generator(800, 800, 18)
  {
    generator.map = generator.generateMap();
  }

  void RRT_search(cv::Mat &map, Location start, Location goal, int step_size)
  {

    cv::circle(map, cv::Point(start.x, start.y), 10, cv::Scalar(255, 0, 255), -1);
    cv::circle(map, cv::Point(goal.x, goal.y), 10, cv::Scalar(255, 0, 0), -1);
    generator.showMap(map, 1);
    std::unordered_map<Location, Location> came_from;
    search(map, start, goal, step_size, came_from);
    std::vector<Location> path = reconstruct_path(start, goal, came_from);

    for (Location loc : path)
    {
      cv::circle(map, cv::Point(loc.x, loc.y), 5, cv::Scalar(0, 0, 255), -1);
    }
    for (size_t i = 1; i < path.size(); ++i)
    {
      cv::line(map,
               cv::Point(path[i - 1].x, path[i - 1].y),
               cv::Point(path[i].x, path[i].y),
               cv::Scalar(255, 0, 0), 2); // 蓝色线，宽度2
    }
    generator.showMap(map, 0);
  }

  std::vector<Location> reconstruct_path(Location start, Location goal,
                                         std::unordered_map<Location, Location> &came_from)
  {

    std::vector<Location> path;
    if (came_from.find(goal) == came_from.end())
    {
      return std::vector<Location>{};
    }
    Location current = goal;
    while (current != start)
    {
      path.push_back(current);
      current = came_from[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<Location> randomLocation(int Xmax, int Ymax, int size)
  {
    std::vector<Location> loc;
    std::random_device rd;  // 随机数生成器
    std::mt19937 gen(rd()); // 以随机设备作为种子
    std::uniform_int_distribution<> disX(0, Xmax - 1);
    std::uniform_int_distribution<> disY(0, Ymax - 1);
    for (int i = 0; i < size; i++)
    {
      Location LOC = {disY(gen), disX(gen)};
      // 将坐标点添加到列表中
      loc.push_back(LOC);
    }
    return loc;
  }

  void search(cv::Mat &map, Location start, Location goal, int step_size,
              std::unordered_map<Location, Location> &came_from)
  {
    //用于储存起点到某一点的代价
    std::unordered_map<Location,double> F;
    F[start]=0.0f;
    came_from[start] = start;
    std::vector<Location> VERTEX;
    std::random_device rd;  // 随机数生成器
    std::mt19937 gen(rd()); // 以随机设备作为种子
    std::uniform_int_distribution<> disX(0, map.cols - 1);
    std::uniform_int_distribution<> disY(0, map.rows - 1);
    VERTEX.push_back(start);
    while (start != goal)
    {
      // 创建优先队列
      std::priority_queue<LocationCost, std::vector<LocationCost>, CompareCost> frontier;
      // 每次生成一组坐标
      Location loc = {disY(gen), disX(gen)};

      for (Location vnode : VERTEX)
      {
        LocationCost nowCost;
        // 计算欧拉距离
        double Euler_distance = std::hypot(loc.x - vnode.x, loc.y - vnode.y);
        // 将结果添加到优先队列
        nowCost.location = vnode;
        nowCost.cost = Euler_distance;
        frontier.push(nowCost);
      }

      double dx = loc.x - frontier.top().location.x;
      double dy = loc.y - frontier.top().location.y;

      double raw_new_x = frontier.top().location.x + (dx / frontier.top().cost) * step_size;
      double raw_new_y = frontier.top().location.y + (dy / frontier.top().cost) * step_size;
      
      Location new_loc;
      new_loc.x = static_cast<int>(std::floor(raw_new_x)) + 1;
      new_loc.y = static_cast<int>(std::floor(raw_new_y)) + 1;
      static int count=0;
      // 判断是否到达目标点附近
      if (std::abs(new_loc.x - goal.x) < 10 && std::abs(new_loc.y - goal.y) < 10)
      {
        count++;
        if(count==10){
        came_from[new_loc] = frontier.top().location;
        came_from[goal] = new_loc;
        break;
        }
      }
      // 是否超出边界
      if (new_loc.x < 0 || new_loc.x >= map.cols || new_loc.y < 0 || new_loc.y >= map.rows)
        continue;
      cv::Vec3b color = map.at<cv::Vec3b>(new_loc.y, new_loc.x);
      // 判断是否为障碍物
      if (!isLineObstacleFree(map, new_loc, frontier.top().location))
        continue;
      // 不是障碍物

      ///提取邻居点并找到更优化父节点
      std::vector<Location> loc_near;
      std::priority_queue<LocationCost, std::vector<LocationCost>, CompareCost> T;
      for(Location ploc : VERTEX){
        double Euler_distance = std::hypot(ploc.x-new_loc.x,ploc.y-new_loc.y);
        if(Euler_distance<50){
          loc_near.push_back(ploc);
          LocationCost nowCost;
          nowCost.location = ploc;
          nowCost.cost = F[ploc]+Euler_distance;
          T.push(nowCost);
        }
      }
      // 判断是否为障碍物,如果新路经过障碍使用优化前路径
      if (!isLineObstacleFree(map, new_loc,T.top().location)){
          F[new_loc] = F[frontier.top().location]+std::hypot(frontier.top().location.x-new_loc.x,frontier.top().location.y-new_loc.y);
          came_from[new_loc] = frontier.top().location;
      }else{
         F[new_loc] = T.top().cost;
          came_from[new_loc] = T.top().location;
      }
      //邻居节点重连
      for(Location near: loc_near){
        double Euler_distance = std::hypot(near.x-new_loc.x,near.y-new_loc.y);
        if( F[new_loc]+Euler_distance <F[near] && isLineObstacleFree(map,new_loc,near)){

          came_from[near]=new_loc;
          F[near] = F[new_loc]+Euler_distance;
        }
      }

      //came_from[new_loc] = frontier.top().location;
      VERTEX.push_back(new_loc);
      cv::circle(map, cv::Point(frontier.top().location.x, frontier.top().location.y), 5, cv::Scalar(0, 255, 0), -1);
      //  cv::line(map,
      //  cv::Point(path[i-1].x, path[i-1].y),
      //  cv::Point(path[i].x, path[i].y),
      //  cv::Scalar(255, 0, 0), 2);  // 蓝色线，宽度2
      generator.showMap(map,1);
    }
  }

  bool isLineObstacleFree(const cv::Mat &map, const Location &p1, const Location &p2)
  {
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    int x = x1;
    int y = y1;

    while (true)
    {
      // 判断当前点是否为障碍物
      cv::Vec3b color = map.at<cv::Vec3b>(y, x);
      if (color[0] == 0 && color[1] == 0 && color[2] == 0)
      {
        return false; // 发现障碍物
      }
      if (x == x2 && y == y2)
        break;

      int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y += sy;
      }
    }
    return true; // 线段无障碍物
  }

  ~RRT() {}
};

int main()
{
  // RandomMapGenerator generator;
  RRT rrt;
  Location start{50, 50}, goal{700, 700};
  rrt.RRT_search(rrt.generator.map, start, goal, 10);

  return 0;
}
