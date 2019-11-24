#ifndef MAZE_H
#define MAZE_H


class Maze
{
  public:
    byte verticalWalls[16][16];
    byte horizontalWalls[16][16];
    byte startY = 5;//15
    byte startX = 0;//0
    byte endPoints[4][2] = {{3,2},{3,3},{4,2},{4,3}};//{7,8},{7,9},{8,8},{8,9}

    Maze() {}
    ~Maze() {}



};

#endif
