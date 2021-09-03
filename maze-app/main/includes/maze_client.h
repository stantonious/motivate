#pragma once

TaskHandle_t MazeClientHandle;

void maze_client_init(void);
void get_maze(int maze_id,int x,int y,int m[][x],int* entry_x,int* entry_y,int* exit_x,int* exit_y);