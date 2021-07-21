#pragma once

TaskHandle_t MazeClientHandle;

void maze_client_init(void);
void maze_client_task(void *pvParameters);
