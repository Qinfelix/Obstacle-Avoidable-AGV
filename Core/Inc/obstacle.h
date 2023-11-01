#ifndef OBSTACLE_H
#define OBSTACLE_H

uint32_t get_imu_instruction(uint32_t* imu_data);
uint32_t filter(uint32_t* distance);
uint32_t get_r_square(int32_t x1, int32_t y1, int32_t x2,int32_t y2);
uint32_t direction_output(uint32_t* distance);
uint32_t get_yaw(uint8_t* imudata);

#endif
