#ifndef SERVO_H
#define SERVO_H

void grabber_lift(uint32_t angle);//抬起固定角度
void grabber_move(uint32_t angle);//手爪抓取一定角度
void camera_lift(uint32_t angle);//抬起摄像头

void grab();//抓取货物
void drop();//放下货物


#endif
