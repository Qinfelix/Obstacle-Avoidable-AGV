
#include"main.h"
#include"math.h"
#include <stdlib.h>
#include "obstacle.h"

#define k_rep 5000000//斥力系数
#define Q_star 20
#define test_R 5//检测范围
#define pi 3.1416


uint32_t filter(uint32_t* distance)
{
	uint32_t flag=0;
	uint32_t result=0;
	uint32_t sum=0;
	for(int i=0;i<10;i++)
	{
		if(*(distance+i)>10000)
			flag++;
		else
			sum+=*(distance+i);
	}
	if(flag>5)
		result=10000;
	else
		result=sum/(10-flag);
	 return result;
}

uint32_t get_r_square(int32_t x1, int32_t y1, int32_t x2,int32_t y2)
{
	uint32_t r_square;
	r_square = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)+1;
	return r_square;
}



uint32_t direction_output(uint32_t* distance)
{
  uint32_t repu[5];//初始的斥力势能
  uint32_t direction;//返回值
  uint32_t x;//监测点横坐标
  uint32_t y;//监测点纵坐标
  uint32_t min_energy;//最小势能值
  uint32_t r_square;//障碍物与监测点之间的坐标

  for (int j=0; j < 5 ;j++)
  {
      x = test_R * cos( (4-j) * pi / 4);
      y = test_R * sin( (4-j) * pi / 4);

      if (*(distance) <= Q_star+10)
      {
    	  //左侧障碍物，坐标为（-distance，0）
    	  r_square = get_r_square(x,y,-*(distance),0)	;
    	  repu[j] += k_rep/r_square ;
      }

      if ( *(distance+1)<= Q_star)
      {
    	  //左前障碍物，坐标为（-distance*0.7，distance+0.7）
    	  r_square = get_r_square(x,y,-*(distance+1)*0.707,*(distance+1)*0.707);
    	  repu[j] += k_rep/r_square ;
      }

      if (*(distance+2) <= Q_star)
      {
    	  //前方障碍物
    	  r_square=get_r_square(x, y, 0, *(distance+2));
    	  repu[j] += 2*k_rep/r_square ;
      }

      if (*(distance+3) <= Q_star)
      {
    	  //右前障碍物
    	  r_square=get_r_square(x, y, *(distance+3)*0.707, *(distance+3)*0.707);
    	  repu[j] += k_rep/r_square ;
      }

      if (*(distance+4) <= Q_star+10)
      {
    	  //右方障碍物
    	  r_square=get_r_square(x, y, *(distance+4), 0);
    	  repu[j] += k_rep/r_square ;
      }

      repu[j] += (200-20*y)*(200-20*y);
  }

  min_energy = repu[0];
  direction = 1 ;

  for (int j=0;j < 5; j++)
  {
      if (repu [j] < min_energy)
      {
          min_energy = repu[j];
          direction = j+1;
      }
  }

  return direction;

}


uint32_t get_yaw(uint8_t* imudata)
{
	uint32_t yaw;
	uint32_t begin_number;
	for(int i=0;i<11;i++)
	{
		if(*(imudata+i)==0x55 && *(imudata+1+i)==0x53)
		{
			begin_number=i+1;
			break;
		}
	}
	//begin_number是指帧头的位置，从1到11
	if(begin_number!=0)
	{
		  uint32_t sum=0;
		  for(int i=0;i<10;i++)
		  {
		   sum+=*(imudata+i+begin_number-1);
		  }
		  if(sum%256 == *(imudata+10+begin_number-1))
		  {
		   yaw=(*(imudata+6+begin_number-1) | (*(imudata+7+begin_number-1)<<8) )*180/32768;
		  }
	}
	 return yaw;
}




