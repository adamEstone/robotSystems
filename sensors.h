#include <stdio.h>
#include <stdlib.h>

int crossingBlackLine(int32_t *sensors, int oldLinesensorData[])
{
  int lineCrossed = 1;

  for (size_t i = 0; i < 7; i++)
  {

    //printf("%i, ", sensors[i]);

    if (oldLinesensorData[i] != 0 || sensors[i] != 128)
    {
      lineCrossed = 0;
    }

    oldLinesensorData[i] = sensors[i];
  }

  //printf("\n");
  if (lineCrossed == 1)
  {
    printf("Black line crossed\n");
    return 1;
  }
  else
  {
    return 0;
  }

  return 0;
}


enum
{
  irSensorLeft = 0,
  irSensorRight = 8,
  irSensorMiddle = 5
};


int laserTrigger(double laserpar[10], double dist, int irSensor) {



printf("dist: %f, linesensor: %f\n",dist, laserpar[irSensor]);

for (int i = 0; i < 9; i++)
{
  printf("%f, ", laserpar[i]);
}

printf("\n");

if (laserpar[irSensor] < dist){
  return 1;
}

return 0;

}