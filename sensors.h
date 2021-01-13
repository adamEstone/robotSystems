#include <stdio.h>
#include <stdlib.h>

int crossingBlackLine(int32_t *sensors)
{
  int lineCrossed = 1;

  for (size_t i = 1; i < 6; i++)
  {
    printf("%i, ", sensors[i]);
    if (sensors[i] != 0)
      temp = 0;
  }

  printf("\n");

  if (temp == 1)
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

/*printf("dist: %f, linesensor: %f\n",dist, laserpar[irSensor]);

for (int i = 0; i < 9; i++)
{
  printf("%f, ", laserpar[i]);
}

printf("\n");

*/

if (laserpar[irSensor] < dist){
  return 1;
}

return 0;

}