#include <stdio.h>
#include <stdlib.h>

int crossingBlackLine(int32_t *sensors, int oldLinesensorData[], int min, int max)
{
  int temp = 1;

  for (size_t i = min; i < max; i++)
  {

    //printf("%i, ", sensors[i]);

    if (oldLinesensorData[i] != 0 || sensors[i] != 128)
    {
      temp = 0;
    }

    oldLinesensorData[i] = sensors[i];
  }

  //printf("\n");

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