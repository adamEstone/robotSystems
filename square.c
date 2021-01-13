// DET VIRKER
/*
 * An example SMR program.
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#include "sensors.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

/*****************************************
* odometry
*/
#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 8000 //24902
#define sampletime 0.01

typedef struct
{                          //input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w;      // wheel separation
  double cr, cl; // meters per encodertick
  //output signals
  double right_pos, left_pos;
  // internal variables
  int left_enc_old, right_enc_old;

  double x, y, theta;
  double x_old, y_old, theta_old;

} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
* Motion control
*/

typedef struct
{ //input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle;
  double left_pos, right_pos;
  double deltaV;
  // parameters
  double w;
  //output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;

enum
{
  mot_stop = 1,
  mot_move,
  mot_turn,
  mot_direction
};

void update_motcon(motiontype *p);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
int direction(double deltaV, double speed, double dist, int time);

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);
void odometryLogToFile(double *data, int size);
void laserLogToFile(double data[10][1000], int size);
int lineSensorCali(int32_t sensor);
int findSensorRight(int32_t *sensors);
int findSensorLeft(int32_t *sensors);
int getSensorAngleRight(int sensor);
int getSensorAngleLeft(int sensor);
double calcVelocity(double targetVelo, double currVelo, double acc, double dist);

float centerOfMass(int distanceBetweenSensors, int *sensors);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_direction,
  ms_followlineLeft,
  ms_followlineRight,
  ms_end
};

int main()
{
  double odometryLog[100000];
  double laserLog[10][1000];
  int odometryCounter = 0, laserCounter = 0;
  int running, arg, time = 0;
  double dist = 0.0, angle = 0.0, angleDeg = 0.0, acc = 0.0, deltaV = 0.0, odoRef = 0.0;
  double targetVelo = 0.0, currVelo = 0.0, maxVelo = 0.0, wheelDist = 0.0;

  int oldLinesensorData[] = {128, 128, 128, 128, 128, 128, 128, 128};

  /* Establish connection to robot sensors and actuators.
   */
  if (rhdConnect('w', "localhost", ROBOTPORT) != 'w')
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);
  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config)
  {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (camsrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");
  }

  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config)
  {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lmssrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected)
    {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      len = sprintf(buf, "scanpush cmd='zoneobst'\n");
      send(lmssrv.sockfd, buf, len, 0);
    }
  }

  /* Read sensors and zero our position.
   */
  rhdSync();

  odo.w = 0.256;
  odo.cr = DELTA_M;
  odo.cl = odo.cr;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;

  while (running)
  {

    if (lmssrv.config && lmssrv.status && lmssrv.connected)
    {
      while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected)
    {
      while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    /****************************************
****************************************
****************************************
****************************************
****************************************
****************************************
****************************************
****************************************
****************************************
****************************************
****************************************/
    // mission statemachine
    sm_update(&mission);
    switch (mission.state)
    {
    case ms_init:
      dist = 8; //2
      angleDeg = 45;
      angle = angleDeg / 180 * M_PI;
      targetVelo = 0.6;
      acc = 1;
      wheelDist = (M_PI * WHEEL_DIAMETER) * (M_PI * WHEEL_SEPARATION) / (M_PI * WHEEL_DIAMETER) * angleDeg / 360;
      mission.state = ms_followlineRight;
      break;

    case ms_fwd:
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      if (fwd(dist, currVelo, mission.time))
        mission.state = ms_end;
      break;

    case ms_turn:
      if (targetVelo - currVelo > acc * sampletime)
        currVelo += acc * sampletime;
      else
        currVelo = targetVelo;

      maxVelo = sqrt(2 * acc * (wheelDist - (mot.right_pos - mot.startpos)));
      if (maxVelo < currVelo)
        currVelo = maxVelo;

      if (turn(angle, currVelo, mission.time))
          mission.state = ms_end;

      break;

    case ms_followlineLeft:
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      //printf("sensor l: %i sensor r: %i\n", findSensorLeft(linesensor->data, 0, 7), findSensorRight(linesensor->data, 0, 7));
      angleDeg = (double)getSensorAngleLeft(findSensorLeft(linesensor->data));
      //printf("Deg %f %f\n", (double)getSensorAngleLeft(findSensorLeft(linesensor->data, 0, 7)), angleDeg);
      //printf("odoRef %f\n", odoRef);
      // lineSensorLeft turn debug
      /*for (int i = 0; i < 8; i++)
        printf("[%d]", lineSensorCali(linesensor->data[i]));
      printf("\n");*/
      //printf("speed: %f \tCurrVelo: %f \n", deltaV, currVelo);
      odoRef = odo.theta - (angleDeg * M_PI / 180);
      deltaV = currVelo * (odoRef - odo.theta);
      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_end;

      break;

    case ms_followlineRight:
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      //printf("sensor l: %i sensor r: %i\n", findSensorRight(linesensor->data, 0, 7), findSensorRight(linesensor->data, 0, 7));
      angleDeg = (double)getSensorAngleRight(findSensorRight(linesensor->data));
      //printf("Deg %f %f\n", (double)getSensorAngleRight(findSensorRight(linesensor->data, 0, 7)), angleDeg);
      //printf("odoRef %f\n", odoRef);

      // lineSensorRight turn debug
      /*
      for (int i = 0; i < 8; i++)
        printf("[%d]", lineSensorCali(linesensor->data[i]));
      printf("\n");
      */
      //deltaV = currVelo * (odoRef - odo.theta);
      //printf("speed: %f \tCurrVelo: %f \n", deltaV, currVelo);

      odoRef = odo.theta - (angleDeg * M_PI / 180);
      deltaV = currVelo * (odoRef - odo.theta);
      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_end;

      break;

    case ms_direction:
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      odoRef = angleDeg * M_PI / 180; // VARIABEL TIL VINKEL SOM SKAL DREJES MED

      if (odo.theta < 0 && odoRef > 0)
        deltaV = currVelo * (odoRef - odo.theta - (2 * M_PI));
      else if (odo.theta > 0 && odoRef < 0)
        deltaV = currVelo * (odoRef - odo.theta + (2 * M_PI));
      else
        deltaV = currVelo * (odoRef - odo.theta);

      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_end;

      break;

    case ms_end:
      mot.cmd = mot_stop;
      running = 0;
      break;
    }
    /*  end of mission  */

    odometryLog[odometryCounter] = (double)mission.time;
    odometryLog[odometryCounter + 1] = odo.x;
    odometryLog[odometryCounter + 2] = odo.y;
    odometryLog[odometryCounter + 3] = odo.theta;
    odometryCounter += 4;

    for (int i = 0; i < 10; i++)
    {
      laserLog[i][laserCounter] = laserpar[i];
    }
    laserCounter++;

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0)
      //    printf(" laser %f \n",laserpar[3]);
      time++;
    /* stop if keyboard is activated
*
*/
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
      running = 0;

  } /* end of main control loop */
  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;

  odometryLogToFile(odometryLog, odometryCounter);
  laserLogToFile(laserLog, laserCounter);

  rhdSync();
  rhdDisconnect();

  exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;

  p->x_old = 0.5;
  p->y_old = 2.0;
  p->theta_old = 0.0;
}

void update_odo(odotype *p)
{
  int delta;
  double tempDU;
  double tempDTheta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;

  double uR = delta * p->cr;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;

  double uL = delta * p->cl;

  tempDU = (uR + uL) / 2;
  tempDTheta = (uR - uL) / WHEEL_SEPARATION;

  p->theta = p->theta_old + tempDTheta;
  if (p->theta > M_PI || p->theta < -M_PI)
  {
    p->theta *= -1;
  }

  p->x = p->x_old + tempDU * cos(p->theta);
  p->y = p->y_old + tempDU * sin(p->theta);

  p->x_old = p->x;
  p->y_old = p->y;
  p->theta_old = p->theta;
}

void update_motcon(motiontype *p)
{
  if (p->cmd != 0)
  {

    p->finished = 0;
    switch (p->cmd)
    {
    case mot_stop:
      p->curcmd = mot_stop;
      break;
    case mot_move:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    case mot_turn:
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;
      p->curcmd = mot_turn;
      break;

    case mot_direction:
      p->curcmd = mot_direction;
      break;
    }

    p->cmd = 0;
  }

  switch (p->curcmd)
  {
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;
  case mot_move:
    if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist)
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else
    {
      p->motorspeed_l = p->speedcmd;
      p->motorspeed_r = p->speedcmd;
    }
    break;

  case mot_turn:
    if (p->angle > 0)
    {
      p->motorspeed_l = 0;
      if (p->right_pos - p->startpos < (p->angle * p->w) / 2)
      {
        p->motorspeed_r = p->speedcmd;
        p->motorspeed_l = -p->speedcmd;
      }
      else
      {
        p->motorspeed_r = 0;
        p->finished = 1;
      }
    }
    else
    {
      p->motorspeed_r = 0;
      if (p->left_pos - p->startpos < fabs(p->angle) * p->w)
      {
        p->motorspeed_l = p->speedcmd;
      }
      else
      {
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    break;

  case mot_direction:
    if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist)
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else
    {
      if (-p->speedcmd > p->speedcmd - p->deltaV)
        p->motorspeed_l = -p->speedcmd;
      else if (p->speedcmd < p->speedcmd - p->deltaV)
        p->motorspeed_l = p->speedcmd;
      else
        p->motorspeed_l = p->speedcmd - p->deltaV;

      if (-p->speedcmd > p->speedcmd + p->deltaV)
        p->motorspeed_r = -p->speedcmd;
      else if (p->speedcmd < p->speedcmd + p->deltaV)
        p->motorspeed_r = p->speedcmd;
      else
        p->motorspeed_r = p->speedcmd + p->deltaV;
    }
    break;
  }
}

int fwd(double dist, double speed, int time)
{
  mot.speedcmd = speed;
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.dist = dist;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed, int time)
{
  mot.speedcmd = speed;
  if (time == 0)
  {
    mot.cmd = mot_turn;
    mot.angle = angle;
    return 0;
  }
  else
    return mot.finished;
}

int direction(double deltaV, double speed, double dist, int time)
{
  mot.speedcmd = speed;
  mot.deltaV = deltaV;

  if (time == 0)
  {
    mot.cmd = mot_direction;
    mot.dist = dist;
    return 0;
  }
  else
    return mot.finished;
}

void sm_update(smtype *p)
{
  if (p->state != p->oldstate)
  {
    p->time = 0;
    p->oldstate = p->state;
  }
  else
  {
    p->time++;
  }
}

void odometryLogToFile(double *data, int size)
{
  FILE *pFile = NULL;

  pFile = fopen("odometry_log.txt", "w");
  for (size_t i = 0; i < size - 1; i += 4)
  {
    fprintf(pFile, "%f %f %f %f\n", data[i], data[i + 1], data[i + 2], data[i + 3]);
  }

  fclose(pFile);
}

void laserLogToFile(double data[10][1000], int size)
{
  FILE *pFile = NULL;

  pFile = fopen("laser_log.txt", "w");
  for (size_t i = 0; i < 800 - 1; i++)
  {
    fprintf(pFile, "%f %f %f %f %f %f %f %f %f\n", data[0][i], data[1][i], data[2][i], data[3][i], data[4][i], data[5][i], data[6][i], data[7][i], data[8][i]);
  }

  fclose(pFile);
}

int lineSensorCali(int32_t sensor)
{
  if (sensor > 0)
    return 1;

  return 0;
}

int findSensorRight(int32_t *sensors)
{
  int value = 1;
  int index = -1;
  for (size_t i = 0; i <= 7; i++)
  {
    //printf("%ld:%d ",i, sensors[i]);
    if (lineSensorCali(sensors[i]) < value)
    {
      return i;
    }
  }
  //makeprintf("\n");
  return index;
}

int findSensorLeft(int32_t *sensors)
{
  int value = 1;
  int index = -1;
  for (size_t i = 7; i > 0; i--)
  {
    //printf("%ld:%d ",i, sensors[i]);
    if (lineSensorCali(sensors[i]) < value)
    {
      return i;
    }
  }
  //printf("\n");
  return index;
}

int getSensorAngleRight(int sensor)
{
  switch (sensor)
  {
  case 0:
    return 20;
    break;
  case 1:
    return 15;
    break;
  case 2:
    return 10;
    break;
  case 3:
    return 5;
    break;
  case 4:
    return 0;
    break;
  case 5:
    return -10;
    break;
  case 6:
    return -15;
    break;
  case 7:
    return -20;
    break;

  default:
    break;
  }
  return 20;
}

int getSensorAngleLeft(int sensor)
{
  switch (sensor)
  {
  case 0:
    return 20;
    break;
  case 1:
    return 15;
    break;
  case 2:
    return 10;
    break;
  case 3:
    return 0;
    break;
  case 4:
    return -5;
    break;
  case 5:
    return -10;
    break;
  case 6:
    return -15;
    break;
  case 7:
    return -20;
    break;

  default:
    break;
  }
  return -20;
}

//xi er afstand fra center af linje sensorene til den individuelle sensor
float centerOfMass(int distanceBetweenSensors, int *sensors)
{
  int numberOfSensors = 8;
  float sensorSum = 0;
  float sensorSumTimesXi = 0;

  float center = ((numberOfSensors - 1)) / 2;

  for (int i = 0; i < numberOfSensors; i++)
  {
    sensorSum += 1 - lineSensorCali(sensors[i]);
  }

  //calc Xi * I_i
  for (int i = 0; i < numberOfSensors; i++)
  {
    sensorSumTimesXi += i * (1 - lineSensorCali(sensors[i]));
  }

  //debug
  for (int i = 0; i < numberOfSensors; i++)
  {
    printf("[%d]", lineSensorCali(sensors[i]));
  }
  printf("\n");
  printf("Center of Mass: %f \n", (center - (sensorSumTimesXi / sensorSum)) * distanceBetweenSensors);

  return (center - (sensorSumTimesXi / sensorSum)) * distanceBetweenSensors;
}

double calcVelocity(double targetVelo, double currVelo, double acc, double dist)
{
  double maxVelo = 0.0;
  if (targetVelo - currVelo > acc * sampletime)
  {
    currVelo += acc * sampletime;
  }
  else
  {
    currVelo = targetVelo;
  }

  maxVelo = sqrt(2 * acc * (dist - (mot.right_pos + mot.left_pos) / 2) + mot.startpos);
  if (maxVelo < currVelo)
  {
    currVelo = maxVelo;
  }

  return currVelo;
}