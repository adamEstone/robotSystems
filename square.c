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

enum
{
  exit_dist,
  exit_crossBlackLine,
  exit_irDistRightLess,
  exit_irDistLeftLess,
  exit_irDistMiddelLess,
  exit_irDistRightLarger,
  exit_irDistLeftLarger,
  exit_irDistMiddelLarger
};

void update_motcon(motiontype *p, int exitC, int32_t *sensors);

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
int lineSensorCaliBlack(int32_t sensor);
int lineSensorCaliWhite(int32_t sensor);
int findSensorRight(int32_t *sensors);
int findSensorLeft(int32_t *sensors);
int getSensorAngleRight(int sensor);
int getSensorAngleLeft(int sensor);
double calcVelocity(double targetVelo, double currVelo, double acc, double dist);
double calcAngle(double angleDeg);
double centerOfMass(int distanceBetweenSensors, int *sensors, int blackLine);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum
{
  ms_init,
  ms_nextState,
  ms_fwd,
  ms_turn,
  ms_direction,
  ms_followBlackLineLeft,
  ms_followBlackLineRight,
  ms_followBlackLineMiddle,
  ms_followWhiteLineMiddle,
  ms_end,
  ms_calcDistance,
  ms_readGotlBot,
  ms_readGotlTop
};

int main()
{
  double odometryLog[100000];
  double laserLog[10][1000];
  int odometryCounter = 0, laserCounter = 0, stateCounter = 0;
  int running, arg, time = 0, exitCondition = 0;
  double dist = 0.0, angle = 0.0, angleDeg = 0.0, acc = 0.0, deltaV = 0.0, odoRef = 0.0, irOffset = 0.0;
  double targetVelo = 0.0, currVelo = 0.0, maxVelo = 0.0, wheelDist = 0.0;

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

  sleep(1);
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

    // mission statemachine
    sm_update(&mission);

    switch (mission.state)
    {
    case ms_init:
      dist = 1;
      stateCounter = 0;
      targetVelo = 0.2;
      acc = 1;
      irOffset = 0.26;
      wheelDist = (M_PI * WHEEL_DIAMETER) * (M_PI * WHEEL_SEPARATION) / (M_PI * WHEEL_DIAMETER) * angleDeg / 360;
      mission.state = ms_nextState;
      break;

    case ms_nextState:
      stateCounter++;
      printf("statecount: %d\n", stateCounter);

      switch (stateCounter)
      {
      case 1: // followline "br" @v0.2 :($irdistfrontmiddle < 0.2)
        mission.state = ms_followBlackLineRight;
        exitCondition = exit_irDistMiddelLess;
        dist = 0.2;
        targetVelo = 0.2;
        break;

      case 9: // followline "bl" @v0.2 :($irdistfrontmiddle < 0.5)
        mission.state = ms_followBlackLineLeft;
        exitCondition = exit_irDistMiddelLess;
        dist = 0.5;
        targetVelo = 0.2;
        break;

      case 10: // followline "bm" @v1.0 :($crossingblackline > 0)
      case 17:
      case 20:
      case 22:
      case 31:
      case 43:
      case 49:
        mission.state = ms_followBlackLineMiddle;
        exitCondition = exit_crossBlackLine;
        targetVelo = 0.4;
        break;

      case 23: // followline "bm" @v0.3 :($irdistleft < 0.8)
        mission.state = ms_followBlackLineMiddle;
        exitCondition = exit_irDistLeftLess;
        dist = 0.8;
        targetVelo = 0.4;
        break;

      case 46: // followline "wm" @v0.5 :($crossingblackline > 0)
        mission.state = ms_followWhiteLineMiddle;
        exitCondition = exit_crossBlackLine;
        targetVelo = 0.2;
        break;

      case 2:
        mission.state = ms_calcDistance;
        break;

      case 3: // turn -pi/2
      case 13:
        mission.state = ms_turn;
        angle = -M_PI - odo.theta;
        targetVelo = 0.2;
        break;

      case 33: // turn pi
      case 50:
        mission.state = ms_turn;
        angle = M_PI - odo.theta;
        targetVelo = 0.2;
        break;

      case 5: // turn -90 @v1.0
      case 8:
      case 27:
      case 48:
      case 53:
      case 55:
        mission.state = ms_turn;
        angle = calcAngle(-90);
        targetVelo = 0.3;
        break;

      case 58:  // turn -80 @v1.0
        mission.state = ms_turn;
        angle = calcAngle(-80);
        targetVelo = 0.3;
        break;

      case 45:
        mission.state = ms_turn;
        angle = calcAngle(45);
        targetVelo = 0.3;
        break;

      case 61:  // turn 80 @v1.0
        mission.state = ms_turn;
        angle = calcAngle(80);
        targetVelo = 0.3;
        break;

      case 16: // turn 90 @v1.0
      case 19:
      case 25:
      case 30:
      case 37:
      case 39:
      case 42:
      case 64:
        mission.state = ms_turn;
        angle = calcAngle(90);
        targetVelo = 0.3;
        break;

      case 6: // drive @v1.0 :($crossingblackline > 0)
      case 14:
      case 28:
      case 40:
      case 62:
        mission.state = ms_fwd;
        exitCondition = exit_crossBlackLine;
        dist = 1;
        targetVelo = 0.3;
        break;

      case 26: // drive @v1.0 :($irdistfrontmiddle > 0.2)
      case 56:
      case 66:
        mission.state = ms_fwd;
        exitCondition = exit_irDistMiddelLess;
        dist = 0.2;
        targetVelo = 0.3;
        break;

      case 35: // drive @v0.3 :($irdistleft > 0.8)
      case 51:
        mission.state = ms_fwd;
        exitCondition = exit_irDistRightLarger;
        dist = 0.8;
        targetVelo = 0.3;
        break;

      case 12: // fwd -1.3 @v1.0
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = -1.3;
        targetVelo = 0.3;
        break;

      case 60:  // fwd -0.1 @v1.0
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = -0.1;
        targetVelo = 0.3;
        break;

      case 57: // fwd 0.1 @v1.0
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 0.1;
        targetVelo = 0.3;
        break;

      case 7: // fwd 0.2 @v1.0
      case 15:
      case 18:
      case 21:
      case 29:
      case 32:
      case 41:
      case 47:
      case 63:
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 0.2;
        targetVelo = 0.3;
        break;

      case 11: // fwd 0.3 @v1.0
      case 65:
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 0.3;
        targetVelo = 0.3;
        break;

      case 4: // fwd 0.5 @v1.0
      case 36:
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 0.5;
        targetVelo = 0.3;
        break;

      case 44: // fwd 0.6 @v1.0
      case 52:
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 0.6;
        targetVelo = 0.2;
        break;

      case 24: // fwd 0.65 @v1.0
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 0.65;
        targetVelo = 0.3;
        break;

      case 34: // fwd 1 @v1.0
      case 38:
      case 59:
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 1;
        targetVelo = 0.3;
        break;

      case 54: // fwd 1.2 @v1.0
        mission.state = ms_fwd;
        exitCondition = exit_dist;
        dist = 1.2;
        targetVelo = 0.3;
        break;

      default:
        printf("Program end\n");
        mission.state = ms_end;
        break;
      }
      break;

    case ms_fwd:
      //printf("fwd\n");
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      if (fwd(dist, currVelo, mission.time))
        mission.state = ms_nextState;
      break;

    case ms_turn:
      //printf("turn\n");
      if (targetVelo - currVelo > acc * sampletime)
        currVelo += acc * sampletime;
      else
        currVelo = targetVelo;

      maxVelo = sqrt(2 * acc * (wheelDist - (mot.right_pos - mot.startpos)));
      if (maxVelo < currVelo)
        currVelo = maxVelo;

      if (turn(angle, currVelo, mission.time))
        mission.state = ms_nextState;

      break;

    case ms_followBlackLineLeft:
      //printf("followBlackLineLeft\n");
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      angleDeg = (double)getSensorAngleLeft(findSensorLeft(linesensor->data));

      odoRef = odo.theta - (angleDeg * M_PI / 180);
      deltaV = 0.6 * (odoRef - odo.theta);

      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_nextState;

      break;

    case ms_followBlackLineRight:
      //printf("followBlackLineRight\n");
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      angleDeg = (double)getSensorAngleRight(findSensorRight(linesensor->data));

      odoRef = odo.theta - (angleDeg * M_PI / 180);
      deltaV = 0.6 * (odoRef - odo.theta);
      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_nextState;

      break;

    case ms_followBlackLineMiddle:
      //printf("followBlackLineMiddle\n");
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      angleDeg = centerOfMass(5, linesensor->data, 1);

      odoRef = odo.theta - (angleDeg * M_PI / 180);
      deltaV = 0.6 * (odoRef - odo.theta);
      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_nextState;

      break;

    case ms_followWhiteLineMiddle:
      //printf("followWhiteLineMiddle\n");
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      angleDeg = centerOfMass(5, linesensor->data, 0);

      odoRef = odo.theta - (angleDeg * M_PI / 180);
      deltaV = 0.6 * (odoRef - odo.theta);
      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_nextState;

      break;

    case ms_direction:
      //printf("direction\n");
      dist = 0.4;
      currVelo = calcVelocity(targetVelo, currVelo, acc, dist);

      odoRef = angleDeg * M_PI / 180; // VARIABEL TIL VINKEL SOM SKAL DREJES MED

      if (odo.theta < 0 && odoRef > 0)
        deltaV = currVelo * (odoRef - odo.theta - (2 * M_PI));
      else if (odo.theta > 0 && odoRef < 0)
        deltaV = currVelo * (odoRef - odo.theta + (2 * M_PI));
      else
        deltaV = currVelo * (odoRef - odo.theta);

      if (direction(deltaV, currVelo, dist, mission.time))
        mission.state = ms_nextState;

      break;

    case ms_calcDistance:
      //printf("ms_calcDistance\n");
      //printf("irDist %f«n", measureIRDist(laserpar, irSensorMiddle));
      //printf("odoy %f\n", odo.y);
      printf("Distance from start to box: %f\n", measureIRDist(laserpar, irSensorMiddle) + (odo.y * -1) + irOffset);
      mission.state = ms_nextState;
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
    update_motcon(&mot, exitCondition, linesensor->data);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0)
      time++;
    /* stop if keyboard is activated*/

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

  p->x_old = 0.0;
  p->y_old = 0.0;
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

void update_motcon(motiontype *p, int exitC, int32_t *sensors)
{
  int temp = 0;

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
    switch (exitC)
    {
    case exit_dist:
      if ((p->dist > 0) && ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist))
        temp = 1;
      else if ((p->dist < 0) && ((((p->right_pos + p->left_pos) / 2 - p->startpos) * -1) > fabs(p->dist)))
        temp = 1;
      break;

    case exit_crossBlackLine:
      if (crossingBlackLine(linesensor->data))
        temp = 1;
      break;

    case exit_irDistLeftLess:
      if (laserTriggerLess(laserpar, p->dist, irSensorLeft))
        temp = 1;
      break;

    case exit_irDistMiddelLess:
      if (laserTriggerLess(laserpar, p->dist, irSensorMiddle))
        temp = 1;
      break;

    case exit_irDistRightLess:
      if (laserTriggerLess(laserpar, p->dist, irSensorRight))
        temp = 1;
      break;

    case exit_irDistLeftLarger:
      if (laserTriggerLarger(laserpar, p->dist, irSensorLeft))
        temp = 1;
      break;

    case exit_irDistMiddelLarger:
      if (laserTriggerLarger(laserpar, p->dist, irSensorMiddle))
        temp = 1;
      break;

    case exit_irDistRightLarger:
      if (laserTriggerLarger(laserpar, p->dist, irSensorRight))
        temp = 1;
      break;

    default:
      printf("Unknown Exit Condition\n");
      break;
    }

    if (temp)
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else if (p->dist > 0)
    {
      p->motorspeed_l = p->speedcmd;
      p->motorspeed_r = p->speedcmd;
    }
    else
    {
      p->motorspeed_l = -p->speedcmd;
      p->motorspeed_r = -p->speedcmd;
    }

    break;

  case mot_turn:
    if (p->angle > 0)
    {
      if (p->right_pos - p->startpos < (p->angle * p->w) / 2)
      {
        p->motorspeed_r = p->speedcmd / 2;
        p->motorspeed_l = -p->speedcmd / 2;
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    else
    {
      if (p->left_pos - p->startpos < fabs(p->angle) * p->w / 2)
      {
        p->motorspeed_r = -p->speedcmd / 2;
        p->motorspeed_l = p->speedcmd / 2;
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    break;

  case mot_direction:

    switch (exitC)
    {
    case exit_dist:
      if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist)
        temp = 1;
      break;

    case exit_crossBlackLine:
      if (crossingBlackLine(linesensor->data))
        temp = 1;
      break;

    case exit_irDistLeftLess:
      if (laserTriggerLess(laserpar, p->dist, irSensorLeft))
        temp = 1;
      break;

    case exit_irDistMiddelLess:
      if (laserTriggerLess(laserpar, p->dist, irSensorMiddle))
        temp = 1;
      break;

    case exit_irDistRightLess:
      if (laserTriggerLess(laserpar, p->dist, irSensorRight))
        temp = 1;
      break;

    case exit_irDistLeftLarger:
      if (laserTriggerLarger(laserpar, p->dist, irSensorLeft))
        temp = 1;
      break;

    case exit_irDistMiddelLarger:
      if (laserTriggerLarger(laserpar, p->dist, irSensorMiddle))
        temp = 1;
      break;

    case exit_irDistRightLarger:
      if (laserTriggerLarger(laserpar, p->dist, irSensorRight))
        temp = 1;
      break;

    default:
      printf("Unknown Exit Condition\n");
      break;
    }

    if (temp)
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

int lineSensorCaliBlack(int32_t sensor)
{
  if (sensor > 0)
    return 1;

  return 0;
}

int lineSensorCaliWhite(int32_t sensor)
{
  if (sensor < 255)
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
    if (lineSensorCaliBlack(sensors[i]) < value)
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
    if (lineSensorCaliBlack(sensors[i]) < value)
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
double centerOfMass(int distanceBetweenSensors, int *sensors, int blackLine)
{
  int numberOfSensors = 8;
  double sensorSum = 0;
  double sensorSumTimesXi = 0;

  double center = ((numberOfSensors)) / 2;
  if (blackLine)
  {
    for (int i = 0; i < numberOfSensors; i++)
    {
      sensorSum += 1 - lineSensorCaliBlack(sensors[i]);
    }

    for (int i = 0; i < numberOfSensors; i++)
    {
      sensorSumTimesXi += i * (1 - lineSensorCaliBlack(sensors[i]));
    }
  }
  else
  {
    for (int i = 0; i < numberOfSensors; i++)
    {
      sensorSum += 1 - lineSensorCaliWhite(sensors[i]);
    }

    for (int i = 0; i < numberOfSensors; i++)
    {
      sensorSumTimesXi += i * (1 - lineSensorCaliWhite(sensors[i]));
    }
  }

  if (!sensorSum)
    return 0;

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

double calcAngle(double angleDeg)
{
  return angleDeg / 180 * M_PI;
}