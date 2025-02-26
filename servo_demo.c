#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <pigpio.h>

/*
# servo_demo.c
# 2016-10-08
# Public Domain

gcc -Wall -pthread -o servo_demo servo_demo.c -lpigpio

sudo ./servo_demo          # Send servo pulses to GPIO 4.
sudo ./servo_demo 23 24 25 # Send servo pulses to GPIO 23, 24, 25.
*/

#define NUM_GPIO 32

#define MIN_WIDTH 1000
#define MAX_WIDTH 2000

int run=1;

int step[NUM_GPIO];
int width[NUM_GPIO];
int used[NUM_GPIO];

int randint(int from, int to)
{
   return (random() % (to - from + 1)) + from;
}

void stop(int signum)
{
   run = 0;
}

int main(int argc, char *argv[])
{
   int i, g;

   if (gpioInitialise() < 0) return -1;

   gpioSetSignalFunc(SIGINT, stop);//SIGINT IS interupt from keyboard

   if (argc == 1) used[4] = 1;
   else
   {
      for (i=1; i<argc; i++)
      {
         g = atoi(argv[i]);
         if ((g>=0) && (g<NUM_GPIO)) used[g] = 1;
      }
   }

   printf("Sending servos pulses to GPIO");

   for (g=0; g<NUM_GPIO; g++)
   {
      if (used[g])
      {
         printf(" %d", g);
         step[g] = randint(5, 25);
         if ((step[g] % 2) == 0) step[g] = -step[g];
         width[g] = randint(MIN_WIDTH, MAX_WIDTH);
      }
   }

   printf(", control C to stop.\n");

   while(run)
   {
      for (g=0; g<NUM_GPIO; g++)
      {
         if (used[g])
         {
            gpioServo(g, width[g]);

            // printf("%d %d\n", g, width[g]);

            width[g] += step[g];

            if ((width[g]<MIN_WIDTH) || (width[g]>MAX_WIDTH))
            {
               step[g] = -step[g];
               width[g] += step[g];
            }
         }
      }

      time_sleep(0.1);
   }

   printf("\ntidying up\n");

   for (g=0; g<NUM_GPIO; g++)
   {
      if (used[g]) gpioServo(g, 0);
   }

   gpioTerminate();

   return 0;
}

