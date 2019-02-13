#include "MyRobot.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

bool running = true;
MyRobot* myRobot;

void shutdown()
{
    if(myRobot != nullptr)
    {
        myRobot->haltMessagePublished = false;
        while(true)
        {
            if (myRobot->halt())
            {
                delete myRobot;
                return;
            }
            else
            {
                ros::spinOnce();
                sleep(1);
            }
        }
    }
}

void sigHandler(int s)
{
  running = false;
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "robofinal", ros::init_options::NoSigintHandler);

  struct sigaction sigIntHandler{};
  sigIntHandler.sa_handler = sigHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, nullptr);

  myRobot = new MyRobot(argc, argv);

  ros::Rate rosrate(15);

  while(ros::ok() && running)
  {
      if(myRobot != nullptr)
      {
          running = myRobot->update();
      }
      ros::spinOnce();
      rosrate.sleep();
  }

  if(!ros::ok())
  {
      printf("ROS was not ok...\n");
  }

  printf("Shutting down...\n");
  shutdown();
  ros::shutdown();
  return 0;
}
