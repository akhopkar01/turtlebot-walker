#include "Walker.h"
#include <iostream>

int main() { 
//   if (argc > 1)
  turtlebot::Walker turtleSim(0.1, 0.3);
  turtleSim.pubROSNode();
  return 0;
}