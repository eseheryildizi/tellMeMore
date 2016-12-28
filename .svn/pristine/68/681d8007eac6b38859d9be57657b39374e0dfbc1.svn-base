/*
 * serviceHelper.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: baris
 */

#include "serviceHelper.hpp"

//using namespace IRCP_HANDLER;

//namespace IRCP_HANDLER {

//global variables ftw!!!!
std::ofstream logFile;
const char *speechCommands[] = {"SAVE_KEYFRAME","START_DEMO", "END_DEMO", "OPEN_HAND", "CLOSE_HAND", "START_TRAJECTORY", "END_TRAJECTORY", "NEW_SKILL"};

void mapSpeechToBools() { //differentiate between close open trajectory etc.?
  if(globalRequests.speechCommand == RECORD_KEYFRAME &&
     !globalRequests.justTra)
  {
    globalRequests.recordFrame = true;
  }
  else if((globalRequests.speechCommand == START_TRAJECTORY  ||
       globalRequests.speechCommand == END_TRAJECTORY  ) &&
      !globalRequests.justKF)
  {
    globalRequests.recordFrame = true;
  }
  else if((globalRequests.speechCommand == CLOSE_HAND  ||
           globalRequests.speechCommand == OPEN_HAND ) &&
      !globalRequests.justTra &&
      !globalRequests.noHand)
  {
    globalRequests.recordFrame = true;
  }
  else if(globalRequests.speechCommand == NEW_DEMO)
  {
    globalRequests.newDemo = true;
  }
  else if(globalRequests.speechCommand == END_DEMO) {
    globalRequests.endDemo = true;
  }
  else if(globalRequests.speechCommand == NEW_SKILL) {
    globalRequests.newSkill = true;
  }
}

void fillObjectInfo(pc_cluster_features &obj_features) {
  objectInfoBuffer[0] = 1.0; //number of objects
  //I will make this more versatile somehow
  obj_features.fillFeatureContainer(objectInfoBuffer,1); //1 is the starting index of the fill
}

//decide on a command set, preferably in a portable way
void genericRequestCallback(unsigned char src,  const int val) {
  std::cout << "Got command: " << (int) val << std::endl;
  logFile << "Got command: " << (int) val << std::endl;
  std::cout.flush();

  switch (val) {
    case 0:
      break;
    case 1:
      globalRequests.bgTrain = true;
      break;
    case 2:
      globalRequests.startLoop = true;
      break;
    case 3:
      globalRequests.stopLoop = true;
      break;
    case 4:
      globalRequests.recordFrame = true;
      break;
    case 5:
      globalRequests.newDemo = true;
      break;
    case 6:
      globalRequests.endDemo = true;
      break;
    case 7:
      globalRequests.newSkill = true;
      break;
    case 8:
      globalRequests.isViz = true;
      break;
    case 9:
      globalRequests.quit = true;
      break;
    case 10:
      globalRequests.isObservationOnly  = true;
      globalSkillInfo.isObservationOnly = true;
      break;
    case 11:
      globalRequests.isObservationOnly  = false;
      globalSkillInfo.isObservationOnly = false;
      break;
    default:
      std::cout << "Unknown request id: " << val << std::endl;
      break;
  }
}


