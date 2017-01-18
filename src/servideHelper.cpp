/*
Copyright (c) 2016, Baris Akgun
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Koc University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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


