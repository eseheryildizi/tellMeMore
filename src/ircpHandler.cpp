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
#include "ircpHandler.hpp"

//using namespace IRCP_HANDLER;

//namespace IRCP_HANDLER {

//global variables ftw!!!!
std::ofstream logFile;
serviceRequest ircpHandler::globalRequests;
skillInfo      ircpHandler::globalSkillInfo;
const char *speechCommands[] = {"SAVE_KEYFRAME","START_DEMO", "END_DEMO", "OPEN_HAND", "CLOSE_HAND", "START_TRAJECTORY", "END_TRAJECTORY", "NEW_SKILL"};

//decode incoming speech here, start very very basic (just keyframes and new demo)
void speechCommandCallback(unsigned char src,  SpeechCommandData::iterator begin,  SpeechCommandData::iterator end) {
  /*for(SpeechCommandData::iterator i=begin; i!=end; i++)
    std::cout << std::string(i->second) << " ";
  std::cout << std::endl;*/

  SpeechCommandData::iterator i = end; i--;

  std::string command = std::string(i->second);
  ircpHandler::globalRequests.speechCommand = NONE;
  for(int i = 1; i <= NUM_SPEECH_COMMANDS; i++)
  {
          if(!command.compare(speechCommands[i-1]))
          {
            ircpHandler::globalRequests.speechCommand = SpeechCommandType(i);
            std::cout << "Received Command: " << command  << std::endl;
            break;
          }
  }
  mapSpeechToBools();
}

void mapSpeechToBools() { //differentiate between close open trajectory etc.?
  if(ircpHandler::globalRequests.speechCommand == RECORD_KEYFRAME &&
     !ircpHandler::globalRequests.justTra)
  {
    ircpHandler::globalRequests.recordFrame = true;
  }
  else if((ircpHandler::globalRequests.speechCommand == START_TRAJECTORY  ||
       ircpHandler::globalRequests.speechCommand == END_TRAJECTORY  ) &&
      !ircpHandler::globalRequests.justKF)
  {
    ircpHandler::globalRequests.recordFrame = true;
  }
  else if((ircpHandler::globalRequests.speechCommand == CLOSE_HAND  ||
           ircpHandler::globalRequests.speechCommand == OPEN_HAND ) &&
      !ircpHandler::globalRequests.justTra &&
      !ircpHandler::globalRequests.noHand)
  {
    ircpHandler::globalRequests.recordFrame = true;
  }
  else if(ircpHandler::globalRequests.speechCommand == NEW_DEMO)
  {
    ircpHandler::globalRequests.newDemo = true;
  }
  else if(ircpHandler::globalRequests.speechCommand == END_DEMO) {
    ircpHandler::globalRequests.endDemo = true;
  }
  else if(ircpHandler::globalRequests.speechCommand == NEW_SKILL) {
    ircpHandler::globalRequests.newSkill = true;
  }
}

//what if num objects drop, do we still try to send everything? should we try to resize the object info?
//solution: when you send instead of globalObjectInfo.end, send whatever is necessary
void fillObjectInfo(std::vector<pc_cluster_features> &objFeatures) {
  int numObjects = objFeatures.size();
  objectInfoBuffer[0] = (float)numObjects;
  //I will make this more versatile somehow
  for (int i = 0; i < numObjects;i++)
  {
    //for(int j = 0; j < objFeatures[i].numFeatures; j++)

    //in case I want to have different number of features per object...
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 1] = objFeatures[i].numFeatures;

    /*objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 2] = (float)objFeatures[i].centroid.x;
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 3] = (float)objFeatures[i].centroid.y;
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 4] = objFeatures[i].ellipseOrientation;
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 5] = (float)objFeatures[i].area;
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 6] = (float)objFeatures[i].minorAxisLength;
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 7] = (float)objFeatures[i].majorAxisLength;
    objectInfoBuffer[(objFeatures[i].numFeatures+1)*i + 8] = (float)objFeatures[i].dominantColor;*/
  }
  /*globalObjectInfo[0] = (float)numObjects;
  //I will make this more versatile somehow
  for (int i = 0; i < numObjects;i++)
  {
    //for(int j = 0; j < objFeatures[i].numFeatures; j++)

    //in case I want to have different number of features per object...
    globalObjectInfo[(objFeatures[i].numFeatures+1)*i + 1] = objFeatures[i].numFeatures;

    globalObjectInfo[(objFeatures[i].numFeatures+1)*i + 2] = (float)objFeatures[i].centroid.x;
    globalObjectInfo[(objFeatures[i].numFeatures+1)*i + 3] = (float)objFeatures[i].centroid.y;
    globalObjectInfo[(objFeatures[i].numFeatures+1)*i + 4] = objFeatures[i].ellipseOrientation;
    globalObjectInfo[(objFeatures[i].numFeatures+1)*i + 5] = (float)objFeatures[i].area;
  }*/
}

void fillObjectInfo(pc_cluster_features &obj_features) {
  objectInfoBuffer[0] = 1.0; //number of objects
  //I will make this more versatile somehow
  obj_features.fillFeatureContainer(objectInfoBuffer,1); //1 is the starting index of the fill
}

//decide on a command set, preferably in a portable way
void genericRequestCallback(unsigned char src,  const IRCP::Integer& val){
  std::cout << "Got command: " << (int) val << std::endl;
  logFile << "Got command: " << (int) val << std::endl;
  std::cout.flush();

  switch (val) {
    case 0:
      break;
    case 1:
      ircpHandler::globalRequests.bgTrain = true;
      break;
    case 2:
      ircpHandler::globalRequests.startLoop = true;
      break;
    case 3:
      ircpHandler::globalRequests.stopLoop = true;
      break;
    case 4:
      ircpHandler::globalRequests.recordFrame = true;
      break;
    case 5:
      ircpHandler::globalRequests.newDemo = true;
      break;
    case 6:
      ircpHandler::globalRequests.endDemo = true;
      break;
    case 7:
      ircpHandler::globalRequests.newSkill = true;
      break;
    case 8:
      ircpHandler::globalRequests.isViz = true;
      break;
    case 9:
      ircpHandler::globalRequests.quit = true;
      break;
    case 10:
      ircpHandler::globalRequests.isObservationOnly  = true;
      ircpHandler::globalSkillInfo.isObservationOnly = true;
      break;
    case 11:
      ircpHandler::globalRequests.isObservationOnly  = false;
      ircpHandler::globalSkillInfo.isObservationOnly = false;
      break;
    default:
      std::cout << "Unknown request id: " << val << std::endl;
      break;
  }
}

//info sent from c6m. what should be the format? Decided on indexed array of string
//this can probably wait till later
void informationCallback(unsigned char src, InfoData::iterator begin, InfoData::iterator end) {
  std::cout << "Received skill Info:" << std::endl;
  for(InfoData::iterator i=begin; i!=end; i++)
  {
    if(!std::string(*i).compare("Poke"))
    {
      continue;
    }
    else if(!std::string(*i).compare("skill"))
    {
      i++;
      ircpHandler::globalSkillInfo.skillName = std::string(*i);
      std::cout << " skill: " << ircpHandler::globalSkillInfo.skillName;
      logFile << " skill: " << ircpHandler::globalSkillInfo.skillName;
      ircpHandler::globalSkillInfo.firstInfo = false;
    }
    else if(!std::string(*i).compare("demoNum"))
    {
      i++;
      ircpHandler::globalSkillInfo.demoNum = atoi(std::string(*i).c_str());
      std::cout << " demoNum: " << ircpHandler::globalSkillInfo.demoNum;
      logFile << " demoNum: " << ircpHandler::globalSkillInfo.demoNum;
    }
    else if(!std::string(*i).compare("obsDemoNum"))
    {
      i++;
      ircpHandler::globalSkillInfo.obsDemoNum = atoi(std::string(*i).c_str());
      std::cout << " obsDemoNum: " << ircpHandler::globalSkillInfo.obsDemoNum;
      logFile << " obsDemoNum: " << ircpHandler::globalSkillInfo.obsDemoNum;
    }
    else if(!std::string(*i).compare("kfNum"))
    {
      i++;
      ircpHandler::globalSkillInfo.kfNum = atoi(std::string(*i).c_str());
      std::cout << " kfNum: " << ircpHandler::globalSkillInfo.kfNum;
      logFile  << " kfNum: " << ircpHandler::globalSkillInfo.kfNum;
    }
    else if(!std::string(*i).compare("subject"))
    {
      i++;
      ircpHandler::globalSkillInfo.subjectNum = atoi(std::string(*i).c_str());
      std::cout << " subject: " << ircpHandler::globalSkillInfo.subjectNum;
      logFile << " subject: " << ircpHandler::globalSkillInfo.subjectNum;
    }
    else if(!std::string(*i).compare("observation"))
    {
      i++;
      ircpHandler::globalSkillInfo.isObservationOnly = atoi(std::string(*i).c_str());
      std::cout << " Observation Only Mode: " << ircpHandler::globalSkillInfo.isObservationOnly;
      logFile << " Observation Only Mode: " << ircpHandler::globalSkillInfo.isObservationOnly;
    }
    else if(!std::string(*i).compare("hueValue"))
    {
      i++;
      ircpHandler::globalSkillInfo.hueValue = atoi(std::string(*i).c_str());
      std::cout << " hueValue: " << ircpHandler::globalSkillInfo.hueValue;
      logFile << " hueValue: " << ircpHandler::globalSkillInfo.hueValue;
    }
    else {
      std::cout << " Unknown info type: " << std::string(*i);
      logFile << " Unknown info type: " << std::string(*i);
      continue;
    }
    ircpHandler::globalSkillInfo.newInfo = true;
  }
   std::cout << std::endl;
   logFile << std::endl;
}


//}

ircpHandler::ircpHandler(unsigned char targetModule, unsigned char module, unsigned char robot, bool registerCallbacks) {
  // TODO Auto-generated constructor stub
  _robot = robot;
  _module = module;
  _targetModule = targetModule;
  //_speechCommandCallback = speechCommandCallback;
  _registerCallbacks = registerCallbacks;
  ircp = NULL;
  idleMode = true;
  
  logFile.open("ircp.log");
}

void ircpHandler::sendObjectInformation() {

  //int numObjects = globalObjectInfo[0]; //numobjects

  int numObjects = objectInfoBuffer[0];

  if(!numObjects)
    return;

  int tmp=1;

  for(int i = 0; i < numObjects; i++)
    tmp += objectInfoBuffer[tmp] + 1;

  ircp->sendto(_targetModule, ObjectData(objectInfoBuffer, objectInfoBuffer+tmp));

  //std::cout << tmp << std::endl;

  /*ObjectData::iterator begin = globalObjectInfo.begin();
  ObjectData::iterator end;

  //calculating the end, assuming different objects can have different number of features
  int tmp=1;
  for(int i = 0; i < numObjects; i++)
    tmp += globalObjectInfo[tmp] + 1;

  //inefficiency at its best
  //end = begin + tmp;
  end = begin;
  for(int i = 0; i < tmp; i++)
    end++;

  ircp->sendto(_targetModule, ObjectData(begin, end));*/

  ircp->flush_ircp();
  //std::cout << "sent " << numObjects << " blobs to " << (int)_targetModule << std::endl;
}

ircpHandler::~ircpHandler() {
  // TODO Auto-generated destructor stub
  if(ircp != NULL)
	ircp->quit();
	
  logFile.close();
  //delete ircp;
}


void ircpHandler::initialize() {
  ircp = new Module(_robot, _module, _bcast);

  if(_registerCallbacks)
  {
    ircp->register_callback(IRCP::SpeechCommandData(), speechCommandCallback);
    ircp->register_callback(GenericInfoRequest(), genericRequestCallback);
    ircp->register_callback(InfoData(), informationCallback);
  }

  ircp->sendto(_module, IRCP::EmptySubpacket<0xFE,0>());
  ircp->flush_ircp();
  sleep(1);

}

commands4Loop ircpHandler::handleCommands(char *base_name)
{
  commands4Loop c4l;

  if(this->globalRequests.quit) {
    std::cout << "Quiting." << std::endl;
    this->globalRequests.recordFrame = false;
    this->globalRequests.newDemo = false;
    c4l.loop_break = true;
    logFile << "Quit" << std::endl;
    //loopFlag = false; //redundant
    //break;
  }
  
  //logFile << "Received commands: " << std::endl;

  if((this->globalRequests.newSkill || ircpHandler::globalSkillInfo.newInfo) && idleMode) {
    std::cout << "Next skill" << std::endl;
    sleep(1);
#ifdef WITH_C6M
    while(!ircpHandler::globalSkillInfo.newInfo) {usleep(10000);} //for next skill wait info from c6?
#endif
    if(ircpHandler::globalSkillInfo.newInfo)
    {
      ircpHandler::globalSkillInfo.newInfo = false;
      ircpHandler::globalSkillInfo.kfNum = 0;
    }
    else
    {
      ircpHandler::globalSkillInfo.rogueSkillCounter++; // be careful, if this is 9 (57?), we are doomed because this is a char!
      ircpHandler::globalSkillInfo.skillName = ircpHandler::globalSkillInfo.rogueSkillName + ircpHandler::globalSkillInfo.rogueSkillCounter;
      ircpHandler::globalSkillInfo.demoNum = 0;
      ircpHandler::globalSkillInfo.obsDemoNum = 0;
      ircpHandler::globalSkillInfo.kfNum = 0;
      logFile << "Resetting demoNum, obsDemoNum, kfNum " << std::endl;
    }

    c4l.make_skill_folder = true;
    this->globalRequests.newSkill = false;
    //demoCount = 0;
    //kfCount = 0;
  }

  if(this->globalRequests.newDemo)
  {
    idleMode = false;
  }

  if(idleMode) {
    if(this->globalRequests.endDemo)
      std::cout << "wtf" << std::endl;
    this->globalRequests.endDemo = false;
    c4l.loop_continue = true;
    return c4l;
  }

  if(this->globalRequests.newDemo)
  {
    //Deal with new demo
    //Save demo information. This might be redundant with c6
    if(!(ircpHandler::globalSkillInfo.demoNum && ircpHandler::globalSkillInfo.obsDemoNum) && !ircpHandler::globalSkillInfo.kfNum)
    {
      std::cout << "First demo!" << std::endl;
    } else{
      std::cout << "Getting a new demonstration. Current demo:  " << ircpHandler::globalSkillInfo.demoNum << " obs: "<< ircpHandler::globalSkillInfo.obsDemoNum << std::endl;
    }
    ircpHandler::globalSkillInfo.kfNum = 0;
    this->globalRequests.newDemo = false;
    this->globalRequests.recordFrame = true;
    
    logFile << "New demo " << std::endl;
  }

  if(this->globalRequests.endDemo)
  {
    if(ircpHandler::globalSkillInfo.isObservationOnly)
      std::cout << "Ending obs demo:  " << ircpHandler::globalSkillInfo.obsDemoNum << std::endl;
    else
      std::cout << "Ending demo:  " << ircpHandler::globalSkillInfo.demoNum << std::endl;
    if (!ircpHandler::globalSkillInfo.kfNum) {
      std::cout << "No keyframes prior to this recorded!";
    }

    //Deal with keyframe
    //Save the image
    std::cout << "Last keyframe: " << ircpHandler::globalSkillInfo.kfNum << std::endl;//kfCount << std::endl;
    c4l.save_raw = true;
    ircpHandler::globalSkillInfo.getFileName(base_name);

    this->globalRequests.recordFrame = false;
    this->globalRequests.endDemo = false;

    if(ircpHandler::globalSkillInfo.isObservationOnly)
      ircpHandler::globalSkillInfo.obsDemoNum++;
    else
      ircpHandler::globalSkillInfo.demoNum++;

    ircpHandler::globalSkillInfo.kfNum = 0;

    idleMode = true;

	logFile << "End demo " << std::endl;
  }

  if(this->globalRequests.recordFrame) {
    //Deal with keyframe
    //Save the image
    std::cout << "New keyrame: " << ircpHandler::globalSkillInfo.kfNum << std::endl;
    c4l.save_raw = true;
    ircpHandler::globalSkillInfo.getFileName(base_name);

    this->globalRequests.recordFrame = false;
    ircpHandler::globalSkillInfo.kfNum++;
    
    logFile << "Record Frame " << std::endl;
  }
  
  if(c4l.save_raw)
  {
	logFile << "Saving raw base file name: " << base_name << std::endl;
  }

  return c4l;
}


