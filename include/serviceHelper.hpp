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

#ifndef SERVICEHELPER_HPP_
#define SERVICEHELPER_HPP_

#include <string>
#include "objectFeatures.hpp"
#include <fstream>
#include <iostream>
//#define USE_INDEX_ARRAY

enum SpeechCommandType {
	NONE = 0,
	RECORD_KEYFRAME = 1,
	NEW_DEMO = 2,
	END_DEMO = 3,
	OPEN_HAND = 4,
	CLOSE_HAND = 5,
	START_TRAJECTORY =6,
	END_TRAJECTORY =7,
	NEW_SKILL = 8
};

//precision grasp?
#define NUM_SPEECH_COMMANDS 8

struct serviceRequest;
void resetServiceRequest(serviceRequest &request);

struct serviceRequest {
	bool recordFrame;
	bool newDemo;
	bool endDemo;
	bool bgTrain;
	bool startLoop;
	bool stopLoop;
	bool quit;
	bool isViz;
	bool newSkill;
	bool isObservationOnly;
	bool justTra;
	bool justKF;
	bool noHand;
	SpeechCommandType speechCommand;
	serviceRequest (){resetServiceRequest(*this);} //: recordFrame(false), newDemo(false), bgTrain(false), startLoop(false), stopLoop(false), quit(false), isViz(true), speechCommand(NONE)  {}
};

class skillInfo {
private:
	char tmp[100];

public:
	int kfNum;
	int demoNum;
	int obsDemoNum;
	int subjectNum;
	int hueValue;
	std::string skillName;
	std::string dataLocation;
	std::string rogueSkillName;
	char rogueSkillCounter;
	bool newInfo;
	bool isObservationOnly;
	bool firstInfo;

	skillInfo () : kfNum(0), demoNum(0), obsDemoNum(0), subjectNum(0), skillName("skill"), newInfo(false),
			dataLocation("/home/baris/data/agLearning/sim"), isObservationOnly(false), firstInfo(true),
			rogueSkillName("skill"), rogueSkillCounter('0'), hueValue(-1) {}

	void getDataFolder(char *folderName) {
		sprintf(folderName,"%s", dataLocation.c_str());
	}
	void getSubjectFolder(char *folderName) {
		getDataFolder(tmp);
		sprintf(folderName,"%s/subject%d", tmp, subjectNum);
	}
	void getSkillFolder(char *folderName) {
		getSubjectFolder(tmp);
		sprintf(folderName,"%s/%s", tmp, skillName.c_str());
	}

	void getFileName(char *imageName) {
		getSkillFolder(tmp);
		if(isObservationOnly)
			sprintf(imageName,"%s/observation_demo%d_kf%d", tmp, obsDemoNum, kfNum);
		else
			sprintf(imageName,"%s/demo%d_kf%d", tmp, demoNum, kfNum);
	}

	void getFileNameWithExt(char *imageName, const char *ext) {
		getFileName(tmp);
		sprintf(imageName,"%s.%s", tmp, ext);
	}
};

void resetServiceRequest(serviceRequest &request) {
	request.recordFrame = false;
	request.newDemo = false;
	request.endDemo = false;
	request.bgTrain = false;
	request.startLoop = false;
	request.stopLoop = false;
	request.quit = false;
	request.isViz = true;
	request.newSkill = false;
	request.speechCommand = NONE;
	request.isObservationOnly = false;
	request.justTra = false;
	request.justKF  = !request.justTra && true;
	request.noHand  = true;
	return;
}

//bunlar gotunde patlar dikkat et
serviceRequest globalRequests;
skillInfo globalSkillInfo;

struct commands4Loop {
        bool loop_break;
        bool loop_continue;
        bool save_raw;
        bool save_features;
        bool send_featurs;
        bool send_misc_info;
        bool make_skill_folder;
        commands4Loop() : loop_break(false), loop_continue(false), save_raw(false),
                                  save_features(false), send_featurs(false), send_misc_info(false),
                                  make_skill_folder(false){}
};

//convenience function
void mapSpeechToBools();


//}
void fillObjectInfo(std::vector<pc_cluster_features> &objFeatures);
void fillObjectInfo(pc_cluster_features &objFeature);

void genericRequestCallback(unsigned char src,  const int val);
float objectInfoBuffer[5000];

class comHandler {

private:


public:


};

#endif /* SERVICEHELPER_HPP_ */

