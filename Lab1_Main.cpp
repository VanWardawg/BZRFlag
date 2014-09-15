#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include "470bot.h"
using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;

int main(int argc, char *argv[]) {
	const char *pcHost;
	int nPort;

	if(argc < 2) {
		pcHost = kDefaultServerName;
	}
	else {
		pcHost = argv[1];
	}
    if(argc < 3) {
		nPort = kDefaultServerPort;
	}
	else {
        nPort = atoi(argv[2]);
    }

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	// Calling agent code
	world_init(&MyTeam);
	for(int i=1; i>0; i++) {
		robot_pre_update();
		robot_update();
		robot_post_update();
		//Sleep(50);
	}

	MyTeam.Close();
	//free(&MyTeam);
	return 0;
}

// Fill in your code here
double normalize_angle(double angle){};
void world_init(BZRC *my_team){};
void robot_pre_update(){};
void robot_update(){};
void robot_post_update(){};