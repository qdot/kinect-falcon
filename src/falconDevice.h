#pragma once

#include "ofMain.h"
#include "ofThread.h"
#include "falcon/core/FalconDevice.h"
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <deque>
// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class falconDevice : public ofThread {
public:
	falconDevice();
	void threadedFunction();
	void updateMesh();
	ofVec3f getPosition() { return falconPos; }
	void die() { d = true; }
	void setMesh(ofMesh m);
private:
	libnifalcon::FalconDevice falcon;
	ofVec3f normalizeFalconCoordinates(boost::array<double, 3> coords);
	boost::array<double, 3> calculateForceVector();
	void findClosestPoint();
	ofVec3f falconPos;
	ofVec3f closest;
	ofMesh mesh;
	bool d;
	int last_index;
	std::deque<ofVec3f> avg;
	struct timeval m_tstart, m_tend;

	void tinit()
	{
	}

	void tstart()
	{
		gettimeofday(&m_tstart, NULL);
	}

	void tend()
	{
		gettimeofday(&m_tend, NULL);
	}

	double tval()
	{
		double t1, t2;
		t1 =  (double)m_tstart.tv_sec + (double)m_tstart.tv_usec/(1000*1000);
		t2 =  (double)m_tend.tv_sec + (double)m_tend.tv_usec/(1000*1000);
		return t2-t1;
	}
};
