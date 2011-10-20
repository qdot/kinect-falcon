#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falconDevice.h"

using namespace libnifalcon;
using namespace std;

falconDevice::falconDevice() :
	d(false),
	last_index(-1)
{
}

void falconDevice::threadedFunction()
{
	unsigned int num_falcons = 0;

	falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
	falcon.setFalconKinematic<FalconKinematicStamper>();

	if(!falcon.getDeviceCount(num_falcons))
	{
		std::cout << "Cannot get device count" << std::endl;
		return;
	}

	std::cout << "Falcons found: " << (int)num_falcons << std::endl;

	if(num_falcons == 0)
	{
		std::cout << "No falcons found, exiting..." << std::endl;
		return;
	}

	std::cout << "Opening falcon " << std::endl;

	if(!falcon.open(0))
	{
		std::cout << "Cannot open falcon - Error: " << std::endl; // << falcon.getErrorCode() << std::endl;
		return;
	}
	std::cout << "Opened falcon" << std::endl;
	
	if(!falcon.isFirmwareLoaded())
	{
		std::cout << "Loading firmware" << std::endl;
		for(int i = 0; i < 10; ++i)
		{
			if(!falcon.getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
			{
				std::cout << "Could not load firmware" << std::endl;
				return;
			}
			else
			{
				std::cout <<"Firmware loaded" << std::endl;
				break;
			}
		}
		if(!falcon.isFirmwareLoaded())
		{
			std::cout << "Firmware didn't load correctly. Try running findfalcons again" << std::endl;
			return;
		}
	}
	double i = 0;
	int count = 0;
	while(!d)
	{
		tstart();		
		falcon.runIOLoop();
		findClosestPoint();
		boost::array<double, 3> f = calculateForceVector();
		falcon.setForce(f);
		boost::array<double, 3> p;
		p = falcon.getPosition();
		ofVec3f pos = normalizeFalconCoordinates(p);
		ofVec3f screen_pos;
		falconPos = pos * ofVec3f(800, 600, 1000);
		tend();
		i += tval();
		++count;
		if(i > 1)
		{
			std::cout << count << std::endl;
			i = 0;
			count = 0;

		}
	}
}

ofVec3f falconDevice::normalizeFalconCoordinates(boost::array<double, 3> pos)
{
	ofVec3f c;
	c[0] = pos[0] * (1/.06);
	c[1] = pos[1] * (1/.06);
	c[2] = (pos[2] - .125) * (1/.06);
	return c;
}

void falconDevice::setMesh(ofMesh m)
{
	//lock();
	mesh = m;
	//unlock();
}

void falconDevice::findClosestPoint()
{
	closest = ofVec3f(10000, 10000, 10000);
	int change = 5000;
	if(last_index >= 0)
	{
		for(int i = (last_index-change < 0 ? 0 : last_index-change); i < mesh.getNumVertices() && i < last_index+change; ++i)
		{
			if(falconPos.distance(mesh.getVertex(i)) < falconPos.distance(closest))
			{
				closest = mesh.getVertex(i);
				last_index = i;
			}
		}
		if(closest.distance(ofVec3f(10000,10000,10000)) < 100)
		{
			last_index = -1;
		}
	}
	else
	{
		for(int i = 0; i < mesh.getNumVertices(); ++i)
		{
			if(falconPos.distance(mesh.getVertex(i)) < falconPos.distance(closest))
			{
				closest = mesh.getVertex(i);
				last_index = i;
			}
		}
	}
}

boost::array<double, 3> falconDevice::calculateForceVector()
{
	static const int avg_size = 50;
	boost::array<double, 3> f;
	float d = falconPos.distance(closest);
	//printf("%f\n", d);
	int dist = 40;
	ofVec3f nf = (falconPos - closest).normalize();
	ofVec3f l;
	if(nf[2] < 0 && fabs(nf[2]) > fabs(nf[1]) && fabs(nf[2]) > fabs(nf[0]))
	{
		nf[2] = -nf[2];
		l = (nf * log((dist+d)+1)) * 0.75;
	}
	else if(d > dist)
	{
		f[0] = 0;
		f[1] = 0;
		f[2] = 0;
		return f;
	}
	else
	{
		l = (nf * log((dist - d)+1)) * 0.75;
	}
	if(avg.size() > avg_size)
	{
		avg.pop_front();
	}
	avg.push_back(l);
	ofVec3f a(0,0,0);
	for(int i = 0; i < avg.size(); ++i)
	{
		a += avg[i];
	}
	a /= avg_size;
	f[0] = a[0];
	f[1] = a[1];
	f[2] = a[2];		
	return f;
}

