#include "testApp.h"

using namespace libnifalcon;

// ofVec3f getNormal(const ofVec3f& v1, const ofVec3f& v2, const ofVec3f& v3) {
//        ofVec3f a = v1 - v2;
//        ofVec3f b = v3 - v2;
//        ofVec3f normal = b.cross(a);
//        normal.normalize();
//        return normal;
// }

// void buildNormals(ofMesh& mesh) {
//        vector<ofVec3f>& vertices = mesh.getVertices();
//        for(int i = 0; i < mesh.getNumVertices(); i += 3) {
//                ofVec3f normal = getNormal(
//                                                                                                                         mesh.getVertices()[i+0],
//                                                                                                                         mesh.getVertices()[i+1],
//                                                                                                                         mesh.getVertices()[i+2]);
//                for(int j = 0; j < 3; j++) {
//                        mesh.addNormal(normal);
//                }
//        }
// }

//--------------------------------------------------------------
void testApp::setup() {
	
	ofSetLogLevel(OF_LOG_VERBOSE);
	falcon.startThread();
	kinect.setUseRegistration(true);
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.setVerbose(true);
	kinect.open();
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	// start with the live kinect source
	kinectSource = &kinect;
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = true;

}

//--------------------------------------------------------------
void testApp::update() {
	ofBackground(100, 100, 100);
	
	kinectSource->update();
	
	// there is a new frame and we are connected
	if(kinectSource->isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinectSource->getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		ofTranslate(0,0,500);
		drawPointCloud();
		drawFalconPoint();
		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
		
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	/*
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
	*/
}

void testApp::drawPointCloud() {
	static const int avg_size = 3;
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);	
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y)/avg_size);
				ofVec3f s(1, -1, -1);
				ofVec3f t(0, 0, -1000);
				ofVec3f f = (kinect.getWorldCoordinateAt(x, y) + t) * s;
				mesh.addVertex(f/avg_size/4);
			}
			else
			{
				mesh.addColor(ofFloatColor(0,0,0,0));
				mesh.addVertex(ofVec3f(10000,10000,10000));
			}
		}
	}
	if(avg_size > 1)
	{
		mesh_deque.push_front(mesh);
	}
	if(mesh_deque.size() > avg_size)
	{
		ofMesh m = mesh_deque.back();
		mesh_deque.pop_back();
		for(int i = 0; i < mesh.getNumVertices(); ++i) {
			mesh_avg.setColor(i, mesh_avg.getColor(i) - m.getColor(i));
			mesh_avg.setVertex(i, mesh_avg.getVertex(i) - m.getVertex(i));
		}
	}
	
	if(avg_size == 1 || mesh_deque.size() == 1)
	{
		printf("SETTING\n");
		mesh_avg = mesh;
	}
	else if(avg_size > 1)
	{
		for(int i = 0; i < mesh.getNumVertices(); ++i) {
			mesh_avg.setColor(i, mesh_avg.getColor(i) + mesh.getColor(i));
			mesh_avg.setVertex(i, mesh_avg.getVertex(i) + mesh.getVertex(i));
		}
	}
	falcon.setMesh(mesh_avg);
	glPointSize(3);
	ofPushMatrix();
	glEnable(GL_DEPTH_TEST);
	mesh_avg.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}


void testApp::drawFalconPoint()
{
	ofVec3f falconPos = falcon.getPosition();
	//normalize falcon endpoint
	ofSetColor(255, 0, 0);	
	ofPushMatrix();
	glEnable(GL_DEPTH_TEST);
	ofTranslate(falconPos[0], falconPos[1], falconPos[2]);
	// before setup
    GLUquadricObj *quadratic;      
    // in draw  
    quadratic = gluNewQuadric();
    gluSphere(quadratic, 10.0, 20, 20);
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
	easyCam.end();
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "pos is: " << falconPos[0] << " " << falconPos[1] << " " << falconPos[2] << endl <<
		// "closest is: " << closest << endl <<
		// "distance: " << falconPos.distance(closest) << endl<<
		"fps: " << ofGetFrameRate() << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
	easyCam.begin();
}

//--------------------------------------------------------------
void testApp::exit() {
	falcon.die();
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
