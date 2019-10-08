#pragma once

#include "ofMain.h"
#include "ofxAruco.h"
#include "ofxPS3EyeGrabber.h"

class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
        bool movePoint(vector<ofVec2f>& points, ofVec2f point);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
        ofImage warpedColor;
        vector<ofVec2f> destPoints, srcPoints;
        bool movingPoint;
        ofVec2f* curPoint;
        bool saveMatrix;
        bool homographyReady;
        bool settingNewPoints;
        int currentPoint;
        
        cv::Mat homography;
    
		ofVideoGrabber grabber;
		ofVideoPlayer player;

		ofBaseVideoDraws * video;

		ofxAruco aruco;
		bool useVideo;
		bool showMarkers;
		bool showBoard;
		bool showBoardImage;
		ofImage board;
		ofImage marker;
    
    ofImage letter1;
};
