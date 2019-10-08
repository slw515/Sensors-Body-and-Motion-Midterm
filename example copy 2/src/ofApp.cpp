#include "ofApp.h"
#include "ofxCv.h"
#include "ofBitmapFont.h"

using namespace ofxCv;
using namespace cv;

//void drawMarker(float size, const ofColor & color){
void drawMarker(float size, ofImage alphabet){
	ofDrawAxis(size / 2);
	ofPushMatrix();
		// move up from the center by size*.5
		// to draw a box centered at that point
    ofTranslate(0,size*0.5,0);
    ofFill();
//    ofSetColor(color, 50);
    alphabet.draw(0, 0);
//		ofDrawBox(size);
    ofDrawEllipse(0, 0, 0.1, 0.1);
//		ofNoFill();
////		ofSetColor(color);
//		ofDrawBox(size);
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::setup(){
    letter1.load("letter1.png");
	ofSetWindowTitle("ofxAruco - example");
	ofSetVerticalSync(true);
	useVideo = false;
	string boardName = "boardConfiguration.yml";
    grabber.setGrabber(std::make_shared<ofxPS3EyeGrabber>());
    grabber.setup(640, 480);
    
	//aruco.setThreaded(false);
	showMarkers = true;
	showBoard = true;
	showBoardImage = false;

	ofEnableAlphaBlending();
    warpedColor.allocate(640, 480, OF_IMAGE_COLOR);
    
    movingPoint = false;
    saveMatrix = false;
    homographyReady = false;
    settingNewPoints = false;
    
    destPoints.push_back(ofVec2f(0,0));
    destPoints.push_back(ofVec2f(warpedColor.getWidth(),0));
    destPoints.push_back(ofVec2f(warpedColor.getWidth(),warpedColor.getHeight()));
    destPoints.push_back(ofVec2f(0,warpedColor.getHeight()));
    
    srcPoints.push_back(ofVec2f(0+warpedColor.getWidth()+100,30));
    srcPoints.push_back(ofVec2f(warpedColor.getWidth()+warpedColor.getWidth()-100,30));
    srcPoints.push_back(ofVec2f(warpedColor.getWidth()+warpedColor.getWidth()-100,warpedColor.getHeight()-30));
    srcPoints.push_back(ofVec2f(0+warpedColor.getWidth()+100,warpedColor.getHeight()-30));
    
    // load the previous homography if it's available
    ofFile previous("homography.yml");
    if(previous.exists()) {
        FileStorage fs(ofToDataPath("homography.yml"), FileStorage::READ);
        fs["homography"] >> homography;
        homographyReady = true;
        cout<<warpedColor.getWidth()<<endl;
    }
    ofSetVerticalSync(true);
    
    aruco.setup("intrinsics.int", warpedColor.getWidth(), warpedColor.getHeight(), boardName);
    aruco.getBoardImage(board.getPixels());
    board.update();

}

//--------------------------------------------------------------
void ofApp::update(){
    grabber.update();
    if(grabber.isFrameNew()){
        if(settingNewPoints) {
            vector<Point2f> sPoints, dPoints;
            for(int i = 0; i < destPoints.size(); i++) {
                sPoints.push_back(Point2f(srcPoints[i].x - warpedColor.getWidth(), srcPoints[i].y));
                dPoints.push_back(Point2f(destPoints[i].x, destPoints[i].y));
            }
                // generate a homography from the two sets of points
                homography = findHomography(Mat(sPoints), Mat(dPoints));
                homographyReady = true;
                
                if(saveMatrix) {
                    FileStorage fs(ofToDataPath("homography.yml"), FileStorage::WRITE);
                    fs << "homography" << homography;
                    saveMatrix = false;
                    settingNewPoints = false;
                }
            }
            
            if(homographyReady) {
                // this is how you warp one ofImage into another ofImage given the homography matrix
                // CV INTER NN is 113 fps, CV_INTER_LINEAR is 93 fps
                warpPerspective(grabber, warpedColor, homography, CV_INTER_LINEAR);
                warpedColor.update();
            }
        aruco.detectBoards(warpedColor.getPixels());
    }
}

void drawPoints(vector<ofVec2f>& points) {
    ofNoFill();
    for(int i = 0; i < points.size(); i++) {
        ofDrawCircle(points[i], 10);
        ofDrawCircle(points[i], 1);
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255);
    ofFile previous("homography.yml");
    if (previous.exists() == false) {
        grabber.draw(640, 0);
    }
    if(homographyReady) {
        warpedColor.draw(0, 0);
    }
    if (settingNewPoints){
        ofPushStyle();
        ofSetColor(ofColor::blue);
        drawPoints(srcPoints);
        ofSetColor(128);
        ofSetLineWidth(2);
        for(int i = 1; i < srcPoints.size(); i++) {
            ofDrawLine(srcPoints[i-1], srcPoints[i]);
        }
        ofDrawLine(srcPoints[srcPoints.size()-1], srcPoints[0]);
        ofPopStyle();
    }
    
    ofSetColor(255);
    string message = ofToString((int) ofGetFrameRate());
    message += "\nPress 'n' to set points\nPress 's' to save";
    ofDrawBitmapString(message, 10, 20);

	if (showMarkers) {
		for (int i = 0; i<aruco.getNumMarkers(); i++) {
			aruco.begin(i);
			drawMarker(0.12, letter1);
			aruco.end();
		}
	}


	if (showBoard && aruco.getBoardProbability()>0.03) {
		for (int i = 0; i<aruco.getNumBoards(); i++) {
			aruco.beginBoard(i);
            drawMarker(.5, letter1);
			aruco.end();
		}
	}

    cout<<ofGetMouseX()<<endl;
	ofSetColor(255);
	if (showBoardImage) {
		board.draw(ofGetWidth() - 320, 0, 320, 320 * float(board.getHeight()) / float(board.getWidth()));
	}
	ofDrawBitmapString("markers detected: " + ofToString(aruco.getNumMarkers()), 20, 20);
	ofDrawBitmapString("fps " + ofToString(ofGetFrameRate()), 20, 40);
	ofDrawBitmapString("m toggles markers", 20, 60);
	ofDrawBitmapString("b toggles board", 20, 80);
	ofDrawBitmapString("i toggles board image", 20, 100);
	ofDrawBitmapString("s saves board image", 20, 120);
	ofDrawBitmapString("0-9 saves marker image", 20, 140);
    
    letter1.draw(20, 20, 200, 200);
}


bool ofApp::movePoint(vector<ofVec2f>& points, ofVec2f point) {
    for(int i = 0; i < points.size(); i++) {
        if(points[i].distance(point) < 20) {
            movingPoint = true;
            curPoint = &points[i];
            return true;
        }
    }
    return false;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'm') showMarkers = !showMarkers;
	if (key == 'b') showBoard = !showBoard;
	if (key == 'i') showBoardImage = !showBoardImage;
	if (key == 'l') board.save("boardimage.png");
	if (key >= '0' && key <= '9') {
		// there's 1024 different markers
		int markerID = key - '0';
		aruco.getMarkerImage(markerID, 240, marker);
		marker.save("marker" + ofToString(markerID) + ".png");
	}
    
    if(key == 's') {
        saveMatrix = true;
    }
    if(key == 'n') {
        settingNewPoints = true;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if(movingPoint) {
        curPoint->set(x, y);
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    ofVec2f cur(x, y);
    ofVec2f rightOffset(warpedColor.getWidth(), 0);
    movePoint(srcPoints, cur);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    movingPoint = false;
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
