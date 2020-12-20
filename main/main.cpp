//Revised by YanggeLiu 2020-6-27
#include "mutex"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include "opencv2/imgproc.hpp"
#include "thread"
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
//#include <ctime.h>
#include <errno.h>
#include <unistd.h>
//#include <ioctl.h>
#include <fcntl.h>
#include <termios.h>

#define NUM_ROWS 16
#define NUM_COLS 16

#define BLUE   0x4      // bit that will be set if grid location is "blue" track
#define ORANGE 0x8      // bit that will be set if grid location is orange. (cone)
#define NORMAL 0
#define FORWARD 0
#define REVERSE 1
#define NUM_REVERSE_FRAMES 50
#define CHANGE 2
#define NO_PATH_FRAMES 3
#define MAX_TURN 700
#define CENTER (NUM_COLS+1)/2
#define SCALE 1
#define TURN_SCALE 10

//using namespace FlyCapture2;

float finalHoleSize = 0;
float finalx = 0.0;
int FrameWidth = 640;       //Value will be updated with incoming video feed
int FrameHeight = 480;      //Value will be updated with incoming video feed
int driveState = 0;
int frameCount = 0;
int turnValueTemp;
int turnValue = 0;
int lastTurnValueSent = 0;
double lastSpeedSent = 0.0;
int    lastDirectionSent = -1;

cv::Mat hsv;
cv::Mat cones;
cv::Mat track;
cv::Mat work;
cv::Mat image;
cv::Mat tempImage1;
cv::Mat tempImage2;
cv::Mat* pointer = &tempImage1;
std::mutex myMutex1;
std::mutex myMutex2;
int mutexToUnlock = 0;
cv::Rect rect = cv::Rect(0,0,0,0);
cv::VideoCapture capture;

int countingFramerate = 0;
int Framerate = 0;

int countingFramerate2 = 0;
int Framerate2 = 0;

clock_t begin = clock();
clock_t end   = clock();

clock_t begin2 = clock();
clock_t end2   = clock();

cv::Scalar orange(0,51,255);        // In BGR
cv::Scalar blue(255,51,0);          // In BGR
cv::Scalar white(255,255,255);
std::vector<float> holeXLocations(NUM_ROWS, 0.0);

int fd, n, i;
char buf[128] = "temp text";

std::vector <std::vector<int> > trackGrid((NUM_COLS+2), std::vector<int>(NUM_ROWS, 0));


int setupSerial(){

    struct termios toptions;

    /* open serial port */
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    printf("fd opened as %i\n", fd);

    /* wait for the Arduino to reboot */
    usleep(1500000);


    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 115200 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);
    printf("Attempting to communicate with arduino... \n");

    return 0;
}

void sendCommand(const char* command){
    printf("Sending Comand: %s",command);
    write(fd,command,strlen(command));
    n = read(fd,buf,64);
    buf[n] = 0;
    printf("Comand Returned: %s\n",buf);
}

void calculateFramerate2(){
    // Calculate the framerate
    end2 = clock();
    double elapsed_secs = double(end2-begin2) / CLOCKS_PER_SEC;
    countingFramerate2++;
    if(elapsed_secs > 1.0){
        begin2 = end2;
        Framerate2 = countingFramerate2;
        countingFramerate2 = 0;
        char frameString[20];
        sprintf(frameString, "%d", Framerate2);
        printf("Framerate: %s \n", frameString);

    }
}

void getFrameFromCamera(){
    // Get the image



    while(1){

        calculateFramerate2();
        if(pointer == &tempImage1){
            myMutex2.lock();
            capture >> tempImage2;
            pointer = &tempImage2;
            myMutex2.unlock();
        }else{
            myMutex1.lock();
            capture >> tempImage1;
            pointer = &tempImage1;
            myMutex1.unlock();
        }

    }
}

void resetMatrix(){
    //function sets all the values of the matrix to 0, except for left-most and right-most columns
    // which are off the screen, to be the "left track" and "right track"
    int i,j = 0;
    for(i=0; i<(NUM_COLS+2);i++){
        for(j=0; j<NUM_ROWS;j++){
            if(i==0 && j>(NUM_ROWS/2)) {
                trackGrid[i][j] = BLUE;
            }else if(i==(NUM_COLS+1) && j>(NUM_ROWS/2)){
                trackGrid[i][j] = BLUE;
            }else{
                trackGrid[i][j] = 0;
            }
        }
    }
}

void formMatrix(){

    rect.width = cones.cols/NUM_COLS;
    rect.height = cones.rows/NUM_ROWS;
    int total_pixels = rect.width * rect.height;
    int threshold = total_pixels/8;
    int i = 0;
    int j = 0;
    for(i=0;i<NUM_COLS;i++){
        for(j=0;j<NUM_ROWS;j++){
            rect.x = i*rect.width;
            rect.y = j*rect.height;
            work = cones(rect);
            int count = cv::countNonZero(work);
            if(count > threshold){
                cv::rectangle(image,rect, orange,2);
                trackGrid[i][j] = ORANGE | trackGrid[i][j];
            }

            work = track(rect);
            count = cv::countNonZero(work);
            if(count > threshold){
                cv::rectangle(image,rect, blue,2);
                trackGrid[i][j] = BLUE | trackGrid[i][j];
            }
        }
    }


}

int checkIfLogicalSequence(int nextx, int y){
    // Checks to see if the possible path has cones/track in the way.
    int oldx = (int) holeXLocations[y+1];
    if(oldx == nextx) return true;

    if((oldx-nextx) > 0 ){
        for(int i=oldx; i >= nextx; i--){
            if((trackGrid[i][y+1] != 0)) return false;
        }

    }else{
        for(int i=oldx; i <= nextx; i++){
            if((trackGrid[i][y+1] != 0)) return false;
        }

    }
    return true;
}

void analyzeMatrix(){
    int i,j = 0;
    float tempx = 0;
    float tempHoleSize = 0;
    finalHoleSize = 0;

    cv::Point base = cv::Point((FrameWidth)/ 2,FrameHeight);
    cv::Point dir = cv::Point((finalx + 0.5) * rect.width,0);
    for(i=0;i<NUM_ROWS;i++){
        holeXLocations[i] = 0.0;
    }

    holeXLocations[NUM_ROWS] = NUM_COLS /2;
    bool found_next = false;
    bool frame_broken = false;

    for(j=NUM_ROWS-1;j>=0;j--){
        found_next = false;
        for(i = 0; i < (NUM_COLS + 1); i++){
            if(((trackGrid[i][j] & BLUE) == BLUE) || ((trackGrid[i][j] & ORANGE) == ORANGE)){
                tempx = tempx / tempHoleSize;
                if((tempHoleSize > finalHoleSize) && checkIfLogicalSequence((int)tempx,j)){
                    finalx = tempx - (float)0.5;
                    finalHoleSize = tempHoleSize;
                    found_next = true;
                }
                tempx = 0;
                tempHoleSize = 0;
            }
            else{
                tempHoleSize++;
                tempx = tempx + i;
            }
        }
        tempx = tempx / tempHoleSize;
        if((tempHoleSize > finalHoleSize) && checkIfLogicalSequence((int)tempx,j)){
            finalx = tempx - (float)0.5;
            finalHoleSize = tempHoleSize;
            found_next = true;
        }
        tempx = 0;
        tempHoleSize = 0;
        finalHoleSize = 0;

        if((!found_next) && (driveState == NORMAL) && (j > NUM_ROWS/3)){
            frame_broken = true;
        }

        holeXLocations[j] = finalx;

        dir = cv::Point((finalx) * rect.width,(j)* rect.height);
        cv::arrowedLine(image,base,dir,cv::Scalar(255,0,0,255));

        base = dir;
    }
    if(frame_broken){
        frameCount++;
        if(frameCount==NO_PATH_FRAMES && driveState==NORMAL){
            driveState = REVERSE;
            frameCount = 0;
        }
    }
    else{
        if(driveState == NORMAL){
            frameCount = 0;
        }
    }
}

int calculateTurnValue(){
        // This function calculates the turn value for the car. It will determines how far from
        // the center it is, and decreases the weight as it gets farther away.
        int j = NUM_ROWS-1;
        turnValueTemp = 0;
        for(j=NUM_ROWS-1; j>=(NUM_ROWS/2-4);j--) {

            turnValueTemp += (int) (holeXLocations[j]-CENTER) * (j-4) * (j-4) * TURN_SCALE;
        }

        if(turnValueTemp > MAX_TURN) turnValueTemp = MAX_TURN;
        if(turnValueTemp <(MAX_TURN*-1)) turnValueTemp = MAX_TURN*-1;
        return turnValueTemp;
}

void printMatrixInfo(){

    int i = 0;
    int j = 0;
    for(i=0;i<NUM_COLS;i++){
        for(j=0;j<NUM_ROWS;j++){
            cv::Point location(rect.width*i, rect.height*(j+1));

            cv::putText(image,std::to_string(trackGrid[i][j]),location, cv::FONT_HERSHEY_COMPLEX, 0.5, white);
        }
    }
}


int setTurnValue(int value){
    if(value==lastTurnValueSent){
        return 0;
    }
    lastTurnValueSent = value;
    std::string stringValue = std::to_string(1500-value);
    std::string command = "!s" + stringValue + "\n";
    const char * c = command.c_str();
    sendCommand(c);

}

int setSpeedValue(double value, int direction){
    if((abs(value-lastSpeedSent) < 0.1) && (lastDirectionSent == direction)){
        return 0;
    }
    lastSpeedSent = value;
    lastDirectionSent = direction;
    char speedString[20];
    if(direction == REVERSE){
        sprintf(speedString, "!p1400\n");

    }else{
        sprintf(speedString, "!p1600\n");

    }
    sendCommand(speedString);

}

void handleDriveState(){

    if(driveState == NORMAL){
        setSpeedValue(0.35, FORWARD);
        cv::putText(image, "F", cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX, 2.0/SCALE, cv::Scalar(0, 255, 0, 255), 2);
        setTurnValue(turnValue);

    }else if(driveState == REVERSE){
        setSpeedValue(0.25, REVERSE);
        cv::putText(image, "R", cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX, 2.0/SCALE, cv::Scalar(0, 255, 0, 255), 2);
        setTurnValue(600);
        frameCount++;
        if(frameCount>NUM_REVERSE_FRAMES){
            frameCount = 0;
            driveState = NORMAL;
        }
    }
}



void processFrames(){
    char key = 0;
    while(key != 'q')
    {

        if(pointer==&tempImage1){
            myMutex1.lock();
            image = *pointer;
            mutexToUnlock  = 1;
        }else if(pointer == &tempImage2){
            myMutex2.lock();
            image = *pointer;
            mutexToUnlock  = 2;

        }
        if(!image.empty()){
            cv::cvtColor(image, hsv, CV_BGR2HSV);

            cv::Scalar cone_low(0, 150, 50);
            cv::Scalar cone_high(15, 255, 255);
            cv::Scalar track_low(90,85,50);
            cv::Scalar track_high(110,255,255);
            cv::inRange(hsv, cone_low, cone_high, cones);
            cv::inRange(hsv, track_low, track_high, track);
            resetMatrix();
            formMatrix();
            analyzeMatrix();
            printMatrixInfo();
            turnValue = calculateTurnValue();
            cv::putText(image, std::to_string(turnValue), cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 2.0/SCALE, cv::Scalar(0, 255, 0, 255), 2);
            handleDriveState();

            //calculateFramerate();
            cv::imshow("1",cones);
            cv::imshow("2",track);
            cv::imshow("Video Capture", image);
            key = cv::waitKey(1);
            if(key == 'q'){
                sendCommand("!s1500\n");
                sendCommand("!s1500\n");
                sendCommand("!p1500\n");
                sendCommand("!p1500\n");
                close(fd);
                std::cout << "done!" << std::endl;
            }
        }
        if(mutexToUnlock == 1){
            myMutex1.unlock();
        }else{
            myMutex2.unlock();
        }

    }

}

int main(){

    setupSerial();
    sendCommand("!p1500\n");
    sendCommand("!p1500\n");
    sendCommand("!p1500\n");
    capture.open(0);
    capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,360);


    std::thread frameGrabber(getFrameFromCamera);
    std::thread mvThings(processFrames);

    mvThings.join();


    return 0;
}
