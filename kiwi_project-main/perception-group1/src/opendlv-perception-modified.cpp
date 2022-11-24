/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "math.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <deque>
#include <unordered_map>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <fstream>
#include <sstream> 

// TODO: add documentation
std::vector<int> parseHsvFlags(std::string strData) {
    std::string s;
    std::stringstream streamData(strData);
    std::vector<int> parsedVector;

    while (std::getline(streamData, s, ',')) {
        parsedVector.push_back(std::stoi(s));
    }

    return parsedVector;
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ||
         (0 == commandlineArguments.count("bluelow")) ||
         (0 == commandlineArguments.count("bluehi")) ||
         (0 == commandlineArguments.count("yellowlow")) ||
         (0 == commandlineArguments.count("yellowhi")) ||
         (0 == commandlineArguments.count("steeringfactor")) ||
         (0 == commandlineArguments.count("maxsteering")) ||
         (0 == commandlineArguments.count("pedalpos")) ||
         (0 == commandlineArguments.count("pedalposslow")) ||
         (0 == commandlineArguments.count("crop")) ||
         (0 == commandlineArguments.count("bumper")) ||
         (0 == commandlineArguments.count("movingavg")) ||
         (0 == commandlineArguments.count("historybuffersize")) ||
         (0 == commandlineArguments.count("yellowturnthres")) ||
         (0 == commandlineArguments.count("blueturnthres")) ||
         (0 == commandlineArguments.count("maxhaltingdelay"))
         ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
            std::mutex steeringMutex;
            std::mutex pedalMutex;
            std::mutex distancesMutex;
            std::mutex centerMutex;

            float front{0};
            float rear{0};
            float left{0};
            float right{0};

            double centerX{0};
            double centerY{0};
            float widthX{0};
            float widthY{0};

            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
                auto senderStamp = env.senderStamp();
                // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
                opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

                // Store distance readings.
                std::lock_guard<std::mutex> lck(distancesMutex);
                switch (senderStamp) {
                    case 0: front = dr.distance(); break;
                    case 2: rear = dr.distance(); break;
                    case 1: left = dr.distance(); break;
                    case 3: right = dr.distance(); break;
                }
            };

            auto onGeolocation = [&centerMutex, &centerX, &centerY, &widthX, &widthY](cluon::data::Envelope &&env){
                 //uint32_t const senderStamp = envelope.senderStamp();
                 opendlv::logic::sensation::Geolocation gl = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(env));
                 // Store distance readings.
                 std::lock_guard<std::mutex> lck(centerMutex);
                 centerX = gl.latitude();
                 centerY = gl.longitude();
                 widthX = gl.altitude();
                 widthY = gl.heading();
            };
        
            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);
            od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(), onGeolocation);
            // od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
            // od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(), onPedalPositionRequest);

            // Parse flag values into usable data structures
            std::vector<int> bluelowValues {parseHsvFlags(commandlineArguments["bluelow"])};
            std::vector<int> bluehiValues {parseHsvFlags(commandlineArguments["bluehi"])};
            std::vector<int> yellowlowValues {parseHsvFlags(commandlineArguments["yellowlow"])};
            std::vector<int> yellowhiValues {parseHsvFlags(commandlineArguments["yellowhi"])};

            double steeringFactor {std::stod(commandlineArguments["steeringfactor"])};
            double maxSteering {std::stod(commandlineArguments["maxsteering"])};
            float pedalPos {std::stof(commandlineArguments["pedalpos"])};
            float pedalPosSlow {std::stof(commandlineArguments["pedalposslow"])};

            int cropHeight {std::stoi(commandlineArguments["crop"])};
            int bumperHeight {std::stoi(commandlineArguments["bumper"])};

            double movingAvgFactor {std::stod(commandlineArguments["movingavg"])};

            int turningBufferSize {std::stoi(commandlineArguments["historybuffersize"])};
            int yellowturnthres {std::stoi(commandlineArguments["yellowturnthres"])};
            int blueturnthres {std::stoi(commandlineArguments["blueturnthres"])};
            int maxHaltingDelay {std::stoi(commandlineArguments["maxhaltingdelay"])};
            
            
            std::deque<int> yellowHistoryQueue;
            std::deque<int> blueHistoryQueue;
            int haltingDelay {0};

            int imageHeight {720 - cropHeight};
            int bumperDisplacement {imageHeight - bumperHeight};
            double groundSteeringPrevious {0.0};

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy image into cvMat structure.
                    // Be aware of that any code between lock/unlock is blocking
                    // the camera to provide the next frame. Thus, any
                    // computationally heavy algorithms should be placed outside
                    // lock/unlock
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // CROPPING IMAGE BEFORE PROCESSING (VERSION 0.0.8)

                // Creating a matrix for the image in HSV space
                cv::Mat imgCropped = img(cv::Range(cropHeight,720), cv::Range(0,1280));


                // TRANSFORMING FROM BGR TO HSV SPACE (VERSION 0.0.2)

                // Creating a matrix for the image in HSV space
                cv::Mat hsv;
                
                // Transforming the original "img" to the "hsv" space 
                cv::cvtColor(imgCropped, hsv, cv::COLOR_BGR2HSV);


                // BLUE COLOUR FILTERING IN HSV SPACE (VERSION 0.0.2)

                // Setting the lower and upper limits
                cv::Scalar hsvBlueLow(bluelowValues.at(0), bluelowValues.at(1), bluelowValues.at(2)); // Note : H [0 ,180] , S [0 ,255] , V [0 , 255]
                cv::Scalar hsvBlueHi(bluehiValues.at(0), bluehiValues.at(1), bluehiValues.at(2));

                // YELLOW COLOUR FILTERING IN HSV SPACE (VERSION 0.0.5)

                // Setting the lower and upper limits
                cv::Scalar hsvYellowLow(yellowlowValues.at(0), yellowlowValues.at(1), yellowlowValues.at(2)); // Note : H [0 ,180] , S [0 ,255] , V [0 , 255]
                cv::Scalar hsvYellowHi(yellowhiValues.at(0), yellowhiValues.at(1), yellowhiValues.at(2));

                // // RED COLOUR FILTERING IN HSV SPACE (VERSION 0.0.6)

                // // Setting the lower and upper limits for both ranges of red 
                // // In HSV space, the red color wraps around 180. So you need the H values to be both in [0,10] and [170, 180].

                // Creating a pair of matrices for the image used to detect the red cones in both intervals
                cv::Mat mask1, mask2;

                // Setting the lower and upper limits
                cv::Scalar hsvMask1RedLow(0, 115, 115);
                cv::Scalar hsvMask1RedHi(10, 255, 255);
                cv::Scalar hsvMask2RedLow(170, 90, 90);
                cv::Scalar hsvMask2RedHi(180, 255, 255);

                // INTRODUCING DETECTION BY SECTION (VERSION 0.1.0)

                // Creating a matrix for the image used to detect the blue cones in the right
                cv::Mat hsvRight = hsv(cv::Range(0,imageHeight), cv::Range(1030,1280));
                
                // Creating a matrix for the image used to detect the yellow cones in the left
                cv::Mat hsvLeft = hsv(cv::Range(0,imageHeight), cv::Range(0,250));

                // Creating a matrix for the image used to detect the both cones in the middle
                cv::Mat hsvMiddle = hsv(cv::Range(0,bumperDisplacement), cv::Range(250,1030));

                // Creating a black matrix for the bumper section
                cv::Mat blackMiddle = hsv(cv::Range(bumperDisplacement,imageHeight), cv::Range(250,1030));

                // Creating a black matrix for the left side of the frame section
                cv::Mat blackLeft = hsv(cv::Range(0,imageHeight), cv::Range(0,250));

                // Creating a black matrix for the right side of the frame section
                cv::Mat blackRight = hsv(cv::Range(0,imageHeight), cv::Range(1030,1280));


                // Creating a matrix for the image used to detect the blue cones in the right
                cv::Mat blueConesRight;
                
                // Creating a matrix for the image used to detect the yellow cones in the left
                cv::Mat yellowConesLeft;

                // Creating a matrix for the image used to detect the both cones in the middle
                cv::Mat yellowConesMiddle;
                cv::Mat blueConesMiddle;

                // Storing the pixels which are within the defined range for black matrices
                cv::inRange(blackMiddle, hsvBlueLow, hsvBlueLow, blackMiddle);
                cv::inRange(blackLeft, hsvBlueLow, hsvBlueLow, blackLeft);
                cv::inRange(blackRight, hsvBlueLow, hsvBlueLow, blackRight);

                // Storing the pixels which are within the defined range for Right side -blue
                cv::inRange(hsvRight, hsvBlueLow, hsvBlueHi, blueConesRight);

                // Storing the pixels which are within the defined range for Left side - yellow
                cv::inRange(hsvLeft, hsvYellowLow, hsvYellowHi, yellowConesLeft);

                // Storing the pixels which are within the defined range for Middle zone both colors
                cv::inRange(hsvMiddle, hsvBlueLow, hsvBlueHi, blueConesMiddle);
                cv::inRange(hsvMiddle, hsvYellowLow, hsvYellowHi, yellowConesMiddle);

                // NON-FUNCTIONAL (RED)

                // Storing the pixels which are within the defined range for each mask
                cv::inRange(hsvMiddle, hsvMask1RedLow, hsvMask1RedHi, mask1);
                cv::inRange(hsvMiddle, hsvMask2RedLow, hsvMask2RedHi, mask2);

                // Creating a matrix for the image used to detect the red cones
                cv::Mat redConesMiddle = mask1 | mask2;

                // Storing the pixels which are within the defined range for each mask
                cv::inRange(hsvRight, hsvMask1RedLow, hsvMask1RedHi, mask1);
                cv::inRange(hsvRight, hsvMask2RedLow, hsvMask2RedHi, mask2);

                // Creating a matrix for the image used to detect the red cones
                cv::Mat redConesRight = mask1 | mask2;

                // Storing the pixels which are within the defined range for each mask
                cv::inRange(hsvLeft, hsvMask1RedLow, hsvMask1RedHi, mask1);
                cv::inRange(hsvLeft, hsvMask2RedLow, hsvMask2RedHi, mask2);

                // Creating a matrix for the image used to detect the red cones
                cv::Mat redConesLeft = mask1 | mask2;

                // Setting two point objects to draw the center points
                cv::Point pt1(-2,-2);
                cv::Point pt2(2,2);

                ///////////////////////////////////////

                // 1. Creating matrices for the image used to detect the blue cones in the middle right
                cv::Mat blueConesVector;
                cv::Mat blueConesBlackLeftMiddle;
                cv::Mat blueMiddle;
            
                // 1.1. Getting the middle frame using vertical concatenation
                cv::vconcat(blueConesMiddle, blackMiddle, blueMiddle);

                // 1.2. Getting the whole matrix using horizontal concatenation in two steps
                cv::hconcat(blackLeft, blueMiddle, blueConesBlackLeftMiddle);
                cv::hconcat(blueConesBlackLeftMiddle, blueConesRight, blueConesVector);

                // 1.3. Performing dilate and erode on blueConesVector matrix
                uint32_t iterations { 5 };
                uint32_t iterations2 { 3 };
                
                cv::Mat dilateBlue;
                cv::dilate(blueConesVector, dilateBlue, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);

                cv::Mat erodeBlue;
                cv::erode(dilateBlue, erodeBlue, cv::Mat(), cv::Point(-1, -1), iterations2, 1, 1);

                // 1.4. Creating vector of vectors of points to generate contours for blue images
                std::vector < std::vector < cv::Point >> blueContours;
                cv::findContours(erodeBlue,blueContours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE); // Find the countours of the cone

                // 1.5. Creating a vector to store the center points for blue cones
                std::vector < cv::Point > centerPointsBlue;

                // 1.6. For loop used to draw all the relevant data in the image
                for (size_t i =0; i< blueContours.size();++i) {
                    cv::Rect boundRect = cv::boundingRect(blueContours[i]); // Creating rectangles based in the contours vector
                    if (boundRect.area() < 8000 && boundRect.area() > 500 && (boundRect.height / boundRect.width < 2.2 && boundRect.height / boundRect.width > 0.8 )) {

                        if (VERBOSE) {
                            cv::rectangle(imgCropped,boundRect.tl(),boundRect.br(),cv::Scalar(0,230,64),3);
                            cv::rectangle(imgCropped,((boundRect.tl()+boundRect.br())/2 + pt2),((boundRect.tl()+boundRect.br())/2 + pt1),cv::Scalar(230,30,0),2);
                        }
                        centerPointsBlue.push_back((boundRect.tl()+boundRect.br())/2);
                    }
                }
                
                // 2. Creating matrices for the image used to detect the yellow cones in the left middle
                cv::Mat yellowConesVector;
                cv::Mat yellowConesLeftMiddle;
                cv::Mat yellowMiddle;

                // 2.1. Getting the middle frame using vertical concatenation
                cv::vconcat(yellowConesMiddle, blackMiddle, yellowMiddle);

                // 2.2. Getting the whole matrix using horizontal concatenation in two steps
                cv::hconcat(yellowConesLeft, yellowMiddle, yellowConesLeftMiddle);
                cv::hconcat(yellowConesLeftMiddle, blackRight, yellowConesVector);

                // 2.3. Performing dilate and erode on blueConesVector matrix
                cv::Mat dilateYellow;
                cv::dilate(yellowConesVector, dilateYellow, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);

                cv::Mat erodeYellow;
                cv::erode(dilateYellow, erodeYellow, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);

                // 2.4. Creating vector of vectors of points to generate contours for yellow cones
                std::vector < std::vector < cv::Point >> yellowContours;
                cv::findContours(erodeYellow,yellowContours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE); // Find the countours of the cone

                // 2.5. Creating a vector to store the center points for yellow cones
                std::vector < cv::Point > centerPointsYellow;

                // 2.6. For loop used to draw all the relevant data in the image
                for (size_t i =0; i< yellowContours.size();++i) {
                    cv::Rect boundRect = cv::boundingRect(yellowContours[i]); // Creating rectangles based in the contours vector
                    if (boundRect.area() < 8000 && boundRect.area() > 500 && (boundRect.height / boundRect.width < 2.2 && boundRect.height / boundRect.width > 0.8 )) {

                        if (VERBOSE) {
                            cv::rectangle(imgCropped,boundRect.tl(),boundRect.br(),cv::Scalar(0,230,64),3); // Drawing rectangles based in the contours vector
                            cv::rectangle(imgCropped,((boundRect.tl()+boundRect.br())/2 + pt2),((boundRect.tl()+boundRect.br())/2 + pt1),cv::Scalar(230,30,0),2); // Creating mini rectangles to evaluate the middle points of the cone
                        }
                        centerPointsYellow.push_back((boundRect.tl()+boundRect.br())/2);
                    }
                }


                // 3. Creating matrices for the image used to detect the red and orange cones in the middle frame
                cv::Mat redConesVector;
                cv::Mat redConesLeftMiddle;
                cv::Mat redMiddle;

                // 3.1. Getting the middle frame using vertical concatenation
                cv::vconcat(redConesMiddle, blackMiddle, redMiddle);

                // 3.2. Getting the whole matrix using horizontal concatenation in two steps
                cv::hconcat(redConesLeft, redMiddle, redConesLeftMiddle);
                cv::hconcat(redConesLeftMiddle, redConesRight, redConesVector);

                // 3.3. Performing dilate and erode on blueConesVector matrix

                cv::Mat dilateRed;
                cv::dilate(redConesVector, dilateRed, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);

                cv::Mat erodeRed;
                cv::erode(dilateRed, erodeRed, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);

                // 3.4. Creating vector of vectors of points to generate contours for yellow cones
                std::vector < std::vector < cv::Point >> redContours;
                cv::findContours(erodeRed,redContours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE); // Find the countours of the cone
                // cv::drawContours(imgCropped,contours,-1,cv::Scalar(0,230,64),3); // Draw the borders of the cone

                // 3.5. Creating a vector to store the center points for yellow cones
                std::vector < cv::Point > centerPointsRed;

                // 3.6. For loop used to draw all the relevant data in the image
                for (size_t i =0; i< redContours.size();++i) {
                    cv::Rect boundRect = cv::boundingRect(redContours[i]); // Creating rectangles based in the contours vector
                    if (boundRect.area() < 8000 && boundRect.area() > 500 && (boundRect.height / boundRect.width < 2.2 && boundRect.height / boundRect.width > 0.8 )) {
                        
                        if (VERBOSE) {
                            cv::rectangle(imgCropped,boundRect.tl(),boundRect.br(),cv::Scalar(0,230,64),3); // Drawing rectangles based in the contours vector
                            cv::rectangle(imgCropped,((boundRect.tl()+boundRect.br())/2 + pt2),((boundRect.tl()+boundRect.br())/2 + pt1),cv::Scalar(230,30,0),2); // Creating mini rectangles to evaluate the middle points of the cone
                        }
                        centerPointsRed.push_back((boundRect.tl()+boundRect.br())/2);
                    }
                }

                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
                {
                    std::lock_guard<std::mutex> lck(distancesMutex);
                }

                {
                    std::lock_guard<std::mutex> lck(steeringMutex);
                    
                    double blueSum = 0.0;
                    for (auto &p : centerPointsBlue) {
                        blueSum += p.x;
                    }
                    double yellowSum = 0.0;
                    for (auto &p : centerPointsYellow) {
                        yellowSum += p.x;
                    }

                    double blueMeanX = blueSum / centerPointsBlue.size();
                    double yellowMeanX = yellowSum / centerPointsYellow.size();

                    if (std::isnan(blueMeanX)) {
                        blueMeanX = 1280.0;
                    }
                    if (std::isnan(yellowMeanX)) {
                        yellowMeanX = 0.0;
                    }

                    double OFFSET_Y = 480.0; // Tune these parameters
                    double globalMeanX = (blueMeanX + yellowMeanX) / 2.0;
                    double deltaX = globalMeanX - 640.0;
                    double deltaY = OFFSET_Y - 720.0;

                    double angleToGlobalMean = - atan2(deltaY, deltaX) - 1.57;

                    double groundSteeringSent {0.0};
                    float pedalPosSent {0.0};
                    double groundSteeringCurrent = angleToGlobalMean * steeringFactor;

                    // Empty the buffer if size is reached
                    if ((int) yellowHistoryQueue.size() > turningBufferSize) {
                        yellowHistoryQueue.pop_front();
                        blueHistoryQueue.pop_front();
                    }
                    yellowHistoryQueue.push_back(centerPointsYellow.size());
                    blueHistoryQueue.push_back(centerPointsBlue.size());

                    // Algorithm to find max frequency of buffer and its corresponding element
                    int numZeroYellow {0};
                    int numZeroBlue {0};
                    for(int i = 0; i < (int) yellowHistoryQueue.size(); i ++) {
                        if (yellowHistoryQueue.at(i) == 0) {
                            numZeroYellow++;
                        }
                        if (blueHistoryQueue.at(i) == 0) {
                            numZeroBlue++;
                        }
                    }
                

                    bool isLeftTurn = false;
                    bool isRightTurn = false;
                    bool isYellowThresTriggered = numZeroYellow > yellowturnthres;
                    bool isBlueThresTriggered = numZeroBlue > blueturnthres;
                    bool isKiwiDetected = (centerX >= 0.001f) && (centerY >= 0.001f);
                    bool areOrangeConesDetected = centerPointsRed.size() > 0;

                    if (areOrangeConesDetected && isKiwiDetected) {
                        haltingDelay = maxHaltingDelay;
                    }

                    // MAIN CONTROL LOGIC

                    if (haltingDelay > 0) {
                        pedalPosSent = 0.0;
                        haltingDelay--;
                    }
                    else if (isKiwiDetected) {
                        pedalPosSent = pedalPosSlow;
                    }
                    else if (isYellowThresTriggered && !isBlueThresTriggered) {
                        groundSteeringSent = -maxSteering;
                        pedalPosSent = pedalPos;
                        isLeftTurn = true;
                    }
                    else if (isBlueThresTriggered && !isYellowThresTriggered) {
                        groundSteeringSent = maxSteering;
                        pedalPosSent = pedalPos;
                        isRightTurn = true;
                    }
                    else {
                        // Moving Average
                        groundSteeringSent = (groundSteeringCurrent * movingAvgFactor) + (groundSteeringPrevious * (1 - movingAvgFactor));
                        pedalPosSent = pedalPos;
                    }

                    // For monitoring
                    std::cout << "Y: ";
                    for(int i = 0; i < (int) yellowHistoryQueue.size(); i ++) {
                        std::cout << yellowHistoryQueue.at(i) << " ";
                    }
                    std::cout << " | n = " << numZeroYellow << std::endl << "B: ";
                    for(int i = 0; i < (int) blueHistoryQueue.size(); i ++) {
                        std::cout << blueHistoryQueue.at(i) << " ";
                    }
                    std::cout << "| n = " << numZeroBlue << " > " << isLeftTurn << isRightTurn << std::endl;
                    std::cout << "orangedetected: " << areOrangeConesDetected << " kiwidetected: " << isKiwiDetected << std::endl;
                    std::cout << "pedal: " << pedalPosSent << " steering: " << groundSteeringSent << " halting: " << haltingDelay << std::endl << std::endl;

                    // Display image.
                    if (VERBOSE) {
                        cv::line(imgCropped, cv::Point((int) blueMeanX, 0), cv::Point((int) blueMeanX, 600), 
                                        cv::Scalar(255, 0, 0), 5, cv::LINE_8);
                        cv::line(imgCropped, cv::Point((int) yellowMeanX, 0), cv::Point((int) yellowMeanX, 600), 
                                        cv::Scalar(0, 255, 255), 5, cv::LINE_8);
                        cv::line(imgCropped, cv::Point((int) globalMeanX, 0), cv::Point((int) globalMeanX, 600), 
                                        cv::Scalar(255, 255, 255), 5, cv::LINE_8);
                        cv::line(imgCropped, cv::Point(640 , 0), cv::Point(640 , 600), 
                                        cv::Scalar(0, 0, 255), 5, cv::LINE_4);
                        cv::imshow("Processed", imgCropped);
                        cv::imshow("Blue", erodeBlue); 
                        cv::imshow("Yellow", erodeYellow);    
                        cv::waitKey(1);
                    }

                    opendlv::proxy::GroundSteeringRequest GSR;
                    GSR.groundSteering((float) groundSteeringSent);
                    od4.send(GSR);

                    opendlv::proxy::PedalPositionRequest PPR;
                    PPR.position(pedalPosSent);
                    od4.send(PPR);

                    opendlv::proxy::AngleReading ar;
                    ar.angle((float) angleToGlobalMean);
                    od4.send(ar);

                    // Save history
                    groundSteeringPrevious = groundSteeringCurrent;
                }

                ////////////////////////////////////////////////////////////////
                // Example for creating and sending a message to other microservices; can
                // be removed when not needed.
                // opendlv::proxy::AngleReading ar;
                // ar.angle(angleToGlobalMean);
                // od4.send(ar);

                ////////////////////////////////////////////////////////////////
                // Steering and acceleration/decelration.
                //
                // Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
                // Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
                //opendlv::proxy::GroundSteeringRequest gsr;
                //gsr.groundSteering(0);
                //od4.send(gsr);

                // Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
                // Be careful!
                //opendlv::proxy::PedalPositionRequest ppr;
                //ppr.position(0);
                //od4.send(ppr);
            }
            
        }
        retCode = 0;
    }
    return retCode;
}

