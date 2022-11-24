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

#include "opencv2/video/tracking.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>


//Initilize parameters 
std::string configurationPath = "/weights/yolov3-tiny.cfg";
std::string weightsPath = "/weights/yolov3-tiny_final.weights";
std::string objects = "kiwi_car";
std::vector<std::string> classes;

float confThreshold = 0.1f; // Confidence threshold
float nmsThreshold = 0.4f;  // Non-maximum suppression threshold
int inpWidth = 416;        // Width of network's input image
int inpHeight = 416;       // Height of network's input image



//Functions 

std::tuple <uint16_t, uint16_t, uint16_t, uint16_t> postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs)
{
    // ----------------------------------
    // Remove the bounding boxes with low confidence using non-maxima suppression
    // ----------------------------------
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    float max_confidence = 0.0f;
    uint16_t centre_x_fun = 0;
    uint16_t centre_y_fun = 0;
    uint16_t width_fun = 0;
    uint16_t height_fun = 0;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        if (confidences[idx] > max_confidence) {
            max_confidence = confidences[idx];
            centre_x_fun = box.x + box.width/2;
            centre_y_fun = box.y + box.height/2;
            width_fun = box.width;
            height_fun = box.height;
        }
    }
    //static_cast<uint16_t>(centre_y)
    return {centre_x_fun, centre_y_fun, width_fun, height_fun};
}


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("freq")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
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
        const uint32_t FREQ{static_cast<uint32_t>(std::stoi(commandlineArguments["freq"]))};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
            std::mutex distancesMutex;
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
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
            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);


            cv::dnn::Net net = cv::dnn::readNetFromDarknet(configurationPath, weightsPath);
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            std::vector<std::string> outNames = net.getUnconnectedOutLayersNames();
            classes.push_back(objects);

            // Endless loop; end the program by pressing Ctrl-C.

            auto atFrequency{[&]() -> bool
              { 

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
                //Change from rgba to rgb
                cv::cvtColor(img, img, cv::COLOR_RGBA2RGB);


                // Create a 4D blob from a frame.
                cv::Mat blob;
                cv::dnn::blobFromImage(img, blob, 1/255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

                //Sets the input to the network
                net.setInput(blob);

                // Runs the forward pass to get output of the output layers
                std::vector<cv::Mat> outs;
                net.forward(outs, outNames);

                // Remove the bounding boxes with low confidence
                auto [centre_x_msg, centre_y_msg, width_msg, height_msg] = postprocess(img, outs);
                std::cout << "center point is:" << centre_x_msg <<","<< centre_y_msg<<","<< "width is:" << width_msg << "," << height_msg << std::endl;



                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
                {
                    std::lock_guard<std::mutex> lck(distancesMutex);
                    //std::cout << "front = " << front << ", "
                    //          << "rear = " << rear << ", "
                    //          << "left = " << left << ", "
                    //          << "right = " << right << "." << std::endl;
                }

                ////////////////////////////////////////////////////////////////
                // Example for creating and sending a message to other microservices; can
                // be removed when not needed.


                    opendlv::logic::sensation::Geolocation GSR;
                    GSR.latitude((double) centre_x_msg);
                    GSR.longitude((double) centre_y_msg);
                    GSR.altitude((float) width_msg);
                    GSR.heading((float) height_msg);
                    od4.send(GSR);
                return true;
              }};
              
            od4.timeTrigger(FREQ, atFrequency);
            while (od4.isRunning()) {
                std::this_thread::sleep_for(std::chrono::seconds((unsigned int)(-1)));
            }
        }
        retCode = 0;
    }
    return retCode;
}

