/**
* This file is part of ORB-SLAM3.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM3>
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <string>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>


using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


// how to run:
// 0 for sequence, 1 for webcam, 2 for drone
// ./Examples/Monocular/mono_tum 0 Vocabulary/ORBvoc.txt Examples/Monocular/DRONE_PARAMS.yaml PATH_TO_SEQUENCE
// ./Examples/Monocular/mono_tum 1 Vocabulary/ORBvoc.txt Examples/Monocular/DRONE_PARAMS.yaml
// ./Examples/Monocular/mono_tum 2 Vocabulary/ORBvoc.txt Examples/Monocular/DRONE_PARAMS.yaml

int main(int argc, char **argv)
{

    std::string str0 ("0");
    std::string str1 ("1");
    std::string str2 ("2");
    if (str1.compare(argv[1]) == 0) {
        if(argc != 4) {
            cerr << endl << "argc:" << argc << "!= 4"<< endl;
        }

        cv::VideoCapture cap(0);

        if (!cap.isOpened()) {
            cerr << endl << "Could not open camera feed." << endl;
            return -1;
        }
        
         // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
        float imageScale = SLAM.GetImageScale();

        double t_resize = 0.f;
        double t_track = 0.f;

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point initT = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point initT = std::chrono::monotonic_clock::now();
#endif

        // Main loop
        while(true)//cv::waitKey(0) != 27)
        {
            //Create a new Mat
            cv::Mat frame;

            //Send the captured frame to the new Mat
            cap >> frame;

            if(frame.empty())
                break;
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point nowT = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point nowT = std::chrono::monotonic_clock::now();
#endif
            // Pass the image to the SLAM system
            SLAM.TrackMonocular(frame, std::chrono::duration_cast<std::chrono::duration<double> >(nowT-initT).count());
        }

        // Save points
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
        std::ofstream pointData;
        pointData.open("/tmp/pointData.csv");
        for(auto p : mapPoints) {
            if (p != NULL)
            {
                auto point = p->GetWorldPos();
                Eigen::Matrix<double, 3, 1> v = ORB_SLAM3::Converter::toVector3d(point);
                pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
            }
        }
        pointData.close();
    
        // Stop all threads
        SLAM.Shutdown();

        //slam->SaveSeperateKeyFrameTrajectoryTUM("KeyFrameTrajectory-1.txt", "KeyFrameTrajectory-2.txt", "KeyFrameTrajectory-3.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    if (str2.compare(argv[1]) == 0) {
        if(argc != 4) {
            cerr << endl << "argc:" << argc << "!= 4"<< endl;
        }

        cv::VideoCapture cap("udp://@0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000");

        if (!cap.isOpened()) {
            cerr << endl << "Could not open camera feed." << endl;
            return -1;
        }
        ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::MONOCULAR, true);
        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point initT = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point initT = std::chrono::monotonic_clock::now();
#endif

        // Main loop
        while(true)//cv::waitKey(0) != 27)
        {
            //Create a new Mat
            cv::Mat frame;

            //Send the captured frame to the new Mat
            cap >> frame;

            if(frame.empty())
                break;
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point nowT = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point nowT = std::chrono::monotonic_clock::now();
#endif
            // Pass the image to the SLAM system
            SLAM.TrackMonocular(frame, std::chrono::duration_cast<std::chrono::duration<double> >(nowT-initT).count());
        }

        // Save points
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
        std::ofstream pointData;
        pointData.open("/tmp/pointData.csv");
        for(auto p : mapPoints) {
            if (p != NULL)
            {
                auto point = p->GetWorldPos();
                Eigen::Matrix<double, 3, 1> v = ORB_SLAM3::Converter::toVector3d(point);
                pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
            }
        }
        pointData.close();
    
        // Stop all threads
        SLAM.Shutdown();

        //slam->SaveSeperateKeyFrameTrajectoryTUM("KeyFrameTrajectory-1.txt", "KeyFrameTrajectory-2.txt", "KeyFrameTrajectory-3.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }


    if (str0.compare(argv[1]) == 0) {
        if(argc != 5){
            cerr << endl << "Usage: NUM ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
            return 1;
        }

        // Retrieve paths to images
        vector<string> vstrImageFilenames;
        vector<double> vTimestamps;
        string strFile = string(argv[4])+"/rgb.txt";
        LoadImages(strFile, vstrImageFilenames, vTimestamps);

        int nImages = vstrImageFilenames.size();

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM3::System SLAM(argv[2],argv[3],ORB_SLAM3::System::MONOCULAR,true);

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(nImages);

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;

        // Main loop
        cv::Mat im;
        for(int ni=0; ni<nImages; ni++)
        {
            // Read image from file
            im = cv::imread(string(argv[4])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                    << string(argv[4]) << "/" << vstrImageFilenames[ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
                    {
            #ifdef REGISTER_TIMES
                #ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
                #else
                        std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
                #endif
            #endif
                        int width = im.cols * imageScale;
                        int height = im.rows * imageScale;
                        cv::resize(im, im, cv::Size(width, height));
            #ifdef REGISTER_TIMES
                #ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
                #else
                        std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
                #endif
                        t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                        SLAM.InsertResizeTime(t_resize);
            #endif
                    }


    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

    #ifdef REGISTER_TIMES
                t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
                SLAM.InsertTrackTime(t_track);
    #endif
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
        
        // Save points
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
        std::ofstream pointData;
        pointData.open("/tmp/pointData.csv");
        for(auto p : mapPoints) {
            if (p != NULL)
            {
                auto point = p->GetWorldPos();
                Eigen::Matrix<double, 3, 1> v = ORB_SLAM3::Converter::toVector3d(point);
                pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
            }
        }
        pointData.close();
        
        // Stop all threads
        SLAM.Shutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;

        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
