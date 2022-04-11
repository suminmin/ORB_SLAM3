/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_mp4 path_to_times_file " << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    //vector<string> vstrImageFilenames;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsCamImu;
    int nImages;
    int nImu;

    int tot_images = 0;
    cout << "Loading images for sequence " << "...";

    string pathSeq(argv[3]);
    string pathTimeStamps(argv[4]);

    string pathCam0 = pathSeq;
    string pathImu = pathSeq + "/imu0/data.csv";

    cv::VideoCapture vVideo;
    vVideo.open(pathCam0);
    if (vVideo.isOpened() == false) {
        std::cout << " failed to open " << pathCam0 << std::endl;
        // 動画ファイルが開けなかったときは終了する
        return -1;
    }
    nImages = int(vVideo.get(cv::CAP_PROP_FRAME_COUNT));
    cout << "LOADED! : " << nImages << endl;
    
    LoadIMU(pathTimeStamps, vTimestampsCamImu, vAcc, vGyro);
    cout << "LOADED!" << endl;

    tot_images += nImages;
    nImu = vTimestampsCamImu.size();
    std::cout << nImages << " , " << nImu << std::endl;

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU for sequence" << endl;
        return 1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;


    // Main loop
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    int proccIm = 0;
    for(int ni=0; ni<nImages; ni++, proccIm++)
    {
        // Read image from file
        vVideo >> im;
        if (im.empty() == true) break;

        // Crop
        //cv::Rect crop_roi(0, 0, im.cols/2, im.rows); //left
        cv::Rect crop_roi(im.cols/2, 0, im.cols/2, im.rows); //right
        im = im(crop_roi);
        //

        double tframe = vTimestampsCamImu[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " <<  ni << endl;
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

        // Load imu measurements from previous frame
        vImuMeas.clear();

        if(ni>0)
        {
            // cout << "t_cam " << tframe << endl;

            vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[ni-1].x,vAcc[ni - 1].y,vAcc[ni - 1].z,
                                                        vGyro[ni - 1].x,vGyro[ni].y,vGyro[ni - 1].z,
                                                        vTimestampsCamImu[ni - 1]));
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[ni].x, vAcc[ni].y, vAcc[ni].z,
                                vGyro[ni].x, vGyro[ni].y, vGyro[ni].z,
                                vTimestampsCamImu[ni]));
        }

    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

        // Pass the image to the SLAM system
        // cout << "tframe = " << tframe << endl;
        SLAM.TrackMonocular(im, tframe, vImuMeas); // TODO change to monocular_inertial

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
        ttrack_tot += ttrack;
        // std::cout << "ttrack: " << ttrack << std::endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCamImu[ni+1] - tframe;
        else if(ni>0)
            T = tframe - vTimestampsCamImu[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }
    //if(seq < num_seq - 1)
    //{
    //    cout << "Changing the dataset" << endl;

    //    SLAM.ChangeDataset();
    //}

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    std::cout << "load IMU" << std::endl;

    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
