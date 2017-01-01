#include "BGRAVideoFrame.h"
#include "CameraReader.h"
#include "Marker.hpp"
#include "MarkerDetector.hpp"
#include "djicam.h"
#include <opencv2/opencv.hpp>
#include <time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

int main()
{
//      CameraReader reader(smallUSB);
//      reader.SetMarkerSize(5.4);
//      bool ret = reader.Start();
//      if (!ret) {
//        cout<<"failed"<<endl;
//      };
//      Mat large;
//      Size dsize;
//      dsize.width=1280;
//      dsize.height=1024;
//      while (true) {
//        if (reader.isImageGot) {
//            //resize(reader.input,large,Size(1280,1024));
//          imshow("1", reader.input);
//          //resizeWindow("1",1280,1024);
//          // cout<<reader.height<<"\t"<<reader.width<<endl;
//        }
//        if (reader.isImageGot && reader.isMarkerCatched) {
//          cout << reader.xPoint << "\t" << reader.yPoint << "\t"
//               << reader.foundMarker.x<<"\t"<<
//                  reader.foundMarker.y<<"\t" <<
//                  reader.foundMarker.z<<"\t"<< reader.foundMarker.id<<"\t"<<
//                  endl;
//        }
//        waitKey(30);
//      }
    /////////////////////////
    VideoCapture cap;
    cap.open(0);
    Mat input;
    float fc1 = 730.166036369399 / 1.6,
            fc2 = 729.992389107947 / 1.6,
            cc1 = 489.104940065542 / 1.6,
            cc2 = 383.685983738317 / 1.6,
            kc1 = -0.0116300151234399,
            kc2 = 0.0407467972044594,
            kc3 = 0,
            kc4 = 0,
            distorsionCoeff[4];
    distorsionCoeff[0] = kc1;
    distorsionCoeff[1] = kc2;
    distorsionCoeff[2] = kc3;
    distorsionCoeff[3] = kc4;
    CameraCalibration calibration = CameraCalibration(fc1, fc2, cc1, cc2, distorsionCoeff);
    MarkerDetector markerDetector(calibration);
    markerDetector.ChangeMarkerSize(5.4, 0.945,0.81);
    vector<Transformation> trans;
    vector<Transformation>::iterator tr;
    Mat_<float> srcMat=Mat_<float>(3, 3), dstMat=Mat_<float>(1, 3);
    while(true)
    {
        if(cap.isOpened())
        {
            cap >> input;
            imshow("1",input);
            BGRAVideoFrame frame;
            frame.height = input.rows;
            frame.width = input.cols;
            frame.stride = input.step;
            frame.data = input.data;
            markerDetector.processFrame(frame);
            trans = markerDetector.getTransformations();
            int count = trans.size();
            cout<<count<<endl;
            for (tr = trans.begin(); tr != trans.end(); tr++) {
                for (int col = 0; col < 3; col++) {
                    for (int row = 0; row < 3; row++) {
                        srcMat(row, col) = tr.base()->r().mat[row][col];
                    }
                }
                Rodrigues(srcMat, dstMat);
                if(tr.base()->id()!=-1)
                {
                    cout<<tr.base()->id()<<"\t"<<
                          tr.base()->t().data[0]<<"\t"<<
                          tr.base()->t().data[1]<<"\t"<<
                          tr.base()->t().data[2]<<"\t"<<
                          dstMat(0,0)*180/M_PI<<"\t"<<
                          dstMat(0,1)*180/M_PI<<"\t"<<
                          dstMat(0,2)*180/M_PI<<endl;

                }
            }
        }
        else
        {
            cout<<"camera error"<<endl;
        }
        waitKey(1);
    }

    return 0;
}
