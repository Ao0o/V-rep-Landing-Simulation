/*****************************************************************************
*   Marker.cpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include "DebugHelpers.hpp"
#include "Marker.hpp"

cv::Mat Marker::bigMarker =
    (cv::Mat_<uchar>(3, 3) << 0, 0, 1, 0, 0, 0, 1, 0, 1);
cv::Mat Marker::smallMarker =
    (cv::Mat_<uchar>(3, 3) << 1, 0, 1, 1, 1, 0, 1, 0, 0);
cv::Mat Marker::frontMarker =
    (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 0, 1, 0, 1, 0, 1);
cv::Mat Marker::frontSmallMarker =
        (cv::Mat_<uchar>(3, 3) << 0, 0, 1, 1, 1, 0, 0, 1, 0);
cv::Mat Marker::markers[10];
bool Marker::IsMarkersInited;
// cv::Mat Marker::bigMarker(3,3,CV_8UC1),Marker::smallMarker(3,3,CV_8UC1);
Marker::Marker() : id(-1) {
    if(!IsMarkersInited)
    {
        markers[0]=(cv::Mat_<uchar>(3, 3) << 0, 0, 1, 1, 0, 0, 1, 0, 1);
        markers[1]=(cv::Mat_<uchar>(3, 3) << 1, 0, 1, 1, 1, 0, 1, 0, 0);
//        markers[2]=(cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 0, 0, 0, 1, 0);
//        markers[3]=(cv::Mat_<uchar>(3, 3) << 1, 1, 0, 0, 0, 0, 0, 0, 0);
//        markers[4]=(cv::Mat_<uchar>(3, 3) << 1, 0, 0, 1, 0, 0, 0, 0, 0);
//        markers[5]=(cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 0, 0, 0, 0, 0);
//        markers[6]=(cv::Mat_<uchar>(3, 3) << 0, 1, 0, 0, 0, 0, 0, 0, 0);
//        markers[7]=(cv::Mat_<uchar>(3, 3) << 0, 1, 0, 1, 0, 0, 0, 0, 0);
//        markers[8]=(cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 0, 1, 0, 0, 0);
//        markers[9]=(cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 0);
        IsMarkersInited=true;
    }

}

bool operator<(const Marker &M1, const Marker &M2) { return M1.id < M2.id; }

cv::Mat Marker::rotate(cv::Mat in) {
  cv::Mat out;
  in.copyTo(out);
  for (int i = 0; i < in.rows; i++) {
    for (int j = 0; j < in.cols; j++) {
      out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
    }
  }
  return out;
}

int Marker::hammDistMarker(cv::Mat bits) {
  int ids[4][5] = {
      {1, 0, 0, 0, 0}, {1, 0, 1, 1, 1}, {0, 1, 0, 0, 1}, {0, 1, 1, 1, 0}};

  int dist = 0;

  for (int y = 0; y < 5; y++) {
    int minSum = 1e5; // hamming distance to each possible word

    for (int p = 0; p < 4; p++) {
      int sum = 0;
      // now, count
      for (int x = 0; x < 5; x++) {
        sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
      }

      if (minSum > sum)
        minSum = sum;
    }

    // do the and
    dist += minSum;
  }

  return dist;
}

int Marker::MarkerConfirm(cv::Mat bits) {
  // std::cout<<bits.channels()<<"\t"<<bigMarker.channels()<<"\t"<<bits.depth()<<"\t"<<bigMarker.depth()<<std::endl;
//  if (cv::countNonZero(bits == smallMarker) == 9) {
//    return 2;
//  } else if (cv::countNonZero(bits == bigMarker) >= 8) {
//    return 1;
//  }
//  else if(cv::countNonZero(bits==frontMarker)==9){
//    return 3;
//  }
//  else if(cv::countNonZero(bits==frontSmallMarker)==9){
//      return 4;
//  }
    if (cv::countNonZero(bits == markers[0]) >= 8) {
      return 0;
    }
  for(int j=1;j<2;j++)
  {
      if (cv::countNonZero(bits == markers[j]) == 9) {
        return j;
      }
  }

  return -1;
}

int Marker::mat2id(const cv::Mat &bits) {
  int val = 0;
  for (int y = 0; y < 5; y++) {
    val <<= 1;
    if (bits.at<uchar>(y, 1))
      val |= 1;
    val <<= 1;
    if (bits.at<uchar>(y, 3))
      val |= 1;
  }
  return val;
}

int Marker::getMarkerId(cv::Mat &markerImage, int &nRotations) {
  assert(markerImage.rows == markerImage.cols);
  assert(markerImage.type() == CV_8UC1);

  cv::Mat grey = markerImage;
  long int average = 0;
  for (int row = 0; row < markerImage.rows; row++) {
    uchar *p = markerImage.ptr(row);
    for (int col = 0; col < markerImage.cols; col++) {
      average = average + *(p + col);
    }
  }
  average = average / (markerImage.rows * markerImage.cols);
  // average=(average+128)/2;
  // Threshold image
  cv::threshold(grey, grey, average, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

#ifdef SHOW_DEBUG_IMAGES
  cv::showAndSave("Binary marker", grey);
#endif

  // Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to
  // marker info
  // the external border should be entirely black

  int cellSize = markerImage.rows / 7;

  for (int y = 0; y < 7; y++) {
    int inc = 6;

    if (y == 0 || y == 6)
      inc = 1; // for first and last row, check the whole border

    for (int x = 0; x < 7; x += inc) {
      int cellX = x * cellSize;
      int cellY = y * cellSize;
      cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));

      int nZ = cv::countNonZero(cell);

      if (nZ > (cellSize * cellSize) / 2) {
        return -1; // can not be a marker because the border element is not
                   // black!
      }
    }
  }

  cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);

  // get information(for each inner square, determine if it is  black or white)
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      int cellX = (x + 1) * cellSize;
      int cellY = (y + 1) * cellSize;
      cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));

      int nZ = cv::countNonZero(cell);
      if (nZ > (cellSize * cellSize) / 2)
        bitMatrix.at<uchar>(y, x) = 1;
    }
  }
  bitMatrix.at<uchar>(2, 2) = 1; // center square is always considered as white
  // check all possible rotations
  cv::Mat rotations[4];
  int distances[4];

  rotations[0] = bitMatrix;
  distances[0] = hammDistMarker(rotations[0]);

  std::pair<int, int> minDist(distances[0], 0);

  for (int i = 1; i < 4; i++) {
    // get the hamming distance to the nearest possible word
    rotations[i] = rotate(rotations[i - 1]);
    distances[i] = hammDistMarker(rotations[i]);

    if (distances[i] < minDist.first) {
      minDist.first = distances[i];
      minDist.second = i;
    }
  }

  nRotations = minDist.second;
  if (minDist.first == 0) {
    return mat2id(rotations[minDist.second]);
  }

  return -1;
}

void Marker::drawContour(cv::Mat &image, cv::Scalar color) const {
  float thickness = 2;

  cv::line(image, points[0], points[1], color, thickness, CV_AA);
  cv::line(image, points[1], points[2], color, thickness, CV_AA);
  cv::line(image, points[2], points[3], color, thickness, CV_AA);
  cv::line(image, points[3], points[0], color, thickness, CV_AA);
}

int Marker::CheckMarker(cv::Mat &markerImage, int &nRotations) {
  assert(markerImage.rows == markerImage.cols);
  assert(markerImage.type() == CV_8UC1);

  cv::Mat grey = markerImage;
  long int average = 0;
  for (int row = 0; row < markerImage.rows; row++) {
    uchar *p = markerImage.ptr(row);
    for (int col = 0; col < markerImage.cols; col++) {
      average = average + *(p + col);
    }
  }
  average = average / (markerImage.rows * markerImage.cols);

  // Threshold image
  cv::threshold(grey, grey, average, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

#ifdef SHOW_DEBUG_IMAGES
  cv::showAndSave("Binary marker", grey);
#endif
//  cv::imshow("Binary marker", grey);

  // Markers  are divided in 5x5 regions, of which the inner 3x3 belongs to
  // marker info
  // the external border should be entirely black

  int cellSize = markerImage.rows / 5;

  for (int y = 0; y < 5; y++) {
    int inc = 4;

    if (y == 0 || y == 4)
      inc = 1; // for first and last row, check the whole border

    for (int x = 0; x < 5; x += inc) {
      int cellX = x * cellSize;
      int cellY = y * cellSize;
      cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));

      int nZ = cv::countNonZero(cell);

      if (nZ > (cellSize * cellSize) / 2) {
        return -1; // can not be a marker because the border element is not
                   // black!
      }
    }
  }

  cv::Mat bitMatrix = cv::Mat::zeros(3, 3, CV_8UC1);

  // get information(for each inner square, determine if it is  black or white)
  for (int y = 0; y < 3; y++) {
    for (int x = 0; x < 3; x++) {
      int cellX = (x + 1) * cellSize;
      int cellY = (y + 1) * cellSize;
      cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));

      int nZ = cv::countNonZero(cell);
      if (nZ > (cellSize * cellSize) / 2)
        bitMatrix.at<uchar>(y, x) = 1;
    }
  }

  // check all possible rotations
  cv::Mat rotations[4];
  int result;

  rotations[0] = bitMatrix;
  result = MarkerConfirm(rotations[0]);
//  if (result[0] == 1) {
//    nRotations = 0;
//    return 1;
//  } else if (result[0] == 2) {
//    nRotations = 0;
//    return 2;
//  } else if (result[0] == 3) {
//    nRotations = 0;
//    return 3;
//  }else if (result[0] == 4) {
//      nRotations = 0;
//      return 4;
//    }
  if(result!=-1)
  {
      nRotations = 0;
      return result;
  }
  // std::pair<int,int> minDist(result[0],0);

  for (int i = 1; i < 4; i++) {
    // get the hamming distance to the nearest possible word
    rotations[i] = rotate(rotations[i - 1]);
    result = MarkerConfirm(rotations[i]);
//    if (result[i] == 1) {
//      nRotations = i;
//      return 1;
//    } else if (result[i] == 2) {
//      nRotations = i;
//      return 2;
//    } else if (result[i] == 3) {
//      nRotations = i;
//      return 3;
//    } else if (result[i] == 4) {
//        nRotations = i;
//        return 4;
//      }
    if(result!=-1)
    {
        nRotations = i;
        return result;
    }
  }

  return -1;
}
