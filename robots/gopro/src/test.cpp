#include <iostream>
#include <opencv2/opencv.hpp>  //OpenCV header to use VideoCapture class//

using namespace std;
using namespace cv;

int main() {
  Mat myImage;                  //Declaring a matrix to load the frames//
  namedWindow("Video Player");  //Declaring the video to show the video//
  char filename[] = "/dev/video0";
  // VideoCapture cap("usb-Elgato_Elgato_HD60_X_A00XB320216NRW-video-index0");
  VideoCapture cap("/dev/video0");
  if (!cap.isOpened()) {  //This section prompt an error message if no video stream is found//
    cout << "No video stream detected" << endl;
    system("pause");
    return -1;
  }
  while (true) {  //Taking an everlasting loop to show the video//
    cout << "Reading new frame" << endl;
    cap >> myImage;
    if (myImage.empty()) {  //Breaking the loop if no video frame is detected//
      cout << "Empty frame. Terminate" << endl;
      break;
    }

    imshow("Video Player", myImage);  //Showing the video//
    char c = (char)waitKey(
        25);  //Allowing 25 milliseconds frame processing time and initiating break condition//
    if (c == 27) {  //If 'Esc' is entered break the loop//
      break;
    }
  }
  cout << "Video ended. Release buffer memory" << endl;
  cap.release();  //Releasing the buffer memory//
  return 0;
}