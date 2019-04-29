#ifndef VIDEO_HANDLER_H
#define VIDEO_HANDLER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "img_handler.h"

#include <iostream>

using namespace cv;
using namespace std;

enum cmp_enum{
    less,
    equal,
    large,
    cross,
};

class Cursor {
    Mat img;
public:
    Cursor(){};
    void find_cursor(VideoCapture cap, int hit_lim, int shift);

};

int video_main();
size_t get_duradion(VideoCapture src);
VideoWriter get_VideoWriter(VideoCapture cap);


double cmp(Mat x, Mat y);
cmp_enum cmp_shape(Mat x, Mat y);
// Return abs_diff form current time + shift img and current time img
// Change VideoCapture posintion
void shift_video_get_difference(VideoCapture src, int shift, Mat &dst);


#endif // VIDEO_HANDLER_H
