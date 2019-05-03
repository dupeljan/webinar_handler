#ifndef VIDEO_HANDLER_H
#define VIDEO_HANDLER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "img_handler.h"

#define MIN_CURSOR_AREA 250//300

using namespace cv;
using namespace std;

enum cmp_enum{
    less,
    equal,
    large,
    cross,
};

struct Diff_dict{
    Mat first;
    Mat second;
    Mat diff;
    void operator ()(Rect rect){
        first  = first(rect);
        second = second(rect);
        diff   = diff(rect);
    }
};

class Cursor {
    Mat img;
    Diff_dict diff;
    vector<Rect>  b_rects;

    void filter_rects();
    void threshold_diff();
    void find_bound_rects_diff();
public:
    Cursor(){};
    void find_cursor(VideoCapture cap, int hit_lim, int shift);
    Mat get(){ return img;}

};

class Presentation {
    Mat cursor;
    Mat cursor_mask;
    vector<Mat> slides;
    void get_cursor_mask(Mat src, Mat& dst);
public:
    Presentation(Mat cursor);
    void generate(VideoCapture cap,Rect area, int shift);
};

int video_main();
size_t get_duradion(VideoCapture src);
VideoWriter get_VideoWriter(VideoCapture cap);


double cmp(Mat x, Mat y);
cmp_enum cmp_shape(Mat x, Mat y);
// Return abs_diff form current time + shift img and current time img
// Change VideoCapture posintion
void shift_video_get_difference(VideoCapture src, int shift, Diff_dict &dst);
void matchTemplateCoords(Mat img, Mat templ,Mat mask,Point& matchLoc);



#endif // VIDEO_HANDLER_H
