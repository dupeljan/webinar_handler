#ifndef VIDEO_HANDLER_H
#define VIDEO_HANDLER_H


#include "all_headers.h"
#include "img_handler.h"

#define MIN_CURSOR_AREA 250//300
#define VIDEO_PATCH "/home/dupeljan/Projects/webinar_analisator/webinar.mp4"
#define PIC_A "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/6.png"
#define PIC_B "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/c.png"
#define CURSOR "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/cursor1.png"
#define SLIDE_PATH "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/gen_slides_shorter/"
#define FRAME_EACH_MSECOND 50 * 10e3
#define DEBUG_VIDEO 0

using namespace cv;
using namespace std;



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
    Mat cursor_covering_mask; // For error reduction
    Rect area;
    vector<Mat> slides;

    void get_cursor_mask(Mat src, Mat& dst);
    void shift_video_get_difference_local(VideoCapture cap, int shift, Diff_dict &dst);
public:
    Presentation(Mat cursor,Rect area);
    void generate(VideoCapture cap, int shift);
    void write_slides(string patch);
    vector<Mat> get_slides(){ return slides; }

};

int video_main();
size_t get_duradion(VideoCapture src);
VideoWriter get_VideoWriter(VideoCapture cap);


// Return abs_diff form current time + shift img and current time img
// Change VideoCapture posintion
void shift_video_get_difference(VideoCapture src, int shift, Diff_dict &dst);




#endif // VIDEO_HANDLER_H
