#include "video_handler.h"

#define VIDEO_PATCH "/home/dupeljan/Projects/webinar_analisator/webinar.mp4"
#define FRAME_EACH_MSECOND 50 * 10e3

int video_main(){
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    size_t time = 300000;
    //cap.set(CAP_PROP_POS_MSEC,time);
    //cap.set(CAP_PROP_FPS,1000);
    Mat edges;
    namedWindow("edges",1);

    int frame_width=   cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter video("/home/dupeljan/Projects/webinar_analisator/dipa.avi", VideoWriter::fourcc('M','J','P','G'),24, Size(frame_width,frame_height),true);

    for(;;)
    {
        Mat frame[2], motion;
        cap >> frame[0];
        time += FRAME_EACH_MSECOND;
        //cap.set(CAP_PROP_POS_MSEC,time);
        cap >> frame[1]; // get a new frame from camera
        /*
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        cvtColor(edges, edges, COLOR_GRAY2BGR);
        */
        //video.write(edges);
        absdiff(frame[0],frame[1],motion);
        imshow("video",motion);
        if(waitKey(30) >= 0) break;
    }
    //video.release();
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
