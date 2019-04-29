#include "video_handler.h"


#define VIDEO_PATCH "/home/dupeljan/Projects/webinar_analisator/webinar.mp4"
#define PIC_A "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/a.png"
#define PIC_B "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/c.png"
#define FRAME_EACH_MSECOND 50 * 10e3
#define DEBUG_VIDEO 1

int video_main(){
    Mat a = imread(PIC_A, IMREAD_COLOR);
    Mat b = imread(PIC_B, IMREAD_COLOR);
    vec_imshow("inp",{a,b});
    double result = cmp(a,b);
    cout << result << endl;
    /*
    VideoCapture cap(VIDEO_PATCH); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    size_t time = 300000;
    cap.set(CAP_PROP_POS_MSEC,time);
    //cap.set(CAP_PROP_FPS,1000);
    Mat edges;
    namedWindow("edges",1);


    auto duration = get_duradion(cap);
    //auto x = cap.get(CAP_PROP_POS_MSEC)  ;
    while(cap.get(CAP_PROP_POS_MSEC)  < duration )
    {
        Mat frame[2], motion;
        cap >> frame[0];
        time += FRAME_EACH_MSECOND;
        //cap.set(CAP_PROP_POS_MSEC,time);
        cap >> frame[1]; // get a new frame from camera
        */
        /*
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        cvtColor(edges, edges, COLOR_GRAY2BGR);
        */
        //video.write(edges);
        /*absdiff(frame[0],frame[1],motion);
        imshow("video",motion);
        if(waitKey(30) >= 0) break;
    }

    */
    //video.release();
    // the camera will be deinitialized automatically in VideoCapture destructor
    waitKey(0); // Wait for a keystroke in the window
    return 0;
}

// Cursor
void Cursor::find_cursor(VideoCapture cap, int hit_lim, int shift){
    vector<Rect>  b_rects;
    Mat diff;
    //size_t frame_count = cap.get(CAP_PROP_FRAME_COUNT);
    const float accuracy = 0.9;
    //function<bool(VideoCapture)> EoF = [frame_count](VideoCapture cap){ return cap.get(CAP_PROP_POS_FRAMES) >= frame_count;};
    function<bool(pair<int,Mat>,pair<int,Mat>)> cmp_first = [](pair<int,Mat> x,pair<int,Mat> y){
        return x.first < y.first;
    };

    while(! b_rects.size() ){
        shift_video_get_difference(cap,shift,diff);
        if (diff.empty())
            break;
        find_bound_rects(diff,b_rects);
    }
    // Inicialize chain
    vector<pair<int,Mat>> chains;
    for( auto &x : b_rects)
        chains.push_back(make_pair(1,diff(x)));
    if ( chains.size() ){
        while( max_element(chains.begin(),chains.end(),cmp_first)->first < hit_lim  ){

            shift_video_get_difference(cap,shift,diff);
            if(diff.empty())
                break;
            find_bound_rects(diff,b_rects);

            for(auto &rect : b_rects){
                bool find = false;
                Mat piece = diff(rect);
                for ( auto chain = chains.begin(); chain != chains.end() && !find; chain++ )
                    if ( cmp(piece,chain->second) >= accuracy ){
                        chain->first++;
                        find = true;
                    }
                    // if don't find appropriate template
                    if ( !find )
                    // add rectangile to the chains
                        chains.push_back(make_pair(1,piece));
            }
        }


        this->img = max_element(chains.begin(),chains.end(),cmp_first)->second;
    }
}

// end Cursor

double cmp(Mat x, Mat y){
    // make x in y shape
    auto shape = cmp_shape(x,y);
    if ( shape != cmp_enum::equal){
        switch (shape) {
        case cmp_enum::large:{
                // swap x and y
                Mat tmp = x.clone();
                x = y.clone();
                y = tmp.clone();
                break;
            }
        case cmp_enum::cross:{
                // Expand y
                auto height =  x.cols + y.cols;
                auto weight =  x.rows + y.rows;
                Mat expand_y = Mat::zeros(weight,height,x.type());
                expand_y = Scalar(255,255,255);

                Mat insetImage;
                insetImage = Mat(expand_y , Rect(0,0, y.cols, y.rows ));
                y.copyTo(insetImage);
                y = expand_y.clone();
                break;
            }
        }




    } // now x in y

    // Find out mathes locatoin
    Mat map;
    auto match_method = TM_CCORR_NORMED;
    Mat mask;
    thresh_otsu(x,mask);
    cvtColor(mask,mask,COLOR_GRAY2BGR);
#if DEBUG_VIDEO == 1
    imshow("mask",mask);
#endif
    matchTemplate(y,x,map,match_method,mask);
    normalize( map, map, 0, 1, NORM_MINMAX, -1, Mat() );

    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    minMaxLoc( map, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    imshow("map",map);
    if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
    else
        { matchLoc = maxLoc; }
#if DEBUG_VIDEO == 1
    rectangle(y,Rect(matchLoc.x,matchLoc.y,x.rows,x.cols),Scalar(255,255,0),1);
    imshow("find piece here",y);
#endif
    // compare pieces
    Mat cmp_result;
    matchTemplate(x,y(Rect(matchLoc.x,matchLoc.y,x.rows,x.cols)),cmp_result,TM_SQDIFF,mask);
    //cout << endl << cmp_result.type() << endl;
    switch (cmp_result.type()) {
        case CV_8U:  return cmp_result.at<uchar>(0,0);
        case CV_8S:  return cmp_result.at<schar>(0,0);
        case CV_16U: return cmp_result.at<ushort>(0,0);
        case CV_16S: return cmp_result.at<short>(0,0);
        case CV_32S: return cmp_result.at<int>(0,0);
        case CV_32F: return cmp_result.at<float>(0,0);
        case CV_64F: return cmp_result.at<double>(0,0);
        default:     return 0;
    }

}

cmp_enum cmp_shape(Mat x, Mat y){
    if ( x.cols  < y.cols && x.rows  < y.rows )
        return cmp_enum::less;
    if(  x.cols == y.cols && x.rows == y.rows )
        return cmp_enum::equal;
    if( x.cols  > y.cols && x.rows  > y.rows )
        return cmp_enum::large;
    return cmp_enum::cross;
}

void shift_video_get_difference(VideoCapture src, int shift, Mat &dst){
    Mat frame[2];
    src >> frame[0];
    size_t current_pos = src.get(CAP_PROP_POS_FRAMES);
    if ( current_pos + shift < src.get(CAP_PROP_FRAME_COUNT) /* Might be a param */ ){
        src.set(CAP_PROP_POS_FRAMES,current_pos + shift);
        src >> frame[1];
        src.set(CAP_PROP_POS_FRAMES,current_pos + shift - 1);
        absdiff(frame[1],frame[0],frame[0]);
    }
    dst = frame[0];
}

size_t get_duradion(VideoCapture src){
    return  10e3 * src.get(CAP_PROP_FRAME_COUNT)  / src.get(CAP_PROP_FPS)  ;
}

VideoWriter get_VideoWriter(VideoCapture cap){
    int frame_width=   cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CAP_PROP_FRAME_HEIGHT);
    return VideoWriter("/home/dupeljan/Projects/webinar_analisator/dipa.avi", VideoWriter::fourcc('M','J','P','G'),24, Size(frame_width,frame_height),true);
}

