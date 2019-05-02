#include "video_handler.h"


#define VIDEO_PATCH "/home/dupeljan/Projects/webinar_analisator/webinar.mp4"
#define PIC_A "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/a.png"
#define PIC_B "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/c.png"
#define FRAME_EACH_MSECOND 50 * 10e3
#define DEBUG_VIDEO 0

int video_main(){
    /*
    Mat a = imread(PIC_A, IMREAD_COLOR);
    Mat b = imread(PIC_B, IMREAD_COLOR);
    vec_imshow("inp",{a,b});
    double result = cmp(a,b);
    cout << result << endl;
    */
    VideoCapture cap(VIDEO_PATCH); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Cursor cursor;
    cursor.find_cursor(cap,5,1000);
    imshow("cursor",cursor.get());
    waitKey(0);
    /*

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
    //waitKey(0); // Wait for a keystroke in the window
    return 0;
}

// Cursor
void Cursor::find_bound_rects_diff(){
    threshold_diff();
    find_bound_rects(diff.diff,b_rects);
    filter_rects();
}

void Cursor::threshold_diff(){
    cvtColor(diff.diff,diff.diff,COLOR_BGR2GRAY);
    threshold(diff.diff,diff.diff,127,255,THRESH_BINARY);
    morphologyEx(diff.diff,diff.diff,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(5,5)));// нижняя граница
}

void Cursor::filter_rects(){
    Mat x = diff.diff;
    //cvtColor(diff.diff,x,COLOR_BGR2GRAY);
    auto it = remove_if(b_rects.begin(), b_rects.end(),[x](Rect i){ return i.area() < MIN_CURSOR_AREA /*|| ! countNonZero(x)*/; } );
    b_rects.erase(it, b_rects.end());
}

void Cursor::find_cursor(VideoCapture cap, int hit_lim, int shift){
    //size_t frame_count = cap.get(CAP_PROP_FRAME_CURSOR_AREA _COUNT);
    const float accuracy = 0.95;
    function<bool(pair<int,Mat>,pair<int,Mat>)> cmp_first =
    [](pair<int,Mat> x,pair<int,Mat> y){
        return x.first < y.first;
    };

    while(! b_rects.size() ){
        shift_video_get_difference(cap,shift,diff);
        if (diff.diff.empty())
            break;

        find_bound_rects_diff();
        //threshold_diff();
        //show_rects(diff.diff,b_rects,"diff");
        //waitKey(0);
    }

    // Inicialize chain
    vector<pair<int,Mat>> chains;
    for( auto &x : b_rects)
        chains.push_back(make_pair(1,diff.second(x)));

    if ( chains.size() ){
        while( max_element(chains.begin(),chains.end(),cmp_first)->first < hit_lim  ){

            shift_video_get_difference(cap,shift,diff);
            if(diff.diff.empty())
                break;
            find_bound_rects_diff();
            //show_rects(diff.diff,b_rects,"diff");
            //waitKey(0);

            set<int> visited;
            vector<pair<int,Mat>> append;
            for(auto &rect : b_rects){
                bool find = false;
                Mat piece = diff.second(rect);
                for ( size_t i = 0; i < chains.size(); i++)
                    if ( cmp(piece,chains[i].second) >= accuracy ){
                        if (visited.find(i) == visited.end()){
                            imshow("piece",piece);
                            imshow("chains[i].second",chains[i].second);
                            waitKey(0);
                            chains[i].first++;
                            visited.insert(i);
                        }
                        find = true;
                    }
                    // if don't find appropriate template
                if ( !find )
                    // add rectangile to the chains
                    append.push_back(make_pair(1,piece));
            }
            chains.insert(chains.end(),append.begin(),append.end());
        }


        this->img = max_element(chains.begin(),chains.end(),cmp_first)->second;
    }
}

// end Cursor

double cmp(Mat x, Mat y){
    // if one piece area much more than another
    // then they different
    //const double area_diff = 2;
    //auto area_x = x.cols * x.rows;
    //auto area_y = y.cols * y.rows;
    //if ( max(area_x,area_y) / (double) min(area_x,area_y) > area_diff )
    const auto quotient_lim = 1.5;
    auto quotient_cols = max(x.cols,y.cols) / (double) min(x.cols,y.cols);
    auto quotient_rows = max(x.rows,y.rows) / (double) min(x.rows,y.rows);
    if ( quotient_cols > quotient_lim || quotient_rows > quotient_lim )
        return 0;

    // if one piece is colorfull and another not
    // then they different
    const auto color_diff_lim = 10;
    Mat x_gray,y_gray;
    cvtColor(x,x_gray,COLOR_BGR2GRAY);
    cvtColor(y,y_gray,COLOR_BGR2GRAY);
    threshold(x_gray,x_gray,127,255,THRESH_BINARY);
    threshold(y_gray,y_gray,127,255,THRESH_BINARY);

    auto area_x = x.cols * x.rows;
    auto area_y = y.cols * y.rows;
    auto cz_x = max(area_x - countNonZero(x_gray),1);
    auto cz_y = max(area_y - countNonZero(y_gray),1);

    if ( max(cz_x,cz_y) / min(cz_x,cz_y) > color_diff_lim )
        return 0;

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
    //imshow("mask",mask);
#endif
    matchTemplate(y,x,map,match_method,mask);
    normalize( map, map, 0, 1, NORM_MINMAX, -1, Mat() );

    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    minMaxLoc( map, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    //imshow("map",map);
    if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
    else
        { matchLoc = maxLoc; }

    //rectangle(y,Rect(matchLoc.x,matchLoc.y,x.rows,x.cols),Scalar(255,255,0),1);
    //imshow("find piece here",y);

    // compare pieces
    Mat cmp_result;
    matchTemplate(x,y(Rect(matchLoc.x,matchLoc.y,x.cols,x.rows)),cmp_result,TM_SQDIFF,mask);
    //cout << endl << cmp_result.type() << endl;
    double res;
    switch (cmp_result.type()) {
        case CV_8U:  res = cmp_result.at<uchar> (0,0);break;
        case CV_8S:  res = cmp_result.at<schar> (0,0);break;
        case CV_16U: res = cmp_result.at<ushort>(0,0);break;
        case CV_16S: res = cmp_result.at<short> (0,0);break;
        case CV_32S: res = cmp_result.at<int>   (0,0);break;
        case CV_32F: res = cmp_result.at<float> (0,0);break;
        case CV_64F: res = cmp_result.at<double>(0,0);break;
        default:     return 0;
    }
#if DEBUG_VIDEO == 1
    if ( 1 - res / 255. > 0.95 ){
        imshow("mask",mask);
        imshow("x",x);
        imshow("x_gray",x_gray);
        imshow("y",y);
        imshow("y_gray",y_gray);
        waitKey();
    }
#endif
    return 1 - res / 255.;

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

void shift_video_get_difference(VideoCapture src, int shift, Diff_dict &dst){
    Mat frame[3];
    src >> frame[0];
    size_t current_pos = src.get(CAP_PROP_POS_FRAMES);
    if ( current_pos + shift < src.get(CAP_PROP_FRAME_COUNT) /* Might be a param */ ){
        src.set(CAP_PROP_POS_FRAMES,current_pos + shift);
        src >> frame[1];
        src.set(CAP_PROP_POS_FRAMES,current_pos + shift - 1);
        absdiff(frame[1],frame[0],frame[2]);
    }
    dst = {frame[0],frame[1],frame[2]};
    //dst = frame[0];
}

size_t get_duradion(VideoCapture src){
    return  10e3 * src.get(CAP_PROP_FRAME_COUNT)  / src.get(CAP_PROP_FPS)  ;
}

VideoWriter get_VideoWriter(VideoCapture cap){
    int frame_width=   cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CAP_PROP_FRAME_HEIGHT);
    return VideoWriter("/home/dupeljan/Projects/webinar_analisator/dipa.avi", VideoWriter::fourcc('M','J','P','G'),24, Size(frame_width,frame_height),true);
}

