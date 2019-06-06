#include "video_handler.h"




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

//    Cursor cursor;
//    cursor.find_cursor(cap,5,1000);
//    //cursor.find_cursor(cap,5,30);
//    imshow("cursor",cursor.get());
//    //imwrite(CURSOR,cursor.get());
//    waitKey(0);

    //cap.set(CAP_PROP_POS_FRAMES,178000);

    Mat cursor = imread(CURSOR);
    Presentation presentation(cursor,Rect(460,230,810,600));
    presentation.generate(cap,2000);
    //presentation.write_slides(SLIDE_PATH);
    Diff_dict diff;
    shift_video_get_difference(cap,100,diff);
    show_rects(diff.first,{Rect(460,230,810,600)},"img");
    waitKey();
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
    // 3,3 better than 5,5
    morphologyEx(diff.diff,diff.diff,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(3,3)));
}

void Cursor::filter_rects(){
    Mat x;
    cvtColor(diff.second,x,COLOR_BGR2GRAY);
    threshold(x,x,127,255,THRESH_BINARY_INV);
    //cvtColor(diff.diff,x,COLOR_BGR2GRAY);
    auto it = remove_if(b_rects.begin(), b_rects.end(),[x](Rect i){
         return i.area() < MIN_CURSOR_AREA || !countNonZero(x(i))/*piece is white*/; } );
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
                            //imshow("piece",piece);
                            //imshow("chains[i].second",chains[i].second);
                            //waitKey(0);
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
#if DEBUG_VIDEO == 1
        for(int i = 0; i < chains.size(); i++)
            imshow(to_string(chains[i].first) + "_pic_" + to_string(i),chains[i].second);
#endif
    }
}

// end Cursor

// Presentation
Presentation::Presentation(Mat cursor, Rect area){
    this->cursor = cursor;
    this->area = area;
    thresh_otsu(cursor,cursor_mask);

    //cursor_covering_mask = Mat::zeros(cursor_mask.rows,cursor_mask.cols,cursor_mask.type());
    //cursor_covering_mask = Scalar(255,255,255);

    //dilate(cursor_mask,cursor_covering_mask,getStructuringElement(MORPH_RECT,Size(5,5)));

    cursor_covering_mask = cursor_mask.clone();

    cvtColor(cursor_mask,cursor_mask,COLOR_GRAY2BGR);
    cvtColor(cursor_covering_mask,cursor_covering_mask,COLOR_GRAY2BGR);
}

void Presentation::get_cursor_mask(Mat src, Mat &dst){
    // Find out cursor's coords
    Point matchLoc;
    matchTemplateCoords(src,cursor,cursor_mask,matchLoc);

    // Create black mask
    dst = Mat::zeros(src.rows,src.cols,src.type());
    dst = Scalar(0,0,0);

    const auto accuracy = 0.9;
    //auto tmp = cmp( cursor,src(Rect(matchLoc.x,matchLoc.y, cursor_mask.cols, cursor_mask.rows))) ;
    if ( cmp( cursor,src(Rect(matchLoc.x,matchLoc.y, cursor_mask.cols, cursor_mask.rows)) )
                                                                               > accuracy ){
        // Insert cursor mask into cursor coords
        // might improve here
        Mat insetImage(dst, Rect(matchLoc.x,matchLoc.y, cursor_mask.cols, cursor_mask.rows ));
        cursor_covering_mask.copyTo(insetImage);
    }
    cvtColor(dst,dst,COLOR_BGR2GRAY);
    threshold(dst,dst,127,255,THRESH_BINARY);
}

void Presentation::shift_video_get_difference_local(VideoCapture cap, int shift, Diff_dict &dst){
    shift_video_get_difference(cap,shift,dst);
    if (!dst.diff.empty()){
        dst(area);
        cvtColor(dst.diff,dst.diff,COLOR_BGR2GRAY);
        morphologyEx(dst.diff,dst.diff,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(3,3)));
        threshold(dst.diff,dst.diff,127,255,THRESH_BINARY);
    }
}

void Presentation::generate(VideoCapture cap,int shift){
    Diff_dict diff;
    Mat cursor_mask_gray;
    cvtColor(cursor_mask,cursor_mask_gray,COLOR_BGR2GRAY);
    auto cursor_area = countNonZero(cursor_mask_gray);
    do{
       shift_video_get_difference_local(cap,shift,diff);

       Mat slide = diff.first.clone();
       Mat mask;
       get_cursor_mask(diff.first,mask);

#if DEBUG_VIDEO == 2
       imshow("init slide",slide);
       waitKey();
       imshow("mask",mask);
       waitKey();
       imshow("diff",diff.diff);
       waitKey();
#endif

       while( countNonZero(mask) &&
              countNonZero(diff.diff) < 3 * cursor_area &&
              !diff.diff.empty()){

            Mat local_mask;
            get_cursor_mask(diff.second,local_mask);

#if DEBUG_VIDEO == 2
            imshow("local mask",local_mask);
            waitKey();
#endif


            // Compute fill_mask = mask 'minus' ( mask 'intersect' local_mask)
            Mat alpha;
            Mat not_lm;
            bitwise_not(local_mask,not_lm);
            bitwise_and(mask,not_lm,alpha);

#if DEBUG_VIDEO == 2
            imshow("fill mask",alpha);
            waitKey();
#endif

            // Compute new mask
            bitwise_and(mask,local_mask,mask);

#if DEBUG_VIDEO == 2
            imshow("new mask",mask);
            waitKey();
#endif

            // Blend pieces
//            slide.convertTo(slide,CV_32FC3);
//            diff.second.convertTo(diff.second,CV_32FC3);
//            alpha.convertTo(alpha, CV_32FC3, 1.0/255);
//            Mat result = Mat::zeros(slide.size(), slide.type());
            // Multiply the foreground with the alpha matte
            //multiply(alpha, diff.second, diff.second);

            // Multiply the background with ( 1 - alpha )
            //multiply(Scalar::all(1.0)-alpha, slide, slide);

            // Add the masked foreground and background.
            //add(slide, diff.second, result);
//            imshow("diff.second",diff.second);
//            imshow("slide",slide);
//            imshow("alpha",alpha);
//            waitKey();
//            alpha_blend(diff.second,slide,alpha,result);
//            slide = result / 255;
            blend_with_mask(slide,diff.second,alpha,slide);

#if DEBUG_VIDEO == 2
            imshow("new slide",slide);
            waitKey();
#endif

            shift_video_get_difference_local(cap,shift,diff);
       }

#if DEBUG_VIDEO == 2
            imshow("diff",diff.diff);
            waitKey();
#endif
       // Wait for new slide
       while( countNonZero(diff.diff) < 3 * cursor_area &&
            !diff.diff.empty()){
               shift_video_get_difference_local(cap,shift,diff);
       }

#if DEBUG_VIDEO == 2
            imshow("diff",diff.diff);
            waitKey();        
            imshow("final slide",slide);
            waitKey();
#endif
       slides.push_back(slide);
    }while(!diff.diff.empty());
}

void Presentation::write_slides(string patch){
    for(size_t i = 0; i < slides.size(); i++)
        imwrite(patch + "slide_" + to_string(i) + ".png",slides[i]);
}
// end Presentation


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

