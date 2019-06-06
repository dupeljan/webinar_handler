#include "slide_proc.h"

Slide_proc::Slide_proc(){

}

void Slide_proc::proccess(Mat image){

    add_white_border(image,image,50);

#if SLIDE_PROC_DEBUG == 1
    imshow("got img",image);
    imwrite("/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/got_img.png",image);
    waitKey();
#endif

    Mat gaus_im;
    Mat src_gray;
    GaussianBlur(image, gaus_im, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cvtColor(gaus_im, src_gray, COLOR_BGR2GRAY);
    threshold(src_gray,src_gray,180,255,THRESH_BINARY );

    // My filter
    float left_filter[9] = {1,  0,   -1,
                           2,   0,   -2,
                           1,   0,   -1};
    float right_filter[9] = {-1,  0,   1,
                             -2,   0,   2,
                             -1,   0,   1};
    // Prepairing
    erode(src_gray,src_gray,getStructuringElement(MORPH_RECT,Size(1,BOTTOM_STICK_LENGTH)));
    //
    Mat left_border, right_border;
    filter2D(src_gray,left_border,-1, Mat(3,3,CV_32F,left_filter) );
    filter2D(src_gray,right_border,-1, Mat(3,3,CV_32F,right_filter));
    addWeighted(left_border,1,right_border,1,0,left_border);
    // end filter

#if SLIDE_PROC_DEBUG == 1
    imshow("after filter",left_border);
    imwrite("/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/left_border.png",left_border);
    waitKey();
#endif

    // Start morphology
    Mat mask;
    // оставляем палки длинны от bottom до upper
    morphologyEx(left_border,left_border,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(1,BOTTOM_STICK_LENGTH)));// нижняя граница
    morphologyEx(left_border,mask,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(1,UPPER_SICK_LENGTH)));// верхняя граница
    my_inv(left_border);//инвертируем
    Mat result;
    addWeighted(left_border,1,mask,1,0,result);//Соединияем
    morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(1,BOTTOM_STICK_LENGTH)));// нижняя граница

#if SLIDE_PROC_DEBUG == 1
    imshow("intermediate result",result);
    imwrite("/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/intermediate_result.png",result);
#endif

    morphologyEx(result,result,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(UPPER_SICK_LENGTH * 25/20,2)),Point(-1,-1),3);//добавим
    morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(5,BOTTOM_STICK_LENGTH)));

#if SLIDE_PROC_DEBUG == 1
    imshow("after morphology",result);
    imwrite("/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/after_morphology.png",result);
    waitKey();
#endif

    // gen bounding rect
    RNG rng(12345);
    vector<vector<Point> > contours;
    Mat countoured_image = image.clone();
    findContours( result, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<Rect> boundRect( contours.size() );
    vector<Mat> pieces ( contours.size());
    for( size_t i = 0; i < contours.size(); i++ ){
        boundRect[i] = boundingRect( Mat(contours[i]) );
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( image, contours, (int)i, color);
        pieces[i] = image(boundRect[i]);
        rectangle(countoured_image,boundRect[i],color);
        //if (! same_shape(image, boundRect[i] ) )
          //  pieces.push_back(image(boundRect[i]));

    }

#if SLIDE_PROC_DEBUG == 1
    imshow("founded contours",countoured_image);
    imwrite("/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/founded_contours.png",countoured_image);
    waitKey();
#endif

    // filter pieces
    vector<Piece> filtered_pieces;
    to_Piece(pieces,boundRect,filtered_pieces);
    filter_pieces(image,filtered_pieces,filtered_pieces);

#if SLIDE_PROC_DEBUG == 1
    {
    vector<Rect> _;
    for(auto &x : filtered_pieces)
        _.push_back(x.coord);
    show_rects(image,_,"filtered contours");
    waitKey();
    RNG rng(12345);
    Mat tmp = image.clone();
    for( auto &x : _){
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( image, contours, (int)i, color);
        rectangle(tmp,x,color);
    }
    imwrite("/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/filtered_contours.png",tmp);
    }
#endif

    //drop_non_text(filtered_pieces,filtered_pieces);

#if SLIDE_PROC_DEBUG == 2
    {
    vector<Rect> _;
    for(auto &x : filtered_pieces)
        _.push_back(x.coord);
    show_rects(image,_,"contours after dnt");
    waitKey();
    }
#endif

    // Reconize text
    // Create Tesseract object
    auto *ocr = new tesseract::TessBaseAPI();

    // Initialize tesseract to use English (eng) and the LSTM OCR engine.
    ocr->Init(NULL, "rus", tesseract::OEM_LSTM_ONLY);

    // Set Page segmentation mode to PSM_AUTO (3)
    ocr->SetPageSegMode(tesseract::PSM_AUTO);

    for( auto &piece : filtered_pieces ){
        ocr->SetImage(piece.pic.data,piece.pic.cols,piece.pic.rows,3,piece.pic.step);
        text_blocks.push_back({piece.coord,string(ocr->GetUTF8Text())});
#if SLIDE_PROC_DEBUG == 1
        imshow(text_blocks.back().text,piece.pic);
        cout <<endl<< text_blocks.back().text << endl;
#endif
    }


#if SLIDE_PROC_DEBUG == 1
    waitKey();
#endif


}
