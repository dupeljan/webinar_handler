#include "img_handler.h"

void vertical_hist(Mat src, Mat& dst,int cols /*= 250*/){
    dst = Mat::zeros(src.rows,cols,src.type());
    //rectangle(dst,Point(0,0),Point(src.rows,cols),Scalar(255,255,255));
    //int rows = int(50/*columns.size()*/);
    for (int i = 0; i < src.rows; i++){
        double hist = sum(src.row(i))[0] / (double) src.cols;
        line(dst,Point(0,i),Point( hist * cols / (double) 255,i ),Scalar(255,255,255));
    }
    //for( int i = 1; i <= cols; i++)
    //src.rowRange(0,50).copyTo(dst);

}

void vertical_hist(Mat src,vector<int>& hist){
    hist.resize(src.rows);
    for (int i = 0; i < src.rows; i++)
        hist[i] = (int)trunc( sum(src.row(i))[0] / (double) src.cols );
}

void horizontal_hist(Mat src, Mat& dst,int rows /*= 180*/){
    dst = Mat::zeros(rows,src.cols,src.type());

    for (int i = 0; i < src.cols; i++){
        double hist = sum(src.col(i))[0] / (double) src.rows;
        line(dst,Point(i,0),Point( i, hist * rows / (double) 255 ),Scalar(255,255,255));
    }

}

void horizontal_hist(Mat src,vector<int>& hist){
    hist.resize(src.cols);
    for (int i = 0; i < src.cols; i++)
        hist[i] = (int)trunc( sum(src.col(i))[0] / (double) src.rows );
}

void cut_text_line(Mat in,vector<Mat>& out,int threshold /*= 1*/){
    vector<int> v_hist;
    vertical_hist(in,v_hist);
    bool is_line = false;
    int top;
    for( int i = 0; i < v_hist.size(); i++ ){

        if( ! is_line  && 255 - v_hist[i] > threshold ){
            top = i;
            is_line = true;
        }
        else if( is_line && 255 - v_hist[i] < threshold ){
            out.resize(out.size() + 1);
            in.rowRange(top,i).copyTo(out[out.size()-1]);
            is_line = false;

            // computing horizontal hist for piece and cut borders
            vector<int> h_hist;
            horizontal_hist(out[out.size()-1],h_hist);
            int left(-1), right;
            for( int j = 0; left < 0 && j < h_hist.size(); left = ( 255 - h_hist[j] > threshold )? j : -1, j++ );
            if ( left > -1 ){
                right = left;
                for( int j = left; j < h_hist.size(); right = ( 255 - h_hist[j] > threshold )? j : right, j++ );
                out[out.size()-1].colRange(left,right).copyTo(out[out.size()-1]); // Cut borders
            }
        }

    }
    if (!out.size()){
        out.resize(1);
        in.copyTo(out[0]);
    }

    // Kill empty lines
    auto it = remove_if(out.begin(), out.end(),[](Mat i){ return i.empty();} );
    out.erase(it, out.end());
}

void cut_words(Mat in,vector<vector<Mat>>& out,int threshold /*= 10*/){

    auto in_set = [](pair<int,int> elem,pair<int,int> set){
        return set.first <= elem.first && elem.second <= set.second;
    };
    const double delta = 1.5;
    struct character{
        Mat s;
        pair<int,int> pos; // character position
    };
    vector<character> chr_list;
    vector<int> h_hist;
    vector<int> chops;


    Mat h_hist_pic;
    horizontal_hist(in,h_hist_pic);

    horizontal_hist(in,h_hist);
    bool is_char = false;
    int left;
    for( int i = 0; i < h_hist.size(); i++){
        if( ! is_char  && 255 - h_hist[i] > threshold ){
            left = i;
            is_char = true;
        }
        else if( is_char && ( 255 - h_hist[i] < threshold || i == h_hist.size() - 1)){
            chr_list.resize(chr_list.size() + 1);
            chops.push_back(i - left);
            chr_list[chr_list.size()-1].pos = make_pair(left,i);
            in.colRange(left,i).copyTo(chr_list[chr_list.size()-1].s);
            is_char = false;
        }
    }
    // Compute median chop
    //sort(chops.begin(),chops.end());
    auto chop = median<int>(chops);
    /*
    if ( chops.size() % 2 )
        chop = (chops[chops.size() / 2] + chops[(chops.size() / 2) + 1]) / 2;
    else
        chop = chops[chops.size() / 2];
    */
    // Cut characters
    for(int i = 0; i < chr_list.size(); i++)
        if(chr_list[i].s.cols > chop*delta){
           Mat piece[2];
           pair<int,int> piece_pos = chr_list[i].pos;
           chr_list[i].s.colRange(0,chr_list[i].s.cols / 2).copyTo(piece[0]);
           chr_list[i].s.colRange(chr_list[i].s.cols / 2, chr_list[i].s.cols).copyTo(piece[1]);
           // update out & pos
           chr_list[i] = { piece[0],
           make_pair(piece_pos.first, (piece_pos.first + piece_pos.second) / 2) };
           chr_list.insert(chr_list.begin()+i,{ piece[1],
           make_pair( (piece_pos.first + piece_pos.second) / 2, + piece_pos.second) });
           i++;
        }
    // Sort chr_list
    sort(chr_list.begin(),chr_list.end(),[](character lhs, character rhs) { return lhs.pos.first < rhs.pos.first; });
    // compute spaces
    bool white_line = false;
    vector<pair<int,int>> spaces_pos;
    //int left;
    for(int i = 0; i < h_hist.size(); i++)
        if(! white_line && 255 - h_hist[i] < threshold){
            left = i;
            white_line = true;
        }else if( white_line &&  255 - h_hist[i] > threshold){
            if(i - left >= chop)
               spaces_pos.push_back(make_pair(left,i));

            white_line = false;
        }
    // Cut words
    vector<vector<Mat>> words       (spaces_pos.size()+1);
    vector<Mat> words_pic           (spaces_pos.size()+1);
    vector<pair<int,int>> words_pos (spaces_pos.size()+1);

    // Compute words_pos
    if( spaces_pos.size()){
        words_pos[0] = make_pair(0,spaces_pos[0].first);
        for(int i = 1; i < spaces_pos.size(); i++)
            words_pos[i] = make_pair(spaces_pos[i-1].second,spaces_pos[i].first);
        words_pos[words_pos.size()-1] = make_pair(spaces_pos[spaces_pos.size()-1].second,in.cols);
    } else
        words_pos[0] = make_pair(0,h_hist.size());

    // Compute words

    for(int i = 0, j = 0; i < chr_list.size(); i++){
        if( chr_list[i].pos.second > words_pos[j].second )
            j++;
        words[j].push_back(chr_list[i].s);
     }

    out = words;
    //out.resize(chr_list.size());
    //for(int i = 0; i < out.size(); i++)
        //out[i] = chr_list[i].s;

}

void drop_non_text(Piece src,Piece &dst){
    Mat thres;
    my_grad(src.pic,thres);
    morphologyEx(thres,thres,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(8,5)));
#if IMG_HANDLER_DEBUG == 1
    imshow("dnt filter",thres);
    waitKey();
#endif
    vector<Rect> b_rect;

    find_bound_rects(thres,b_rect);

#if IMG_HANDLER_DEBUG == 1
    show_rects(src.pic,b_rect,"dnt rects");
    waitKey();
#endif

    dst = src;
    vector<Rect> pieces;
    for(int i = 0; i < b_rect.size(); i++ ){
        Mat piece = src.pic(b_rect[i]);
        if (  piece_is_word(piece) )
            // fill him white
            //rectangle(dst.pic,b_rect[i],Scalar(255,255,255),FILLED);
        //else
            pieces.push_back(b_rect[i]);

    }
    if ( pieces.size() ){
        // bound text
        Rect bound;
        bound.x = min_element(pieces.begin(),pieces.end(),[](Rect l, Rect r){ return l.x  < r.x; }) -> x;
        bound.y = min_element(pieces.begin(),pieces.end(),[](Rect l, Rect r){ return l.y  < r.y; }) -> y;
        Rect tmp;
        tmp = *max_element(pieces.begin(),pieces.end(),[](Rect l, Rect r){ return l.x + l.width <  r.x + r.width; });
        bound.width = tmp.width + tmp.x - bound.x;
        tmp = *max_element(pieces.begin(),pieces.end(),[](Rect l, Rect r){ return l.y + l.height < r.y + r.height;}) ;
        bound.height = tmp.height + tmp.y - bound.y;
        dst.pic = dst.pic(bound);
        // update countur
        dst.coord.x += bound.x;
        dst.coord.y += bound.y;
        dst.coord.width =  bound.width;
        dst.coord.height = bound.height;
    }
}

void drop_non_text(vector<Piece> src,vector<Piece> &dst){
    dst.clear();
    for( auto &x : src ){
        drop_non_text(x,x);
        if ( ! is_white(x.pic) )
            dst.push_back(x);
    }
}

void my_grad(Mat src,Mat& dst){

    Mat src_gray;

    thresh_otsu(src,src_gray);

    vector<vector<Point>> contours;
    Canny( src_gray, dst, 50, 150);
    dilate(dst,dst,getStructuringElement(MORPH_RECT,Size(3,3)));
    findContours( dst, contours,RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for(int i = 0; i < contours.size();i++)
        drawContours(dst, contours,i, Scalar(255, 255, 255), FILLED);
    my_inv(dst);

}

void find_bound_rects(Mat src,vector<Rect> &dst){

    my_find_contours( src, dst);

    // filter
    // TODO: remove nested contours
    auto it = remove_if(dst.begin(), dst.end(),[src](Rect i){ return same_shape(src,i);} );
    dst.erase(it, dst.end());
}

void find_bound_rects_rgb(Mat src,vector<Rect> &b_rect){
    cvtColor(src,src,COLOR_BGR2GRAY);
    find_bound_rects(src,b_rect);
}

bool piece_is_word(Mat dst,int threshold /*= 1*/){
    //Mat h_hist_pic;
    vector<int> h_hist;
    vector<int> chops;

    thresh_otsu(dst,dst);
    my_inv(dst);

    // Try to cut lines
    vector<Mat> lines;
    vector<double> l_proport;
    cut_text_line(dst,lines);

#if IMG_HANDLER_DEBUG == 1
    {
    Mat _;
    vertical_hist(dst,_);
    imshow("piece is word vert hist",_);
    }
#endif

    for ( auto &line : lines){
        // Try to cut characters
        horizontal_hist(line,h_hist);

#if IMG_HANDLER_DEBUG == 1
        {
        imshow("piece_is_word thres",line);
        waitKey();
        Mat _;
        horizontal_hist(line,_);
        imshow("piece is word horiz hist",_);
        waitKey();
        }
#endif

        bool is_char = false;
        int left;
        for( int i = 0; i < h_hist.size(); i++){
            if( ! is_char  && 255 - h_hist[i] > threshold ){
                left = i;
                is_char = true;
            }
            else if( is_char && ( 255 - h_hist[i] < threshold || i == h_hist.size() - 1)){
                chops.push_back(i - left);
                is_char = false;
            }
        }
        if ( !chops.size() )
            return false;


        auto chop = expected_value(chops);
        l_proport.push_back(line.rows / (double)chop);


    }
    auto m = median(l_proport);


#if IMG_HANDLER_DEBUG == 1
     if (lines.size()){
        imshow("result " + to_string(m),lines.back());
        waitKey();
     }
#endif
    return 1 <= m && m <= 3.5 ;
}

void thresh_otsu(Mat src,Mat &dst){
    cvtColor(src, dst, COLOR_BGR2GRAY);
    threshold(dst,dst, 127, 255, THRESH_BINARY_INV | THRESH_OTSU);
}

void my_find_contours(Mat src,vector<Rect> &b_rect,int border /*=10*/){

    vector<vector<Point>> contours;

    if (! border ){
        findContours( src, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        for( size_t i = 0; i < contours.size(); i++ )
            b_rect[i] = boundingRect( Mat(contours[i]) );
    }
    else{
        // Extent src pic
        Rect src_shape = Rect(0,0,src.cols,src.rows);
        add_white_border(src,src,border);
        findContours( src, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );

        b_rect.resize(contours.size());
        for( size_t i = 0; i < contours.size(); i++ ){
            b_rect[i] = boundingRect( Mat(contours[i]) );
            // Shift contours
            b_rect[i].x = max(0, b_rect[i].x - border);
            b_rect[i].y = max(0, b_rect[i].y - border);
            b_rect[i].width = min(  src_shape.width  - b_rect[i].x, b_rect[i].width );
            b_rect[i].height = min( src_shape.height - b_rect[i].y, b_rect[i].height);
        }
    }
}

void my_inv(Mat in){
    threshold(in,in,127,255,THRESH_BINARY_INV);
}

void filter_pieces(Mat src_img, vector<Piece> src_vec, vector<Piece> &dst){
    Rect image_shape = Rect(0,0,src_img.cols,src_img.rows);
    auto it = remove_copy_if(src_vec.begin(),src_vec.end(),dst.begin(),[image_shape](Piece i){ return i.coord == image_shape;});
    dst.erase(it,dst.end());
}

void add_white_border(Mat src,Mat& dst,int border_size /*= 1*/){
    dst = Mat::zeros(src.rows + 2 * border_size,src.cols + 2 * border_size,src.type());
    dst = Scalar(255,255,255);
    Mat insetImage(dst, Rect(border_size,border_size, src.cols, src.rows ));
    src.copyTo(insetImage);
}

void vec_imshow(string name, vector<Mat> src){
    for(int i = 0; i < src.size();i++)
        imshow(name + ' ' + to_string(i),src[i]);
}

void vec_imshow(string name, vector<Piece> src){
    for(int i = 0; i < src.size();i++)
        imshow(name + ' ' + to_string(i),src[i].pic);
}

void show_rects(Mat src, vector<Rect> rects,string title){
    RNG rng(12345);
    Mat tmp = src.clone();
    for( auto &x : rects ){
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( image, contours, (int)i, color);
        rectangle(tmp,x,color);
    }
    imshow(title,tmp);
}

bool same_shape(Mat a, Rect b){
    return ( a.cols == b.width ) && ( a.rows == b.height );
}

template <typename T>
int expected_value(vector<T> row){
    T sum = 0;
    for (auto& i : row)
        sum += i;
    return  (row.size())? sum / T(row.size()) : 0;
}

template <typename T>
int median(vector<T> row){
    if ( ! row.size())
        return NULL;
    sort(row.begin(),row.end());
    return ( row.size() % 2 && row.size() > 1 )? \
                (row[row.size() / 2] + row[(row.size() / 2) + 1]) / T(2) :\
                 row[row.size() / 2];
}

bool is_white(Mat src){
   cvtColor(src, src, COLOR_BGR2GRAY);
   my_inv(src);
   return ( ! countNonZero(src) );
}

void to_Piece(vector<Mat> pic, vector<Rect> rect, vector<Piece> &dst){
    size_t size = min(pic.size(),rect.size());
    for(size_t i = 0; i < size; i++)
        dst.push_back({pic[i],rect[i]});
}

void blend_with_mask(Mat &base, Mat &src, Mat &mask, Mat &out){
    char ch = base.channels();
    double alpha = 0;
    for( int y = 0; y < base.rows; y++ ){
        uchar* pBS = base.ptr<uchar>(y);
        uchar* pSR = src.ptr<uchar>(y);
        uchar* pMK = mask.ptr<uchar>(y);
        uchar* pOU = out.ptr<uchar>(y);
        for( int x = 0; x < base.cols*ch; x++ ){
            int ix = x / ch;
            if(pMK[ix] == 0){
                pOU[x] = pBS[x];
            } else if(pMK[ix] == 255){
                pOU[x] = pSR[x];
            } else {
                alpha = pMK[ix] / 255.0;
                pOU[x] = pSR[x] * alpha + pBS[x] * (1-alpha);
            }
        }
    }
}


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
    Mat mask;
    thresh_otsu(x,mask);
    cvtColor(mask,mask,COLOR_GRAY2BGR);
#if IMG_HANDLER_DEBUG == 1
    //imshow("mask",mask);
#endif
    Point matchLoc;
    matchTemplateCoords(y,x,mask,matchLoc);

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
#if IMG_HANDLER_DEBUG == 1
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

void matchTemplateCoords(Mat img, Mat templ,Mat mask,Point& matchLoc){
    auto match_method = TM_CCORR_NORMED;
    Mat map;
    matchTemplate(img,templ,map,match_method,mask);
    normalize( map, map, 0, 1, NORM_MINMAX, -1, Mat() );

    double minVal; double maxVal; Point minLoc; Point maxLoc;
    minMaxLoc( map, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
    else
        { matchLoc = maxLoc; }
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

int countNonZero_rgb(Mat src){
    cvtColor(src,src,COLOR_BGR2GRAY);
    return countNonZero(src);
}

