#include <QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <algorithm>
#include <set>
#include <string>
#include <math.h>
#include <numeric>


#define PIC "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/slides/test2.png"

#define BOTTOM_STICK_LENGTH 7//11
#define UPPER_SICK_LENGTH 25//25

using namespace cv;
using namespace std;

struct Piece{
    Mat pic;
    Rect coord;
};

void vertical_hist(Mat src, Mat& dst,int cols = 250);
void vertical_hist(Mat src,vector<int>& hist);
void horizontal_hist(Mat src, Mat& dst,int rows = 180);
void horizontal_hist(Mat src,vector<int>& hist);
void cut_text_line(Mat in,vector<Mat>& out,int threshold= 1);
void cut_words(Mat in,vector<vector<Mat>>& out,int threshold= 10);
void drop_non_text(Piece src, Piece &dst);

void my_inv(Mat in);
void my_grad(Mat src, Mat &dst);
void find_bound_rects(Mat src, vector<Rect> &dst);

void add_white_border(Mat src, Mat &dst, int border_size = 1);
void vec_imshow(string name, vector<Mat> src);
void thresh_otsu(Mat src,Mat &dst);
bool same_shape(Mat a, Rect b);
bool piece_is_word(Mat dst, int threshold = 1);
int expected_value(vector<int> row);
int median(vector<int> row);


int main(int argc, char *argv[]){

    string imageName(PIC);
    Mat image,src_gray,grad;
    RNG rng(12345);

    image = imread(imageName.c_str(), IMREAD_COLOR); // Read the file

    add_white_border(image,image,50);
    if( image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    Mat gaus_im;

    GaussianBlur(image, gaus_im, Size(3, 3), 0, 0, BORDER_DEFAULT);

    cvtColor(gaus_im, src_gray, COLOR_BGR2GRAY);

    threshold(src_gray,src_gray,180,255,THRESH_BINARY );

   // addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    Mat morphology_out;
    //morphologyEx(grad,morphology_out,MORPH_CLOSE, getStructuringElement( MORPH_RECT,Size(3,3) ) );

    //Mat inv;
    //threshold(morphology_out,inv,127,255,THRESH_BINARY_INV);
    // end Canny

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
    Mat my_filtered;
    addWeighted(left_border,1,right_border,1,0,left_border);
    // end filter
    //my_inv(my_filtered);
    // Start morphology
    Mat mask;
    //erode(left_border,result,getStructuringElement(MORPH_RECT,Size(1,20)));
    // оставляем палки длинны от bottom до upper
    morphologyEx(left_border,left_border,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(1,BOTTOM_STICK_LENGTH)));// нижняя граница
    morphologyEx(left_border,mask,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(1,UPPER_SICK_LENGTH)));// верхняя граница
    my_inv(left_border);//инвертируем
    Mat result;
    addWeighted(left_border,1,mask,1,0,result);//Соединияем
    morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(1,BOTTOM_STICK_LENGTH)));// нижняя граница
    imshow("intermediate result",result);

    //my_inv(left_border);
    //my_inv(result);
    // Must to improve:
    //
    //
    /*
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5) );
    erode(result,result,getStructuringElement(MORPH_RECT,Size(15,5)));
    morphologyEx(result, result, MORPH_OPEN, element,Point(-1,-1),3);
    */
    //erode(result,result,getStructuringElement(MORPH_RECT,Size(15,5)));//добавим
    //imshow("after erode",result);
    morphologyEx(result,result,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(UPPER_SICK_LENGTH * 25/20,2)),Point(-1,-1),3);//добавим
    morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(5,BOTTOM_STICK_LENGTH)));// нижняя граница
    //Mat dist;
    //add_white_border(result,result);
    //add_white_border(image,image);
    //imshow("ins img",dist);
    //imshow("after opening",result);
   // morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(10,UPPER_SICK_LENGTH * 20/25)));// Уберем
    //
    //
    // end
    /*
    int n = 10;
    Mat vect[n];
    namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
    for ( int i = 1; i < n; i++){
         erode(my_filtered,vect[i],getStructuringElement(MORPH_RECT,Size(i*10,5)));
         imshow(to_string(i*10),vect[i]);
    }
    */
    //cvtColor(result,result,COLOR_GRAY2BGR);
    //my_inv(result);
    //Mat conduct = image & result;

    // gen bounding rect
    vector<vector<Point> > contours;
    findContours( result, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<Rect> boundRect( contours.size() );
    vector<Mat> pieces ;//( contours.size());
    for( size_t i = 0; i < contours.size(); i++ ){
        boundRect[i] = boundingRect( Mat(contours[i]) );
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( image, contours, (int)i, color);
        //rectangle(image,boundRect[i],color);
        if (! same_shape(image, boundRect[i] ) )
            pieces.push_back(image(boundRect[i]));

    }

    imshow("result",result);
    imshow("gray",src_gray);
    imshow("image",image);
    //vec_imshow("p",pieces);
    Piece just_piece = {image,Rect(0,0,image.cols,image.rows)};
    drop_non_text(just_piece,just_piece);
    imshow("new piece",just_piece.pic);
    // drop not text pieces

    vector<vector<Mat>> words;
    //GaussianBlur(pieces[3], pieces[3], Size(3, 3), 0, 0, BORDER_DEFAULT);
    //thresh_otsu(pieces[3],pieces[3]);
    //cvtColor(pieces[3],pieces[3],COLOR_BGR2GRAY);
    //adaptiveThreshold(pieces[3],pieces[3],255,ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,3,0);
    //my_inv(pieces[3]);
    //cut_words(pieces[3],words);
    //vec_imshow("words",words[0]);
/*
    vec_imshow("pieces",pieces);

    threshold(pieces[2],src_gray,180,255,THRESH_BINARY );
    vector<Mat> text_lines;
    cut_text_line(src_gray,text_lines);
    imshow("gray",src_gray);
    for ( int i = 0; i < text_lines.size() && i < 5; i++)
        imshow("pice" + to_string(i) ,text_lines[i]);

    vector<vector<Mat>> words;
    cut_words(text_lines[0],words);
    for ( int i = 0; i < words[0].size(); i++)
        imshow("char" + to_string(i) ,words[0][i]);
*/
    /*
    for ( int i = 0; i < pieces.size(); i++)
        imshow("pice" + to_string(i) ,pieces[i]);
    */
    //imshow("conduct",conduct);
    waitKey(0); // Wait for a keystroke in the window
    return 0;
}



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
            //out.push_back( in( Rect(top,0,i,in.cols) ) );
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
    imshow("h_hist_pic",h_hist_pic);

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
    int chop = median(chops);
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

    /*
    // Compute words_pic
    in.colRange(0,spaces_pos[0].first).copyTo(words_pic[0]);
    for(int i = 1; i < spaces_pos.size(); i++)
        in.colRange(spaces_pos[i-1].second,spaces_pos[i].first).copyTo(words_pic[i]);
    in.colRange(spaces_pos[spaces_pos.size()-1].second,in.cols).copyTo(words_pic[words_pic.size() - 1]);

    for(int i = 0; i < words_pic.size(); i++)
        imshow("word " + to_string(i),words_pic[i]);
    */
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
    imshow("piiic",thres);
    morphologyEx(thres,thres,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(8,5)));
    vector<Rect> b_rect;
    find_bound_rects(thres,b_rect);
    dst = src;
    //set<Rect> non_text;
    vector<Rect> pieces;
    for(int i = 0; i < b_rect.size(); i++ ){
        Mat piece = src.pic(b_rect[i]);
        imshow("h" + to_string(i), piece);
        if ( ! piece_is_word(piece) )
            // fill him white
            rectangle(dst.pic,b_rect[i],Scalar(255,255,255),FILLED);
        else
            pieces.push_back(b_rect[i]);

    }
    if ( pieces.size() ){
        // bound text
        Rect bound;
        // Error here
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
    //copy_if(b_rect.begin(),b_rect.end(),back_inserter(non_text),[](Rect x){return piece_is_word(src(x));});
    //for (int i = 0; i < pieces.size(); i++)
        //imshow( ( piece_is_word(pieces[i]))? "word" + to_string(i) : "trash" + to_string(i), pieces[i]  );
      //  if ( ! piece_is_word(pieces[i]) )
        //    non_text.insert(b_rect[i]);
    //imshow("thres",thres);
}
void my_grad(Mat src,Mat& dst){

    Mat src_gray;

    thresh_otsu(src,src_gray);
    /*
    //cvtColor(src, src_gray, COLOR_BGR2GRAY);
    //threshold(src_gray,src_gray, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    //threshold(src_gray,src_gray,127,255,THRESH_BINARY_INV);

    float left_filter[9] = {1,  0,   -1,
                           2,   0,   -2,
                           1,   0,   -1};
    float right_filter[9] = {-1,  0,   1,
                             -2,   0,   2,
                             -1,   0,   1};

    Mat left_border, right_border;
    filter2D(src_gray,left_border,-1, Mat(3,3,CV_32F,left_filter) );
    filter2D(src_gray,right_border,-1, Mat(3,3,CV_32F,right_filter));
    Mat my_filtered;
    addWeighted(left_border,1,right_border,1,0,dst);
    */
    vector<vector<Point>> contours;
    Canny( src_gray, dst, 50, 150);
    imshow("canny",dst);
    findContours( dst, contours,RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for(int i = 0; i < contours.size();i++)
        drawContours(dst, contours,i, Scalar(255, 255, 255), FILLED);
    my_inv(dst);

}

void find_bound_rects(Mat src,vector<Rect> &dst){

    vector<vector<Point> > contours;
    findContours( src, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    dst.resize( contours.size() );
    //vector<Mat> pieces ;//( contours.size());
    for( size_t i = 0; i < contours.size(); i++ ){
        dst[i] = boundingRect( Mat(contours[i]) );
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( dst, contours, (int)i, color);
        //rectangle(dst,boundRect[i],Scalar(0,0,0));
        //if (! same_shape(image, boundRect[i] ) )
        //    pieces.push_back(image(boundRect[i]));

    }
    // filter
    auto it = remove_if(dst.begin(), dst.end(),[src](Rect i){return same_shape(src,i);} );
    dst.erase(it, dst.end());
}

bool piece_is_word(Mat dst,int threshold /*= 1*/){
    //Mat h_hist_pic;
    vector<int> h_hist;
    vector<int> chops;
    //horizontal_hist(dst,h_hist_pic);
    //imshow("h_hist_pic",h_hist_pic);

    thresh_otsu(dst,dst);
    my_inv(dst);
    imshow("g",dst);
    /*
    // Try to cut line
    vector<Mat> line;
    cut_text_line(dst,line);
    // if not one line
    if ( line.size() > 1 )
        return false;
    dst = line[0];
    */
    horizontal_hist(dst,h_hist);
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

    //sort(chops.begin(),chops.end());
    int chop = expected_value(chops);
    //if ( chops.size() % 2 && chops.size() > 1 )
      //  chop = (chops[chops.size() / 2] + chops[(chops.size() / 2) + 1]) / 2;
    //else
      //  chop = chops[chops.size() / 2];
    double l_proport = dst.rows / (double)chop;
    //double r_proport = dst.rows / (double)(*min_element(chops.begin(),chops.end()));
    return ( 1  <= l_proport && l_proport <= 3 /*&& 1 / 3. <= r_proport && r_proport <= 5*/ )? true : false;
}

void thresh_otsu(Mat src,Mat &dst){
    cvtColor(src, dst, COLOR_BGR2GRAY);
    threshold(dst,dst, 127, 255, THRESH_BINARY_INV | THRESH_OTSU);
}
void my_inv(Mat in){
    threshold(in,in,127,255,THRESH_BINARY_INV);
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

bool same_shape(Mat a, Rect b){
    bool x = ( a.cols == b.width ) && ( a.rows == b.height );
    return ( a.cols == b.width ) && ( a.rows == b.height );
}

int expected_value(vector<int> row){
    int sum = 0;
    for (auto& i : row)
        sum += i;
    return  (row.size())? sum / row.size() : 0;
}

int median(vector<int> row){
    if ( ! row.size())
        return NULL;
    sort(row.begin(),row.end());
    return ( row.size() % 2 && row.size() > 1 )? \
                (row[row.size() / 2] + row[(row.size() / 2) + 1]) / 2 :\
                 row[row.size() / 2];

}
