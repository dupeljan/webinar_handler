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

#define PIC "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/block.png"

#define BOTTOM_STICK_LENGTH 5//11
#define UPPER_SICK_LENGTH 20//25

using namespace cv;
using namespace std;

void vertical_hist(Mat src, Mat& dst,int cols = 250){
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

void horizontal_hist(Mat src, Mat& dst,int rows = 180){
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

void cut_text_line(Mat in,vector<Mat>& out,int threshold= 1){
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


}

void cut_words(Mat in,vector<Mat>& out){

}
void cut_chars(Mat in,vector<Mat>& out,int threshold= 10){
    const double delta = 1.5;
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
            out.resize(out.size() + 1);
            chops.push_back(i - left);
            in.colRange(left,i).copyTo(out[out.size()-1]);
            is_char = false;
        }
    }

    // Compute median chop
    sort(chops.begin(),chops.end());
    int chop;
    if ( chops.size() % 2 )
        chop = (chops[chops.size() / 2] + chops[(chops.size() / 2) + 1]) / 2;
    else
        chop = chops[chops.size() / 2];
    // Cut characters
    for(int i = 0; i < out.size(); i++)
        if(out[i].cols > chop*delta){
           Mat piece[2];
           out[i].colRange(0,out[i].cols / 2).copyTo(piece[0]);
           out[i].colRange(out[i].cols / 2, out[i].cols).copyTo(piece[1]);
           out[i] = piece[0];
           out.insert(out.begin()+i,piece[1]);
           i++;
        }
    // TESTING
    // compute spaces
    bool white_line = false;
    vector<pair<int,int>> spaces;
    vector<Mat> words;
    //int left;
    for(int i = 0; i < h_hist.size(); i++)
        if(! white_line && 255 - h_hist[i] < threshold){
            left = i;
            white_line = true;
        }else if( white_line &&  255 - h_hist[i] > threshold){
            if(i - left >= chop)
               spaces.push_back(make_pair(left,i));

            white_line = false;
        }
    // Cut words
    words.resize(spaces.size()+1);
    in.colRange(0,spaces[0].first).copyTo(words[0]);
    for(int i = 1; i < spaces.size(); i++)
        in.colRange(spaces[i-1].second,spaces[i].first).copyTo(words[i]);
    in.colRange(spaces[spaces.size()-1].second,in.cols).copyTo(words[words.size() - 1]);

    for(int i = 0; i < words.size(); i++)
        imshow("word " + to_string(i),words[i]);




}

void my_inv(Mat in){
    threshold(in,in,127,255,THRESH_BINARY_INV);
}
int main(int argc, char *argv[])
{
    string imageName(PIC);
    Mat image,src_gray,grad;
    RNG rng(12345);

    image = imread(imageName.c_str(), IMREAD_COLOR); // Read the file

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

    Mat inv;
    threshold(morphology_out,inv,127,255,THRESH_BINARY_INV);
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

    //my_inv(left_border);
    //my_inv(result);
    // Must to improve:
    erode(result,result,getStructuringElement(MORPH_RECT,Size(30,5)));//добавим
    morphologyEx(result,result,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(40,5)),Point(-1,-1),2);//добавим
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
    vector<Mat> pieces( contours.size());
    for( size_t i = 0; i < contours.size(); i++ ){
        boundRect[i] = boundingRect( Mat(contours[i]) );
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( image, contours, (int)i, color);
        //rectangle(image,boundRect[i],color);
        pieces[i] = image(boundRect[i]);
    }

    imshow( "left_border", left_border);                // Show our image inside it.
    imshow("result",result);
    imshow("gray",src_gray);
    imshow("image",image);
    Mat hist;
    horizontal_hist(image,hist);
    imshow("hist",hist);
    vector<Mat> text_lines;
    cut_text_line(image,text_lines);
    for ( int i = 0; i < /*text_lines.size()*/2; i++)
        imshow("pice" + to_string(i) ,text_lines[i]);

    vector<Mat> chars;
    cut_chars(text_lines[1],chars);
    for ( int i = 0; i < chars.size(); i++)
        imshow("char" + to_string(i) ,chars[i]);
    /*
    for ( int i = 0; i < pieces.size(); i++)
        imshow("pice" + to_string(i) ,pieces[i]);
    */
    //imshow("conduct",conduct);
    waitKey(0); // Wait for a keystroke in the window
    return 0;
}

