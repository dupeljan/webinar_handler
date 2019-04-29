#ifndef IMG_HANDLER_H
#define IMG_HANDLER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
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

#define BOTTOM_STICK_LENGTH 7 //11
#define UPPER_SICK_LENGTH 25 //25

using namespace std;
using namespace cv;

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
void drop_non_text(vector<Piece> src,vector<Piece> &dst);
void filter_pieces(Mat src_img, vector<Piece> src_vec, vector<Piece> &dst);

void my_inv(Mat in);
void my_grad(Mat src, Mat &dst);
void find_bound_rects(Mat src, vector<Rect> &dst);
void to_Piece(vector<Mat> pic, vector<Rect> rect, vector<Piece> &dst);

void add_white_border(Mat src, Mat &dst, int border_size = 1);
//void insert(Mat src, Mat inset, Point coord);
void vec_imshow(string name, vector<Mat> src);
void vec_imshow(string name, vector<Piece> src);
void show_rects(Mat src, vector<Rect> rects, string title);
void thresh_otsu(Mat src,Mat &dst);
void my_find_contours(Mat src, vector<Rect> &b_rect, int border = 10);
bool same_shape(Mat a, Rect b);
bool piece_is_word(Mat dst, int threshold = 1);
bool is_white(Mat src);
int expected_value(vector<int> row);
int median(vector<int> row);


#endif // IMG_HANDLER_H
