#ifndef IMG_HANDLER_H
#define IMG_HANDLER_H

#include "all_headers.h"

#define IMG_HANDLER_DEBUG 0

#define BOTTOM_STICK_LENGTH 7 //11
#define UPPER_SICK_LENGTH 25 //25
#define MIN_TEXT_BLOCK_AREA 50

using namespace std;
using namespace cv;

struct Piece{
    Mat pic;
    Rect coord;
};

enum cmp_enum{
    less,
    equal,
    large,
    cross,
};

void vertical_hist(Mat src, Mat& dst,int cols = 250);
void vertical_hist(Mat src,vector<int>& hist);
void horizontal_hist(Mat src, Mat& dst,int rows = 180);
void horizontal_hist(Mat src,vector<int>& hist);
void cut_text_line(Mat in,vector<Mat>& out,int threshold= 1);
void cut_words(Mat in,vector<vector<Mat>>& out,int threshold= 10);
void drop_non_text(Piece src, Piece &dst);
void drop_non_text(vector<Piece> src,vector<Piece> &dst);
void filter_pieces(Mat src_img, vector<Piece> src_vec, vector<Piece> &dst,int min_area = 100);

void my_inv(Mat in);
void my_grad(Mat src, Mat &dst);
void find_bound_rects(Mat src, vector<Rect> &dst);
void find_bound_rects_rgb(Mat src,vector<Rect> &b_rect); // find bound rect for color Mat
void to_Piece(vector<Mat> pic, vector<Rect> rect, vector<Piece> &dst);
void blend_with_mask(Mat &base, Mat &src, Mat &mask, Mat &out);
void matchTemplateCoords(Mat img, Mat templ,Mat mask,Point& matchLoc);

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
template<typename T>
int expected_value(vector<T> row);
template<typename T>
int median(vector<T> row);
int countNonZero_rgb(Mat src);
double cmp(Mat x, Mat y); // 1 - equal, 0 - different
cmp_enum cmp_shape(Mat x, Mat y);


#endif // IMG_HANDLER_H
