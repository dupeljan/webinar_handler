#include <QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>

#define PIC "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/ex.png"

using namespace cv;
using namespace std;
void my_inv(Mat in){
    threshold(in,in,127,255,THRESH_BINARY_INV);
}
int main(int argc, char *argv[])
{
    string imageName(PIC);
    Mat image,src_gray,grad;

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


    // Sobel
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    int ddepth = CV_8U;
    Sobel(src_gray, grad_x, ddepth, 1, 0);//, ksize, scale, delta, BORDER_DEFAULT);
    Sobel(src_gray, grad_y, ddepth, 0, 1);//, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    // end Sobel

    // Canny
    Mat canny_out;
    Canny(src_gray,canny_out,3,3);
    //OutputArrayOfArrays counturs;
    //findContours(src_gray,counturs,RetrievalModes,ContourApproximationModes);

    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    Mat morphology_out;
    morphologyEx(grad,morphology_out,MORPH_CLOSE, getStructuringElement( MORPH_RECT,Size(3,3) ) );

    Mat inv;
    threshold(morphology_out,inv,127,255,THRESH_BINARY_INV);
    // end Canny

    // My filter
    float left_filter[9] = {1,  0,   -1,
                           2,   0,   -2,
                           1,   0,   -1};
    float bottom_filter[9] {-1, -2, -1,
                             0,  0,  0,
                             1,  2,  1};
    Mat left_border, bottom_border;
    filter2D(src_gray,left_border,-1, Mat(3,3,CV_32F,left_filter) );
    filter2D(src_gray,bottom_border,-1, Mat(3,3,CV_32F,bottom_filter));
    Mat my_filtered;
    addWeighted(left_border,0.5,bottom_border,0.5,0,my_filtered);
    // end filter
    my_inv(my_filtered);
    // Start morphology
    Mat mask;
    //erode(left_border,result,getStructuringElement(MORPH_RECT,Size(1,20)));
    morphologyEx(left_border,left_border,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(1,10)));// нижняя граница
    morphologyEx(left_border,mask,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(1,15)));// верхняя граница
    my_inv(left_border);//инвертируем
    Mat result;
    addWeighted(left_border,1,mask,1,0,result);//Соединияем

    //my_inv(left_border);
    //my_inv(result);
    erode(result,result,getStructuringElement(MORPH_RECT,Size(11,5)));//добавим
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
    imshow( "left_border", left_border);                // Show our image inside it.
    imshow("result",result);
    waitKey(0); // Wait for a keystroke in the window
    return 0;
}

