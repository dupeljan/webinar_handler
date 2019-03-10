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

    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    int ddepth = CV_8U;
    Sobel(src_gray, grad_x, ddepth, 1, 0);//, ksize, scale, delta, BORDER_DEFAULT);
    Sobel(src_gray, grad_y, ddepth, 0, 1);//, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);

    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "Display window", grad );                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window
    return 0;
}

