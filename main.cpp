#include <QCoreApplication>

// Only that order
#include "slide_proc.h"
#include "video_handler.h"
#include "img_handler.h"

#define PIC "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/gen_slides/slide_8.png"

#define SLIDE_DEBUG 0
#define VIDEO_DEBUG 1
#define FREE_TEST 0


using namespace cv;
using namespace std;

#if FREE_TEST == 0

int main(int argc, char *argv[]){

#if VIDEO_DEBUG == 1
    video_main();
#endif
#if SLIDE_DEBUG == 1
    string imageName(PIC);

    auto image = imread(imageName.c_str(), IMREAD_COLOR); // Read the file

    if( image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    Slide_proc s;
    s.proccess(image);
    return 0;

#endif

}

#endif
#if FREE_TEST == 1
int main(){
    string root = "/home/dupeljan/Documents/courcework_2019/Latex/pictures/solution/";
    auto cursor = imread(root+"cursor.png");
    Mat cmp_img[2] = { imread(root + "cmp3.png"),imread(root+"cmp4.png")};
    imshow("cmp0",cmp_img[0]);
    imshow("cmp1",cmp_img[1]);
    waitKey();
    cout << cmp(cmp_img[0],cmp_img[1]) << endl << cmp(cursor,cmp_img[1]) << endl    ;
    return 0;
}
/*
int main(int argc, char ** argv)
{
   string root = "/home/dupeljan/Documents/courcework_2019/graphics/2/";
   string filename =  root + "input1.jpg";
    Mat I = imread(filename, 0);

    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( I.rows );
    int n = getOptimalDFTSize( I.cols ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];

    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1,NORM_MINMAX); // Transform the matrix with float values into a
                                            // viewable image form (float between values 0 and 1).

    imwrite(root + "gray_" + filename + ".jpg",I);
    imshow("Input Image"       , I   );    // Show the result
    imshow("spectrum magnitude", magI);
    waitKey();

    return 0;
}
*/
//int main( int argc, char** argv )
//{
//    auto img = imread(PIC,1);
//    imwrite("/home/dupeljan/Documents/courcework_2019/photos/tmpl_matching_inp.png",img);
//    auto templ = imread(CURSOR,1);
//    imwrite("/home/dupeljan/Documents/courcework_2019/photos/cursor.png",templ);
//    Mat mask;
//    thresh_otsu(templ,mask);
//    cvtColor(mask,mask,COLOR_GRAY2BGR);
//    Point matchLoc;
//    matchTemplateCoords(img,templ,mask,matchLoc);
//    rectangle(img,Rect(matchLoc.x,matchLoc.y,mask.cols,mask.rows),Scalar(255,0,0),2);
//    imwrite("/home/dupeljan/Documents/courcework_2019/photos/templ_matching_result.png",img);
//}



#endif

