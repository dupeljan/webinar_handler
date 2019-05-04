#include <QCoreApplication>

// Only that order
#include "slide_proc.h"
#include "video_handler.h"
#include "img_handler.h"

#define PIC "/home/dupeljan/Projects/webinar_analisator/web_analis_opencv/gen_slides/slide_25.png"

#define SLIDE_DEBUG 1
#define VIDEO_DEBUG 0


using namespace cv;
using namespace std;


int main(int argc, char *argv[]){

#if VIDEO_DEBUG == 1
    video_main();
#elif SLIDE_DEBUG == 1
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


