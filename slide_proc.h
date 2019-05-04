#ifndef SLIDE_PROC_H
#define SLIDE_PROC_H

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>


#include "all_headers.h"
#include "img_handler.h"

#define SLIDE_PROC_DEBUG 1

using namespace cv;
using namespace std;

struct Text_block{
    Rect coord;
    string text;
};

class Slide_proc
{
    vector<Text_block> text_blocks;
public:
    Slide_proc();
    void proccess(Mat image);
};

#endif // SLIDE_PROC_H
