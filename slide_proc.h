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

struct Area{
    size_t slide;
    size_t text;
    size_t pics;
    Area(){
        slide = text = pics = 0;
    }
};

class Slide_proc
{
    vector<Text_block> text_blocks;
    Area area;
public:
    Slide_proc();
    void proccess(Mat image);
    vector<Text_block> get(){return text_blocks;}
    Area get_area(){ return area; }
};

#endif // SLIDE_PROC_H
