#include "Macro.h"
#include "Database.h"

using namespace db;

void Macro::init(){
    if (rectNum == 4) {
        _width  = rects[4];
        _height = rects[3];
    }
}