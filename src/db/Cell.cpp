
#include "Cell.h"
#include "Database.h"

using namespace db;


Cell::Cell(const string& name){
    _name = name;
    pos = new Point();
}