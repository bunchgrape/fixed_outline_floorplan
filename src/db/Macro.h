#pragma once
#include "Database.h"

namespace db {

class Macro {
private:
    string _name;
    int _width;
    int _height;

public:
    int id = -1;

    int rectNum = 0;
    bool rotated = 0;
    vector<int> rects;

    Macro(const string& name = "") : _name(name) {}
    ~Macro();

    const std::string& name() const { return _name; }

    void init();

    int width() {return _width;}
    int height() {return _height;}

    friend ostream& operator<<(ostream& os, const Macro& c) {
        return os << c._name << "\t(" << ')';
    }
};

}