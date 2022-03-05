#pragma once
#include "Database.h"


namespace db {

class Net {
private:
    string _name;

public:
    int min_x_;
    int min_y_;
    int max_x_;
    int max_y_;
    int id = -1;
    bool cut = false;
    vector<Cell*> cell_list;
    vector<Macro*> macro_list;


    Net(const std::string& name = "") : _name(name) { }
    ~Net();

    const std::string& name() const { return _name; }

    void init_bx();

    int ComputeWirelength(
        std::vector<std::pair<Point, Point>> macro_bounding_box_by_id) const;

    friend ostream& operator<<(ostream& os, const Net& c) {
        return os << c._name << "\n";
    }
};

}
