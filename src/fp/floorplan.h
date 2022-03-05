#include "db/Database.h"
#include "b_star_tree.h"
#include "contour.h"
#include "global.h"

namespace db {

class Floorplan {
public:
    
    Floorplan(db::Database* database_);
    db::Database* database;

    int num_macros() const;
    double width() const;
    double height() const;
    double area() const;
    double wirelength() const;
    const pair<Point, Point>& macro_bounding_box(int macro_id) const;

    void Perturb(const Database& database);
    void Pack();
    void print() {b_star_tree_.Print();}

    void write(const string& output_path);

private:
    double width_;
    double height_;
    double wirelength_;
    BStarTree b_star_tree_;
    vector<int> macro_id_by_node_id_;
    vector<bool> is_macro_rotated_by_id_;
    vector<pair<Point, Point>> macro_bounding_box_by_id_;
};

}