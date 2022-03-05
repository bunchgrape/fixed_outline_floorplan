#include "global.h"
#include "db/Database.h"

namespace fp {

class Contour {
public:
    Contour();

    void Print(std::ostream& os = std::cout, int indent_level = 0) const;

    double max_x() const;
    double max_y() const;

    std::pair<db::Point, db::Point> Update(double x, double width, double height);

private:
    double FindMaxYBetween(double x_begin, double x_end) const;

    double max_y_;
    std::list<db::Point> coordinates_;
};

}