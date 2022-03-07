#include "global.h"
#include "db/Database.h"

#include "floorplan.h"

namespace fp {

class Floorplanner {
public:
    Floorplanner(db::Database* database_, double alpha,
                const std::string& sa_mode, bool is_verbose,
                const string fp_path = " ");

    db::Database* database;

    string fp_path_;

    double alpha() const;
    const Floorplan& best_floorplan() const;

    void write(const string& output_path);
    void init();
    void Run();

private:
    void SA();
    void FastSA();
    double ComputeCostNaive(const Floorplan& floorplan) const;
    double ComputeCost(const Floorplan& floorplan, double alpha,
                        double beta) const;

    
    double alpha_;
    std::string sa_mode_;
    bool is_verbose_;
    double min_area_;
    double max_area_;
    double min_wirelength_;
    double max_wirelength_;
    double average_area_;
    double average_wirelength_;
    double average_uphill_cost_;
    Floorplan best_floorplan_;
    Floorplan best_floorplan_invalid_;
};

}