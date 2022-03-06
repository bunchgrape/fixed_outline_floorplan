#include "floorplanner.h"

using namespace fp;


Floorplanner::Floorplanner(db::Database* database_, double alpha,
                const std::string& sa_mode, bool is_verbose, 
                const string fp_path)
    :   database(database_),
        fp_path_(fp_path),
        alpha_(alpha),
        sa_mode_(sa_mode),
        is_verbose_(is_verbose),
        min_area_(numeric_limits<double>::max()),
        max_area_(0.0),
        min_wirelength_(numeric_limits<double>::max()),
        max_wirelength_(0.0),
        average_area_(0.0),
        average_wirelength_(0.0),
        average_uphill_cost_(0.0),
        best_floorplan_(Floorplan(database_))
{
    const int num_macros = database_->nMacros;
    const int num_perturbations = num_macros;
    Floorplan floorplan(best_floorplan_);

    floorplan.Pack();

    const double area = floorplan.area();
    const double wirelength = floorplan.wirelength();
    average_area_ = area;
    average_wirelength_ = wirelength;


    double adaptive_alpha = alpha_ / 4.0;
    double adaptive_beta = (1.0 - alpha_) / 4.0;
    double cost = ComputeCost(floorplan, adaptive_alpha, adaptive_beta);
    average_uphill_cost_ = cost;

    log() << floorplan.width() <<' '<<floorplan.height()<<endl;
    log() << average_area_ << ' ' << average_wirelength_ << endl;
    log() << average_uphill_cost_ << endl;
    // exit(1);
} //END MODULE

//---------------------------------------------------------------------

double Floorplanner::ComputeCost(const Floorplan& floorplan, double alpha,
                                 double beta) const {
                                     // FIXME:
    const double outline_width = database->outline_width;
    const double outline_height = database->outline_height;
    const double width = floorplan.width();
    const double height = floorplan.height();
    
    // ratio cost
    const double ratio = height / width; 
    const double outline_ratio = outline_height / outline_width; 
    double outline_penalty = (ratio - outline_ratio) * (ratio - outline_ratio); 
    
    //outline cost
    double width_penalty = 0;
    double height_penalty = 0;
    if (width > outline_width) {
        width_penalty = (width / outline_width);
    }
    if (width > outline_width) {
        height_penalty = (height / outline_height);
    }

    // log() << "-------------------\n";
    // log() << width <<" | "<<outline_width<<endl;
    // log() << height<<" | "<<outline_height<<endl;

    // log()<<floorplan.area() <<" | "<<average_area_<<endl;

    // log()<<floorplan.wirelength() <<" | "<<average_wirelength_<<endl;



    // TODO:
    const double normalized_area = floorplan.area() / average_area_;
    const double normalized_wirelength = floorplan.wirelength() / average_wirelength_;

    // const double cost  = alpha * normalized_area + beta * normalized_wirelength +
    //                     (1.0 - alpha - beta) * (outline_penalty + width_penalty + height_penalty);

    const double cost  = normalized_area + normalized_wirelength +
                        (outline_penalty + width_penalty + height_penalty);

    // const double cost = alpha * normalized_area + beta * normalized_wirelength +
    //                     (1.0 - alpha - beta) * penalty_weight * penalty;

    return cost;
} //END MODULE

//---------------------------------------------------------------------

void Floorplanner::write(const string& output_path) {
    best_floorplan_.write(output_path);
}