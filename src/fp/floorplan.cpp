#include "floorplan.h"

using namespace fp;


Floorplan::Floorplan(db::Database* database_, double alpha,
                const std::string& sa_mode, bool is_verbose, 
                const string fp_path)
    :   database(database_),
        fp_path_(fp_path),
        alpha_(alpha),
        lambda_(100),
        sa_mode_(sa_mode),
        is_verbose_(is_verbose),
        min_area_(numeric_limits<double>::max()),
        max_area_(0.0),
        min_wirelength_(numeric_limits<double>::max()),
        max_wirelength_(0.0),
        average_area_(0.0),
        average_wirelength_(0.0),
        average_uphill_cost_(0.0),
        best_floorplan_(Packer(database_)),
        best_floorplan_invalid_(Packer(database_))
{
    const int num_macros = database_->nMacros;
    const int num_perturbations = num_macros;
    Packer floorplan(best_floorplan_);

    floorplan.PackInt();

    const double area = floorplan.area();
    const double wirelength = floorplan.wirelength();
    average_area_ = area;
    average_wirelength_ = wirelength;

    double cost = Calc_Cost_Naive(floorplan);
    average_uphill_cost_ = cost;

} //END MODULE

//---------------------------------------------------------------------

void Floorplan::init(){
    const int num_macros = database->nMacros;
    const int num_perturbations = num_macros * num_macros;
    Packer floorplan(best_floorplan_);

    best_floorplan_invalid_ = best_floorplan_;
    floorplan.PackInt();

    // init WL and Area
    double total_area = 0.0;
    double total_wirelength = 0.0;

    // init cost
    beta_ = 1 - alpha_;
    // alpha_ /= 2;
    // beta_ /= 2;
    double adaptive_alpha = alpha_ * balance_;
    double adaptive_beta = beta_ * balance_;
    double total_uphill_cost = 0.0;
    int num_uphills = 0;
    double last_cost =
            Calc_Cost(floorplan, adaptive_alpha, adaptive_beta);
    for (int i = 0; i < num_perturbations; i++) {
        floorplan.Perturb();
        floorplan.PackInt();
        double cost = Calc_Cost(floorplan, adaptive_alpha, adaptive_beta);
        double delta_cost = cost - last_cost;
        if (delta_cost > 0) {
            num_uphills++;
            total_uphill_cost += delta_cost;
        }
        last_cost = cost;
        int area = floorplan.area();
        total_area += area;
        int wirelength = floorplan.wirelength();
        total_wirelength += wirelength;

        if (area < min_area_) {
            min_area_ = area;
        }
        if (area > max_area_) {
            max_area_ = area;
        }
        if (wirelength < min_wirelength_) {
            min_wirelength_ = wirelength;
        }
        if (wirelength > max_wirelength_) {
            max_wirelength_ = wirelength;
        }
    }

    average_area_ = total_area / num_perturbations;
    average_wirelength_ = total_wirelength / num_perturbations;
    average_uphill_cost_ = total_uphill_cost / num_uphills;
    
} //END MODULE

//---------------------------------------------------------------------

double Floorplan::Calc_Cost_Naive(const Packer& floorplan) const {
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

    // TODO:
    const double normalized_area = floorplan.area() / average_area_;
    const double normalized_wirelength = floorplan.wirelength() / average_wirelength_;

    const double cost  = normalized_area + normalized_wirelength +
                        (outline_penalty + width_penalty + height_penalty);

    return cost;
} //END MODULE

//---------------------------------------------------------------------

double Floorplan::Calc_Cost(const Packer& floorplan, double alpha,
                                 double beta) const {
                                     // FIXME:
    const double outline_width = database->outline_width;
    const double outline_height = database->outline_height;
    const double width = floorplan.width();
    const double height = floorplan.height();
    
    //outline cost
    double width_penalty = 0;
    double height_penalty = 0;
    if (width > outline_width) {
        width_penalty = (width / outline_width);
    }
    if (width > outline_width) {
        height_penalty = (height / outline_height);
    }

    double box_penalty = width_penalty + height_penalty;

    // ratio cost
    const double ratio = height / width; 
    const double outline_ratio = outline_height / outline_width; 
    double outline_penalty = (ratio - outline_ratio) * (ratio - outline_ratio); 


    // TODO:
    const double normalized_area = floorplan.area() / average_area_;
    const double normalized_wirelength = floorplan.wirelength() / average_wirelength_;

    // double cost = (alpha * normalized_area + beta * normalized_wirelength +
    //                     (1.0 - alpha - beta) * lambda_
    //                     * (outline_penalty + box_penalty))
    //                     / (1 + lambda_);
    
    double cost = alpha * normalized_area + beta * normalized_wirelength +
                        (1.0 - alpha - beta) * (outline_penalty);

    return cost;
}  //END MODULE

//---------------------------------------------------------------------

void Floorplan::write(const string& output_path) {
    best_floorplan_.write(output_path);
}