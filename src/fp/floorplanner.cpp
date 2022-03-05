#include "floorplanner.h"

using namespace fp;


Floorplanner::Floorplanner(db::Database* database_, double alpha,
                const std::string& sa_mode, bool is_verbose)
    :   database(database_),
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
    const int num_perturbations = num_macros * num_macros;
    Floorplan floorplan(best_floorplan_);

    double total_area = 0.0;
    double total_wirelength = 0.0;
    for (int i = 0; i < num_perturbations; ++i) {
        // log() << "iter " << i << endl;
        floorplan.Perturb();
        floorplan.Pack();
        const double area = floorplan.area();
        const double wirelength = floorplan.wirelength();
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
        total_area += area;
        total_wirelength += wirelength;
    }
    average_area_ = total_area / num_perturbations;
    average_wirelength_ = total_wirelength / num_perturbations;

    double total_uphill_cost = 0.0;
    int num_uphills = 0;
    double adaptive_alpha = alpha_ / 4.0;
    double adaptive_beta = (1.0 - alpha_) / 4.0;
    double last_cost =
        ComputeCost(best_floorplan_, adaptive_alpha, adaptive_beta);
    for (int i = 0; i < num_perturbations; ++i) {
        // log() << "iter " << i << endl;
        floorplan.Perturb();
        floorplan.Pack();
        double cost = ComputeCost(floorplan, adaptive_alpha, adaptive_beta);
        double delta_cost = cost - last_cost;
        if (delta_cost > 0) {
            ++num_uphills;
            total_uphill_cost += delta_cost;
        }
        last_cost = cost;
    }
    average_uphill_cost_ = total_uphill_cost / num_uphills;
} //END MODULE

//---------------------------------------------------------------------

double Floorplanner::ComputeCost(const Floorplan& floorplan, double alpha,
                                 double beta) const {
                                     // FIXME:
    const double outline_width = database->outline_width;
    const double outline_height = database->outline_height;
    const double width = floorplan.width();
    const double height = floorplan.height();

    const double penalty_weight = 100.0;
    double penalty = 0.0;
    if (width > outline_width || height > outline_height) {
        if (width > outline_width && height > outline_height) {
        penalty += (width * height - outline_width * outline_height);
        } else if (width > outline_width) {
        penalty += ((width - outline_width) * height);
        } else if (height > outline_height) {
        penalty += (width * (height - outline_height));
        }
        penalty += ((width - outline_width) * (width - outline_width) +
                    (height - outline_height) * (height - outline_height));
        penalty /= (max_area_ - min_area_);
    }

    /* const double penalty_weight = 1.0; */
    /* double penalty = 0.0; */
    /* if (width > outline_width || height > outline_height) { */
    /*   const double ratio = height / width; */
    /*   const double outline_ratio = outline_height / outline_width; */
    /*   penalty = (ratio - outline_ratio) * (ratio - outline_ratio); */
    /* } */

    /* const double penalty_weight = 1.0; */
    /* const double ratio = height / width; */
    /* const double outline_ratio = outline_height / outline_width; */
    /* const double penalty = (ratio - outline_ratio) * (ratio - outline_ratio);
    */

    const double normalized_area = floorplan.area() / (max_area_ - min_area_);
    const double normalized_wirelength =
        floorplan.wirelength() / (max_wirelength_ - min_wirelength_);

    const double cost = alpha * normalized_area + beta * normalized_wirelength +
                        (1.0 - alpha - beta) * penalty_weight * penalty;
    // log()<<alpha<<' '<<beta<<' '<<normalized_wirelength;
    // exit(1);
    return cost;
} //END MODULE

//---------------------------------------------------------------------

void Floorplanner::write(const string& output_path) {
    best_floorplan_.write(output_path);
}