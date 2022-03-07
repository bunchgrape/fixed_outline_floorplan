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
        best_floorplan_(Floorplan(database_)),
        best_floorplan_invalid_(Floorplan(database_))
{
    const int num_macros = database_->nMacros;
    const int num_perturbations = num_macros;
    Floorplan floorplan(best_floorplan_);


    floorplan.PackInt();
    // log() << floorplan.width() <<' '<<floorplan.height()<<endl;
    // floorplan.Pack();
    // log() << floorplan.width() <<' '<<floorplan.height()<<endl;
    // exit(1);

    const double area = floorplan.area();
    const double wirelength = floorplan.wirelength();
    average_area_ = area;
    average_wirelength_ = wirelength;


    double adaptive_alpha = alpha_ / 2;
    double adaptive_beta = (1.0 - alpha_) / 2;
    double cost = ComputeCostNaive(floorplan);
    average_uphill_cost_ = cost;

    log() << floorplan.width() <<' '<<floorplan.height()<<endl;
    log() << average_area_ << ' ' << average_wirelength_ << endl;
    log() << average_uphill_cost_ << endl;
    // exit(1);
} //END MODULE

//---------------------------------------------------------------------


void Floorplanner::init(){
    const int num_macros = database->nMacros;
    const int num_perturbations = num_macros * num_macros;
    Floorplan floorplan(best_floorplan_);

    floorplan.PackInt();

    // init WL and Area
    double total_area = 0.0;
    double total_wirelength = 0.0;
    for (int i = 0; i < num_perturbations; i++) {
        floorplan.Perturb();
        floorplan.PackInt();
        
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

    // init cost
    double adaptive_alpha = alpha_ / 4.0;
    double adaptive_beta = (1.0 - alpha_) / 4.0;
    double total_uphill_cost = 0.0;
    int num_uphills = 0;
    double last_cost =
            ComputeCost(floorplan, adaptive_alpha, adaptive_beta);
    for (int i = 0; i < num_perturbations; i++) {
        floorplan.Perturb();
        floorplan.PackInt();
        double cost = ComputeCost(floorplan, adaptive_alpha, adaptive_beta);
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
    
    // log() << floorplan.width() <<' '<<floorplan.height()<<endl;
    // log() << average_area_ << ' ' << average_wirelength_ << endl;
    // log() << num_uphills << ' ' << average_uphill_cost_ << endl;
    // exit(1);
} //END MODULE

//---------------------------------------------------------------------

double Floorplanner::ComputeCostNaive(const Floorplan& floorplan) const {
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

    const double cost  = normalized_area + normalized_wirelength +
                        (outline_penalty + width_penalty + height_penalty);



    return cost;
} //END MODULE

//---------------------------------------------------------------------

double Floorplanner::ComputeCost(const Floorplan& floorplan, double alpha,
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

    // ratio cost
    const double ratio = height / width; 
    const double outline_ratio = outline_height / outline_width; 
    double outline_penalty = (ratio - outline_ratio) * (ratio - outline_ratio); 

    // // cost
    // const double penalty_weight = 1;
    // double penalty = 0.0;
    // if (width > outline_width || height > outline_height) {
    //     if (width > outline_width && height > outline_height) {
    //         penalty += (width * height - outline_width * outline_height);
    //     } 
    //     else if (width > outline_width) {
    //         penalty += ((width - outline_width) * height);
    //     } 
    //     else if (height > outline_height) {
    //         penalty += (width * (height - outline_height));
    //     }
    //     penalty += ((width - outline_width) * (width - outline_width) +
    //                 (height - outline_height) * (height - outline_height));
    //     penalty /= (max_area_ - min_area_);
    // }



    // TODO:
    const double normalized_area = floorplan.area() / average_area_;
    const double normalized_wirelength = floorplan.wirelength() / average_wirelength_;
    // const double normalized_area1 = floorplan.area() / (max_area_ - min_area_);
    // const double normalized_wirelength1 =
    //             floorplan.wirelength() / (max_wirelength_ - min_wirelength_);

    const double cost = alpha * normalized_area + beta * normalized_wirelength +
                        (1.0 - alpha - beta) * 100
                        * (outline_penalty + width_penalty + height_penalty);

    // const double cost = alpha * (normalized_area + outline_penalty)
    //                     + beta * normalized_wirelength;

    // log() << "-------------------\n";
    // // log() << width <<" | "<<outline_width<<endl;
    // // log() << height<<" | "<<outline_height<<endl;

    // log() << normalized_area << " | " << normalized_wirelength << endl;
    // log() << normalized_area1 << " | " << normalized_wirelength1 << endl;

    // // log()<< floorplan.area() << " | " << average_area_ << endl;

    // // log() << floorplan.wirelength() << " | " << average_wirelength_ << endl;
    // log() << "penalty " << outline_penalty << " | " << penalty << endl;
    // log() << "cost " << cost << endl;

    // exit(1);

    return cost;
}  //END MODULE

//---------------------------------------------------------------------

void Floorplanner::write(const string& output_path) {
    best_floorplan_.write(output_path);
}