#include "floorplanner.h"
using namespace fp;


//---------------------------------------------------------------------

void Floorplanner::SA() {
    const double P = 0.95;
    const double r = 0.9;
    const double init_T =
            average_uphill_cost_ * database->nMacros / (-1 * log(P));
    const int N = database->nMacros * 20;
    log() << "Init temperature:  " << init_T << endl;
    // exit(1);

    const double stop_T = init_T / 1.0e15;
    const double stop_accept_rate = 0.01;

    const double alpha = alpha_;
    const double beta = (1 - alpha);
    const double adaptive_alpha_base = alpha;
    const double adaptive_beta_base = beta;


    Floorplan floorplan(best_floorplan_);
    floorplan.Pack();

    double adaptive_alpha = adaptive_alpha_base;
    double adaptive_beta = adaptive_beta_base;
    double best_cost = average_uphill_cost_;
    // double best_cost_invalid = best_cost;
    double last_cost = best_cost;
    double T = init_T;
    int num_iteration = 0;
    utils::timer runtime;
    double run_time = 0;
    // 1. Outter loop: T
    // while (run_time < 300) {
    while (T > stop_T) {
        int num_up_hill = 0;
        int num_M = 0;
        num_iteration++;

        int num_accepted_floorplans = 0;
        int num_feasible_floorplans = 0;

        
        // for (int i = 0; i < num_perturbations; ++i) {
        while (num_up_hill <= N && num_M <= 2 * N) {
            // 1. rotate
            // 2. swap
            // 3. delete and insert
            num_M++;
            Floorplan new_floorplan(floorplan);
            new_floorplan.Perturb();
            new_floorplan.PackInt();
      

            const double cost =
                ComputeCostNaive(new_floorplan);
            const double delta_cost = cost - last_cost;
            const double p = exp(-1 * delta_cost / T);
            // log() << delta_cost << endl;
            if ((delta_cost <= 0) ||
                    (rand() / static_cast<double>(RAND_MAX) < p)) 
            {   
                if (delta_cost > 0)
                    num_up_hill++;
                num_accepted_floorplans++;
                floorplan = new_floorplan;
                last_cost = cost;

                const double outline_width = database->outline_width;
                const double outline_height = database->outline_height;
                if (new_floorplan.width() <= outline_width &&
                    new_floorplan.height() <= outline_height) 
                {
                    num_feasible_floorplans++;
                    if (cost < best_cost) {
                        best_floorplan_ = new_floorplan;
                        best_cost = cost;
                    }
                }
                else {
                    if (cost < best_cost) {
                        best_floorplan_invalid_ = new_floorplan;
                        best_cost = cost;
                    }
                }
            } 
        } //END FOR
        double run_time = runtime.elapsed();
        cout << "========== total time: " << run_time << " s ==========\n";
        if (is_verbose_) {
            log() << "--------------- " << num_iteration << " ---------------" << endl;
            log() << "    Moves:    " << num_M << endl;
            log() << "    T:    " << T << endl;
            log() << "    num_accepted_floorplans:  " << num_accepted_floorplans
                << endl;
            log() << "    num_feasible_floorplans:  " << num_feasible_floorplans
                << endl;
            log() << "    num_perturbations:    " << N << endl;
            log() << "    adaptive_alpha: " << adaptive_alpha
                << "\tadaptive_beta: " << adaptive_beta << endl;
            log() << "    Best area: " << best_floorplan_.area()
                << "\tBest wirelength: " << best_floorplan_.wirelength() << endl;
        }

        T *= r;
    }

    if (best_floorplan_.area() == 0) {
        log() << "No feasible solution\n";
        log() << fp_path_ << endl;
        best_floorplan_invalid_.PackInt();
        best_floorplan_invalid_.write(fp_path_);
        best_floorplan_invalid_.Pack();
        best_floorplan_invalid_.write("/data/ssd/bqfu/hw/fp/output/n100.floorplan");
    }
    else{
        best_floorplan_.PackInt();
        best_floorplan_.write(fp_path_);
        best_floorplan_.Pack();
        best_floorplan_.write("/data/ssd/bqfu/hw/fp/output/n100.floorplan");
    }
} //END MODULE

//---------------------------------------------------------------------

void Floorplanner::FastSA() {
    const double P = 0.99;
    const double r = 0.9;
    const double init_T =
            average_uphill_cost_ / (-1 * log(P));

    log() << "Init cost:  " << average_uphill_cost_ << endl;

    log() << "Init temperature:  " << init_T << endl;
    // TODO: solution space
    const int num_perturbations = database->nMacros * database->nMacros;

    const double stop_T = init_T / 1.0e15;
    const double stop_accept_rate = 0.01;
    
    double adaptive_alpha_base = alpha_ / 2;
    double adaptive_beta_base = beta_ / 2;
    const int c = 10;
    const int k = 21;

    Floorplan floorplan(best_floorplan_);
    floorplan.PackInt();

    double adaptive_alpha = adaptive_alpha_base;
    double adaptive_beta = adaptive_beta_base;
    double best_cost = ComputeCost(floorplan, adaptive_alpha, adaptive_beta);
    double last_cost = best_cost;
    double T = init_T;
    utils::timer runtime;
    double run_time = 0;
    double p ;
    int num_iteration = 0;
    // 1. Outter loop: T
    while (T > stop_T) {
        num_iteration++;

        int num_M = 0;
        double total_delta_cost = 0.0;
        int num_accepted_floorplans = 0;
        int num_feasible_floorplans = 0;

        // adapt new values
        last_cost =
                ComputeCost(floorplan, adaptive_alpha, adaptive_beta);

        // 2. Inner loop: solution space
        for (int i = 0; i < num_perturbations; ++i) {
            // 1. rotate
            // 2. swap
            // 3. delete and insert
            num_M++;
            Floorplan new_floorplan(floorplan);

            new_floorplan.Perturb();
            new_floorplan.PackInt();

            const double cost =
                ComputeCost(new_floorplan, adaptive_alpha, adaptive_beta);
            const double delta_cost = cost - last_cost;
            
            p = exp(-1 * delta_cost / T);
            if ((delta_cost < 0) ||
                    (rand() / static_cast<double>(RAND_MAX) < p)) 
            {   
                total_delta_cost += delta_cost;
                num_accepted_floorplans++;
                floorplan = new_floorplan;
                last_cost = cost;

                const double outline_width = database->outline_width;
                const double outline_height = database->outline_height;
                if (new_floorplan.width() <= outline_width &&
                    new_floorplan.height() <= outline_height) 
                {
                    num_feasible_floorplans++;
                    if (cost < best_cost ||
                        new_floorplan.wirelength() < best_floorplan_.wirelength()) {
                        best_floorplan_ = new_floorplan;
                        best_cost = cost;
                    }
                }
                else {
                    if (cost < best_cost) {
                        best_floorplan_invalid_ = new_floorplan;
                        best_cost = cost;
                    }
                }
            } 
        } //END FOR
        double run_time = runtime.elapsed();
        cout << "========== total time: " << run_time << " s ==========\n";
        if (is_verbose_) {
            log() << "--------------- " << num_iteration << " ---------------" << endl;
            log() << "    Moves:    " << num_M << endl;
            log() << "    T:    " << T << endl;
            log() << "    stop T:    " << stop_T << endl;
            log() << "    total_delta_cost: " << total_delta_cost << endl;
            log() << "    cost coef: " << abs(total_delta_cost / num_perturbations) << endl;
            log() << "    num_accepted_floorplans:  " << num_accepted_floorplans
                << endl;
            log() << "    num_feasible_floorplans:  " << num_feasible_floorplans
                << endl;
            log() << "    lambda:    " << lambda_ << endl;
            log() << "    adaptive_alpha: " << adaptive_alpha
                << "\tadaptive_beta: " << adaptive_beta << endl;
            log() << "    Best area: " << best_floorplan_.area()
                << "\tBest wirelength: " << best_floorplan_.wirelength() << endl;
            log() << "invalid Best area: " << best_floorplan_invalid_.area() << endl;
            log() << "invalid Best wirelength: " << best_floorplan_invalid_.wirelength() << endl;
        }

        // if (num_accepted_floorplans / static_cast<double>(num_perturbations) <
        //     stop_accept_rate) 
        //     break;

        // adaptive_alpha =
        //     adaptive_alpha_base +
        //     (alpha_ - adaptive_alpha_base) *
        //         (num_feasible_floorplans / static_cast<double>(num_perturbations));
        // adaptive_beta =
        //     adaptive_beta_base +
        //     (beta_ - adaptive_beta_base) *
        //         (num_feasible_floorplans / static_cast<double>(num_perturbations));
        
        double sol_coef = (num_feasible_floorplans 
                    / static_cast<double>(num_perturbations) );
        // if (sol_coef == 0 && num_iteration < k){
        //     lambda_ *= 2;
        // }
        // else {
        //     lambda_ /= 2;
        // }

        const double average_delta_cost = abs(total_delta_cost / num_perturbations);

        if (num_iteration >= 1 && num_iteration < k) {
            T = init_T * average_delta_cost / (num_iteration + 1) / c;
        } 
        else {
            lambda_ = 1;
            T = init_T * average_delta_cost / (num_iteration + 1);

            // adaptive_alpha =
            //     adaptive_alpha_base +
            //     (alpha_ - adaptive_alpha_base) *
            //         (num_feasible_floorplans / static_cast<double>(num_perturbations));
            // adaptive_beta =
            //     adaptive_beta_base +
            //     (beta_ - adaptive_beta_base) *
            //         (num_feasible_floorplans / static_cast<double>(num_perturbations));
        }
    } //END WHILE

    log() << "    T:    " << T << endl;
    log() << "    stop T:    " << stop_T << endl;
    
    if (best_floorplan_.area() == 0) {
        log() << "No feasible solution\n";
        log() << fp_path_ << endl;
        best_floorplan_invalid_.PackInt();
        best_floorplan_invalid_.write(fp_path_);
        best_floorplan_invalid_.Pack();
        best_floorplan_invalid_.write("/data/ssd/bqfu/hw/fp/output/n100.floorplan");
    }
    else{
        best_floorplan_.PackInt();
        best_floorplan_.write(fp_path_);
        best_floorplan_.Pack();
        best_floorplan_.write("/data/ssd/bqfu/hw/fp/output/n100.floorplan");
    }
} //END MODULE

//---------------------------------------------------------------------

void Floorplanner::Run() {
    if (sa_mode_ == "classical") {
        SA();
    } 
    else if (sa_mode_ == "fast") {
        init();
        int nth_fsa = 1;
        do {
            nth_fsa++;
            if (is_verbose_) {
                log() << nth_fsa << " th Fast SA" << endl;
            }

            FastSA();
        } while (best_floorplan_.area() == 0.0);
    } 
    // else if (sa_mode_ == "both") {
    //     Floorplan initial_floorplan(best_floorplan_);

    //     int nth_sa = 0;
    //     do {
    //         ++nth_sa;
    //         if (is_verbose_) {
    //             cout << nth_sa << " th SA" << endl;
    //         }

    //         SA();
    //     } while (best_floorplan_.area() == 0.0);

    //     Floorplan floorplan_sa(best_floorplan_);

    //     best_floorplan_ = initial_floorplan;
    //     int nth_fsa = 0;
    //     do {
    //         ++nth_fsa;
    //         if (is_verbose_) {
    //             cout << nth_fsa << " th Fast SA" << endl;
    //         }

    //         FastSA();
    //     } while (best_floorplan_.area() == 0.0);
    //     Floorplan floorplan_fsa(best_floorplan_);

    //     const double beta = 1.0 - alpha_;
    //     if (ComputeCost(floorplan_sa, alpha_, beta) <
    //         ComputeCost(floorplan_fsa, alpha_, beta)) 
    //     {
    //         best_floorplan_ = floorplan_sa;
    //     } 
    //     else 
    //     {
    //         best_floorplan_ = floorplan_fsa;
    //     }
    // } 
    // else if (sa_mode_ == "fast-5") {
    //     const int num_fsa = 5;

    //     Floorplan initial_floorplan(best_floorplan_);
    //     Floorplan best_floorplan(initial_floorplan);
    //     int nth_fsa = 0;
    //     for (int i = 0; i < num_fsa; ++i) {
    //         ++nth_fsa;

    //         if (is_verbose_) {
    //             cout << nth_fsa << " th Fast SA" << endl;
    //         }

    //         FastSA();

    //         if (best_floorplan_.area() != 0.0) {
    //             const double beta = 1.0 - alpha_;
    //             if (best_floorplan.area() == 0.0 ||
    //                 ComputeCost(best_floorplan_, alpha_, beta) <
    //                     ComputeCost(best_floorplan, alpha_, beta)) 
    //             {
    //                 best_floorplan = best_floorplan_;
    //             }
    //         }

    //         best_floorplan_ = initial_floorplan;
    //     }

    //     best_floorplan_ = best_floorplan;

    //     while (best_floorplan_.area() == 0.0) {
    //         ++nth_fsa;

    //         if (is_verbose_) {
    //             cout << nth_fsa << " th Fast SA" << endl;
    //         }

    //         FastSA();
    //     }
    // }
} //END MODULE

//---------------------------------------------------------------------
