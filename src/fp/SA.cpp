#include "floorplanner.h"

using namespace fp;


//---------------------------------------------------------------------

void Floorplanner::SA() {
    const double P = 0.95;
    const double r = 0.9;
    const double init_T =
            average_uphill_cost_ * database->nMacros / (-1 * log(P));
    const int N = database->nMacros * 20;
    log() << database->nMacros<<endl;
    log() << average_uphill_cost_<<endl;
    log() << "Init temperature:  " << init_T << endl;
    // exit(1);

    const double stop_T = init_T / 1.0e15;
    const double stop_accept_rate = 0.01;

    const double alpha = alpha_;
    const double beta = (1 - alpha);
    const double adaptive_alpha_base = alpha / 4.0;
    const double adaptive_beta_base = beta / 4.0;


    Floorplan floorplan(best_floorplan_);
    floorplan.Pack();
    // log()<<floorplan.width()<<' '<<floorplan.height()<<endl;
    // int i =0;
    // while(i<10000){
    //     i++;
    //     // floorplan.Perturb();
    //     floorplan.PackInt();
    // }
    // // log()<<floorplan.width()<<' '<<floorplan.height()<<endl;
    // exit(1);

    // utils::timer runtime;
    // floorplan.PackInt();
    // double pack_time = runtime.elapsed();
    // log() << "========== Pack time: " << pack_time << " s ==========\n\n\n";
    // exit(1);

    double adaptive_alpha = adaptive_alpha_base;
    double adaptive_beta = adaptive_beta_base;
    double best_cost = average_uphill_cost_;
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
                ComputeCost(new_floorplan, adaptive_alpha, adaptive_beta);
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

        // TODO:
        // write(fp_path);
        // best_floorplan_.print();
        // exit(1);


        // if (num_accepted_floorplans / static_cast<double>(N) <
        //     stop_accept_rate) 
        //     break;

        // adaptive_alpha =
        //     adaptive_alpha_base +
        //     (alpha - adaptive_alpha_base) *
        //         (num_feasible_floorplans / static_cast<double>(N));
        // adaptive_beta =
        //     adaptive_beta_base +
        //     (beta - adaptive_beta_base) *
        //         (num_feasible_floorplans / static_cast<double>(N));
        T *= r;
    }
} //END MODULE

//---------------------------------------------------------------------

void Floorplanner::FastSA() {
    const double P = 0.95;
    const double init_T =
        average_uphill_cost_ / (-1 * log(P));

    // TODO: solution space
    const int num_perturbations = database->nMacros * 40;

    const double stop_T = init_T / 1.0e50;
    const double stop_accept_rate = 0.01;

    const double alpha = alpha_;
    const double beta = (1 - alpha);
    const double adaptive_alpha_base = alpha / 4.0;
    const double adaptive_beta_base = beta / 4.0;
    const int c = 100;
    const int k = 7;

    Floorplan floorplan(best_floorplan_);
    floorplan.Pack();

    double adaptive_alpha = adaptive_alpha_base;
    double adaptive_beta = adaptive_beta_base;
    double best_cost = ComputeCost(floorplan, adaptive_alpha, adaptive_beta);
    double last_cost = best_cost;
    double T = init_T;
    int num_iteration = 0;
    // 1. Outter loop: T
    while (T > stop_T) {
        num_iteration++;

        double total_delta_cost = 0.0;
        int num_accepted_floorplans = 0;
        int num_feasible_floorplans = 0;

        // 2. Inner loop: solution space
        for (int i = 0; i < num_perturbations; ++i) {
            // 1. rotate
            // 2. swap
            // 3. delete and insert
            Floorplan new_floorplan(floorplan);
            new_floorplan.Perturb();
            new_floorplan.Pack();

            const double cost =
                ComputeCost(new_floorplan, adaptive_alpha, adaptive_beta);
            const double delta_cost = cost - last_cost;
            const double p = exp(-1 * delta_cost / T);
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
                    if (cost < best_cost) {
                        best_floorplan_ = new_floorplan;
                        best_cost = cost;
                    }
                }
            }
        } //END FOR

        if (is_verbose_) {
            log() << "  num_iteration: " << num_iteration << endl;
            log() << "    temperature: " << T << endl;
            log() << "    total_delta_cost: " << total_delta_cost << endl;
            log() << "    num_accepted_floorplans: " << num_accepted_floorplans
                << endl;
            log() << "    num_feasible_floorplans: " << num_feasible_floorplans
                << endl;
            log() << "    num_perturbations: " << num_perturbations << endl;
            log() << "    adaptive_alpha: " << adaptive_alpha
                << "\tadaptive_beta: " << adaptive_beta << endl;
            log() << "    Best area: " << best_floorplan_.area()
                << "\tBest wirelength: " << best_floorplan_.wirelength() << endl;
        }

        if (num_accepted_floorplans / static_cast<double>(num_perturbations) <
            stop_accept_rate) 
            break;

        adaptive_alpha =
            adaptive_alpha_base +
            (alpha - adaptive_alpha_base) *
                (num_feasible_floorplans / static_cast<double>(num_perturbations));
        adaptive_beta =
            adaptive_beta_base +
            (beta - adaptive_beta_base) *
                (num_feasible_floorplans / static_cast<double>(num_perturbations));

        const double average_delta_cost = abs(total_delta_cost / num_perturbations);
        if (num_iteration >= 1 && num_iteration < k) {
            T =
                init_T * average_delta_cost / (num_iteration + 1) / c;
        } 
        else {
            T =
                init_T * average_delta_cost / (num_iteration + 1);
        }
    } //END WHILE
} //END MODULE

//---------------------------------------------------------------------

void Floorplanner::Run() {
    if (sa_mode_ == "classical") {
        // int nth_sa = 0;
        // do {
        //     ++nth_sa;
        //     if (is_verbose_) {
        //         log() << nth_sa << " th SA" << endl;
        //     }

        //     SA();
        // } while (best_floorplan_.area() == 0.0);
        SA();
    } 
    else if (sa_mode_ == "fast") {
        int nth_fsa = 0;
        do {
            ++nth_fsa;
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
