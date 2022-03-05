#include "floorplan.h"

using namespace fp;

//---------------------------------------------------------------------

Floorplan::Floorplan(db::Database* database_):
      width_(0.0),
      height_(0.0),
      wirelength_(0.0),
      b_star_tree_(database_->nMacros),
      macro_id_by_node_id_(database_->nMacros, -1),
      is_macro_rotated_by_id_(database_->nMacros, false),
      macro_bounding_box_by_id_(database_->nMacros,
                                make_pair(db::Point(0, 0), db::Point(0, 0)))
    {
        for (int i = 0; i < macro_id_by_node_id_.size(); i++) {
            macro_id_by_node_id_[i] = i;
        }
        database = database_;
} //END MODULE

//---------------------------------------------------------------------

void Floorplan::print() {
    const int indent_size = 2;
    int indent_level = 0;
    cout << string(indent_level * indent_size, ' ') << "BStarTree:" << endl;
    indent_level++;
    cout << string(indent_level * indent_size, ' ') << "root_name_:" << database->macros[
                    macro_id_by_node_id_.at(b_star_tree_.root_id())]->name() << endl;
    cout << string(indent_level * indent_size, ' ') << "nodes_:" << endl;
    indent_level++;
    for (int i = 0; i < b_star_tree_.nodes_.size(); i++) {
        const Node& node = b_star_tree_.nodes_[i];
        cout << string(indent_level * indent_size, ' ') << "Node: " << database->macros[macro_id_by_node_id_.at(i)]->name() << endl;
        indent_level++;
        if (node.parent_id_ != -1)
            cout << string(indent_level * indent_size, ' ')
                << "parent_name_: " << database->macros[macro_id_by_node_id_.at(node.parent_id_)]->name() << endl;
        if (node.left_child_id_ != -1)
            cout << string(indent_level * indent_size, ' ')
                << "left_child_name_: " << database->macros[macro_id_by_node_id_.at(node.left_child_id_)]->name() << endl;
        if (node.right_child_id_ != -1)
            cout << string(indent_level * indent_size, ' ')
                << "right_child_name_: " << database->macros[macro_id_by_node_id_.at(node.right_child_id_)]->name() << endl;
        indent_level--;
    }
} //END MODULE

//---------------------------------------------------------------------

int Floorplan::num_macros() const { return b_star_tree_.num_nodes(); } //END MODULE

//---------------------------------------------------------------------

double Floorplan::width() const { return width_; } //END MODULE

//---------------------------------------------------------------------

double Floorplan::height() const { return height_; } //END MODULE

//---------------------------------------------------------------------

double Floorplan::area() const { return width_ * height_; } //END MODULE

//---------------------------------------------------------------------

double Floorplan::wirelength() const { return wirelength_; } //END MODULE

//---------------------------------------------------------------------

const pair<db::Point, db::Point>& Floorplan::macro_bounding_box(int macro_id) const {
    return macro_bounding_box_by_id_.at(macro_id);
} //END MODULE

//---------------------------------------------------------------------

void Floorplan::Perturb() {
    const int num_nodes = b_star_tree_.num_nodes();
    const int num_macros = num_nodes;
    const int op = rand() % 3;
    switch (op) {
        case 0: {
            const int macro_id = rand() % num_macros; // TODO: all rotatable
            is_macro_rotated_by_id_.at(macro_id) =
                !is_macro_rotated_by_id_.at(macro_id);
            break;
        }
    case 1: {
        const int node_a_id = rand() % num_nodes;
        const int node_b_id = [&]() {
        int node_id = rand() % num_nodes;
            while (node_id == node_a_id) {
                node_id = rand() % num_nodes;
            }
            return node_id;
        }();
        int& node_a_macro_id = macro_id_by_node_id_.at(node_a_id);
        int& node_b_macro_id = macro_id_by_node_id_.at(node_b_id);
        swap(node_a_macro_id, node_b_macro_id);
        break;
    }
    case 2: {
        const int node_a_id = rand() % num_nodes;
        const int node_b_id = [&]() {
        int node_id = rand() % num_nodes;
            while (node_id == node_a_id) {
                node_id = rand() % num_nodes;
            }
        return node_id;
        }();
        auto inserted_positions = make_pair(rand(), rand());
        b_star_tree_.DeleteAndInsert(node_a_id, node_b_id, inserted_positions);

        break;
    }
    default:
        break;
    }
} //END MODULE

//---------------------------------------------------------------------

void Floorplan::Pack() {
    // log() << "Packing \n";
    b_star_tree_.UnvisitAll();

    const int root_id = b_star_tree_.root_id();
    const int root_macro_id = macro_id_by_node_id_.at(root_id);
    db::Macro* root_macro = database->macros[root_macro_id];

    int root_macro_width = root_macro->width();
    int root_macro_height = root_macro->height();
    const bool is_root_macro_rotated = is_macro_rotated_by_id_.at(root_macro_id);

    if (is_root_macro_rotated) {
        swap(root_macro_width, root_macro_height);
    }

    Contour contour;
    macro_bounding_box_by_id_.at(root_macro_id) =
        contour.Update(0.0, root_macro_width, root_macro_height);

    // traverse
    stack<int> unvisited_node_ids;
    unvisited_node_ids.push(root_id);
    while (!unvisited_node_ids.empty()) {
        const int current_node_id = unvisited_node_ids.top();
        const int current_macro_id = macro_id_by_node_id_.at(current_node_id);
        const pair<db::Point, db::Point> current_macro_bounding_box =
            macro_bounding_box_by_id_.at(current_macro_id);
        const int left_child_id = b_star_tree_.left_child_id(current_node_id);
        const int right_child_id = b_star_tree_.right_child_id(current_node_id);

        if (left_child_id != -1 && !b_star_tree_.is_visited(left_child_id)) {
            unvisited_node_ids.push(left_child_id);

            const int left_child_macro_id = macro_id_by_node_id_.at(left_child_id);
            db::Macro* left_child_macro = database->macros[left_child_macro_id];
            int left_child_macro_width = left_child_macro->width();
            int left_child_macro_height = left_child_macro->height();
            const bool is_left_child_macro_rotated =
                is_macro_rotated_by_id_.at(left_child_macro_id);

            if (is_left_child_macro_rotated) {
                swap(left_child_macro_width, left_child_macro_height);
            }

            macro_bounding_box_by_id_.at(left_child_macro_id) =
                contour.Update(current_macro_bounding_box.second.x(),
                                left_child_macro_width, left_child_macro_height);

        } else if (right_child_id != -1 &&
               !b_star_tree_.is_visited(right_child_id)) {
            unvisited_node_ids.push(right_child_id);

            const int right_child_macro_id = macro_id_by_node_id_.at(right_child_id);
            db::Macro* right_child_macro = database->macros[right_child_macro_id];
            double right_child_macro_width = right_child_macro->width();
            double right_child_macro_height = right_child_macro->height();
            const bool is_right_child_macro_rotated =
                is_macro_rotated_by_id_.at(right_child_macro_id);

            if (is_right_child_macro_rotated) {
                swap(right_child_macro_width, right_child_macro_height);
            }

            macro_bounding_box_by_id_.at(right_child_macro_id) =
                contour.Update(current_macro_bounding_box.first.x(),
                                right_child_macro_width, right_child_macro_height);

        } else {
            unvisited_node_ids.pop();

            b_star_tree_.Visit(current_node_id);
        }
    }



    width_ = contour.max_x();
    height_ = contour.max_y();



    // for (Net* net :database.nets){
    //     log() << net->max_x_ <<' '<< net->max_y_ <<' '<< net->min_y_ <<endl;
    // }
    // exit(1);
    
    wirelength_ = 0.0;
    for (int i = 0; i < database->nNets; i++) {
        wirelength_ += database->nets[i]->ComputeWirelength(macro_bounding_box_by_id_);
    }
    // log() << "Wirelength: " << wirelength_ << endl;
} //END MODULE

//---------------------------------------------------------------------


void Floorplan::write(const string& output_path) {
    ofstream outfile;
    outfile.open(output_path, ios::out);
    outfile << "Wirelength " << wirelength_ << "\n";
    outfile <<"Blocks\n";

    for (int i = 0; i < b_star_tree_.num_nodes(); i++) {
        const int macro_id = i;
        const string macro_name = database->macros[macro_id]->name();
        auto bounding_box = macro_bounding_box(macro_id);
        const db::Point& lower_left = bounding_box.first;
        const db::Point& upper_right = bounding_box.second;
        // log() << macro_name << " " << lower_left.x() << " " << lower_left.y()
        //         << " " << upper_right.x() << " " << upper_right.y() << endl;
        
        outfile << macro_name << ' ' << lower_left.x() << ' ' << lower_left.y() << ' '
                << is_macro_rotated_by_id_.at(macro_id) << endl;

    }
} //END MODULE

//---------------------------------------------------------------------
