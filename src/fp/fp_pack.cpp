#include "fp_pack.h"

using namespace fp;

//---------------------------------------------------------------------

Packer::Packer(db::Database* database_):
      width_(0.0),
      height_(0.0),
      wirelength_(0.0),
      b_star_tree_(database_),
      macro_id_by_node_id_(database_->nMacros, -1),
      is_macro_rotated_by_id_(database_->nMacros, false),
      is_macro_rotatable_by_id_(database_->nMacros, false),
      macro_bounding_box_by_id_(database_->nMacros,
                                make_pair(db::Point(0, 0), db::Point(0, 0)))
    {
        for (int i = 0; i < macro_id_by_node_id_.size(); i++) {
            macro_id_by_node_id_[i] = i;
            is_macro_rotatable_by_id_[i] =
                    (database_->macros[i]->width() 
                != database_->macros[i]->height());
        }
        macro_pins.resize(macro_id_by_node_id_.size());
        database = database_;
} //END MODULE

//---------------------------------------------------------------------

void Packer::print() {
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

int Packer::num_macros() const { return b_star_tree_.num_nodes(); } //END MODULE

//---------------------------------------------------------------------

double Packer::width() const { return width_; } //END MODULE

//---------------------------------------------------------------------

double Packer::height() const { return height_; } //END MODULE

//---------------------------------------------------------------------

double Packer::area() const { return area_; } //END MODULE

//---------------------------------------------------------------------

double Packer::wirelength() const { return wirelength_; } //END MODULE

//---------------------------------------------------------------------

const pair<db::Point, db::Point>& Packer::macro_bounding_box(int macro_id) const {
    return macro_bounding_box_by_id_.at(macro_id);
} //END MODULE

//---------------------------------------------------------------------

void Packer::Perturb() {
    const int num_nodes = b_star_tree_.num_nodes();
    const int num_macros = num_nodes;
    const int op = rand() % 3;
    switch (op) {
        case 0: {
            // TODO: all rotatable?
            // const int macro_id = rand() % num_macros;
            const int macro_id = [&]() {
            int macro_id = rand() % num_macros;
                while (!is_macro_rotatable_by_id_[macro_id]) {
                    macro_id = rand() % num_macros;
                }
                return macro_id;
            }();
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
        // TODO:
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

void Packer::traverseTree(int current_node_id, bool left) {
    
    Node &current_node = b_star_tree_.nodes_[current_node_id];
    const int current_macro_id = macro_id_by_node_id_[current_node_id];
    db::Macro* current_macro = database->macros[current_macro_id];

    // parent node and macro
    int parent_node_id = current_node.parent_id_;
    const int parent_macro_id = macro_id_by_node_id_[parent_node_id];
    db::Macro* parent_macro = database->macros[parent_macro_id];
    
    const pair<db::Point, db::Point> parent_macro_bounding_box =
            macro_bounding_box_by_id_[parent_macro_id];

    const bool is_current_macro_rotated = is_macro_rotated_by_id_[current_macro_id];
    int current_macro_width = current_macro->width();
    int current_macro_height = current_macro->height();
    if (is_current_macro_rotated) {
        swap(current_macro_width, current_macro_height);
    }

    // left or right child of parent
    int lower_left_x;
    int lower_left_y;
    if (left)
        lower_left_x = parent_macro_bounding_box.second.x();
    else 
        lower_left_x = parent_macro_bounding_box.first.x();


    int x_start = lower_left_x;
    int x_end = x_start + current_macro_width;
    if (x_end > contour.box.size())
        contour.box.resize(x_end);
    int y_max = 0;
    for (int i = x_start; i < x_end; i++)
        if (contour.box[i] > y_max)
            y_max = contour.box[i];

    lower_left_y = y_max;

    int upper_right_x = lower_left_x + current_macro_width;
    int upper_right_y = lower_left_y + current_macro_height;

    if (upper_right_x > contour.box_x_max)
        contour.box_x_max = upper_right_x;
    if (upper_right_y > contour.box_y_max)
        contour.box_y_max = upper_right_y;

    y_max += current_macro_height;
    for (int i = x_start; i < x_end; i++)
        contour.box[i] = y_max;

    macro_bounding_box_by_id_[current_macro_id] =
            make_pair(db::Point(lower_left_x, lower_left_y),
                    db::Point(upper_right_x, upper_right_y));
    macro_pins[current_macro_id] = db::pin(
                (int)(lower_left_x + upper_right_x) / 2,
                (int)(lower_left_y + upper_right_y) / 2);

    if (current_node.left_child_id_ != -1)
        traverseTree(current_node.left_child_id_, true);
    if (current_node.right_child_id_ != -1)
        traverseTree(current_node.right_child_id_, false);
    
} //END MODULE

//---------------------------------------------------------------------
// FIXME:
void Packer::PackInt() {
    // find root node and macro
    const int root_id = b_star_tree_.root_id();
    Node &root_node = b_star_tree_.nodes_[root_id];
    const int root_macro_id = macro_id_by_node_id_[root_id];
    db::Macro* root_macro = database->macros[root_macro_id];

    // set box
    int root_macro_width = root_macro->width();
    int root_macro_height = root_macro->height();
    const bool is_root_macro_rotated = is_macro_rotated_by_id_[root_macro_id];
    if (is_root_macro_rotated) {
        swap(root_macro_width, root_macro_height);
    }
    macro_bounding_box_by_id_[root_macro_id] =
            make_pair(db::Point(0, 0),
                    db::Point(root_macro_width, root_macro_height));

    macro_pins[root_macro_id] = db::pin((int)root_macro_width / 2,
                                        (int)root_macro_height / 2 );

    contour = Contour(database->outline_width);
    for (int i = 0; i < root_macro_width; i++)
        contour.box[i] = root_macro_height;

    if (root_node.left_child_id_ != -1)
        traverseTree(root_node.left_child_id_, true);
    if (root_node.right_child_id_ != -1)
        traverseTree(root_node.right_child_id_, false);


    // set Packer outline
    width_ = contour.box_x_max;
    height_ = contour.box_y_max;

    // area_ = width_ * height_

    // log() << width_ << " | " << height_ << " " << 
    //        width_*height_<< endl; 

    area_ = 0;
    int idx = 0;
    for (int &i : contour.box) {
        idx++;
        if (i < database->outline_height && 
                idx <= database->outline_width)
            area_ += database->outline_height;
        else
            area_ += i;
    }
    // log() << area_ << endl;
    // exit(1);


    wirelength_ = 0.0;
    for (int i = 0; i < database->nNets; i++) {
        const vector<int> &net = database->nets_by_id[i];
        const db::pin &terminal_box = database->net_terminals[i];

        int min_x = terminal_box.min_x;
        int min_y = terminal_box.min_y;
        int max_x = terminal_box.max_x;
        int max_y = terminal_box.max_y;

        for (const int id : net) {
            const int x = macro_pins[id].min_x;
            const int y = macro_pins[id].min_y;
            if (x < min_x) {
                min_x = x;
            }
            if (x > max_x) {
                max_x = x;
            }
            if (y < min_y) {
                min_y = y;
            }
            if (y > max_y) {
                max_y = y;
            }
        }
        wirelength_ += (max_x - min_x) + (max_y - min_y);
    }
} //END MODULE

//---------------------------------------------------------------------

void Packer::write(const string& output_path) {
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

        outfile << macro_name << ' ' << lower_left.x() << ' ' << lower_left.y() << ' '
                << is_macro_rotated_by_id_.at(macro_id) << endl;

    }
} //END MODULE

//---------------------------------------------------------------------
