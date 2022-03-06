#include "global.h"
#include "db/Database.h"

namespace fp {

class Node {
public:
    Node(int parent_id, int left_child_id, int right_child_id);
    int id;
    int parent_id_;
    int left_child_id_;
    int right_child_id_;
    bool is_visited_;
};

class BStarTree {
public:
    BStarTree(db::Database* database_);
    db::Database* database;

    void Print(std::ostream& os = std::cout, int indent_level = 0) const;

    int num_nodes() const;
    int root_id() const;
    int parent_id(int node_id) const;
    int left_child_id(int node_id) const;
    int right_child_id(int node_id) const;
    bool is_visited(int node_id) const;

    void Visit(int node_id);
    void UnvisitAll();
    void DeleteAndInsert(int deleted_node_id, int target_node_id,
                        std::pair<int, int> inserted_positions);

    std::vector<Node> nodes_;

private:

    const Node& node(int node_id) const;
    Node& node(int node_id);
    void Delete(int deleted_node_id);
    void Insert(int inserted_node_id, int target_node_id,
                std::pair<int, int> inserted_positions);

    int root_id_;
};

}