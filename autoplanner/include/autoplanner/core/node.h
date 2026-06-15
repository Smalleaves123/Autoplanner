#pragma once

namespace autoplanner {

// A graph search node with index, cost, and parent reference.
struct Node {
    int index = -1;
    double g = 0.0;
    double h = 0.0;
    int parent = -1;
};

}  // namespace autoplanner
