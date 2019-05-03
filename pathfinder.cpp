/**
 * Build: g++ -o pathfinder pathfinder.cpp micropather.cpp
 */

#include "pathfinder.h"


using namespace micropather;

/**
 * Constructor.
 * Initializes the 2D vector with empty values.
 */
Pathfinder::Pathfinder() {
    //initialize 2D vector
    for (int i = 0; i < YMAX; i++) {
        std::vector<Node> row;
        for (int j = 0; j < XMAX; j++) {
            struct Node n = {i, j, true}; 
            row.push_back(n);
        }
        map.push_back(row);
    }
}


/**
 * Implementation of core function for path finding library.
 * Determines the distance between two nodes.
 */
float Pathfinder::LeastCostEstimate( void* stateStart, void* stateEnd ) {
    struct Node *start = (struct Node*)stateStart;
    struct Node *end = (struct Node*)stateEnd;
    PrintStateInfo(stateStart);
    PrintStateInfo(stateEnd);
    int dx = abs(start->x - end->x);
    int dy = abs(start->y - end->y);
    float res = (float) sqrt( (double)(dx*dx) + (double)(dy*dy) );
    std::cout << " " << res << std::endl;
    if (res > 0) {
        return res;
    } else {
        return 0;
    }
}


/**
 * Implementation of required Graph function.
 * Returns the adjacent nodes and their costs.
 */
void Pathfinder::AdjacentCost( void* state, MPVector< StateCost > *adjacent ) {
    struct Node *start = (struct Node*)state;
    //                  E  N   W  S
    const int dx[4] = { 1, 0, -1, 0};
    const int dy[4] = { 0, -1, 0, 1};
    for (int i=0; i<4; ++i) {
        int nx = start->x + dx[i];
        int ny = start->y + dy[i];
        if (nx > XMAX-1 || ny > YMAX-1 || nx < 0 || ny < 0) continue; //stay in bounds
        Node node = map.at(ny).at(nx);
        if (node.traversable) {
            StateCost nodeCost = {&node, 1.0f};
            adjacent->push_back(nodeCost);
            std::cout << "Adjacent: ";
            PrintStateInfo(&node);
            std::cout << std::endl;
        }
    }
    return;
}


/**
 * Implementation of required Graph function used for debugging
 */
void Pathfinder::PrintStateInfo( void* state ) {
    struct Node *node = (struct Node*)state;
    printf("(%d,%d)", node->x, node->y);
    return;
}


/**
 * Fill the map with random data for testing
 */
void Pathfinder::testFill() {
    //fill array with random booleans for testing
    srand(time(0));
    for (int i = 0; i < YMAX; i++) {
        for (int j = 0; j < XMAX; j++) {
            if (rand() % 100 < 90) {
                map[i][j].traversable = true;
            } else {
                map[i][j].traversable = false;
            }
        }
    }
    //Ensure the corners are traversable
    map[0][0].traversable = true;
    map[0][XMAX-1].traversable = true;
    map[YMAX-1][0].traversable = true;
    map[YMAX-1][XMAX-1].traversable = true;
}


/**
 * Print the map to standard output, for debugging
 */
void Pathfinder::printMap() {
    for (int i = 0; i < YMAX; i++) {
        for (int j = 0; j < XMAX; j++) {
            if (map[i][j].traversable) {
                std::cout << ". ";
            } else {
                std::cout << "x ";
            }
        }
        std::cout << std::endl;
    }
}


/**
 * Find and return a path to the specified coordinates and
 */
void Pathfinder::findPath(int startX, int startY, int endX, int endY) {
    MicroPather pather(this);
    float totalCost;
    void* start = (void*) &(map.at(startY).at(startX));
    void* end = (void*) &(map.at(endY).at(endX));
    int result = pather.Solve(start, end, &path, &totalCost);
    std::cout <<  "Success (1 is fail): " << result << std::endl;
}


int main() {
    //Example/testing case. Do something like this in the main robot code
    Pathfinder pf;
    pf.testFill();
    pf.findPath(0, 0, 10, 10);
    std::cout << "Path length: " << pf.path.size() << std::endl;
    pf.printMap();
    return 0;
}
