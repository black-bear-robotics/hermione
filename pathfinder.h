#include <iostream>
#include <vector>
#include <math.h>
#include "micropather.h"

#define XMAX 50
#define YMAX 30 

using namespace micropather;

struct Node {
    int x, y;
    bool traversable;
};

class Pathfinder : public micropather::Graph {
    public:
        std::vector <std::vector<Node> > map;
        MPVector<void*> path;
        Pathfinder();
        void testFill();
        void printMap();
        void findPath(int startX, int startY, int endX, int endY);
        virtual float LeastCostEstimate( void* stateStart, void* stateEnd );
        virtual void AdjacentCost( void* state, MPVector< StateCost > *adjacent );
        virtual void PrintStateInfo( void* state );
};

