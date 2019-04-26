#include <iostream>
#include <vector>

/**
 * Base FSMState class to be inherited by tasks
 */
class FSMState {
    public:
        virtual bool condition() { return false; };
        virtual void run() { return; };
};


/**
 * Finite State Machine class
 */
class FSM {
    std::vector<FSMState*> stack;

    public:

        void update() {
            if (stack.size() > 0) {
                FSMState *state = this->current();
                if (state->condition()) {
                    state->run();
                } else {
                    this->pop();
                }
            }
        }

        void pop() {
            if (stack.size() > 0) {
                stack.pop_back();
            }
        }

        void push(FSMState *state) {
            stack.push_back(state);
        }

        void clear() {
            stack.clear();
        }

        FSMState* current() {
            return stack.back();
        }

        void spin() {
            while (true) {
                this->update();
            }
        }
};


/**
 * Example state class. We'll want to create variants for:
 * MappingField, MovingTo, Arriving, Depositing, Digging
 */
class ExampleState : public FSMState {
    int count;
    public:
        void construct() {
            //There will probably be a reference to the main robot class passed in
            count = 0;
        }

        bool condition() {
            return count < 3;
        }

        void run() {
            count++;
            std::cout << "Count: " << count << std::endl;
        }
};


int main() {
    //Do this in the main robot class:
    FSM fsm = FSM();
    fsm.push(new ExampleState()); //initial state
    fsm.spin();
}
