#include <iostream>
#include <vector>
#include "fsm.hpp"

/**
 * FSMState class
 */
bool FSMState::condition() { return false; };
void FSMState::run() { return; };


/**
 * Finite State Machine class
 */
void FSM::update() {
    if (stack.size() > 0) {
        FSMState *state = this->current();
        if (state->condition()) {
            state->run();
        } else {
            this->pop();
        }
    }
}

void FSM::pop() {
    if (stack.size() > 0) {
        FSMState* state = FSM::current();
        stack.pop_back();
        free(state);
    }
}

void FSM::push(FSMState *state) {
    stack.push_back(state);
}

void FSM::clear() {
    stack.clear();
}

FSMState* FSM::current() {
    return stack.back();
}

void FSM::spin() {
    while (true) {
        this->update();
    }
}


/**
 * Example state class.
 */
void ExampleState::construct() {
    //There will probably be a reference to the main robot class passed in
    count = 0;
}

bool ExampleState::condition() {
    return count < 3;
}

void ExampleState::run() {
    count++;
    std::cout << "Count: " << count << std::endl;
}


/**
 * MappingField state
 */
bool MappingField::condition() {
    return !done;
}

void MappingField::run() {
    // get 2DM
    // find path
    // set done to true and push MovingTo state if we need to go somewhere
    // use a robot collected boolean to determine which target to seek
    if (done) {
        fsm->push(new MovingTo());
    }
}


/**
 * MovingTo state
 */
bool MovingTo::condition() {
    //true if robot's path > 0
    return false;
}

void MovingTo::run() {
    //get and pop node off our stack from the pathfinder
    //drive robot, sending command to drive control and waiting for response
    //validate that we've actually moved
    //update current location
    if (!condition()) {
        //If we're out of nodes to travel to, push the next state
        fsm->push(new Arriving());
    }
}


/**
 * Arriving state
 */
bool Arriving::condition() {
    return true;
}

void Arriving::run() {
    //Check to make sure the robot is actually where we think it is
    //Get the FSM and pop this off the stack
    //If the robot has collected something, push Depositing() state
    //If the robot's collector is empty, push Digging() state
}


/**
 * Depositing
 */
bool Depositing::condition() {
    return !done;
}

void Depositing::run() {
    //Repeatedly run motor routine until collection bin is empty
    if (done) {
        //update robot collector status to empty
        //push MappingField() onto the stack and let's go around again.
        fsm->push(new MappingField());
    }
}


/**
 * Digging
 */
bool Digging::condition() {
    return !done;
}

void Digging::run() {
    //Repeatedly run motor routine while bin < full and spot still has ice
    if (done) {
        //update robot collector status to full
        //push MappingField() onto the stack and let's go around again.
        fsm->push(new MappingField());
    }
}


int main() {
    //Do this in the main robot class:
    FSM *fsm = new FSM();
    fsm->push(new ExampleState()); //initial state. Would be MappingField
    fsm->spin();
}
