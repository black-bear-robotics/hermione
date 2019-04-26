#pragma once

#include <iostream>
#include <vector>

/**
 * Base FSMState class to be inherited by tasks
 */
class FSMState {
    public:
        virtual bool condition();
        virtual void run();
};


/**
 * Finite State Machine class
 */
class FSM {
    private:
        std::vector<FSMState*> stack;
    public:
        void update();
        void pop();
        void push(FSMState *state);
        void clear();
        FSMState* current();
        void spin();
};


/**
 * Example state class. We'll want to create variants for:
 * MappingField, MovingTo, Arriving, Depositing, Digging
 */
class ExampleState : public FSMState {
    private:
        int count;
    public:
        void construct();
        bool condition();
        void run();
};


/**
 * Mapping Field state. This is where we start.
 */
class MappingField : public FSMState {
    private:
        FSM *fsm;
        bool done;
    public:
        void construct(FSM *fsm) {
            //There will probably be a reference to the main robot class passed in.
            //For access to drive system, 2DM getter and data members for things like
            //collection status, path, current location, etc.
            done = false;
            this->fsm = fsm;
        }
        bool condition();
        void run();
};


class MovingTo : public FSMState {
    private:
        FSM *fsm;
    public:
        void construct(FSM *fsm) {
            this->fsm = fsm;
        }
        bool condition();
        void run();
};


class Arriving : public FSMState {
    private:
        FSM *fsm;
    public:
        void construct(FSM *fsm) {
            this->fsm = fsm;
        }
        bool condition();
        void run();
};


class Depositing : public FSMState {
    private:
        FSM *fsm;
        bool done;
    public:
        void Depositing::construct(FSM *fsm) {
            this->fsm = fsm;
            done = false;
        }
        bool condition();
        void run();
};


class Digging : public FSMState {
    private:
        FSM *fsm;
        bool done;
    public:
        void construct(FSM *fsm) {
            void Digging::construct(FSM *fsm) {
            this->fsm = fsm;
            done = false;
        }
        bool condition();
        void run();
};
