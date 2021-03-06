#ifndef TIGER_H
#define TIGER_H

#include "simulator.h"
#include <unordered_map>
#include <vector>
#include <random>
#include <algorithm>

class TIGER_STATE : public STATE
{
public:
    bool tiger_on_left;
    std::vector<int> saved_actions;
};

class TIGER : public SIMULATOR
{
public:
    TIGER();
    TIGER(std::vector<double> belief);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action, int& observation,
                      double& reward) const;
    void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation,
                                    std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

protected:
    std::vector<int> saved_actions;

    enum
    {
        A_LISTEN,
        A_LEFT_DOOR,
        A_RIGHT_DOOR,
    };

    enum
    {
        O_LEFT_ROAR,
        O_RIGHT_ROAR,
        O_TREASURE,
        O_TIGER
    };

private:
    mutable MEMORY_POOL<TIGER_STATE> MemoryPool;

    // shield
    double tr_open, tr_listen;
    std::vector<double> initial_belief;
};

#endif // TIGER_H
