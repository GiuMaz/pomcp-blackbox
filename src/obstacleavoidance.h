#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "simulator.h"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <array>

class OBSTACLEAVOIDANCE_STATE : public STATE
{
    public:
        std::vector<int> segDifficulties;
        int curSegI;
        int curSubsegJ;
        int dp;
        int p;
        int v;
        double dt;
        double dt1;
        double dt2;
        double t;
        int o;
        double r;
        double r_total;

        std::vector<int> dps;
        std::vector<int> ps;
        std::vector<int> vs;
        std::vector<int> acs;
        std::vector<double> dts;
        std::vector<double> dt1s;
        std::vector<double> dt2s;
        std::vector<double> ts;
        std::vector<int> os;
        std::vector<double> rs;
        std::vector<double> r_totals;
        std::vector<double> dist_state_beliefs;
};

class OBSTACLEAVOIDANCE : public SIMULATOR
{
    public:

        OBSTACLEAVOIDANCE(std::vector<std::vector<double>> map,
                std::vector<std::vector<double>> prob_col);
        // FIXED distribution of new states
        OBSTACLEAVOIDANCE(std::vector<std::vector<double>> map,
                std::vector<std::vector<double>> prob_col,
                int seg, int subseg, const std::vector<double> &belief);

        virtual STATE* Copy(const STATE& state) const;
        virtual void Validate(const STATE& state) const;
        virtual STATE* CreateStartState() const;
        virtual void FreeState(STATE* state) const;
        virtual bool Step(STATE& state, int action, 
                int& observation, double& reward) const;

        void GenerateLegal(const STATE& state, const HISTORY& history,
                std::vector<int>& legal, const STATUS& status) const;
        void GeneratePreferred(const STATE& state, const HISTORY& history,
                std::vector<int>& legal, const STATUS& status) const;
        virtual bool LocalMove(STATE& state, const HISTORY& history,
                int stepObservation, const STATUS& status) const;

        virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
        virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
        virtual void DisplayAction(int action, std::ostream& ostr) const;
        virtual double JointProb(const STATE& state) const;
        void DisplayStateId(const STATE& state, std::ostream& ostr) const;
        void DisplayStateHist(const STATE& state, const char* fileName) const;
        virtual void DisplayBeliefIds(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        STATE* CreateStartState(std::vector<double*>* stateVarRelationships) const;
        STATE* CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs) const;
        STATE* CreateStartStateFixedValues(std::vector<int> values) const;
        virtual bool LocalMove(STATE& state, const HISTORY& history,
                int stepObservation, const STATUS& status, std::vector<double*>* stateVarRelationships) const;

        virtual double ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const;

    protected:
        int nSeg;
        std::vector<int> nSubSegs;
        std::vector<std::vector<double>> subSegLengths;
        int nDifficultyValues;
        int nVelocityValues;
        int collisionPenaltyTime;

        std::vector<std::vector<double>> prob_collision_map;

        // fixed initial distr
        bool has_initial_distr = false;
        const std::vector<double> initial_belief;
        int initial_seg = -1, initial_subseg =-1;

    private:
        mutable MEMORY_POOL<OBSTACLEAVOIDANCE_STATE> MemoryPool;
        mutable std::uniform_real_distribution<> unif_dist;
};

#endif
