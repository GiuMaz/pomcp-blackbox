#include "obstacleavoidance.h"
#include "utils.h"
#include <fstream>
#include <sstream>

using namespace std;
using namespace UTILS;

OBSTACLEAVOIDANCE::OBSTACLEAVOIDANCE(std::vector<std::vector<double>> map,
        std::vector<std::vector<double>> prob_col) :
    nSubSegs(),
    subSegLengths(map),
    nDifficultyValues(3),
    nVelocityValues(3),
    nSeg(map.size()),
    unif_dist(0.0, 1.0),
    prob_collision_map(prob_col),
    has_initial_distr(false)
{
    // store number of subsegments in each segment
    for (const auto &s : map) nSubSegs.push_back(s.size());

    NumActions = 3;
    NumObservations = 2;

    collisionPenaltyTime=50;

    RewardRange = 52;
    Discount = 0.95;
}

OBSTACLEAVOIDANCE::OBSTACLEAVOIDANCE(std::vector<std::vector<double>> map,
                                     std::vector<std::vector<double>> prob_col,
                                     int seg, int subseg,
                                     const std::vector<double> &bel)
    : nSubSegs(), subSegLengths(map), nDifficultyValues(3), nVelocityValues(3),
      nSeg(map.size()), unif_dist(0.0, 1.0), prob_collision_map(prob_col),
      has_initial_distr(true),
      initial_seg(seg), initial_subseg(subseg), initial_belief(bel) {
  // store number of subsegments in each segment
  for (const auto &s : map)
    nSubSegs.push_back(s.size());

  NumActions = 3;
  NumObservations = 2;

  collisionPenaltyTime = 50;

  RewardRange = 52;
  Discount = 0.95;
}

STATE* OBSTACLEAVOIDANCE::Copy(const STATE& state) const // Makes a copy of the state state
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    OBSTACLEAVOIDANCE_STATE* newstate = MemoryPool.Allocate();
    *newstate = bmState;
    return newstate;
}

// NOT USED
void OBSTACLEAVOIDANCE::Validate(const STATE& state) const
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
}

// Used by standard planner
STATE* OBSTACLEAVOIDANCE::CreateStartState() const
{
    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_totals.clear();
    bmState->segDifficulties.clear();

    if (has_initial_distr) {
        bmState->curSegI = initial_seg;
        bmState->curSubsegJ = initial_subseg;
    }
    else {
        bmState->curSegI = 0;
        bmState->curSubsegJ=0;
    }
    bmState->p=0;                 // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                 // Set initial time
    bmState->ts.push_back(0);
    bmState->r_totals.push_back(0); // Set initial cumulative reward

    if (has_initial_distr) {
        double cumul_sum = 0;
        double rnd = unif_dist(random_state);
        int state_pos = 0;
        for (const double &b : initial_belief) {
            cumul_sum += b;
            if (rnd < cumul_sum) break;
            state_pos++;
        }
        if (state_pos >= 81)
            state_pos = 80; // force last state

        int maxexp = 1;
        for (int i = 0; i < (nSeg-1); ++i)
            maxexp *=3;

        for (int i = 0; i < nSeg; ++i) {
            int diff = (state_pos / maxexp) % nDifficultyValues;
            maxexp /= 3;
            bmState->segDifficulties.push_back(diff);
        }
    } else {
        int rnd;
        for (int i = 0; i < nSeg; ++i) {
            rnd = (random_state() % nDifficultyValues);
            bmState->segDifficulties.push_back(rnd);
        }
    }

    return bmState;
}

/*
STATE* OBSTACLEAVOIDANCE::CreateStartStateFixedValues(std::vector<int> values) const
{

    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_totals.clear();
    bmState->segDifficulties.clear();

    bmState->curSegI = 0;         // Set agent position (segment i)
    bmState->curSubsegJ=0;        // Set agent position (subsegment j)
    bmState->p=0;                 // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                 // Set initial time
    bmState->ts.push_back(0);
    bmState->r_totals.push_back(0); // Set initial cumulative reward

    int rnd;
    for (int i = 0; i < nSeg; ++i)
    {
        bmState->segDifficulties.push_back(values[i]); 
    }

    return bmState;
}

STATE* OBSTACLEAVOIDANCE::CreateStartState(std::vector<double*>* stateVarRelationships) const
{
    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    return bmState;
}

STATE* OBSTACLEAVOIDANCE::CreateStartState(vector<STATE*>* allParticles, vector<double> allParticleCumProbs) const
{
    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_totals.clear();
    bmState->segDifficulties.clear();

    bmState->curSegI = 0;         // Set agent position (segment i)
    bmState->curSubsegJ=0;        // Set agent position (subsegment j)
    bmState->p=0;                 // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                 // Set initial time
    bmState->ts.push_back(0);
    bmState->r_totals.push_back(0); // Set initial cumulative reward

    // Select a particle from allParticles with probability from allParticleProb
    double rnd = unif_dist(random_state);
    int partI=0; // Particle index
    double cumProbTmp=0.0;
    while(cumProbTmp<=rnd){
        partI++;
        cumProbTmp=allParticleCumProbs[partI];
    }
    partI--;
    const OBSTACLEAVOIDANCE_STATE& bmParticle = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(*((*allParticles)[partI]));

    for (int i = 0; i < nSeg; i++) // Set each segment difficulty to that of the particle
    {
        bmState->segDifficulties.push_back(bmParticle.segDifficulties[i]);
    }
    return bmState;
}
*/

void OBSTACLEAVOIDANCE::FreeState(STATE* state) const
{
    OBSTACLEAVOIDANCE_STATE* bmState = safe_cast<OBSTACLEAVOIDANCE_STATE*>(state);
    MemoryPool.Free(bmState);
}


bool OBSTACLEAVOIDANCE::Step(STATE& state, int action,
        int& observation, double& reward) const
{
    OBSTACLEAVOIDANCE_STATE& s = safe_cast<OBSTACLEAVOIDANCE_STATE&>(state);

    s.acs.push_back(action); // In history

    s.v=action;
    s.vs.push_back(action);
    double prob_collision =
        prob_collision_map[s.segDifficulties[s.curSegI]][action];

    // 1  collision, 0 = no collision
    int dp= unif_dist(random_state) < prob_collision ? 1 : 0;

    s.dp=dp;
    s.dps.push_back(dp);

    s.p=s.p+dp;
    s.ps.push_back(s.p);

    double dt, dt1, dt2;
    dt1= (nVelocityValues-s.v) * subSegLengths[s.curSegI][s.curSubsegJ];
    s.dt1s.push_back(dt1);
    s.dt1 = dt1;

    dt2 = dp * collisionPenaltyTime;
    s.dt2s.push_back(dt2);
    s.dt2 = dt2;

    dt = dt1 + dt2;
    s.dts.push_back(dt);
    s.dt=dt;

    s.t=s.t+dt1+dt2;
    s.ts.push_back(s.t);

    double prob_observe_obstacle;
    if (s.segDifficulties[s.curSegI]==0) {
        prob_observe_obstacle=0.1;
    }
    if(s.segDifficulties[s.curSegI]==1){
        prob_observe_obstacle=0.5;
    }
    if(s.segDifficulties[s.curSegI]==2){
        prob_observe_obstacle=0.9;
    }
    int o= unif_dist(random_state) < prob_observe_obstacle ? 1 : 0 ;

    s.o=o;
    s.os.push_back(o);

    observation = o;

    // Reward
    reward= - dt1 - dt2;

    s.r=reward;
    s.rs.push_back(reward);
    s.r_total=s.r_total+reward;
    s.r_totals.push_back(s.r_total);

    // Save current distance state-belief (only for data analysis)
    double dist=0; // Total distance real state <-> belief
    int hammingDist=0; // Hamming distance between single state and belief
    int res=0;

    // Update position, to be ready for next step
    bool is_last = s.curSegI == (nSeg-1) && s.curSubsegJ==(nSubSegs[s.curSegI]-1);

    if (s.curSubsegJ==(nSubSegs[s.curSegI]-1)) {
        s.curSegI += 1;
        s.curSubsegJ = 0;
    }
    else {
        s.curSubsegJ += 1;
    }

    if (is_last)
        return true; // terminal state
    else
        return false;
}

bool OBSTACLEAVOIDANCE::LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const
{
    return true;
}



// Puts in legal a set of legal actions that can be taken from state
void OBSTACLEAVOIDANCE::GenerateLegal(const STATE& state, const HISTORY& history,
        vector<int>& legal, const STATUS& status) const
{
    legal.push_back(0);
    legal.push_back(1);
    legal.push_back(2);
}

void OBSTACLEAVOIDANCE::GeneratePreferred(const STATE& state, const HISTORY& history,
        vector<int>& actions, const STATUS& status) const
{
    actions.push_back(0);
    actions.push_back(1);
    actions.push_back(2);
}

// Display methods -------------------------
void OBSTACLEAVOIDANCE::DisplayBeliefs(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
        std::ostream& ostr) const
{
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefs start" << endl;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayState(*s,cout);
    }
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefs end" << endl;
}

void OBSTACLEAVOIDANCE::DisplayBeliefIds(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
        std::ostream& ostr) const
{
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefIds: [";
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayStateId(*s, cout);cout <<"; ";
    }
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefs end" << endl;
}

void OBSTACLEAVOIDANCE::DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const
{
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(*state);

        id=0;   
        for (int j = 0; j<nSeg; j++)  // For each segment difficulty
        {
            id+=bmState.segDifficulties[j]*(pow(3,nSeg-j-1));
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }
    for (auto it = dist.begin(); it != dist.end(); ++it ){  // For each state in the belief
        ostr << it->first << ":" << it->second << ", ";
    }
}

void OBSTACLEAVOIDANCE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    ostr << endl;

    // Display segment difficulties
    ostr << "## STATE ############" << endl;
    ostr << "Difficulties: ";
    for (int i = 0; i < nSeg; i++)
        ostr << i << ":" << bmState.segDifficulties[i] << ", ";
    ostr << endl;

    // Display agent's position
    ostr << "Position: i:" << bmState.curSegI << ", j:" << bmState.curSubsegJ << endl;

    // 1. Display delta battery voltage in last subsegment
    ostr << "dp: " << bmState.dp << endl;

    // 2. Display voltage
    ostr << "p: " << bmState.p << endl;

    // 3. Display Velocity in the last subsegment
    ostr << "v: " << bmState.v << endl;

    // 5a. Delta time due to normal navigation in last subsegment
    ostr << "dt1: " << bmState.dt1 << endl;

    // 5b. Delta time due to collisions in last subsegment
    ostr << "dt2: " << bmState.dt2 << endl;

    // 5. Delta time in last subsegment
    ostr << "dt: " << bmState.dt << endl;

    // 6. Display time
    ostr << "t: " << bmState.t << endl;

    // 8. Occupancy
    ostr << "o: " << bmState.o << endl;

    // 9. Reward
    ostr << "r: " << bmState.r << endl;

    // 10. Display cumulative reward
    ostr << "r_total: " << bmState.r_total << endl;
    ostr << "#######################" << endl<< endl;
}

void OBSTACLEAVOIDANCE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const // Prints the observation
{
    switch (observation)
    {
        case 0:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=0, dt1=1" << endl;
            break;
        case 1:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=0, dt1=2" << endl;
            break;
        case 2:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=0, dt1=3" << endl;
            break;
        case 3:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=1, dt1=1" << endl;
            break;
        case 4:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=1, dt1=2" << endl;
            break;
        case 5:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=1, dt1=3" << endl;
            break;
    }
}

void OBSTACLEAVOIDANCE::DisplayAction(int action, std::ostream& ostr) const // Prints the action performed
{
    if (action == 0)  
        ostr << "Action: Low power (0)" << endl;
    if (action == 1) 
        ostr << "Action: Medium power (1)" << endl;
    if (action == 2) 
        ostr << "Action: High power (2)" << endl;
}


double OBSTACLEAVOIDANCE::JointProb(const STATE& state) const // Displays the STATE (grid with agent and rocks)
{
    // not implemented yet
    return 1.0;
}

void OBSTACLEAVOIDANCE::DisplayStateId(const STATE& state, std::ostream& ostr) const // Displays an id from rock values
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    int id=0;
    string s="State id: ";
    for (int i = 0; i<nSeg; i++)  // For each segment difficulty
    {
        id+=bmState.segDifficulties[i]*(pow(nDifficultyValues,nSeg-i-1));
        s=s+to_string(bmState.segDifficulties[i]);
        if(i!=(nSeg-1)){
            s=s+"-";
        }
    }
    cout << s << "(" << id << ")";
}

void OBSTACLEAVOIDANCE::DisplayStateHist(const STATE& state, const char* fileName) const // Displays an id from rock values
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    std::ofstream outFile;
    outFile.open(fileName, std::ofstream::out | std::ofstream::app);

    string s;
    s="Step";
    int step=0;
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(step);
            step=step+1;
        }
    }
    outFile << s << endl;

    s="Segments";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(i);
        }
    }
    outFile << s << endl;

    s="SubSegments";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(j);
        }
    }
    outFile << s << endl;

    s="Lengths";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(subSegLengths[i][j]);
        }
    }
    outFile << s << endl;

    int stateId=0;   
    for (int i = 0; i<nSeg; i++)  // For each segment difficulty
    {
        stateId+=bmState.segDifficulties[i]*(pow(3,nSeg-i-1));
    }
    s="Difficulties ("+ to_string(stateId)+")";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(bmState.segDifficulties[i]);
        }
    }
    outFile << s << endl;

    s="dps";
    for (int i = 0; i<bmState.dps.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dps[i]);
    }

    outFile << s << endl;

    // Battery voltage at the end of the last subsegment (2)
    s="p";
    for (int i = 0; i<bmState.ps.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.ps[i]);
    }
    outFile << s << endl;

    // Velocity in the last subsegment (3)
    s="v";
    for (int i = 0; i<bmState.vs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.vs[i]);
    }
    outFile << s << endl;

    // Actions (4)
    s="acs";
    for (int i = 0; i<bmState.acs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.acs[i]);
    }
    outFile << s << endl;

    // Delta time in last segment (5a)
    s="dt1s";
    for (int i = 0; i<bmState.dt1s.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dt1s[i]);
    }
    outFile << s << endl;

    // Delta time in last segment (5b)
    s="dt2s";
    for (int i = 0; i<bmState.dt2s.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dt2s[i]);
    }
    outFile << s << endl;

    // Delta time in last segment (5)
    s="dts";
    for (int i = 0; i<bmState.dts.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dts[i]);
    }
    outFile << s << endl;

    // Time from the beginning (cumulative) (6)
    s="ts";
    for (int i = 0; i<bmState.ts.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.ts[i]);
    }
    outFile << s << endl;

    // Occupancy (o)
    s="os";
    for (int i = 0; i<bmState.os.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.os[i]);
    }
    outFile << s << endl;

    // Reward last subsegment (7)
    s="rs";
    for (int i = 0; i<bmState.rs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.rs[i]);
    }
    outFile << s << endl;

    // Cumulative reward (8)
    s="r_totals";
    for (int i = 0; i<bmState.r_totals.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.r_totals[i]);
    }
    outFile << s << endl;

    // Cumulative reward (8)
    s="dist_state_bel";
    for (int i = 0; i<bmState.dist_state_beliefs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dist_state_beliefs[i]);
    }
    outFile << s << endl<< endl;

    outFile.close();
}

bool OBSTACLEAVOIDANCE::LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status, std::vector<double*>* stateVarRelationships) const
{
    // TO BE IMPLEMENTED
    return true;
}

double OBSTACLEAVOIDANCE::ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const {
    OBSTACLEAVOIDANCE_STATE& oastate = safe_cast<OBSTACLEAVOIDANCE_STATE&>(particle);
    double probab=1;

    int var0, var1, seg0Val, seg1Val;

    double potentialEqualValues, potentialDifferentValues, potential;
    for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){
        var0=(*it)[0];
        var1=(*it)[1];
        potentialEqualValues=(*it)[2]/nDifficultyValues;
        potentialDifferentValues=(1.0-(*it)[2])/6;
        // Apply the constraint to the state and compute the related potential
        seg0Val=oastate.segDifficulties[var0];
        seg1Val=oastate.segDifficulties[var1];
        if(seg0Val==seg1Val){ // They are equal
            potential=potentialEqualValues;
        }
        else{   // They are different
            potential=potentialDifferentValues;
        }
        probab=probab*potential;
    }
    return probab;
}
