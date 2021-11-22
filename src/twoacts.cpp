#include "mcts.h"
#include "network.h"
#include "experiment.h"
#include "obstacleavoidance2.h"
#include <boost/program_options.hpp>
#include <vector>

using namespace std;
using namespace boost::program_options;

/*
void UnitTests()
{
    cout << "Testing UTILS" << endl;
    UTILS::UnitTest();
    cout << "Testing COORD" << endl;
    COORD::UnitTest();
    cout << "Testing MCTS" << endl;
    MCTS::UnitTest();
}
*/

void disableBufferedIO(void)
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int main(int argc, char* argv[])
{
    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    //SIMULATOR::KNOWLEDGE knowledge;
    string outputfile;
    int treeknowledge = 1, rolloutknowledge = 1, smarttreecount = 10;
    double smarttreevalue = 1.0;

    double RewardRange = 52.0;
    size_t seed = 0;

    std::vector<double> belief;
    std::vector<int> pos;
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("outputfile", value<string>(&outputfile)->default_value("output.txt"), "summary output file")
        ("mindoubles", value<int>(&expParams.MinDoubles), "minimum power of two simulations")
        ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
        ("belief", value<vector<double>>(&belief)->multitoken(), "belief velocity regulation (probability distribution over 81 states)")
        ("pos", value<vector<int>>(&pos)->multitoken(), "segment and subsegment")
        ("W", value<double>(&RewardRange), "set reward range (W)")
        ("seed", value<size_t>(&seed), "set seed")
        //("runs", value<int>(&expParams.NumRuns), "number of runs")
        //("test", "run unit tests")
        //("problem", value<string>(&problem), "problem to run")
        //("policy", value<string>(&policy), "policy file (explicit POMDPs only)")
        //("size", value<int>(&size), "size of problem (problem specific)")
        //("number", value<int>(&number), "number of elements in problem (problem specific)")
        //("timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")
        //("accuracy", value<double>(&expParams.Accuracy), "accuracy level used to determine horizon")
        //("horizon", value<int>(&expParams.UndiscountedHorizon), "horizon to use when not discounting")
        //("num steps", value<int>(&expParams.NumSteps), "number of steps to run when using average reward")
        //("autoexploration", value<bool>(&expParams.AutoExploration), "Automatically assign UCB exploration constant")
        //("exploration", value<double>(&searchParams.ExplorationConstant), "Manual value for UCB exploration constant")
        //("usetransforms", value<bool>(&searchParams.UseTransforms), "Use transforms")
        //("transformdoubles", value<int>(&expParams.TransformDoubles), "Relative power of two for transforms compared to simulations")
        //("transformattempts", value<int>(&expParams.TransformAttempts), "Number of attempts for each transform")
        //("userave", value<bool>(&searchParams.UseRave), "RAVE")
        //("ravediscount", value<double>(&searchParams.RaveDiscount), "RAVE discount factor")
        //("raveconstant", value<double>(&searchParams.RaveConstant), "RAVE bias constant")
        //("treeknowledge", value<int>(&knowledge.TreeLevel), "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")
        //("rolloutknowledge", value<int>(&knowledge.RolloutLevel), "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")
        //("smarttreecount", value<int>(&knowledge.SmartTreeCount), "Prior count for preferred actions during smart tree search")
        //("smarttreevalue", value<double>(&knowledge.SmartTreeValue), "Prior value for preferred actions during smart tree search")
        //("disabletree", value<bool>(&searchParams.DisableTree), "Use 1-ply rollout action selection")
        ;


    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      cout << desc << "\n";
      return 1;
    }

    if (vm.count("belief") == 0 || belief.size() != 81) {
      cerr << "probability distribution on 81 states required, your input "
              "contains "
           << belief.size() << " probabilities" << std::endl;
      return 1;
    }

    if (vm.count("pos") == 0 || pos.size() != 2) {
      cerr << "missing segment and subsegment" << std::endl;
      return 1;
    }

    expParams.MaxDoubles = expParams.MinDoubles;

    // -----  Rectangle parameters -----

    // 3 x 5 rectangle, all subsegment has lenght 1.0
    std::vector<std::vector<double>> map = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0}};

    // prob_collision_map[difficulty][speed]:
    // defines the probability of collision given the pair (segment difficulty,speed)
    // e.g.: if the segment has difficulty 2 and the robot move at speed 1 there
    // is a 11% risk of collision
    std::vector<std::vector<double>> prob_collision_map = {
        {0.0, 0.0},
        {0.0, 0.14},
        {0.0, 0.25}
    };

    // ---------------------------------

    // initialize real and simulator (simulator use the forced belief)
    auto real = std::make_unique<TWOACTSOBSTACLE>(map, prob_collision_map);
    auto simulator = std::make_unique<TWOACTSOBSTACLE>(
        map, prob_collision_map, pos[0], pos[1], belief);

    if (vm.count("seed")) {
        simulator->set_seed(seed);
    }

    if (vm.count("W")) {
        real->SetRewardRange(RewardRange);
        simulator->SetRewardRange(RewardRange);
    }

    EXPERIMENT experiment(*real, *simulator, outputfile, expParams,
                          searchParams);
    experiment.DiscountedReturn();

    return 0;
}

