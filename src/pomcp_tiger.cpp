#include "mcts.h"
#include "experiment.h"
#include "tiger.h"
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

    double RewardRange = 103.0;
    size_t seed = 0;

    std::vector<double> belief;
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("outputfile", value<string>(&outputfile)->default_value("output.txt"), "summary output file")
        ("mindoubles", value<int>(&expParams.MinDoubles), "minimum power of two simulations")
        ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
        ("belief", value<vector<double>>(&belief)->multitoken(), "belief velocity regulation (probability distribution over 2 states)")
        ("W", value<double>(&RewardRange), "set reward range (W)")
        ("seed", value<size_t>(&seed), "set seed")
        ;


    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      cout << desc << "\n";
      return 1;
    }

    if (vm.count("belief") == 0 || belief.size() != 2) {
      cerr << "probability distribution on 2 states required, your input "
              "contains "
           << belief.size() << " probabilities" << std::endl;
      return 1;
    }


    expParams.MaxDoubles = expParams.MinDoubles;

    // initialize real and simulator (simulator use the forced belief)
    auto real = std::make_unique<TIGER>();
    auto simulator = std::make_unique<TIGER>(belief);

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

