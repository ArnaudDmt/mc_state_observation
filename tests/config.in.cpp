#include <string>

namespace mc_state_observation
{

std::string get_test_configuration()
{
  return "@CFG_OUT@";
}

size_t nrIter()
{
  // clang-format off
  return @NR_ITER@;
  // clang-format on
}

} // namespace mc_state_observation
