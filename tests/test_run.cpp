#include <mc_control/Ticker.h>

namespace mc_state_observation
{

std::string get_test_configuration();

size_t nrIter();

} // namespace mc_state_observation

int main()
{
  mc_control::Ticker::Configuration config;
  config.mc_rtc_configuration = mc_state_observation::get_test_configuration();
  mc_control::Ticker ticker(config);
  for(size_t i = 0; i < mc_state_observation::nrIter(); ++i)
  {
    bool r = ticker.step();
    if(!r)
    {
      mc_rtc::log::critical("Failed at iter {}", i);
      return 1;
    }
  }
  return 0;
}
