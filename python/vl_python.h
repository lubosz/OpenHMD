#include "vl_driver.h"
#include <string>
#include <vector>
#include <map>

class ViveLibre {
 public:
  ViveLibre();
  ~ViveLibre();
  void connect();
  std::map<uint32_t, std::vector<uint32_t>> poll_angles(std::string channel, uint32_t samples);
  std::pair<std::vector<float>, std::vector<float> > poll_pnp(std::string channel, uint32_t samples);
  std::string get_config();
};
