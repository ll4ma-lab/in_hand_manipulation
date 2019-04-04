#include <ll4ma_collision_wrapper/libs/kris_ccd/collision_checker.h>
using namespace Meshing;
using namespace Geometry;
namespace kris_ccd_ccheck
{
class inHandEnv : public collisionChecker
{
public:
  /// Constructor
  inHandEnv() : collisionChecker(){}

  void load_robot_links(vector<string> &arm_link_names, vector<string> &hand_link_names, string &urdf_text);
};
