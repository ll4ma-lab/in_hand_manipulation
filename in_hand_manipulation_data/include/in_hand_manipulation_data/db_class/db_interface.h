#include <ros/ros.h>

using namespace std;
class dbInterface
{
protected:
  string write_srv_name_;
  string read_srv_name_;
  bool read_data();
  bool write_data();
  bool init();
  
}
