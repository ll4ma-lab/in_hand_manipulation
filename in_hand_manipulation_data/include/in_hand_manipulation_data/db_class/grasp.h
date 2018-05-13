#include <database_interface/db_class.h>
namespace in_hand_manipulation_data
{
class Grasp : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> object_;
  database_interface::DBField <std::vector<double> > grasp_config_;
  database_interface::DBField <std::vector<int> > f_idx_;  
  database_interface::DBField <std::vector<double> > obj_pose_;
  std::string table_name="grasp";
  Grasp():
    id_(database_interface::DBFieldBase::TEXT, this, "id",table_name,true),
    object_(database_interface::DBFieldBase::TEXT, this, "object_name",table_name,true),
    grasp_config_(database_interface::DBFieldBase::TEXT, this, "grasp_js",table_name,true),
    f_idx_(database_interface::DBFieldBase::TEXT, this, "fingers",table_name,true),
    obj_pose_(database_interface::DBFieldBase::TEXT, this, "object_pose",table_name,true)
  {
    primary_key_field_=&id_;
    fields_.push_back(&object_);
    fields_.push_back(&grasp_config_);
    fields_.push_back(&f_idx_);
    fields_.push_back(&obj_pose_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

  }

};
}
