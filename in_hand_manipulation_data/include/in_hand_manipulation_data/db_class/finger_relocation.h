#include <database_interface/db_class.h>
namespace in_hand_manipulation_data
{
class fingerRelocation : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> object_;
  database_interface::DBField <std::vector<double> > grasp_config_;
  database_interface::DBField <std::vector<int> > f_idx_;  
  database_interface::DBField <std::vector<double> > obj_pose_;
  database_interface::DBField<std::string> mode_;
  database_interface::DBField <int> f_;
  database_interface::DBField <std::vector<double> > des_cpt_;
  std::string table_name="finger_relocation";
  fingerRelocation():
    id_(database_interface::DBFieldBase::TEXT, this, "id",table_name,true),
    object_(database_interface::DBFieldBase::TEXT, this, "object_name",table_name,true),
    grasp_config_(database_interface::DBFieldBase::TEXT, this, "grasp_js",table_name,true),
    f_idx_(database_interface::DBFieldBase::TEXT, this, "fingers",table_name,true),
    obj_pose_(database_interface::DBFieldBase::TEXT, this, "object_pose",table_name,true),
    mode_(database_interface::DBFieldBase::TEXT, this, "finger_relocation_mode",table_name,true),
    f_(database_interface::DBFieldBase::TEXT, this, "finger",table_name,true),
    des_cpt_(database_interface::DBFieldBase::TEXT, this, "des_cpt",table_name,true)
  {
    primary_key_field_=&id_;
    fields_.push_back(&object_);
    fields_.push_back(&grasp_config_);
    fields_.push_back(&f_idx_);
    fields_.push_back(&obj_pose_);
    fields_.push_back(&mode_);
    fields_.push_back(&f_);
    fields_.push_back(&des_cpt_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

  }

};
}
