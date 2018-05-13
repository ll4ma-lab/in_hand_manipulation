#include <database_interface/db_class.h>
namespace in_hand_manipulation_data
{
class inGrasp : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> object_;
  database_interface::DBField <std::vector<double> > grasp_config_;
  database_interface::DBField <std::vector<int> > f_idx_;  
  database_interface::DBField <std::vector<double> > obj_pose_;
  database_interface::DBField <std::vector<double> > des_obj_pose_;
  //std::string table_name_;
  inGrasp():
    id_(database_interface::DBFieldBase::TEXT, this, "id","in_grasp",true),
    object_(database_interface::DBFieldBase::TEXT, this, "object_name","in_grasp",true),
    grasp_config_(database_interface::DBFieldBase::TEXT, this, "grasp_js","in_grasp",true),
    f_idx_(database_interface::DBFieldBase::TEXT, this, "fingers","in_grasp",true),
    obj_pose_(database_interface::DBFieldBase::TEXT, this, "object_pose","in_grasp",true),
    des_obj_pose_(database_interface::DBFieldBase::TEXT, this, "desired_object_pose","in_grasp",true)
  {
    //table_name_=table_name;
    primary_key_field_=&id_;
    fields_.push_back(&object_);
    fields_.push_back(&grasp_config_);
    fields_.push_back(&f_idx_);
    fields_.push_back(&obj_pose_);
    fields_.push_back(&des_obj_pose_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    id_.setSequenceName("id_seq");
    id_.setWriteToDatabase(false);

  }

};
}
