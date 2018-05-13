#include <database_interface/db_class.h>
namespace in_hand_manipulation_data
{
class objectMeta : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> obj_name_;
  database_interface::DBField<std::string> obj_stl_;
  database_interface::DBField<std::string> obj_coll_;
  objectMeta():
    id_(database_interface::DBFieldBase::TEXT, this, "id","object_meta",true),
    obj_name_(database_interface::DBFieldBase::TEXT, this, "object_name","object_meta",true),
    obj_stl_(database_interface::DBFieldBase::TEXT, this, "stl_file","object_meta",true),
    obj_coll_(database_interface::DBFieldBase::TEXT, this, "coll_file","object_meta",true)
  {
    primary_key_field_=&id_;
    fields_.push_back(&obj_name_);
    fields_.push_back(&obj_stl_);
    fields_.push_back(&obj_coll_);
    
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);


  }
};
}
