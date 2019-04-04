
#include <string>
#include <vector>

#include <database_interface/db_class.h>
#include <sensor_msgs/JointState.h>
class inGrasp : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> object_;
  database_interface::DBField <std::vector<double> > grasp_config_;
  database_interface::DBField <std::vector<double> > obj_pose_;

  inGrasp():
    id_(database_interface::DBFieldBase::TEXT, this, "id","in_grasp",true),
    object_(database_interface::DBFieldBase::TEXT, this, "object_name","in_grasp",true),
    grasp_config_(database_interface::DBFieldBase::TEXT, this, "grasp_js","in_grasp",true),
    obj_pose_(database_interface::DBFieldBase::TEXT, this, "object_pose","in_grasp",true)

  {
    primary_key_field_=&id_;
    fields_.push_back(&object_);
    fields_.push_back(&grasp_config_);
    fields_.push_back(&obj_pose_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

  }

};

class objectMeta : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> obj_name_;
  database_interface::DBField<std::string> obj_stl_;
  database_interface::DBField<std::string> obj_bvox_;
  objectMeta():
    id_(database_interface::DBFieldBase::TEXT, this, "id","object_meta",true),
    obj_name_(database_interface::DBFieldBase::TEXT, this, "object_name","object_meta",true),
    obj_stl_(database_interface::DBFieldBase::TEXT, this, "stl_file","object_meta",true),
    obj_bvox_(database_interface::DBFieldBase::TEXT, this, "bvox_file","object_meta",true)
  {
    primary_key_field_=&id_;
    fields_.push_back(&obj_name_);
    fields_.push_back(&obj_stl_);
    fields_.push_back(&obj_bvox_);
    
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);


  }
};


#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

int main(int argc, char **argv)
{
  database_interface::PostgresqlDatabase 
    database("localhost", "5432",
	     "bala", "hi", "in_hand");
  if (!database.isConnected())
  {
    std::cerr << "Database failed to connect \n";
    return -1;
  }
  std::cerr << "Database connected successfully \n";

  std::vector< boost::shared_ptr<inGrasp> > in_grasp;
  if (!database.getList(in_grasp))
  {
    std::cerr << "Failed to get in_grasp data\n";
    return -1;
  }
  std::cerr << "Retrieved " << in_grasp.size() << " in-grasp datasets \n";
  
  /*
  std::cerr << "Students: \n";
  for (size_t i=0; i<students.size(); i++)
  {
    std::cerr << students[i]->student_last_name_.data() 
	      << ", " 
	      << students[i]->student_first_name_.data()
	      << ": \n";
    std::cerr << "  GPA: " << students[i]->student_gpa_.data() << "\n";
    std::cerr << "  Major(s): ";
    for (size_t j=0; j<students[i]->student_majors_.data().size(); j++)
    {
      if (j!=0) std::cerr << ", ";
      std::cerr << students[i]->student_majors_.data().at(j);
    }
    std::cerr << "\n";
  }

  std::vector< boost::shared_ptr<Grade> > grades;
  std::string where_clause("student_id=1");
  database.getList(grades, where_clause);
  std::cerr << "Student with id 1 has " << grades.size() << " grade(s) on record\n";

  grades[0]->grade_grade_.data() = 2.5;
  if (!database.saveToDatabase( &(grades[0]->grade_grade_) ) )
    std::cerr << "Failed to modify grade\n";
  else
    std::cerr << "Grade modified successfully\n";

  //we have forgotten a grade
  grades[0]->grade_grade_.data() = 0.0;
  //reload it from the database
  if (!database.loadFromDatabase( &(grades[0]->grade_grade_) ) )
    std::cerr << "Failed to load grade field from database\n";
  else
    std::cerr << "Grade field (re)loaded, its value is "
              << grades[0]->grade_grade_.data() 
              << "\n";

  Grade new_grade;
  new_grade.student_id_.data() = 1;
  new_grade.grade_subject_.data() = "astrology";
  new_grade.grade_grade_.data() = 4.0;
  new_grade.grade_id_.data() = 5;
  if (!database.insertIntoDatabase(&new_grade)) 
    std::cerr << "Grade insertion failed\n";
  else 
    std::cerr << "Grade insertion succeeded\n";

  //get the students, but photos will not be loaded by default
  std::vector< boost::shared_ptr<StudentWithPhoto> > students_with_photos;
  database.getList(students_with_photos);

  //now get the photo for the first student retrieved
  database.loadFromDatabase( &(students_with_photos[0]->student_photo_) );
  std::cerr << "The photo has " 
	    << students_with_photos[0]->student_photo_.data().size() 
	    << " bytes \n";

  //insert a grade with automatic key generation
  GradeWithSequence new_grade_seq;
  new_grade_seq.student_id_.data() = 2;
  new_grade_seq.grade_subject_.data() = "mythology";
  new_grade_seq.grade_grade_.data() = 4.0;
  database.insertIntoDatabase(&new_grade_seq);
  std::cerr << "The newly inserted grade was assigned grade_id=" 
	    << new_grade_seq.grade_id_.data() 
	    << "\n";
  */
  return 0;
}
