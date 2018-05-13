
#include <string>
#include <vector>

#include <database_interface/db_class.h>
#include <sensor_msgs/JointState.h>

// custom db classes:
#include <in_hand_manipulation_data/db_class/in_grasp.h>
#include <in_hand_manipulation_data/db_class/grasp.h>
#include <in_hand_manipulation_data/db_class/object_meta.h>
#include <in_hand_manipulation_data/db_class/finger_relocation.h>

#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

using namespace in_hand_manipulation_data;


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
  
  // Connect to database tables:
  // TODO: find a way to create a table if not already present, currently these tables need to be created using sql. pgadmin3 can be used if you need a gui.
  
  std::vector< boost::shared_ptr<inGrasp> > in_grasp;
  std::vector< boost::shared_ptr<Grasp> > grasp;
  std::vector< boost::shared_ptr<fingerRelocation> > finger_relocation;
  std::vector< boost::shared_ptr<objectMeta> > object_meta;
  

  if (!database.getList(in_grasp))
  {
    std::cerr << "In-grasp table not present, creating new....\n";
  }
  database.getList(in_grasp);
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
