
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt nis_lidar.txt nis_radar.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc != 5) {
    cerr << usage_instructions << endl;
  } 
  else  
	{
    has_valid_args = true;
  } 
  
  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name) {
  
	if (!in_file.is_open())
	{
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) 
	{
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) 
{

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str());

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str());
	
	string out_nis_lidar_ = argv[3];
  ofstream out_file_nis_lidar_(out_nis_lidar_.c_str());
	
	string out_nis_radar_ = argv[4];
  ofstream out_file_nis_radar_(out_nis_radar_.c_str());
	
  //check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) 
	{
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) 
		{
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } 
    else if (sensor_type.compare("R") == 0) 
		{
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

      // read ground truth data to compare later
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      gt_package.gt_values_ = VectorXd(4);
      gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);
  }

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)

  size_t number_of_measurements = measurement_pack_list.size();

  // column names for output file
  out_file_ << "px" << ",";
  out_file_ << "py" << ",";
  out_file_ << "vx" << ",";
  out_file_ << "vy" << ",";
	
  out_file_ << "px_measured" << ",";
  out_file_ << "py_measured" << ",";
	
  out_file_ << "gt_px" << ",";
  out_file_ << "gt_py" << ",";
  out_file_ << "gt_vx" << ",";
  out_file_ << "gt_vy" << "\n";
  	


  for (size_t k = 0; k < number_of_measurements; ++k)
	{
    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
		double x_estimate_ = ukf.x_(0);
    double y_estimate_ = ukf.x_(1);
    double vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    double vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
		
    out_file_ << x_estimate_ << ","; // pos1 - est
    out_file_ << y_estimate_ << ","; // pos2 - est
    out_file_ << vx_estimate_ << ","; // vel_x -est
    out_file_ << vy_estimate_ << ","; // vel_y -est
    

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
		{
      // output the estimation

      // p1 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << ",";

      // p2 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << ",";
			
			out_file_nis_lidar_ << ukf.nis_lidar << "\n";
			
    } 
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
		{
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
			
      out_file_ << ro * cos(phi) << ","; // p1_meas
      out_file_ << ro * sin(phi) << ","; // p2_meas
			
			out_file_nis_radar_ << ukf.nis_radar << "\n";
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << ",";
    out_file_ << gt_pack_list[k].gt_values_(1) << ",";
    out_file_ << gt_pack_list[k].gt_values_(2) << ",";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

		

    // convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);
    
    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);

  }

  out_file_nis_lidar_ << ukf.nis_lidar <<"\n";
	out_file_nis_radar_ << ukf.nis_radar <<"\n";
  
  cout <<"nis lidar value " << ukf.nis_lidar <<"\n";
	cout <<"nis radar value " << ukf.nis_radar <<"\n";
  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << '\n' << tools.CalculateRMSE(estimations, ground_truth) << endl;


  cout << "Done!" << endl;
  return 0;
}
