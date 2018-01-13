#include <fstream>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include "measurement_package.h"
#include "ukf.h"

using namespace std;

class FileNotOpenedException : public runtime_error {
 public:

  FileNotOpenedException(string filepath)
      : runtime_error("File " + filepath + " not opened for write.") {
  }
};

class DataHandler {
 private:
  string filepath;
  bool headerWritten;
  ofstream out;

  void writeHeader() {
    if (!out.is_open()) {
      throw FileNotOpenedException(filepath);
    }

    // column names for output file
    out << "pos1_est" << "\t";
    out << "pos2_est" << "\t";
    out << "vel_est" << "\t";
    out << "yaw_angle_est" << "\t";
    out << "yaw_rate_est" << "\t";

    out << "px_measured" << "\t";
    out << "py_measured" << "\t";

    out << "px_gt" << "\t";
    out << "py_gt" << "\t";
    out << "vx_gt" << "\t";
    out << "vy_gt" << "\t";

    out << "rmse_x" << "\t";
    out << "rmse_y" << "\t";
    out << "rmse_vx" << "\t";
    out << "rmse_vy" << "\t";

    out << "NIS" << "\n";
}

 public:
  DataHandler(string filepath) {
    this->filepath = filepath;
    this->headerWritten = false;
  }

  void openFile() {
    out.open(filepath, ios::out);
    cout << "File " << filepath << " open for write." << endl;
  }

  void closeFile() {
    if (out.is_open()) {
      out.close();
      cout << "File " << filepath << " closed." << endl;
    }
  }

  void write(MeasurementPackage &meas_package,
             UKF &ukf,
             VectorXd &gt_values,
             VectorXd &RMSE) {
    if (!out.is_open()) {
      throw FileNotOpenedException(filepath);
    }

    if (!headerWritten) {
      writeHeader();
      headerWritten = true;
    }

    string type;
    stringstream st;

    st << ukf.x_(0) << "\t";  // pos1_est
    st << ukf.x_(1) << "\t";  // pos2_est
    st << ukf.x_(2) << "\t";  // vel_est
    st << ukf.x_(3) << "\t";  // yaw_angle_est
    st << ukf.x_(4) << "\t";  // yaw_rate_est

    double px_measured = 0.0;
    double py_measured = 0.0;
    double NIS = 0.0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // range (radial distance from origin to the tracking object such as pedestrian, car, etc.)
      double rho = meas_package.raw_measurements_[0];
      // bearing (angle between rho and x)
      double phi = meas_package.raw_measurements_[1];
      // Convert radar from polar to cartesian coordinates
      px_measured = rho * cos(phi);
      py_measured = rho * sin(phi);
      // Normalized innovation squared (NIS)
      NIS = ukf.NIS_radar_;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      px_measured = meas_package.raw_measurements_[0];
      py_measured = meas_package.raw_measurements_[1];
      // Normalized innovation squared (NIS)
      NIS = ukf.NIS_laser_;
    }

    if (fabs(px_measured) < XY_THRESH) {
      px_measured = XY_THRESH;
    }
    if (fabs(py_measured) < XY_THRESH) {
      py_measured = XY_THRESH;
    }

    st << px_measured << "\t";  // px_measured
    st << py_measured << "\t";  // py_measured

    double px_gt = gt_values(0);
    double py_gt = gt_values(1);
    double vx_gt = gt_values(2);
    double vy_gt = gt_values(3);

    st << px_gt << "\t";  // px_gt (ground truth)
    st << py_gt << "\t";  // py_gt (ground truth)
    st << vx_gt << "\t";  // vx_gt (ground truth)
    st << vy_gt << "\t";  // vy_gt (ground truth)

    double rmse_x = RMSE(0);
    double rmse_y = RMSE(1);
    double rmse_vx = RMSE(2);
    double rmse_vy = RMSE(3);

    st << rmse_x << "\t";   // rmse_x
    st << rmse_y << "\t";   // rmse_y
    st << rmse_vx << "\t";  // rmse_vx
    st << rmse_vy << "\t";  // rmse_vy

    st << NIS << "\t";  // NIS (Normalized innovation squared)
    st << endl;

    out << st.str();
  }
};
