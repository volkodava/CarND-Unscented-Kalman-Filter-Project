#include <fstream>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include "measurement_package.h"
#include "ukf.h"

using namespace std;

class DataHandler {
 private:
  string filepath;
  ofstream out;

 public:
  DataHandler(string filepath) {
    this->filepath = filepath;
  }

  void openFile() {
    out.open(filepath, ios::app);
    cout << "File " << filepath << " open for write." << endl;
  }

  void closeFile() {
    if (out.is_open()) {
      out.close();
      cout << "File " << filepath << " closed." << endl;
    }
  }

  void write(MeasurementPackage &meas_package, UKF &ukf,
             vector<VectorXd> &estimations, vector<VectorXd> &ground_truth,
             VectorXd &RMSE) {
    string type;
    stringstream st;
    st << "hello" << endl;
    cout << st.str();
  }
};
