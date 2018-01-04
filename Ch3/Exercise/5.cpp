#include <iostream>
using namespace std;

#include <Eigen/Core>

int main()
{
    Eigen::Matrix<double, 10, 10> m = Eigen::Matrix<double, 10, 10>::Random();
    cout << "Original matrix: \n" << m << endl;

    m.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    cout << "After: \n" << m << endl;

    return 0;

}
