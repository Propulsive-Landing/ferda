#include "Navigation.hpp"

Eigen::Matrix<double, 3, 1> Navigation::GetNavigation()
{
    Eigen::Matrix<double, 3, 1> m = {      // construct a 2x2 matrix
      {1},     // first row
      {3},      // second row
      {5},      // Third row
    };
   
   return m;

}

void Navigation::UpdateNavigation(Eigen::Matrix<double, 4, 1> updates){
    
}
