#ifndef _Saturate_HH_
#define _Saturate_HH_

#include <stdint.h>
#include <Eigen/Dense>

/*! @brief This class is used to saturate an output variable
 */
class Saturate {
   public:
    Saturate();
    Saturate(int64_t size);  //!< class constructor
    ~Saturate();
    void setBounds(Eigen::MatrixXd bounds);
    Eigen::VectorXd saturate(Eigen::VectorXd unsaturatedStates);
    /*!@brief Saturates the given unsaturated states
       @param unsaturated States, a vector of the unsaturated states
       @return saturatedStates*/

   private:
    int64_t numStates;            //!< -- Number of states to generate noise for
    Eigen::MatrixXd stateBounds;  //!< -- one row for each state. lower bounds in left column, upper in right column
};

#endif /* _saturate_HH_ */
