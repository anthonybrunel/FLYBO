#ifndef BOUDINGMAP_HPP
#define BOUDINGMAP_HPP

#include <Eigen/Core>
template <typename Scalar>
class BoundingMap3D{
public:
    BoundingMap3D(){

    }
    BoundingMap3D(const Eigen::Matrix<Scalar,3,1> &min_i, const Eigen::Matrix<Scalar,3,1> &max_i):max_(max_i),min_(min_i){

    }

    float clamp(float n, float lower, float upper) {
      return std::max(lower, std::min(n, upper));
    }
    void clamp(Eigen::Matrix<Scalar,3,1> & pos_i){
        pos_i.x() = clamp(pos_i.x(),min_.x(),max_.x());
        pos_i.y() = clamp(pos_i.y(),min_.y(),max_.y());
        pos_i.z() = clamp(pos_i.z(),min_.z(),max_.z());

    }

    inline bool is_inside(const Eigen::Matrix<Scalar,3,1> &pt) const{
        return
                pt.x() > min_.x() &&
                pt.x() < max_.x() &&
                pt.y() > min_.y() &&
                pt.y() < max_.y() &&
                pt.z() > min_.z() &&
                pt.z() < max_.z();

    }

    Eigen::Matrix<Scalar,3,1> max_;
    Eigen::Matrix<Scalar,3,1> min_;

};
#endif // BOUDINGMAP_HPP
