#ifndef __SLAM_UTILS__
#define __SLAM_UTILS__

#include "gslam/typedefs.h"


namespace gSlam
{

namespace slam_utils
{
    
    customtype::TransformSE3 estimateRelativeTransformBtwnProjections(customtype::ProjMatType src_prj, customtype::ProjMatType tgt_prj);

} // namespace slam_utils

} // namespace gSlam


#endif // __SLAM_UTILS__