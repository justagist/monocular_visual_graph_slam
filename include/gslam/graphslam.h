#ifndef __GRAPH_SLAM__
#define __GRAPH_SLAM__

#include "typedefs.h"

namespace gSlam
{

class GrSLAM
{

public:

    typedef boost::shared_ptr<GrSLAM> Ptr;

    GrSLAM();
    ~GrSLAM();

private:

    bool optimize_now_, optimize_near_, optimize_far_, keep_opt_thread_alive_;

    customtype::TransformSE3 map_correction_;




};// class GrSLAM

}//namespace gSlam


#endif // __GRAPH_SLAM__