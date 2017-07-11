

#include "gslam/graphslam.h"

namespace gSlam
{

GrSLAM::GrSLAM() : map_correction_(customtype::TransformSE3::Identity()), optimize_now_(false), keep_opt_thread_alive_(true), optimize_near_(false), optimize_far_(false) {}

GrSLAM::~GrSLAM(){}

// TODO: DEFINE DESTRUCTOR

} // namespace gSlam