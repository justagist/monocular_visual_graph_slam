
#include "gslam/data_pool.h"
#include "gslam/fabmap.h"

namespace gSlam
{


DataPool::DataPool() : loop_count_far_(0), loop_count_near_(0) {}

void DataPool::addDataSpot(DataSpot3D::DataSpot3DPtr data_spot_ptr)

{
    std::cout << " Adding data spot " << std::endl;

    std::cout << " checking loop closure " << std::endl;

    //Loop closure constraints
    int new_id, loop_id;
    fabmap_.compareAndAdd(data_spot_ptr, new_id, loop_id);

    std::cout << " LoopID: " << loop_id << std::endl;
    // if( loop_id >= 0 && new_id > 0 ){}  // TODO: loop closure constraint


}

} // namespace gSlam