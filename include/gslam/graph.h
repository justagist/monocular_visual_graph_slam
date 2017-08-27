/** @file graph.h (creates graph and performs graph optimisation using g2o library)
*
* @author  Ermano Arruda (eaa3@cin.ufpe.br)
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#ifndef _GSLAM_GRAPH_
#define _GSLAM_GRAPH_

#include <gslam/data_spot.h>
#include <map>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/optimizable_graph.h"

//#include "g2o/solvers/cholmod/linear_solver_cholmod.h" // if using cholmod

#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/core/robust_kernel_impl.h"

namespace gSlam {

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
//typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver; // if using cholmod
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;



class GraphOptimizer {
public:

    typedef customtype::MapTransformSE3 Vertices;
    typedef DataLink3D::Links Links;

    GraphOptimizer();
    ~GraphOptimizer();

    void addVertex(DataSpot3D::DataSpot3DPtr data_spot);

    void addLink(DataLink3D::DataLinkPtr link);

    void getPoses(customtype::SeqTransformSE3& out_poses);

    void updateVertices();

    void updateVertices(DataSpot3D::DataSpotMap& data_spots);

    void setVertices(const DataSpot3D::DataSpotMap& data_spots);

    void optimizeGraph(int iter = 10);

    int getVerticesSize(){
        return vertices_.size();
    }
    int getLinksSize(){
        return links_.size();
    }

    void init();
    void release();
protected:



    void addVertex(customtype::Identifier vertex_id,const customtype::TransformSE3& pose);

    void addLink(customtype::Identifier from_id, customtype::Identifier to_id,const customtype::TransformSE3& rel_transform,const customtype::InformationMatrix3D& inf_matrix = customtype::InformationMatrix3D::Identity());

public:
    Vertices vertices_;
    Links links_;

    g2o::SparseOptimizer optimizer_;
    //g2o::BlockSolverX::LinearSolverType * linear_solver_;
    //g2o::BlockSolverX * block_solver_;
    SlamBlockSolver*  block_solver_;

    g2o::RobustKernelHuber robust_kernel_;

    bool fixed_iter_;

};


};


#endif // _GSLAM_GRAPH_
