#include <gslam/graph.h>


namespace gSlam {

GraphOptimizer::GraphOptimizer() : fixed_iter_(false) {


}

GraphOptimizer::~GraphOptimizer(){

}


void GraphOptimizer::init(){

    // Apply g2o optimization
    //new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    // create the linear solver
    SlamLinearCSparseSolver* linear_solver_ = new SlamLinearCSparseSolver();

    // create the block solver on top of the linear solver
    block_solver_ =  new SlamBlockSolver(linear_solver_);//new g2o::BlockSolverX(linear_solver_);


    // create the algorithm to carry out the optimization
    //g2o::OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    g2o::OptimizationAlgorithmLevenberg* optimization_algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver_);
    optimization_algorithm->setUserLambdaInit(0.001); //0.001

    // create the optimizer to load the data and carry out the optimization
    optimizer_.setVerbose(false);
    optimizer_.setAlgorithm(optimization_algorithm);


    //Add vertices and constraints

    //Adding vertices
    for(Vertices::iterator it = vertices_.begin(); it != vertices_.end(); it++)
        addVertex(it->first,it->second);

    //Adding links (constraints)
    for(Links::iterator it = links_.begin(); it != links_.end(); it++)
        addLink(it->second->from_id_, it->second->to_id_, it->second->transform_, it->second->inf_matrix_);


}

void GraphOptimizer::release(){

    //freeing the graph memory
    optimizer_.clear();

    // destroy all the singletons
    g2o::Factory::destroy();
    g2o::OptimizationAlgorithmFactory::destroy();
    g2o::HyperGraphActionLibrary::destroy();

}

void GraphOptimizer::addVertex(DataSpot3D::DataSpot3DPtr data_spot) {
    vertices_.insert(std::make_pair(data_spot->getId(),data_spot->getPose()));
    Links& links = data_spot->getLinks();
    for(Links::iterator it = links.begin(); it != links.end(); it++){
        addLink(it->second);
    }
}

void GraphOptimizer::addVertex(customtype::Identifier vertex_id,const customtype::TransformSE3& pose){


    Eigen::Isometry3d pose_isometry;
    Eigen::Affine3d a = pose;
    pose_isometry.translation() = a.translation();
    pose_isometry.linear() = a.rotation();
    //v3->setEstimate(pose);
    //vertex = v3;


    // set up node
    g2o::VertexSE3 *vc = new g2o::VertexSE3();

    vc->setEstimate(pose_isometry);
    vc->setMarginalized(false);


    vc->setId(vertex_id); // vertex id

    // set first pose fixed
    if (vertex_id==0){
        vc->setFixed(true);
    }

    // add to optimizer
    optimizer_.addVertex(vc);

}

void GraphOptimizer::addLink(customtype::Identifier from_id, customtype::Identifier to_id,const customtype::TransformSE3& rel_transform,const customtype::InformationMatrix3D& inf_matrix){


    Eigen::Affine3d a = rel_transform;
    Eigen::Isometry3d constraint;
    constraint.translation() = a.translation();
    constraint.linear() = a.rotation();
    //Eigen::Quaternion<double> q(a.rotation());

    g2o::EdgeSE3* edge = new g2o::EdgeSE3;
    edge->vertices()[0] = optimizer_.vertex(from_id);
    edge->vertices()[1] = optimizer_.vertex(to_id);
    edge->setMeasurement(constraint);
    //robust_kernel_.setDelta(0.1);
    //edge->setRobustKernel(&robust_kernel_);

    //Set the information matrix to identity
    edge->setInformation(inf_matrix);


    optimizer_.addEdge(edge);

}

void GraphOptimizer::addLink(DataLink3D::DataLinkPtr link){

    links_.insert(std::make_pair(link->from_id_, link));

    //this->addLink(link.from_id_, link.to_id_,link.transform_, link.inf_matrix_);

}

void GraphOptimizer::updateVertices(){

    for(Vertices::iterator it = vertices_.begin(); it != vertices_.end(); it++)
    {
        //Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(it->first));
        double optimizedPoseQuaternion[7];
        vertex->getEstimateData(optimizedPoseQuaternion);

        customtype::TransformSE3 optimizedPose;
        static double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

        qx=optimizedPoseQuaternion[3];
        qy=optimizedPoseQuaternion[4];
        qz=optimizedPoseQuaternion[5];
        qr=optimizedPoseQuaternion[6];
        qx2=qx*qx;
        qy2=qy*qy;
        qz2=qz*qz;
        qr2=qr*qr;

        optimizedPose(0,0)=qr2+qx2-qy2-qz2;
        optimizedPose(0,1)=2*(qx*qy-qr*qz);
        optimizedPose(0,2)=2*(qz*qx+qr*qy);
        optimizedPose(0,3)=optimizedPoseQuaternion[0];
        optimizedPose(1,0)=2*(qx*qy+qr*qz);
        optimizedPose(1,1)=qr2-qx2+qy2-qz2;
        optimizedPose(1,2)=2*(qy*qz-qr*qx);
        optimizedPose(1,3)=optimizedPoseQuaternion[1];
        optimizedPose(2,0)=2*(qz*qx-qr*qy);
        optimizedPose(2,1)=2*(qy*qz+qr*qx);
        optimizedPose(2,2)=qr2-qx2-qy2+qz2;
        optimizedPose(2,3)=optimizedPoseQuaternion[2];
        optimizedPose(3,0)=0;
        optimizedPose(3,1)=0;
        optimizedPose(3,2)=0;
        optimizedPose(3,3)=1;

        //Set the optimized pose to the vector of poses
        it->second=optimizedPose;
    }
}

void GraphOptimizer::updateVertices(DataSpot3D::DataSpotMap& data_spots){

    for(DataSpot3D::DataSpotMap::iterator it = data_spots.begin(); it != data_spots.end(); it++)
    {
        //Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(it->first));
        double optimizedPoseQuaternion[7];
        vertex->getEstimateData(optimizedPoseQuaternion);

        customtype::TransformSE3 optimizedPose;
        static double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

        qx=optimizedPoseQuaternion[3];
        qy=optimizedPoseQuaternion[4];
        qz=optimizedPoseQuaternion[5];
        qr=optimizedPoseQuaternion[6];
        qx2=qx*qx;
        qy2=qy*qy;
        qz2=qz*qz;
        qr2=qr*qr;

        optimizedPose(0,0)=qr2+qx2-qy2-qz2;
        optimizedPose(0,1)=2*(qx*qy-qr*qz);
        optimizedPose(0,2)=2*(qz*qx+qr*qy);
        optimizedPose(0,3)=optimizedPoseQuaternion[0];
        optimizedPose(1,0)=2*(qx*qy+qr*qz);
        optimizedPose(1,1)=qr2-qx2+qy2-qz2;
        optimizedPose(1,2)=2*(qy*qz-qr*qx);
        optimizedPose(1,3)=optimizedPoseQuaternion[1];
        optimizedPose(2,0)=2*(qz*qx-qr*qy);
        optimizedPose(2,1)=2*(qy*qz+qr*qx);
        optimizedPose(2,2)=qr2-qx2-qy2+qz2;
        optimizedPose(2,3)=optimizedPoseQuaternion[2];
        optimizedPose(3,0)=0;
        optimizedPose(3,1)=0;
        optimizedPose(3,2)=0;
        optimizedPose(3,3)=1;

        //Set the optimized pose to the vector of poses
        it->second->setPose(optimizedPose);
    }

}

void GraphOptimizer::getPoses(customtype::SeqTransformSE3& out_poses){

    out_poses.clear();
    out_poses.resize(vertices_.size());

    int pose_idx = 0;
    for(Vertices::iterator it = vertices_.begin(); it != vertices_.end(); it++)
    {
        //Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(it->first));
        double optimizedPoseQuaternion[7];
        vertex->getEstimateData(optimizedPoseQuaternion);

        customtype::TransformSE3 optimizedPose;
        static double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

        qx=optimizedPoseQuaternion[3];
        qy=optimizedPoseQuaternion[4];
        qz=optimizedPoseQuaternion[5];
        qr=optimizedPoseQuaternion[6];
        qx2=qx*qx;
        qy2=qy*qy;
        qz2=qz*qz;
        qr2=qr*qr;

        optimizedPose(0,0)=qr2+qx2-qy2-qz2;
        optimizedPose(0,1)=2*(qx*qy-qr*qz);
        optimizedPose(0,2)=2*(qz*qx+qr*qy);
        optimizedPose(0,3)=optimizedPoseQuaternion[0];
        optimizedPose(1,0)=2*(qx*qy+qr*qz);
        optimizedPose(1,1)=qr2-qx2+qy2-qz2;
        optimizedPose(1,2)=2*(qy*qz-qr*qx);
        optimizedPose(1,3)=optimizedPoseQuaternion[1];
        optimizedPose(2,0)=2*(qz*qx-qr*qy);
        optimizedPose(2,1)=2*(qy*qz+qr*qx);
        optimizedPose(2,2)=qr2-qx2-qy2+qz2;
        optimizedPose(2,3)=optimizedPoseQuaternion[2];
        optimizedPose(3,0)=0;
        optimizedPose(3,1)=0;
        optimizedPose(3,2)=0;
        optimizedPose(3,3)=1;

        //Set the optimized pose to the vector of poses
        out_poses[pose_idx]=optimizedPose;
    }

}

void GraphOptimizer::setVertices(const DataSpot3D::DataSpotMap& data_spots){

    // TODO: set the minimum spanning tree from data_spots
    for( auto it = data_spots.begin(); it != data_spots.end(); it++){
        addVertex(it->second);
    }

}




void GraphOptimizer::optimizeGraph(int iter){


    init();

    optimizer_.initializeOptimization();
    if( fixed_iter_ ){

        //optimizer_.save("before_optimization.g2o");
        //cerr << "Optimizing" << endl;



        optimizer_.optimize(iter);
        //cerr << "done." << endl;

        //optimizer_.save("after_optimization.g2o");

        //updateVertices();

    }
    else{

        double prev_chi2;
        double chi2 = std::numeric_limits<double>::max();
        int currentIt = 0;
        do {
            prev_chi2 = chi2; //chi2 is numeric_limits::max() in first iteration
            currentIt += optimizer_.optimize(iter);//optimize 10 iterations per step
            optimizer_.computeActiveErrors();
            chi2 = optimizer_.chi2();
        } while(chi2/prev_chi2 < (1.0 - 0.01));//e.g.  999/1000 < (1.0 - 0.01) => 0.999 < 0.99

        //updateVertices();

    }





    // release();
}

};
