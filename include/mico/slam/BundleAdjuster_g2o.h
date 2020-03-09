//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef MICO_BASE_MAP3D_BUNDLEADJUSTERG2O_H_
#define MICO_BASE_MAP3D_BUNDLEADJUSTERG2O_H_

#ifdef USE_G2O
    #include <g2o/config.h>
    #include <g2o/core/sparse_optimizer.h>
    #include <g2o/core/block_solver.h>
    #include <g2o/core/solver.h>
    #include <g2o/core/robust_kernel_impl.h>
    #include <g2o/core/optimization_algorithm_levenberg.h>
    #include <g2o/solvers/dense/linear_solver_dense.h>
    #include <g2o/types/icp/types_icp.h>
    #include <g2o/solvers/structure_only/structure_only_solver.h>
    #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
    #include <g2o/types/sba/types_six_dof_expmap.h>
    #include <g2o/core/block_solver.h>
#endif

#include <mico/slam/BundleAdjuster.h>
#include <Eigen/Eigen>


namespace mico {
  template <typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    class BundleAdjuster_g2o: public mico::BundleAdjuster<PointType_, DebugLevel_, OutInterface_>{
    public:
        BundleAdjuster_g2o();

            
    protected:
        virtual void appendCamera(int _id, Eigen::Matrix4f _pose, cv::Mat _intrinsics = cv::Mat(), cv::Mat _distcoeff = cv::Mat());
        virtual void appendPoint(int _id, Eigen::Vector3f _position);
        virtual void appendProjection(int _idCamera, int _idPoint, cv::Point2f _projection, cv::Mat _intrinsics = cv::Mat(), cv::Mat _distcoeff = cv::Mat());
        virtual void reserveData(int _cameras, int _words);
        virtual void fitSize(int _cameras, int _words);
        virtual void cleanData();
        virtual void checkData();
        virtual bool doOptimize();
        
        virtual void recoverCamera(int _id, Eigen::Matrix4f &_pose, cv::Mat &_intrinsics, cv::Mat &_distcoeff);
        virtual void recoverPoint(int _id, Eigen::Vector3f &_position);
        virtual bool isProjectionEnabled(int _idCamera, int _idPoint);

        
    #ifdef USE_G2O
        g2o::SparseOptimizer *mOptimizer = nullptr;
        std::vector<g2o::EdgeProjectXYZ2UV*> mEdgesList;
    #endif
        std::map<int,int> mPointIdToGraphId;
        std::map<int,int> mGraphIdToPointId;
        std::map<int,int> mCameraIdToGraphId;
        std::map<int,int> mGraphIdToCameraId;
        int mCurrentGraphID = 0;

        std::map<int,std::map<int,bool>> mEdgeToRemove; // {idcam:{idpoint, true|false}}
    };
}

#include <mico/slam/BundleAdjuster_g2o.inl>

#endif
