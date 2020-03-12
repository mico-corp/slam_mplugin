//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifndef MICO_BASE_MAP3D_BUNDLEADJUSTER_H_
#define MICO_BASE_MAP3D_BUNDLEADJUSTER_H_

#include <mico/slam/Dataframe.h>

#include <mico/slam/utils/LogManager.h>

namespace mico {
  template <typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    class BundleAdjuster : public LoggableInterface<DebugLevel_, OutInterface_>{
    public:
        void sequence(std::map<int,std::shared_ptr<Dataframe<PointType_>>> &_dataframes);
        
        bool optimize();

        // ---- Getters ----
        /// \brief Get minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \return minimum error.
        double      minError       () const;

        /// \brief Get number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \return iterations.
        unsigned    iterations     () const;

        /// \brief Get minumim number of times that a points needs to be observed to be used in the Bundle Adjustment.
        /// \return number of aparitions.
        unsigned    minAparitions  () const;

        unsigned    minWords       () const;

        // ---- Setters ----
        /// \brief Set minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \param _error: minimum error.
        void minError         (double _error);

        /// \brief Set number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \param _iterations iterations.
        void iterations       (unsigned _iterations);

        /// \brief Set minumim number of times that a points needs to be observed to be used in the Bundle Adjustment.
        /// \param _aparitions: number of aparitions.
        void minAparitions    (unsigned _aparitions);

        void minWords       (unsigned _minWords) ;
    protected:
        bool prepareDataClusterframes();

        virtual void cleanData() = 0;
        virtual void cleanDataParent();

        virtual void appendCamera(int _id, Eigen::Matrix4f _pose, cv::Mat _intrinsics = cv::Mat(), cv::Mat _distcoeff = cv::Mat()) = 0;

        virtual void appendPoint(int _id, Eigen::Vector3f _position) = 0;

        virtual void appendProjection(int _idCamera, int _idPoint, cv::Point2f _projection, cv::Mat _intrinsics = cv::Mat(), cv::Mat _distcoeff = cv::Mat()) = 0;

        virtual void reserveData(int _cameras, int _words) = 0;

        virtual void fitSize(int _cameras, int _words) = 0;

        virtual void checkData() = 0;

        virtual bool doOptimize() = 0;

        virtual void recoverCamera(int _id, Eigen::Matrix4f &_pose, cv::Mat &_intrinsics, cv::Mat &_distcoeff) = 0;

        virtual void recoverPoint(int _id, Eigen::Vector3f &_position)= 0;

        virtual bool isProjectionEnabled(int _idCamera, int _idPoint) = 0;

    protected:
        // Parameters of Bundle Adjustment.
        double      mBaMinError = 1e-10;
        unsigned    mBaIterations = 500;
        unsigned    mBaMinAparitions = 5;
        unsigned    mMinWords = 10;

    protected:
        std::map<int, std::shared_ptr<Dataframe<PointType_>>> mDataframes;
        std::map<int,bool> mUsedWordsMap;   // 666 placed  here to prevent weird memory crash.

        std::map<int,int> mClustersIdToCameraId;
        std::map<int,int> mCameraIdToClustersId;
        std::map<int,int> mWordIdToPointId;
        std::map<int,int> mPointIdToWordId;

    public: // 666 temporary public
        std::map<int, std::shared_ptr<Word<PointType_>>> mGlobalUsedWordsRef;

    };
}   // namespace mico 

#include <mico/slam/BundleAdjuster.inl>

#endif //RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
