/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <map>

#include <pcl/common/point_operators.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief ZhedaSampleConsensusModelRegistration defines a model for Point-To-Point registration outlier rejection.
    * \author Radu Bogdan Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class ZhedaSampleConsensusModelRegistration : public SampleConsensusModel<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::model_name_;
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModel<PointT>::error_sqr_dists_;
      using SampleConsensusModel<PointT>::isModelValid;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<ZhedaSampleConsensusModelRegistration> Ptr;

      /** \brief Constructor for base ZhedaSampleConsensusModelRegistration.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      ZhedaSampleConsensusModelRegistration (const PointCloudConstPtr &cloud, 
                                        bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random)
        , target_ ()
        , indices_tgt_ ()
        , correspondences_ ()
        , sample_dist_thresh_ (0)
      {
        // Call our own setInputCloud
        setInputCloud (cloud);
        model_name_ = "ZhedaSampleConsensusModelRegistration";
        sample_size_ = 3;
        model_size_ = 16;
      }

      /** \brief Constructor for base ZhedaSampleConsensusModelRegistration.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      ZhedaSampleConsensusModelRegistration (const PointCloudConstPtr &cloud,
                                        const std::vector<int> &indices,
                                        bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random)
        , target_ ()
        , indices_tgt_ ()
        , correspondences_ ()
        , sample_dist_thresh_ (0)
      {
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (cloud, indices);
        model_name_ = "ZhedaSampleConsensusModelRegistration";
        sample_size_ = 3;
        model_size_ = 16;
      }
      
      /** \brief Empty destructor */
      ~ZhedaSampleConsensusModelRegistration () {}

      /** \brief Provide a pointer to the input dataset
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      setInputCloud (const PointCloudConstPtr &cloud) override
      {
        SampleConsensusModel<PointT>::setInputCloud (cloud);
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (cloud);
      }

      /** \brief Set the input point cloud target.
        * \param[in] target the input point cloud target
        */
      inline void
      setInputTarget (const PointCloudConstPtr &target)
      {
        target_ = target;
        indices_tgt_.reset (new std::vector<int>);
        // Cache the size and fill the target indices
        int target_size = static_cast<int> (target->size ());
        indices_tgt_->resize (target_size);

        for (int i = 0; i < target_size; ++i)
          (*indices_tgt_)[i] = i;
        computeOriginalIndexMapping ();
      }

      /** \brief Set the input point cloud target.
        * \param[in] target the input point cloud target
        * \param[in] indices_tgt a vector of point indices to be used from \a target
        */
      inline void
      setInputTarget (const PointCloudConstPtr &target, const std::vector<int> &indices_tgt)
      {
        target_ = target;
        indices_tgt_.reset (new std::vector<int> (indices_tgt));
        computeOriginalIndexMapping ();
      }

      /** \brief Compute a 4x4 rigid transformation matrix from the samples given
        * \param[in] samples the indices found as good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool
      computeModelCoefficients (const std::vector<int> &samples,
                                Eigen::VectorXf &model_coefficients) const override;

      /** \brief Compute all distances from the transformed points to their correspondences
        * \param[in] model_coefficients the 4x4 transformation matrix
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const override;

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the 4x4 transformation matrix
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param[out] inliers the resultant model inliers
        */
      void
      selectWithinDistance (const Eigen::VectorXf &model_coefficients,
                            const double threshold,
                            std::vector<int> &inliers) override;

      /** \brief Count all the points which respect the given model coefficients as inliers.
        *
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      int
      countWithinDistance (const Eigen::VectorXf &model_coefficients,
                           const double threshold) const override;

      /** \brief Recompute the 4x4 transformation using the given inlier set
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed transformation
        */
      void
      optimizeModelCoefficients (const std::vector<int> &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients) const override;

      void
      projectPoints (const std::vector<int> &,
                     const Eigen::VectorXf &,
                     PointCloud &, bool = true) const override
      {
      };

      bool
      doSamplesVerifyModel (const std::set<int> &,
                            const Eigen::VectorXf &,
                            const double) const override
      {
        return (false);
      }

      /** \brief Return an unique id for this model (SACMODEL_REGISTRATION). */
      inline pcl::SacModel
      getModelType () const override { return (SACMODEL_REGISTRATION); }

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check if a sample of indices results in a good sample of points
        * indices.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood (const std::vector<int> &samples) const override;

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero ();

        computeMeanAndCovarianceMatrix (*cloud, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!std::isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::ZhedaSampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        * \param indices
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud,
                                      const std::vector<int> &indices)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        computeMeanAndCovarianceMatrix (*cloud, indices, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!std::isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::ZhedaSampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

    /** \brief Estimate a rigid transformation between a source and a target point cloud using an SVD closed-form
      * solution of absolute orientation using unit quaternions
      * \param[in] cloud_src the source point cloud dataset
      * \param[in] indices_src the vector of indices describing the points of interest in cloud_src
      * \param[in] cloud_tgt the target point cloud dataset
      * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from
      * indices_src
      * \param[out] transform the resultant transformation matrix (as model coefficients)
      *
      * This method is an implementation of: Horn, B. “Closed-Form Solution of Absolute Orientation Using Unit Quaternions,” JOSA A, Vol. 4, No. 4, 1987
      */
      void
      estimateRigidTransformationSVD (const pcl::PointCloud<PointT> &cloud_src,
                                      const std::vector<int> &indices_src,
                                      const pcl::PointCloud<PointT> &cloud_tgt,
                                      const std::vector<int> &indices_tgt,
                                      Eigen::VectorXf &transform) const;

      /** \brief Compute mappings between original indices of the input_/target_ clouds. */
      void
      computeOriginalIndexMapping ()
      {
        if (!indices_tgt_ || !indices_ || indices_->empty () || indices_->size () != indices_tgt_->size ())
          return;
        for (size_t i = 0; i < indices_->size (); ++i)
          correspondences_[(*indices_)[i]] = (*indices_tgt_)[i];
      }

      /** \brief A boost shared pointer to the target point cloud data array. */
      PointCloudConstPtr target_;

      /** \brief A pointer to the vector of target point indices to use. */
      boost::shared_ptr <std::vector<int> > indices_tgt_;

      /** \brief Given the index in the original point cloud, give the matching original index in the target cloud */
      std::map<int, int> correspondences_;

      /** \brief Internal distance threshold used for the sample selection step. */
      double sample_dist_thresh_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}


template <typename PointT> bool
pcl::ZhedaSampleConsensusModelRegistration<PointT>::isSampleGood (const std::vector<int> &samples) const
{
  using namespace pcl::common;
  using namespace pcl::traits;

  PointT p10 = input_->points[samples[1]] - input_->points[samples[0]];
  PointT p20 = input_->points[samples[2]] - input_->points[samples[0]];
  PointT p21 = input_->points[samples[2]] - input_->points[samples[1]];

  return ((p10.x * p10.x + p10.y * p10.y + p10.z * p10.z) > sample_dist_thresh_ && 
          (p20.x * p20.x + p20.y * p20.y + p20.z * p20.z) > sample_dist_thresh_ && 
          (p21.x * p21.x + p21.y * p21.y + p21.z * p21.z) > sample_dist_thresh_);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::ZhedaSampleConsensusModelRegistration<PointT>::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients) const
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::computeModelCoefficients] No target dataset given!\n");
    return (false);
  }
  // Need 3 samples
  if (samples.size () != 3)
    return (false);

  std::vector<int> indices_tgt (3);
  for (int i = 0; i < 3; ++i)
    indices_tgt[i] = correspondences_.at (samples[i]);

  estimateRigidTransformationSVD (*input_, samples, *target_, indices_tgt, model_coefficients);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ZhedaSampleConsensusModelRegistration<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::getDistancesToModel] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    distances.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::getDistanceToModel] No target dataset given!\n");
    return;
  }
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // Get the 4x4 transformation
  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src (input_->points[(*indices_)[i]].x, 
                            input_->points[(*indices_)[i]].y, 
                            input_->points[(*indices_)[i]].z, 1); 
    Eigen::Vector4f pt_tgt (target_->points[(*indices_tgt_)[i]].x, 
                            target_->points[(*indices_tgt_)[i]].y, 
                            target_->points[(*indices_tgt_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    // need to compute the real norm here to keep MSAC and friends general
    distances[i] = (p_tr - pt_tgt).norm ();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ZhedaSampleConsensusModelRegistration<PointT>::selectWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::selectWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    inliers.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::selectWithinDistance] No target dataset given!\n");
    return;
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }
  
  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src (input_->points[(*indices_)[i]].x, 
                            input_->points[(*indices_)[i]].y, 
                            input_->points[(*indices_)[i]].z, 1); 
    Eigen::Vector4f pt_tgt (target_->points[(*indices_tgt_)[i]].x, 
                            target_->points[(*indices_tgt_)[i]].y, 
                            target_->points[(*indices_tgt_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);
  
    float distance = (p_tr - pt_tgt).squaredNorm (); 
    // Calculate the distance from the transformed point to its correspondence
    if (distance < thresh)
    {
      inliers[nr_p] = (*indices_)[i];
      error_sqr_dists_[nr_p] = static_cast<double> (distance);
      ++nr_p;
    }
  }
  inliers.resize (nr_p);
  error_sqr_dists_.resize (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::ZhedaSampleConsensusModelRegistration<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::countWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    return (0);
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::countWithinDistance] No target dataset given!\n");
    return (0);
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);
  
  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  int nr_p = 0; 
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src (input_->points[(*indices_)[i]].x, 
                            input_->points[(*indices_)[i]].y, 
                            input_->points[(*indices_)[i]].z, 1); 
    Eigen::Vector4f pt_tgt (target_->points[(*indices_tgt_)[i]].x, 
                            target_->points[(*indices_tgt_)[i]].y, 
                            target_->points[(*indices_tgt_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    if ((p_tr - pt_tgt).squaredNorm () < thresh)
      nr_p++;
  }
  return (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ZhedaSampleConsensusModelRegistration<PointT>::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::ZhedaSampleConsensusModelRegistration::optimizeModelCoefficients] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients) || !target_)
  {
    optimized_coefficients = model_coefficients;
    return;
  }

  std::vector<int> indices_src (inliers.size ());
  std::vector<int> indices_tgt (inliers.size ());
  for (size_t i = 0; i < inliers.size (); ++i)
  {
    indices_src[i] = inliers[i];
    indices_tgt[i] = correspondences_.at (indices_src[i]);
  }

  estimateRigidTransformationSVD (*input_, indices_src, *target_, indices_tgt, optimized_coefficients);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ZhedaSampleConsensusModelRegistration<PointT>::estimateRigidTransformationSVD (
    const pcl::PointCloud<PointT> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointT> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Eigen::VectorXf &transform) const
{
  transform.resize (16);

  Eigen::Matrix<double, 3, Eigen::Dynamic> src (3, indices_src.size ());
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt (3, indices_tgt.size ());

  for (size_t i = 0; i < indices_src.size (); ++i)
  {
    src (0, i) = cloud_src[indices_src[i]].x;
    src (1, i) = cloud_src[indices_src[i]].y;
    src (2, i) = cloud_src[indices_src[i]].z;

    tgt (0, i) = cloud_tgt[indices_tgt[i]].x;
    tgt (1, i) = cloud_tgt[indices_tgt[i]].y;
    tgt (2, i) = cloud_tgt[indices_tgt[i]].z;
  }

  // Call Umeyama directly from Eigen
  Eigen::Matrix4d transformation_matrix = pcl::umeyama (src, tgt, false);

  // Return the correct transformation
  transform.segment<4> (0).matrix () = transformation_matrix.cast<float> ().row (0); 
  transform.segment<4> (4).matrix () = transformation_matrix.cast<float> ().row (1);
  transform.segment<4> (8).matrix () = transformation_matrix.cast<float> ().row (2);
  transform.segment<4> (12).matrix () = transformation_matrix.cast<float> ().row (3);
}

#define PCL_INSTANTIATE_ZhedaSampleConsensusModelRegistration(T) template class PCL_EXPORTS pcl::ZhedaSampleConsensusModelRegistration<T>;

