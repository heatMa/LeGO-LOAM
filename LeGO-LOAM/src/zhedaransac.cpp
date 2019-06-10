
#include "utility.h"
#include <cfloat>
#include <ctime>
#include <climits>
#include <set>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>

//RANSAC模型类，使用变换矩阵作为模型参数
class RansacModel{
public:

    typedef PointType PointT;

    //命名两个指针，与源码中的名字相同，方便修改
    typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

    //类指针
    typedef boost::shared_ptr<RansacModel> Ptr;
    typedef boost::shared_ptr<const RansacModel> ConstPtr;

    //样本大小 和 模型参数大小
    unsigned int sample_size_;
    unsigned int model_size_;

    //输入输出点云和对应关联点的索引
    pcl::PointCloud<PointT>::ConstPtr input_;
    pcl::PointCloud<PointT>::ConstPtr  target_;
    boost::shared_ptr <std::vector<int> > indices_;
    boost::shared_ptr <std::vector<int> > indices_tgt_;

    /** \brief Given the index in the original point cloud, give the matching original index in the target cloud */
    std::map<int, int> correspondences_;

    /** Data containing a shuffled version of the indices. This is used and modified when drawing samples. */
    std::vector<int> shuffled_indices_;

    std::vector<int> prob_indices_;

    std::vector<double> error_sqr_dists_;
    static const unsigned int max_sample_checks_ = 1000;

    //采样距离，需要注意，初始化为0，后面根据输入点云计算得到
    double sample_dist_thresh_;


    //用于产生随机数的三个变量
    /** \brief Boost-based random number generator algorithm. */
    boost::mt19937 rng_alg_;
    /** \brief Boost-based random number generator distribution. */
    boost::shared_ptr<boost::uniform_int<> > rng_dist_;
    /** \brief Boost-based random number generator. */
    boost::shared_ptr<boost::variate_generator< boost::mt19937&, boost::uniform_int<> > > rng_gen_;

    //    //构造函数
    //    RansacModel (const PointCloudConstPtr &cloud,
    //                 const std::vector<int> &indices,
    //                 const PointCloudConstPtr &target,
    //                 const std::vector<int> &indices_tgt):
    //        input_ (cloud)
    //      , indices_ (new std::vector<int> (indices))
    //      , target_ (target)
    //      , indices_tgt_ (new std::vector<int> (indices_tgt))
    //      , correspondences_ ()
    //      , shuffled_indices_ ()
    //      , sample_dist_thresh_ (0)
    //    {
    //        computeOriginalIndexMapping ();
    //        computeSampleDistanceThreshold (cloud, indices);
    //        sample_size_ = 3;
    //        model_size_ = 16;

    //        shuffled_indices_ = *indices_;

    //        rng_alg_.seed (12345u);
    //        rng_dist_.reset(new boost::uniform_int<> (0, std::numeric_limits<int>::max ()));
    //        rng_gen_.reset (new boost::variate_generator<boost::mt19937&, boost::uniform_int<> > (rng_alg_, *rng_dist_));
    //    }



    //构造函数
    RansacModel (const std::vector<int> &indices,
                 const std::vector<int> &indices_tgt):
        indices_ (new std::vector<int> (indices))
      , indices_tgt_ (new std::vector<int> (indices_tgt))
      , correspondences_ ()
      , shuffled_indices_ ()
      , sample_dist_thresh_ (0)
    {
        sample_size_ = 3;
        model_size_ = 16;

        shuffled_indices_ = *indices_;

        rng_alg_.seed (12345u);
        rng_dist_.reset(new boost::uniform_int<> (0, std::numeric_limits<int>::max ()));
        rng_gen_.reset (new boost::variate_generator<boost::mt19937&, boost::uniform_int<> > (rng_alg_, *rng_dist_));
    }
    /** \brief Empty destructor */
    ~RansacModel () {}


    //bug！这里因为indices_的类型是boost::shared_ptr <std::vector<int> >，传入computeSampleDistanceThreshold会编译报错
    //所以又把indices传了进来
    void setInputAndTargerCloud(const PointCloudConstPtr &cloud,const PointCloudConstPtr &target,const std::vector<int> &indices)
    {
        input_=cloud;
        target_=target;
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (input_, indices);
    }

    //计算对应关系
    void computeOriginalIndexMapping ()
    {
        if (!indices_tgt_ || !indices_ || indices_->empty () || indices_->size () != indices_tgt_->size ())
            return;
        for (size_t i = 0; i < indices_->size (); ++i)
            correspondences_[(*indices_)[i]] = (*indices_tgt_)[i];
    }


    //计算最优的采样距离
    inline void
    computeSampleDistanceThreshold (const PointCloudConstPtr &cloud,
                                    const std::vector<int> &indices)
    {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        pcl::computeMeanAndCovarianceMatrix (*cloud, indices, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (!std::isfinite (covariance_matrix.coeffRef (i, j)))
                    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
    }

    //获取样本大小
    inline unsigned int
    getSampleSize () const
    {
        return sample_size_;
    }

    //获取模型大小
    inline unsigned int
    getModelSize () const
    {
        return model_size_;
    }

    /** \brief Boost-based random number generator. */
    inline int
    rnd ()
    {
        return ((*rng_gen_) ());
    }

    //提取样本
    inline void drawIndexSample (std::vector<int> &sample)
    {
        size_t sample_size = sample.size ();
        size_t index_size = shuffled_indices_.size ();
        for (size_t i = 0; i < sample_size; ++i)
            // The 1/(RAND_MAX+1.0) trick is when the random numbers are not uniformly distributed and for small modulo
            // elements, that does not matter (and nowadays, random number generators are good)
            //std::swap (shuffled_indices_[i], shuffled_indices_[i + (rand () % (index_size - i))]);
            std::swap (shuffled_indices_[i], shuffled_indices_[i + (rnd () % (index_size - i))]);
        std::copy (shuffled_indices_.begin (), shuffled_indices_.begin () + sample_size, sample.begin ());
    }

    //按概率采样
    inline void darwProbSample(const vector<int> &bin, const vector<float> &prob,std::vector<int> &sample)
    {
        size_t sample_size = sample.size ();

        //扩充矩阵
        vector<int> intProb;
        for(size_t i=0;i<prob.size();i++)
            intProb.push_back((int)(prob[i]*100));
        //prob_indices_是扩充后的矩阵
        prob_indices_.clear();
        for(int i=0;i<bin.size();i++)
        {
            for(size_t j=0;j<intProb[i];j++)
                prob_indices_.push_back(bin[i]);
        }

        for (size_t i = 0; i < sample_size; ++i)
        {
            size_t index_size = prob_indices_.size ();

            int idx=(rnd () % (index_size));
            sample.push_back(prob_indices_[idx]);

            //删除已经选择的数据
            vector<int>::iterator frontIte=prob_indices_.begin()+idx;
            vector<int>::iterator backIte=prob_indices_.begin()+idx;
            //这里不能是>=，否则当frontIte为0时，frontIte--会报错
            while(frontIte>prob_indices_.begin())
            {
                if(*frontIte!=prob_indices_[idx])
                {
                    frontIte++;
                    break;
                }
                frontIte--;
            }
            while(backIte<prob_indices_.end())
            {
                if(*backIte!=prob_indices_[idx])
                    break;
                backIte++;
            }
            prob_indices_.erase(frontIte, backIte);
        }
    }

    bool isSampleGood (const std::vector<int> &samples) const
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


    void getSamples (int &iterations, std::vector<int> &samples)
    {
        // We're assuming that indices_ have already been set in the constructor
        if (indices_->size () < getSampleSize ())
        {
            PCL_ERROR ("[pcl::SampleConsensusModel::getSamples] Can not select %lu unique points out of %lu!\n",
                       samples.size (), indices_->size ());
            // one of these will make it stop :)
            samples.clear ();
            iterations = INT_MAX - 1;
            return;
        }

        // Get a second point which is different than the first
        samples.resize (getSampleSize ());
        for (unsigned int iter = 0; iter < max_sample_checks_; ++iter)
        {
            // Choose the random indices
            drawIndexSample (samples);

            // If it's a good sample, stop here
            if (isSampleGood (samples))
            {
                PCL_DEBUG ("[pcl::SampleConsensusModel::getSamples] Selected %lu samples.\n", samples.size ());
                return;
            }
        }
        PCL_DEBUG ("[pcl::SampleConsensusModel::getSamples] WARNING: Could not select %d sample points in %d iterations!\n", getSampleSize (), max_sample_checks_);
        samples.clear ();
    }

    //由随机选择的三个点计算转移矩阵
    bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients) const
    {
        if (!target_)
        {
            PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeModelCoefficients] No target dataset given!\n");
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
    //SVD求解
    void estimateRigidTransformationSVD (
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


    //计算内点数量
    int countWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold) const
    {
        if (indices_->size () != indices_tgt_->size ())
        {
            PCL_ERROR ("[pcl::SampleConsensusModelRegistration::countWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
            return (0);
        }
        if (!target_)
        {
            PCL_ERROR ("[pcl::SampleConsensusModelRegistration::countWithinDistance] No target dataset given!\n");
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


    //选择符合最好模型的内点
    void selectWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
    {
        if (indices_->size () != indices_tgt_->size ())
        {
            PCL_ERROR ("[pcl::SampleConsensusModelRegistration::selectWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
            inliers.clear ();
            return;
        }
        if (!target_)
        {
            PCL_ERROR ("[pcl::SampleConsensusModelRegistration::selectWithinDistance] No target dataset given!\n");
            return;
        }

        double thresh = threshold * threshold;

        //判断所得到的参数大小是否正确
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

    //根据所有内点计算变换矩阵，暂时没有用到
    void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
    {
        if (indices_->size () != indices_tgt_->size ())
        {
            PCL_ERROR ("[pcl::SampleConsensusModelRegistration::optimizeModelCoefficients] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
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

    //判断所得到的参数大小是否正确
    bool isModelValid (const Eigen::VectorXf &model_coefficients) const
    {
        if (model_coefficients.size () != model_size_)
        {
            PCL_ERROR ("[isModelValid] Invalid number of model coefficients given (%lu)!\n",model_coefficients.size ());
            return (false);
        }
        return (true);
    }


};



class ZhedaRansac
{
public:
    int max_iterations_;
    double threshold_;
    int iterations_;

    typedef RansacModel::Ptr RansacModelPtr;

    //RANSAC模型，单独的一个类
    RansacModelPtr sac_model_;
    //所选择的模型index向量
    std::vector<int> model_;
    //变换矩阵
    Eigen::VectorXf model_coefficients_;

    std::vector<int> inliers_;
    double probability_;

    ZhedaRansac (const RansacModelPtr model,
                 double threshold)
        : sac_model_ (model)
        , model_ ()
        , inliers_ ()
        , model_coefficients_ ()
        , probability_ (0.99)
        , iterations_ (0)
        , threshold_ (threshold)
        , max_iterations_ (10000)
    {
    }


    //计算模型
    bool computeModel (int debug_verbosity_level = 0);
    //获取内点
    inline void
    getInliers (std::vector<int> &inliers) const { inliers = inliers_; }
};


//////////////////////////////////////////////////////////////////////////
bool ZhedaRansac::computeModel (int)
{

    // Warn and exit if no threshold was set
    if (threshold_ == std::numeric_limits<double>::max())
    {
        PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No threshold set!\n");
        return (false);
    }

    iterations_ = 0;
    int n_best_inliers_count = -INT_MAX;
    double k = 1.0;

    std::vector<int> selection;
    Eigen::VectorXf model_coefficients;

    double log_probability  = log (1.0 - probability_);
    double one_over_indices = 1.0 / static_cast<double> (sac_model_->indices_->size ());

    int n_inliers_count = 0;
    unsigned skipped_count = 0;
    // suppress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
    const unsigned max_skip = max_iterations_ * 10;

    // Iterate
    while (iterations_ < k && skipped_count < max_skip)
    {
        // Get X samples which satisfy the model criteria
        sac_model_->getSamples (iterations_, selection);
        if (selection.empty ())
        {
            PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!\n");
            break;
        }

        // Search for inliers in the point cloud for the current plane model M
        if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
        {
            //++iterations_;
            ++skipped_count;
            continue;
        }

        // Select the inliers that are within threshold_ from the model
        //sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
        //if (inliers.empty () && k > 1.0)
        //  continue;

        n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);
        // Better match ?
        if (n_inliers_count > n_best_inliers_count)
        {
            n_best_inliers_count = n_inliers_count;

            // Save the current model/inlier/coefficients selection as being the best so far
            model_              = selection;
            model_coefficients_ = model_coefficients;

            // Compute the k parameter (k=log(z)/log(1-w^n))
            double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
            double p_no_outliers = 1.0 - pow (w, static_cast<double> (selection.size ()));
            p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
            p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
            k = log_probability / log (p_no_outliers);
        }

        ++iterations_;
        PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
        if (iterations_ > max_iterations_)
        {
            PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
            break;
        }
    }

    PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Model: %lu size, %d inliers.\n", model_.size (), n_best_inliers_count);

    if (model_.empty ())
    {
        inliers_.clear ();
        return (false);
    }
    // Get the set of inliers that correspond to the best model found so far
    sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_);
    return (true);
}

