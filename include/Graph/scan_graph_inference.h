/*
 * Copyright(c) 2020, SCHOOL OF GEODESY AND GEOMATIC, WUHAN UNIVERSITY
 * WUHAN, CHINA
 * All Rights Reserved
 * Authors: Hao Wu, Pengcheng Wei, et al.
 * Do not hesitate to contact the authors if you have any question or find any bugs
 * Email: haowu2021@whu.edu.cn
 * Thanks to the work P.W.Theiler, et al.
 *
*/

#include <vector>
#include <iostream>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/registration/matching_candidate.h>
#include "transformation_graph.h"

class ScanGraphInference
{

public:
    ScanGraphInference();
    ~ScanGraphInference();

    /** \brief inference invaild edges.
     * \param[in] length the length of loops
     * \returns  UES
     */
    std::set<int> inference(int length, int t_mcs);

    /** \brief set graph nodes.
     * \param[in] pairs scan-pairs (v_i,v_j)  in G_s( V, E)
     * \returns void
     */
    void setScanPairs(std::vector<std::pair<int, int>> pairs) { edges = pairs; }

    /** \brief set scan-to-scan matching results.
     * \param[in] trans transformation in edges
     * \returns void
     */
    void setMatchingCandidates(std::vector<pcl::registration::MatchingCandidate,Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> &trans)
    {
        transformations = trans;
    }

    void setMCS(std::vector<int> vmcs)
    {
        VMCS = vmcs;
    }

    /** \brief set the predefined rotation accuracy.
     * \param[in] accuracy rotation accuracy (default  0.087)
     * \returns void
     */
    inline void setRotationAccuracy(float accuracy)
    {
        rotation_accuracy = accuracy;
    }

    /** \brief set the predefined translation_accuracy.
     * \param[in] accuracy translation accuracy (default 0.5)
     * \returns  void
     */
    inline void setTranslationAccuracy(float accuracy)
    {
        translation_accuracy = accuracy;
    }

private:
    void eliminateUncertaintyEdges();

    bool loopClosure(Eigen::Matrix4f &trans, int n);

    void nextNode(Eigen::Matrix4f &cur_trans, std::vector<std::pair<int, bool>> &loop, int position);

    void combineTransformation(Eigen::Matrix4f &a, Eigen::Matrix4f b, Eigen::Matrix4f &c, bool inverse);

private:
    /** \brief Scan graph. */
    pcl::registration::TransformationGraph<float> graph;

    /** \brief pairwise relationships. */
    std::vector<std::pair<int, int>> edges;

    /** \brief rotation accuracy. */
    float rotation_accuracy;

    /** \brief translation accuracy. */
    float translation_accuracy;

    /** \brief transformations in corresponding edges. */
    std::vector<pcl::registration::MatchingCandidate,Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> transformations;

    std::vector<int> VMCS;

    /** \brief detected loops. */
    std::vector<std::vector<std::pair<int, bool>>> loops;

    /** \brief Uncertain edges set. */
    std::vector<std::vector<std::pair<int, bool>>> UES;

    /** \brief Uncertain edges after knowledge sharing. */
    std::vector<std::vector<std::pair<int, bool>>> UES_pruned;

    /** \brief Vaild edges set. */
    std::vector<std::vector<std::pair<int, bool>>> VES;

    /** \brief Invaild edges. */
    std::set<int> IES;

    /** \brief Vaild edges. */
    std::set<int> VE;
};
