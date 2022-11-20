/*
 * Software License Agreement
 *
 *  Copyright (c) P.W.Theiler, 2015
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
 */

#ifndef TRANSFORMATION_GRAPH_H_
#define TRANSFORMATION_GRAPH_H_

/** \brief Comparator to sort a vectors of DirectedIndices according to the inner vectors length.
 * \param[in] first First directed indices
 * \param[in] second Second directed indices
 * \returns true if first is smaller than second
 */
inline bool smaller(const std::vector<std::pair<int, bool>> &first, const std::vector<std::pair<int, bool>> &second)
{
  return (first.size() < second.size());
};

/** \brief Comparator to find biggest element in vector of pairs, independent of the elements position in the pair.
 * \param[in] a First pair
 * \param[in] b Second pair
 * \returns true if first element contains the bigger value
 */
inline bool bigger_pair_element(const std::pair<int, int> &a, const std::pair<int, int> &b)
{
  return (std::max(b.first, b.second) > std::max(a.first, a.second));
};

namespace pcl
{
  namespace registration
  {
    /** \brief Container for a transformation graph. The graph is derived from cloud pairs
     * defining adjacent point clouds which overlap.
     *
     * The class is intended to provide tools to extract loops or find shortest paths
     * in the transformation graph.
     * So far only loop detection is implemented
     */
    template <typename Scalar = float>
    class TransformationGraph
    {
    public:
      /** \cond */
      typedef boost::shared_ptr<TransformationGraph<Scalar>> Ptr;
      typedef boost::shared_ptr<const TransformationGraph<Scalar>> ConstPtr;

      typedef std::pair<int, int> Pair;
      typedef std::vector<Pair> Pairs;

      typedef std::vector<int> Indices;
      typedef std::vector<Indices> IndicesList;

      typedef std::pair<int, bool> DirectedIndex;
      typedef std::vector<DirectedIndex> DirectedIndices;
      typedef std::vector<DirectedIndices> DirectedIndicesList;
      /** \endcond */

      /** \brief Constructor. */
      TransformationGraph() : graph_(),
                              pairs_(),
                              map_(),
                              graph_is_updated_(false)
      {
        class_name_ = "pcl::registration::TransformationGraph";
      };

      /** \brief Destructor. */
      virtual ~TransformationGraph(){};

      /** \brief Set the point cloud pairs defining the graph.
       * \param[in] pairs the point clouds pairs
       * \note Number of clouds is derived from the pairs itself
       */
      inline void
      setPointCloudPairs(const Pairs &pairs)
      {
        pairs_ = pairs;
        graph_is_updated_ = false;
      };

      /** \brief Extract loops within transformation graph using depth first search.
       * \param[out] loops the pair/transformation loops
       * \param[in] max_loop_size the maximum loop size
       * \returns the number of found loops
       * \note Also bigger loops than loop_size are extracted, but only if it connects
       * a cloud, which is otherwise not contained in a loop.
       */
      int
      detectLoops(
          DirectedIndicesList &loops,
          int max_loop_size = 4);

    protected:
      /** \brief The point cloud pairs which define the transformation graph.*/
      Pairs pairs_;

      /** \brief Method to initialize graph and respective map from the given point cloud pairs.*/
      void
      initCompute();

      /** \brief Apply depth first search to extract loops in graph
       * \param[out] loops the detected loops
       * \param[in] max_loop_size the maximum loop size
       * \returns the number of deteced loops.
       */
      int
      depthFirstSearch(
          IndicesList &loops,
          int max_loop_size);

    private:
      /** \brief The class name. */
      std::string class_name_;

      /** \brief The transformation graph where for each node (point cloud) the
       * connected nodes (overlappoing clouds) are stored.
       */
      IndicesList graph_;

      /** \brief Map to get the position of a respective pair within the vector of pairs.*/
      DirectedIndices map_;

      /** \brief Flag to check actuality of graph.*/
      bool graph_is_updated_;

      /** \brief Remove node as neighbor if already visited.
       * \param[in] visited flag vector to define visited nodes (true)
       * \param[in/out] indices indices of unvisited edges
       */
      void removeVisited(
          const std::vector<bool> &visited,
          Indices &indices);

      /** \brief Recursive function to do depth first search in graph.
       *
       * \param[in] node current node to check
       * \param[in] path indices of currently visited nodes
       * \param[in] visited flag vector of visited nodes (true)
       * \param[in] max_loop_size the maximum loop size
       * \param[in/out] loops resulting node loops
       *
       * \returns if a loop was generated using the current node
       */
      bool nextDepth(
          std::size_t node,
          Indices path,
          std::vector<bool> visited,
          int max_loop_size,
          IndicesList &loops);

      /** \brief Verify if two index lists have the same elements.
       * \param[in] a first directed indices
       * \param[in] b second directed indices
       * \returns true if all elements are equal
       */
      bool
      hasSameElements(
          DirectedIndices a,
          DirectedIndices b);

      /** \brief Adds all pair combinations of a loop to a binary lookup table.
       * \param[in] loop list of pairs to add to lookup table
       * \param[in] lookup pair lookup table
       */
      void addLoopToLookup(
          const DirectedIndices &loop,
          Eigen::MatrixXi &lookup);

      /** \brief Counts the number of pairs of a current loop which are present
       * in the lookup table.
       * \param[in] loop current list of edges
       * \param[in] lookup pair lookup table
       * \returns the number of common pairs between lookup table and a loop
       */
      int compareLoopToLookup(
          const DirectedIndices &loop,
          const Eigen::MatrixXi &lookup);
    };
  };
};

#include "./transformation_graph.hpp"

#endif // TRANSFORMATION_GRAPH_H_