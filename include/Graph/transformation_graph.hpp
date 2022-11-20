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

#ifndef IMPL_TRANSFORMATION_GRAPH_H_
#define IMPL_TRANSFORMATION_GRAPH_H_

#include "transformation_graph.h"


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> int
pcl::registration::TransformationGraph <Scalar>::detectLoops (
  DirectedIndicesList &loops,
  int max_loop_size)
{
  // initialize graph and map
  initCompute ();
  const std::size_t nr_clouds = graph_.size ();
  const std::size_t nr_pairs = pairs_.size ();

  // apply depth first search for initial loop search
  IndicesList node_loops;
  depthFirstSearch (node_loops, max_loop_size);

  // convert node loops to pair/transformation loops
  //loops ´æµÄÊÇ±ß
  const std::size_t nr_loops = node_loops.size ();
  loops.resize (nr_loops);

  for (std::size_t l = 0; l < nr_loops; l++)
  {
    const std::size_t loop_size = node_loops[l].size () - 1;
    loops[l].resize (loop_size);

    for (std::size_t i = 0; i < loop_size; i++)
      loops[l][i] = map_[node_loops[l][i] * nr_clouds + node_loops[l][i + 1]];
  }

  // remove non unique loops
  for (DirectedIndicesList::iterator it_a = loops.begin (); it_a != loops.end () - 1 && it_a != loops.end (); it_a++)
    for (DirectedIndicesList::iterator it_b = it_a + 1; it_b != loops.end ();)
    {
      if (hasSameElements (*it_a, *it_b))
        it_b = loops.erase (it_b);
      else
        it_b++;
    }

  // remove large constraints which have at least 2 common edges compared to smaller constraints
  std::sort (loops.begin (), loops.end (), smaller);
  Eigen::MatrixXi lookup = Eigen::MatrixXi::Zero (nr_pairs, nr_pairs);
  for (DirectedIndicesList::iterator it = loops.begin (); it != loops.end ();)
  {
    if (it->size () <= max_loop_size)
      addLoopToLookup (*it++, lookup);
    else
    {
      if (compareLoopToLookup (*it, lookup) < 2)
        it++;
      else
        it = loops.erase (it);
    }
  }

  return (static_cast <int> (loops.size ()));
};

///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> void
pcl::registration::TransformationGraph <Scalar>::initCompute ()
{
  if (graph_is_updated_)
    return;

  Pairs::const_iterator it = std::max_element (pairs_.begin (), pairs_.end (), bigger_pair_element);
  const std::size_t nr_clouds = static_cast <std::size_t> (std::max (it->first, it->second) + 1);
  graph_.resize (nr_clouds);

  // fill undirected graph
  const std::size_t nr_pairs = pairs_.size ();
  for (std::size_t i = 0; i < nr_pairs; i++)
  {
    graph_[pairs_[i].first].push_back (pairs_[i].second);
    graph_[pairs_[i].second].push_back (pairs_[i].first);
  }

  // sort connections of each cloud
  for (std::size_t i = 0; i < nr_clouds; i++)
    std::sort (graph_[i].begin (), graph_[i].end ());


  // initialize map with maximum size
  map_.resize (nr_clouds * nr_clouds);

  // map the directed (true) and inverse (false) pairs: idx1 * nr_clouds + idx2
  for (std::size_t i = 0; i < nr_pairs; i++)
  {
    map_[pairs_[i].first * nr_clouds + pairs_[i].second] = std::make_pair (static_cast <int> (i), true);
    map_[pairs_[i].second * nr_clouds + pairs_[i].first] = std::make_pair (static_cast <int> (i), false);
  }

  // set to updated
  graph_is_updated_ = true;
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> int
pcl::registration::TransformationGraph <Scalar>::depthFirstSearch (
  IndicesList &loops,
  int max_loop_size)
{
  const std::size_t nr_nodes = graph_.size ();
  std::vector <bool> visited (nr_nodes, false);

  for (std::size_t node = 0; node < nr_nodes; node++)
  {
    // new loop with current as starting node
    Indices path (1, static_cast <int> (node));

    // loop over all unvisited adjacent nodes
    Indices indices = graph_[node];
    removeVisited (visited, indices);
    const std::size_t nr_adjacent = indices.size ();

    for (std::size_t a = 0; a < nr_adjacent; a++)
      nextDepth (indices[a], path, visited, max_loop_size, loops);

    // set node to visited at the end, to ensure loop closure to first node
    visited[node] = true;
  }

  return (static_cast <int> (loops.size ()));
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> void
pcl::registration::TransformationGraph <Scalar>::removeVisited (
  const std::vector <bool> &visited,
  Indices &indices)
{
  Indices::iterator it = indices.begin ();
  while (it != indices.end ())
  {
    if (visited[*it])
      it = indices.erase (it);
    else
      it++;
  }
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> bool
pcl::registration::TransformationGraph <Scalar>::nextDepth (
  std::size_t node,
  Indices path,
  std::vector <bool> visited,
  int max_loop_size,
  IndicesList &loops)
{
  // update path and visited nodes
  path.push_back (static_cast <int> (node));
  visited[node] = true;

  // check if we have a loop
  if (node == path[0] && path.size () > 3)
  {
    loops.push_back (path);
    return (true);
  }

  // return if we have a loop, but it contains to less elements
  if (node == path[0])
    return (false);

  // loop over all unvisited adjacent nodes
  Indices indices = graph_[node];
  removeVisited (visited, indices);
  const std::size_t nr_adjacent = indices.size ();

  for (std::size_t a = 0; a < nr_adjacent; a++)
  {
    if (nextDepth (indices[a], path, visited, max_loop_size, loops))
    {
      // early abort when loops ar getting big, but valid loop was already found 
      if (path.size () < max_loop_size)
        continue;
      return (true);
    }
  }

  return (false);
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> bool
pcl::registration::TransformationGraph <Scalar>::hasSameElements (
  DirectedIndices a,
  DirectedIndices b)
{
  if (a.size () != b.size ())
    return (false);

  // sort the indices vector
  std::sort (a.begin (), a.end ());
  std::sort (b.begin (), b.end ());

  const std::size_t nr_elements = a.size ();

  // check if all elements are equal
  for (std::size_t i = 0; i < nr_elements; i++)
    if (a[i].first != b[i].first)
      return (false);

  return (true);
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> void
pcl::registration::TransformationGraph <Scalar>::addLoopToLookup (
  const DirectedIndices &loop,
  Eigen::MatrixXi &lookup)
{
  // loop over all pair combinations in the present loop
  for (DirectedIndices::const_iterator it_out = loop.begin (); it_out != loop.end () - 1; it_out++)
    for (DirectedIndices::const_iterator it_in = it_out + 1; it_in != loop.end (); it_in++)
    {
      // add combination to lookup table
      lookup (it_out->first, it_in->first) = 1;
      lookup (it_in->first, it_out->first) = 1;
    }
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> int
pcl::registration::TransformationGraph <Scalar>::compareLoopToLookup (
  const DirectedIndices &loop,
  const Eigen::MatrixXi &lookup)
{
  int nr_common_edges = 0;

  // loop over all pair combinations in the present loop
  for (DirectedIndices::const_iterator it_out = loop.begin (); it_out != loop.end () - 1; it_out++)
    for (DirectedIndices::const_iterator it_in = it_out + 1; it_in != loop.end (); it_in++)
    {
      // if pair combination is present in lookup, count as common
      if (lookup (it_out->first, it_in->first) == 1 || lookup (it_in->first, it_out->first) == 1)
        nr_common_edges++;
    }

  return (nr_common_edges);
};


///////////////////////////////////////////////////////////////////////////////////////////

#endif // IMPL_TRANSFORMATION_GRAPH_H_