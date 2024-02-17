#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include <boost/graph/adjacency_list.hpp>

#include <rl/plan/MatrixPtr.h>
#include <rl/plan/Model.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/TransformPtr.h>
#include <rl/plan/VectorPtr.h>
#include <rl/plan/Verifier.h>
#include "YourSampler.h"

using namespace ::rl::plan;
/**
 *	The implementation of your planner.
 *	modify any of the existing methods to improve planning performance.
 */
class YourPlanner : public Planner
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  virtual ::std::size_t getNumEdges() const;

  virtual ::std::size_t getNumVertices() const;

  virtual rl::plan::VectorList getPath();

  virtual void reset();

  virtual bool solve();

  /////////////////////////////////////////////////////////////////////////
  // Planner parameters ///////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////

  /** Configuration step size. */
  ::rl::math::Real delta;

  /** Epsilon for configuration comparison. */
  ::rl::math::Real epsilon;

  /** The sampler used for planning */
  ::rl::plan::YourSampler *sampler;

  uint most_fails = 0;

  bool use_goal_bias = false;
  bool use_neighbor_exhaustion = false;
  bool use_gaussian_sampling = false;
  bool use_bridge_sampling = false;
  bool use_better_connect = false;
  bool use_weighted_distance_metric = false;
  bool use_gaussian_along_c_path = false;
  bool use_gaussian_along_direction = false;
  int exhaustion_limit = 50;
  float goal_bias = 0.05;
  ::rl::math::Real sigma = 0.1;
  ::std::string name = "Test"; //"GoalProbability" + ::std::to_string(goal_bias);
  Eigen::MatrixXd Q;
  Eigen::VectorXd start_to_goal;
  double lengthStartGoal;

protected:
  /////////////////////////////////////////////////////////////////////////
  // boost graph definitions //////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////

  /** This struct defines all variables that are stored in each tree vertex.
  You can access them by calling i.e.: tree[vertex].tmp
  If you need additional parameters for vertices add them here */
  struct VertexBundle
  {
    ::std::size_t index;

    ::rl::plan::VectorPtr q;

    ::rl::math::Real tmp;

    bool exhausted = false;
    uint fails = 0;
    uint successes = 0;
  };

  typedef ::boost::adjacency_list_traits<
      ::boost::listS,
      ::boost::listS,
      ::boost::bidirectionalS,
      ::boost::listS>::vertex_descriptor Vertex;

  /** This defines a boost graph */
  typedef ::boost::adjacency_list<
      ::boost::listS,
      ::boost::listS,
      ::boost::bidirectionalS,
      VertexBundle,
      ::boost::no_property,
      ::boost::no_property>
      Tree;

  typedef ::boost::graph_traits<Tree>::edge_descriptor Edge;

  typedef ::boost::graph_traits<Tree>::edge_iterator EdgeIterator;

  typedef ::std::pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;

  typedef ::boost::graph_traits<Tree>::vertex_iterator VertexIterator;

  typedef ::std::pair<VertexIterator, VertexIterator> VertexIteratorPair;

  typedef ::std::pair<Vertex, ::rl::math::Real> Neighbor;

  ////////////////////////////////////////////////////////////////////////
  // helper functions ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////

  /** Add an edge to the RR-Tree */
  virtual Edge addEdge(const Vertex &u, const Vertex &v, Tree &tree);

  /** Add a vertex to the RR-Tree */
  Vertex addVertex(Tree &tree, const ::rl::plan::VectorPtr &q);

  bool areEqual(const ::rl::math::Vector &lhs, const ::rl::math::Vector &rhs) const;

  ////////////////////////////////////////////////////////////////////////
  // RRT functions ///////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////

  /** Draws a random sample configuration*/
  virtual void choose(::rl::math::Vector &chosen, const ::rl::math::Vector &goal);

  /** Extends vertex nearest of tree towards sample chosen*/
  virtual Vertex extend(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen);

  /** Tries to connect vertex nearest of tree to sample chosen*/
  virtual Vertex connect(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen);

  virtual rl::math::Real compute_distance(const ::rl::math::Vector &q1, const ::rl::math::Vector &q2);

  /** Returns the nearest neighbour of chosen in tree*/
  virtual Neighbor nearest(const Tree &tree, const ::rl::math::Vector &chosen);

  /**generates and orthonormal basis based on a given vector that will be included (in normalized form) in said basis*/
  Eigen::MatrixXd generateOrthonormalBasis(const Eigen::VectorXd &v);

  ////////////////////////////////////////////////////////////////////////
  // members /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////

  /** A vector of RRTs - here it's size 2 because we use two trees that grow towards each other */
  ::std::vector<Tree> tree;

  /** Start and end of the solution path */
  ::std::vector<Vertex> begin;
  ::std::vector<Vertex> end;
  ::std::vector<double> furthestDistanceFromOrigin;

private:
};

#endif // _YOUR_PLANNER_H_
