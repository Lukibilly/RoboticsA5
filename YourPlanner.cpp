#include "YourPlanner.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
#include <iostream>
#include <Eigen/Dense>

YourPlanner::YourPlanner() : Planner(),
                             delta(1.0f),
                             epsilon(1.0e-3f),
                             sampler(NULL),
                             begin(2),
                             end(2),
                             tree(2),
                             furthestDistanceFromOrigin(2)
{
  use_goal_bias = false;
  goal_bias = 0.1;
  use_neighbor_exhaustion = false;
  exhaustion_limit = 100;
  use_gaussian_sampling = false;
  use_better_connect = false;
  use_bridge_sampling = false;
  use_gaussian_along_c_path = true;
  use_gaussian_along_direction = true;
  sigma = 2 * this->delta;
  name = "BridgeSampling2delta";
}

YourPlanner::~YourPlanner()
{
}

YourPlanner::Edge
YourPlanner::addEdge(const Vertex &u, const Vertex &v, Tree &tree)
{
  Edge e = ::boost::add_edge(u, v, tree).first;

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
  }

  return e;
}

//project vector a on vector b
Eigen::VectorXd project(const ::rl::math::Vector &a, const ::rl::math::Vector &b)
{
  double scale = b.dot(a) / b.dot(b);
  return b * scale;
}

YourPlanner::Vertex
YourPlanner::addVertex(Tree &tree, const ::rl::plan::VectorPtr &q)
{
  // when a vertex gets added calculate its distance from the origin if projected along the start->goal vector
  if (this->use_gaussian_along_direction)
  {
    
    Eigen::VectorXd projection = project(*q, this->start_to_goal);
    float length_of_projection = projection.norm();

    int tree_version = -1;
    if (&this->tree[0] == &tree)
    {
      tree_version = 0;
    }
    else
    {
      tree_version = 1;
    }

    if (furthestDistanceFromOrigin[tree_version] < length_of_projection)
    {
      furthestDistanceFromOrigin[tree_version] = length_of_projection;
    }
  }

  Vertex v = ::boost::add_vertex(tree);
  tree[v].index = ::boost::num_vertices(tree) - 1;
  tree[v].q = q;

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationVertex(*tree[v].q);
  }

  return v;
}

bool YourPlanner::areEqual(const ::rl::math::Vector &lhs, const ::rl::math::Vector &rhs) const
{
  if (this->model->distance(lhs, rhs) > this->epsilon)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void // TODO: OPTIMIZE
YourPlanner::choose(::rl::math::Vector &chosen, const ::rl::math::Vector &goal)
{
  float goal_p = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  if (goal_p < this->goal_bias && this->use_goal_bias)
  {
    chosen = goal;
    return;
  }
  if (this->use_gaussian_sampling)
  {
    chosen = this->sampler->generateGaussian();
  }
  else if (this->use_bridge_sampling)
  {
    chosen = this->sampler->generateBridge();
  }
  else if (this->use_gaussian_along_c_path)
  {
    // if we also additionally want to sample with two gaussians along the start->goal direction
    if (this->use_gaussian_along_direction)
    {
      chosen = this->sampler->generateGaussianAlongCPath_Improved(this->Q, this->lengthStartGoal, this->furthestDistanceFromOrigin);
    }
    // if we want our "regular" gaussian along start->goal direction
    else
    {
      chosen = this->sampler->generateGaussianAlongCPath(this->Q, this->lengthStartGoal);
    }
  }
  else
  {
    chosen = this->sampler->generate();
  }
}

YourPlanner::Vertex
YourPlanner::connect(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen)
{
  // Do first extend step

  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  bool reached = false;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr lastQ = ::std::make_shared<::rl::math::Vector>(this->model->getDof());

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *lastQ);

  this->model->setPosition(*lastQ);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    tree[nearest.first].fails += 1;
    if (tree[nearest.first].fails > this->exhaustion_limit && this->use_neighbor_exhaustion)
    {
      tree[nearest.first].exhausted = true;
    }
    if (tree[nearest.first].fails > this->most_fails)
    {
      this->most_fails = tree[nearest.first].fails;
      // std::cout << "Most fails: " << this->most_fails << std::endl;
    }
    return NULL;
  }
  else
  {
    tree[nearest.first].successes += 1;
  }

  //::rl::math::Vector nextQ(this->model->getDof());
  ::rl::plan::VectorPtr nextQ = ::std::make_shared<::rl::math::Vector>(this->model->getDof());

  uint counter = 0;

  Vertex lastVertex = nearest.first;

  while (!reached)
  {
    // Do further extend step

    distance = this->model->distance(*lastQ, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    // move "next" along the line last<->chosen by distance "step / distance"
    this->model->interpolate(*lastQ, chosen, step / distance, *nextQ);

    this->model->setPosition(*nextQ);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      break;
    }

    if (this->use_better_connect)
    {
      Vertex tmp = this->addVertex(tree, nextQ);
      this->addEdge(lastVertex, tmp, tree);
      lastVertex = tmp;
    }
    *lastQ = *nextQ;
    counter += 1;
  }

  // "last" now points to the vertex where the connect step collided with the environment.
  // Add it to the tree
  Vertex connected = this->addVertex(tree, lastQ);
  this->addEdge(lastVertex, connected, tree);
  return connected;
}

Eigen::MatrixXd
YourPlanner::generateOrthonormalBasis(const Eigen::VectorXd &v)
{

  int dim = this->model->getDof();

  if (dim != v.size())
  {
    std::cout << "DOF does not match vector size" << std::endl;
  }

  // Initialize a matrix to store vectors that will form the basis. The matrix has 'dim' rows and 'dim' columns.
  Eigen::MatrixXd mat(dim, dim);
  mat.col(0) = v.normalized();

  // Generate additional vectors to fill the basis.
  // Start from 1 because the first vector is already set.
  for (int i = 1; i < dim; ++i)
  {

    Eigen::VectorXd vec = Eigen::VectorXd::Zero(dim); // Initialize a vector of zeros.

    vec(i) = 1; // Set one element to 1 to create a simple basis vector. This is a naive way to fill the space.

    // The generated vector is added as a column to the matrix.
    // This process is not guaranteed to produce
    // orthogonal vectors yet; QR decomposition will handle orthogonality and normalization later.
    mat.col(i) = vec;
  }

  // Perform QR decomposition on the matrix with the initial vectors.
  // QR decomposition is a method to decompose a matrix into an orthogonal matrix (Q) and an upper triangular matrix (R).
  Eigen::HouseholderQR<Eigen::MatrixXd> qr = mat.householderQr();

  // Extract the Q matrix from the QR decomposition. Q is an orthonormal matrix where columns are the orthonormal basis vectors.
  Eigen::MatrixXd Q = qr.householderQ();

  // Return the Q matrix, which now contains an orthonormal basis for the space.
  // The first vector is the normalized input vector, and the rest are orthogonal to it and each other.

  std::cout << "Orthonormal basis:\n"
            << Q << std::endl;

  /*
  for (int i = 1; i < dim; ++i) {
    float diff = Q.col(i-1).dot(Q.col(i));
    std::cout << "Diff from perfect orthogonality:" << i << "="<< diff << std::endl;
  }
  */

  return Q;
}

YourPlanner::Vertex
YourPlanner::extend(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared<::rl::math::Vector>(this->model->getDof());

  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding())
  {
    Vertex extended = this->addVertex(tree, next);
    this->addEdge(nearest.first, extended, tree);
    return extended;
  }

  return NULL;
}

::std::string
YourPlanner::getName() const
{
  return this->name;
}

::std::size_t
YourPlanner::getNumEdges() const
{
  ::std::size_t edges = 0;

  for (::std::size_t i = 0; i < this->tree.size(); ++i)
  {
    edges += ::boost::num_edges(this->tree[i]);
  }

  return edges;
}

::std::size_t
YourPlanner::getNumVertices() const
{
  ::std::size_t vertices = 0;

  for (::std::size_t i = 0; i < this->tree.size(); ++i)
  {
    vertices += ::boost::num_vertices(this->tree[i]);
  }

  return vertices;
}

rl::plan::VectorList
YourPlanner::getPath()
{

  rl::plan::VectorList path;
  Vertex i = this->end[0];

  while (i != this->begin[0])
  {
    path.push_front(*this->tree[0][i].q);
    i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
  }

  path.push_front(*this->tree[0][i].q);

  i = ::boost::source(*::boost::in_edges(this->end[1], this->tree[1]).first, this->tree[1]);

  while (i != this->begin[1])
  {
    path.push_back(*this->tree[1][i].q);
    i = ::boost::source(*::boost::in_edges(i, this->tree[1]).first, this->tree[1]);
  }

  path.push_back(*this->tree[1][i].q);

  return path;
}

rl::math::Real // TODO: OPTIMIZE
YourPlanner::compute_distance(const ::rl::math::Vector &q1, const ::rl::math::Vector &q2)
{
  return this->model->transformedDistance(q1, q2);
}

YourPlanner::Neighbor // TODO: OPTIMIZE
YourPlanner::nearest(const Tree &tree, const ::rl::math::Vector &chosen)
{
  // create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

  // Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    // TODO: maybe only make them less likely?
    // ignore exhausted nodes
    if (tree[*i.first].exhausted)
      continue;

    ::rl::math::Real d = this->compute_distance(chosen, *tree[*i.first].q);

    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }
  }

  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}

void YourPlanner::reset()
{
  for (::std::size_t i = 0; i < this->tree.size(); ++i)
  {
    this->tree[i].clear();
    this->begin[i] = NULL;
    this->end[i] = NULL;
  }
}

bool // TODO: OPTIMIZE
YourPlanner::solve()
{
  this->sampler->setSigma(this->sigma);
  this->time = ::std::chrono::steady_clock::now();

  // calculate direction in c-space from start to goal
  this->start_to_goal = (*this->goal) - (*this->start);

  // calculate length between start and goal
  this->lengthStartGoal = start_to_goal.norm();

  // calculate orthonormal basis including the first direction as the 0th entry
  this->Q = YourPlanner::generateOrthonormalBasis(this->start_to_goal);

  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared<::rl::math::Vector>(*this->goal));

  // these keep the furthest distance any node has from the origin of its respective tree
  // they decice, where the most of the first values are drawn
  // TODO check good starting values for these, 
  furthestDistanceFromOrigin[0] = 0.1*this->lengthStartGoal;
  furthestDistanceFromOrigin[1] = 0.1*this->lengthStartGoal;

  Tree *a = &this->tree[0];
  Tree *b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());

  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    // First grow tree a and then try to connect b.
    // then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      // Sample a random configuration
      this->choose(chosen, &this->tree[0] == a ? *this->goal : *this->start);

      // Find the nearest neighbour in the tree
      Neighbor aNearest = this->nearest(*a, chosen);

      // Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      // If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          // Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      }

      // Swap the roles of a and b
      using ::std::swap;
      swap(a, b);
    }
  }
  return false;
}