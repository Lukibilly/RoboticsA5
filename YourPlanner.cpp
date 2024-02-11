#include "YourPlanner.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
#include <iostream>

YourPlanner::YourPlanner() :
  Planner(),
  delta(1.0f),
  epsilon(1.0e-3f),
  sampler(NULL),
  begin(2),
  end(2),
  tree(2)
{
}

YourPlanner::~YourPlanner()
{
}

YourPlanner::Edge
YourPlanner::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
{
  Edge e = ::boost::add_edge(u, v, tree).first;

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
  }

  return e;
}
YourPlanner::Vertex
YourPlanner::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
  Vertex v = ::boost::add_vertex(tree);
  tree[v].index = ::boost::num_vertices(tree) - 1;
  tree[v].q = q;

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationVertex(*tree[v].q);
  }

  return v;
}

bool
YourPlanner::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const
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
YourPlanner::choose(::rl::math::Vector& chosen, const ::rl::math::Vector& goal)
{
  float use_goal_probability = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  if (use_goal_probability < this->goal_bias && this->use_goal_bias)
  {
    chosen = goal;
  }
  else
  {
    chosen = this->sampler->generate();
  }
}

YourPlanner::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //Do first extend step

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

  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  uint counter = 0;

  while (!reached)
  {
    //Do further extend step

    distance = this->model->distance(*last, chosen);
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
    this->model->interpolate(*last, chosen, step / distance, next);

    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      break;
    }

    *last = next;
    counter += 1;
  }

  // "last" now points to the vertex where the connect step collided with the environment.
  // Add it to the tree
  Vertex connected = this->addVertex(tree, last);
  this->addEdge(nearest.first, connected, tree);
  return connected;
}

YourPlanner::Vertex
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

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
YourPlanner::compute_distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2)
{
  return this->model->transformedDistance(q1, q2);
}

YourPlanner::Neighbor // TODO: OPTIMIZE
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
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

void
YourPlanner::reset()
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

  this->time = ::std::chrono::steady_clock::now();
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());


  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      //Sample a random configuration
      
      this->choose(chosen, &this->tree[0] == a ? *this->goal : *this->start);

      //Find the nearest neighbour in the tree
      Neighbor aNearest = this->nearest(*a, chosen);

      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          //Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      }

      //Swap the roles of a and b
      using ::std::swap;
      swap(a, b);
    }

  }

  return false;

}