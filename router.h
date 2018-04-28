#ifndef _ROUTER_H
#define _ROUTER_H

#define DEBUG 1

#define VIA_COST 5
#define DIR_PENL 100

#include <algorithm>
#include <bitset>
#include <climits>
#include <vector>
#include <queue>

struct GridNode
{
public:
  /* Variables */
  uint32_t cost = UINT32_MAX;

  /* Functions */
  bool obstacle( void );
  void set_obstacle( const bool& _flag );

  bool visited( void );
  void set_visited( const bool& _flag );

  bool routed( void );
  void set_routed( const bool& _flag );

  bool pin( void );
  void set_pin( const bool& _flag );

  bool source( void );
  void set_source( const bool& _flag );

  bool target( void );
  void set_target( const bool& _flag );

  uint8_t direction( void );
  void set_direction( const uint8_t& _dir );
  void clear_direction( void );

private:
  /// Flags for routing (4-byte in size)
  std::bitset<9> rt_flags = 0;
  /*************************************************************/
  /** Bit Assignment                                          **/
  /** <Bit>  <Flag>                                           **/
  /**   0    obstacle                                         **/
  /**   1    visited                                          **/
  /**         [direction]                                     **/
  /**   2   -|  from            upper(001)/lower(000) layer   **/
  /**   3   -|- from node with higher(011)/lower(010) y-value **/
  /**   4   -|  from node with higher(101)/lower(100) x-value **/
  /**   5    routed by another net                            **/
  /**   6    pin node                                         **/
  /**   7    source node                                      **/
  /**   8    target node                                      **/
  /*************************************************************/
};

struct Node
{
public:
  /* Variables */
  /// The layer, x-, and y-layer index of the node in a grid
  uint8_t l = 0;
  uint32_t x = 0;
  uint32_t y = 0;

  /* Constructors */
  Node( void ) { }
  Node( const uint8_t& _l, const uint32_t& _x, const uint32_t& _y ) :
    l(_l), x(_x), y(_y) { }

  /* Comparison */
  friend bool operator==( const Node& _lhs, const Node& _rhs )
    { return (_lhs.l == _rhs.l) && (_lhs.x == _rhs.x) && (_lhs.y == _rhs.y); }
  friend bool operator!=( const Node& _lhs, const Node& _rhs )
    { return !(_lhs == _rhs); }
};

struct VisitedNode : public Node
{
public:
  /* Variables */
  /// The distance from the source node
  uint32_t cost = 0;

  /* Constructors */
  VisitedNode( void ) { }
  VisitedNode( const Node& _node ) : Node( _node ) { }
};

class Grid
{
public:
  /* Constants */
  /// Preferred directions, y: 01* / x: 10*
  uint8_t preferred_direction[2] = { 1, 2 };

  /* Variables */
  /** Grid Information **/
  /// A 3-dimensional grid node array with size :
  ///   size_x * size_y * size_l
  uint8_t size_l = 0;
  uint32_t size_x = 0;
  uint32_t size_y = 0;

  /// Preferred directions of layers
  /// 0: prefer y-direction / 1: prefer x-direction
  std::vector<bool> layers;

  /// Two vectors that store the x- and y-values of non-uniform grid axes
  std::vector<uint32_t> axis_x;
  std::vector<uint32_t> axis_y;

  /// Derived grid widths of x- and y-axis to improve routing performance
  std::vector<uint32_t> axis_x_w;
  std::vector<uint32_t> axis_y_w;

  std::vector< std::vector< std::vector<GridNode> > > nodes;

  /* Functions */
  /** Initializing **/
  void set_axis_x( const std::vector<uint32_t>& _axis_x );
  void set_axis_y( const std::vector<uint32_t>& _axis_y );
  void set_layers( const std::vector<bool>& _layers );

  /// Add obstacle with two vertices (_l, _ax, _ay) and (_l, _bx, _by)
  void add_obstacle( const uint8_t& _l, const uint32_t& _ax,
    const uint32_t& _ay, const uint32_t& _bx, const uint32_t& _by );

  /** Node with coordinates to node with grid indices conversion **/
  Node convert_to_index( const Node& _node );

  /** Preferred Direction **/
  bool is_preferred_direction( const uint8_t& _l, const uint8_t& _dir );

private:
  /* Functions */
  void resize_grid_nodes( void );
  void calculate_grid_width_x( void );
  void calculate_grid_width_y( void );
  uint32_t get_x_index( const uint32_t& _x );
  uint32_t get_y_index( const uint32_t& _y );
};

struct Net
{
public:
  /* Variables */
  std::vector<Node> pins;
  std::vector<Node> route_nodes;

  /* Constructors */
  Net( void ) { }
  Net( const Node& );
  Net( const std::vector<Node>& );

  /* Functions */
  void add_route_node( const Node& _node );
  void add_route_nodes( const std::vector<Node>& _nodes );
};

struct RoutingOrder
{
public:
  bool operator() ( const VisitedNode& _lhs, const VisitedNode& _rhs )
    { return _lhs.cost > _rhs.cost; }
};

class Router
{
public:
  /* Variables */
  Grid grid;
  std::vector<Net> routed_nets;

  /* Functions */
  void add_net( const std::vector<Node>& _pins );

private:
  /* Variables */
  std::priority_queue<VisitedNode, std::vector<VisitedNode>, RoutingOrder>
    routing_queue;
  std::vector<Node> visited_nodes;

  /* Functions */
  /** Utilities **/
  uint32_t positive_diff( const uint32_t&, const uint32_t& );
  uint32_t manh_distance( const Node&, const Node& );

  /** Maze Routing **/
  /// Find a path to any target node and save the reached node
  bool find_target( const Node& _source, Node& _target );
  /// Backtrack the route and add the add the route nodes to _nodes
  void backtrack( const Node& _source, const Node& _target,
    std::vector<Node>& _nodes );
  void clear_visited_marks( void );
  void clear_net_marks( const Net& _net );
};

#endif
