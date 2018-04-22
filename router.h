#ifndef _ROUTER_H
#define _ROUTER_H

#define DEBUG 1

#define VIA_COST 1

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

  uint8_t direction( void );
  void set_direction( const uint8_t& _dir );
  void clear_direction( void );

private:
  /// Flags for routing (4-byte in size)
  std::bitset<6> rt_flags = 0;
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
  /*************************************************************/
};

class Grid
{
public:
  /* Variables */
  /** Grid Information **/
  /// A 3-dimensional grid node array with size :
  ///   size_x * size_y * size_l
  uint8_t size_l = 0;
  uint32_t size_x = 0;
  uint32_t size_y = 0;

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
  void set_layers( const uint32_t& _l );

  /// Add obstacle with two vertices (_l, _ax, _ay) and (_l, _bx, _by)
  void add_obstacle( const uint8_t& _l, const uint32_t& _ax,
    const uint32_t& _ay, const uint32_t& _bx, const uint32_t& _by );

  /** X-/Y-Coordinate to Index Conversion **/
  uint32_t get_x_index( const uint32_t& _x );
  uint32_t get_y_index( const uint32_t& _y );

private:
  /* Functions */
  void resize_grid_nodes( void );
  void calculate_grid_width_x( void );
  void calculate_grid_width_y( void );
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

struct Path
{
public:
  /* Variables */
  Node source;
  Node target;

  /// Nodes on the path including the source and the target
  std::vector<Node> nodes;
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
  std::vector<Path> routed_paths;

  /* Functions */
  void add_path( const Node& _source, const Node& _target );

private:
  /* Variables */
  std::priority_queue<VisitedNode, std::vector<VisitedNode>, RoutingOrder>
    routing_queue;
  std::vector<Node> visited_nodes;

  /* Functions */
  bool find_path( const Node& _source, const Node& _target );
  void backtrack( const Node& _source, const Node& _target );
  void clear_marks( void );
};

#endif
