#include "router.h"

void Router::add_net( const std::vector<Node>& _pins )
{
  routed_nets.resize( routed_nets.size() + 1 );
  Net& net = routed_nets.back();
  std::vector<Node>& route_nodes = net.route_nodes;

  /* Add pins to the grid */
  for ( const auto& pin : _pins )
  {
    grid.nodes[pin.l][pin.y][pin.x].set_pin( 1 );
  }

  /** Pin order for routing **/
  std::vector<Node> pin_order;
  std::vector<bool> pin_selected( _pins.size(), 0 );
  std::vector< std::vector<uint32_t> > dist_matrix;

  /* Set pin order */
  /// Initialize the adjacent matrix for Prim's algorithm
  dist_matrix.resize( _pins.size() );
  for ( auto& row : dist_matrix)
  {
    row.resize( _pins.size() );
  }

  for ( auto i = 0; i < _pins.size(); ++i )
  {
    for ( auto j = 0; j < _pins.size(); ++j )
    {
      dist_matrix[i][j] = manh_distance( _pins[i], _pins[j] );
    }
  }

  /// Prim's algorithm
  uint32_t p = 0;
  pin_selected[0] = 1;
  while ( pin_order.size() != (_pins.size() - 1) )
  {
    uint32_t next_p = 0;
    uint32_t lowest_dist = UINT32_MAX;
    for ( auto i = 0; i < _pins.size(); ++i )
    {
      if ( pin_selected[i] ) continue;
      if ( dist_matrix[p][i] < lowest_dist )
      {
        lowest_dist = dist_matrix[p][i];
        next_p = i;
      }
    }
    pin_order.push_back( _pins[next_p] );
    pin_selected[next_p] = 1;
    p = next_p;
  }

  /* Iteratively routing by pin order */
  bool has_routed = true;
  Node target = _pins[0];
  grid.nodes[target.l][target.y][target.x].set_target( 1 );
  route_nodes.push_back( target );
  for ( const auto& source : pin_order )
  {
    if ( find_target( source, target ) )
    {
      backtrack( source, target, route_nodes );
      clear_visited_marks();
    } else {
      has_routed = false;
      clear_visited_marks();
      break;
    }
  }
  clear_net_marks( net );

  if ( !has_routed )
    routed_nets.resize( routed_nets.size()-1 );
}

uint32_t Router::positive_diff( const uint32_t& _a, const uint32_t& _b )
{
  return (_a > _b) ? (_a - _b) : (_b - _a);
}

uint32_t Router::manh_distance( const Node& _n1, const Node& _n2 )
{
  return positive_diff( grid.axis_x[_n1.x], grid.axis_x[_n2.x] )
    + positive_diff( grid.axis_y[_n1.y], grid.axis_y[_n2.y] );
}

bool Router::find_target( const Node& _source, Node& _target )
{
  /* Variables */
  VisitedNode node;
  VisitedNode node_q;
  bool target_reached = false;

  /* Initializing */
  node = _source;
  node.cost = 0;

  visited_nodes.clear();

  /// Push the source node to the routing queue
  routing_queue.push( node );

  /* Pathfinding */
  while ( !routing_queue.empty() )
  {
    node = routing_queue.top();
    routing_queue.pop();

    visited_nodes.push_back( node );


    /// Initialize or update the target node if a target node is reached.
    if ( grid.nodes[node.l][node.y][node.x].target() )
    {
      if ( !target_reached
        || node.cost < grid.nodes[_target.l][_target.y][_target.x].cost )
        _target = node;
      target_reached = true;
    }

    /// If no better solution exists, return the function.
    /// Since the cost of every node is initialized to UINT32_MAX, the condition
    /// is only true when the target node is visited.
    if ( target_reached &&
      node.cost >= grid.nodes[_target.l][_target.y][_target.x].cost )
      return true;

    /// Propagate in six directions
    for ( auto dir = 0; dir < 6; ++dir )
    {
      node_q = node;

      switch ( dir )
      {
        case 0:
          node_q.l += 1;
          if ( node_q.l != grid.size_l )
            node_q.cost += VIA_COST;
          else continue;
          break;
        case 1:
          node_q.l -= 1;
          if ( node_q.l != UINT8_MAX )
            node_q.cost += VIA_COST;
          else continue;
          break;
        case 2:
          node_q.y += 1;
          if ( node_q.y != grid.size_y )
            node_q.cost += grid.axis_y_w[node_q.y-1];
          else continue;
          break;
        case 3:
          node_q.y -= 1;
          if ( node_q.y != UINT32_MAX )
            node_q.cost += grid.axis_y_w[node_q.y];
          else continue;
          break;
        case 4:
          node_q.x += 1;
          if ( node_q.x != grid.size_x )
            node_q.cost += grid.axis_x_w[node_q.x-1];
          else continue;
          break;
        case 5:
          node_q.x -= 1;
          if ( node_q.x != UINT32_MAX )
            node_q.cost += grid.axis_x_w[node_q.x];
          else continue;
          break;
      }

      /// Add penalty to cost if not in preferred direction
      if ( !grid.is_preferred_direction( node.l, dir ) )
      {
        node_q.cost += DIR_PENL;
      }

      if ( !grid.nodes[node_q.l][node_q.y][node_q.x].obstacle()
        && node_q.cost < grid.nodes[node_q.l][node_q.y][node_q.x].cost )
      {
        routing_queue.push( node_q );
        grid.nodes[node_q.l][node_q.y][node_q.x].cost = node_q.cost;
        grid.nodes[node_q.l][node_q.y][node_q.x].set_visited( 1 );
        grid.nodes[node_q.l][node_q.y][node_q.x].set_direction( dir );
      }
    }
  }
  /// Return true if the target is reached.
  return target_reached;
}

void Router::backtrack( const Node& _source, const Node& _target,
  std::vector<Node>& _nodes )
{
  Node node = _target;

  /** Backtracking **/
  while ( node != _source )
  {
    grid.nodes[node.l][node.y][node.x].set_target( 1 );

    if ( node != _target )
      _nodes.push_back( node );

    switch ( grid.nodes[node.l][node.y][node.x].direction() )
    {
      case 0:
        node.l -= 1;
        break;
      case 1:
        node.l += 1;
        break;
      case 2:
        node.y -= 1;
        break;
      case 3:
        node.y += 1;
        break;
      case 4:
        node.x -= 1;
        break;
      case 5:
        node.x += 1;
        break;
    }
  }
  grid.nodes[_source.l][_source.y][_source.x].set_target( 1 );
  _nodes.push_back( node );
}

void Router::clear_visited_marks( void )
{
  Node node;

  /** Reset the grid **/
  /// Clear all marks on nodes in the routing queue
  while ( !routing_queue.empty() )
  {
    node = routing_queue.top();
    routing_queue.pop();
    grid.nodes[node.l][node.y][node.x].cost = UINT32_MAX;
    grid.nodes[node.l][node.y][node.x].set_visited( 0 );
  }
  /// Clear all marks on visited nodes
  for ( const auto& node : visited_nodes )
  {
    grid.nodes[node.l][node.y][node.x].cost = UINT32_MAX;
    grid.nodes[node.l][node.y][node.x].set_visited( 0 );
  }
  visited_nodes.clear();
}

void Router::clear_net_marks( const Net& _net )
{
  for ( const auto& node : _net.route_nodes )
  {
    grid.nodes[node.l][node.y][node.x].set_routed( 1 );
    grid.nodes[node.l][node.y][node.x].set_target( 0 );
  }
}
