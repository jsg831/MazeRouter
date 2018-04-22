#include "router.h"

void Router::add_path( const Node& _source, const Node& _target )
{
  if ( find_path( _source, _target ) )
    backtrack( _source, _target );
  clear_marks();
}

bool Router::find_path( const Node& _source, const Node& _target )
{
  /* Variables */
  Path path;
  VisitedNode node;
  VisitedNode node_q;

  /* Initializing */
  path.source = _source;
  path.target = _target;

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

    /// If no better solution exists, return the function.
    /// Since the cost of every node is initialized to UINT32_MAX, the condition
    /// is only true when the target node is visited.
    if ( node.cost >= grid.nodes[_target.l][_target.y][_target.x].cost )
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
  return false;
}

void Router::backtrack( const Node& _source, const Node& _target )
{
  /* Variables */
  Node node;
  Path path;

  /* Initializing */
  node = _target;

  path.nodes.clear();

  /** Backtracking **/
  while ( node != _source )
  {
    path.nodes.push_back( node );
    grid.nodes[node.l][node.y][node.x].set_routed( 1 );

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
  grid.nodes[_source.l][_source.y][_source.x].set_routed( 1 );
  path.nodes.push_back( _source );

  routed_paths.push_back( path );
  return;
}

void Router::clear_marks( void )
{
  /* Variables */
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
