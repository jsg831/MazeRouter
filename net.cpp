#include "router.h"

Net::Net( const Node& _pin )
{
  pins.push_back( _pin );
}

Net::Net( const std::vector<Node>& _pins )
{
  pins = _pins;
}

void Net::add_route_node( const Node& _node )
{
  route_nodes.push_back( _node );
}

void Net::add_route_nodes( const std::vector<Node>& _nodes )
{
  route_nodes.insert( route_nodes.begin(), _nodes.begin(), _nodes.end() );
}
