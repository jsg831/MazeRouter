#include "router.h"

void Grid::set_axis_x( const std::vector<uint32_t>& _axis_x )
{
  axis_x = _axis_x;
  size_x = _axis_x.size();
  std::sort( axis_x.begin(), axis_x.end() );
  calculate_grid_width_x();
  resize_grid_nodes();
}

void Grid::set_axis_y( const std::vector<uint32_t>& _axis_y )
{
  axis_y = _axis_y;
  size_y = _axis_y.size();
  std::sort( axis_y.begin(), axis_y.end() );
  calculate_grid_width_y();
  resize_grid_nodes();
}

void Grid::set_layers( const uint32_t& _l )
{
  size_l = _l;
  resize_grid_nodes();
}

void Grid::add_obstacle( const uint8_t& _l, const uint32_t& _ax,
  const uint32_t& _ay, const uint32_t& _bx, const uint32_t& _by )
{
  for ( auto y = _ay; y <= _by; ++y )
  {
    for ( auto x = _ax; x <= _bx; ++x )
    {
      nodes[_l][y][x].set_obstacle( 1 );
    }
  }
}

uint32_t Grid::get_x_index( const uint32_t& _x )
{
  return std::upper_bound( axis_x.begin(), axis_x.end(), _x ) - axis_x.begin();
}

uint32_t Grid::get_y_index( const uint32_t& _y )
{
  return std::upper_bound( axis_y.begin(), axis_y.end(), _y ) - axis_y.begin();
}

void Grid::resize_grid_nodes( void )
{
  nodes.resize( size_l );
  for ( auto& y : nodes )
  {
    y.resize( size_y );
    for ( auto& x : y )
    {
      x.resize( size_x );
    }
  }
}

void Grid::calculate_grid_width_x( void )
{
  axis_x_w.resize( axis_x.size()-1 );
  for ( auto i = 1; i < axis_x.size(); ++i )
  {
    axis_x_w[i-1] = axis_x[i] - axis_x[i-1];
  }
}

void Grid::calculate_grid_width_y( void )
{
  axis_y_w.resize( axis_y.size()-1 );
  for ( auto i = 1; i < axis_y.size(); ++i )
  {
    axis_y_w[i-1] = axis_y[i] - axis_y[i-1];
  }
}
