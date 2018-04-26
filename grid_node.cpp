#include "router.h"

#define OBS_BIT 0
#define VIS_BIT 1
#define D_0_BIT 2
#define D_1_BIT 3
#define D_2_BIT 4
#define RTD_BIT 5
#define SRC_BIT 6
#define TAR_BIT 7

bool GridNode::obstacle( void )
{
  return rt_flags.test( OBS_BIT );
}

void GridNode::set_obstacle( const bool& _flag )
{
  rt_flags.set( OBS_BIT, _flag );
}

bool GridNode::visited( void )
{
  return rt_flags.test( VIS_BIT );
}

void GridNode::set_visited( const bool& _flag )
{
  rt_flags.set( VIS_BIT, _flag );
}

bool GridNode::routed( void )
{
  return rt_flags.test( RTD_BIT );
}

void GridNode::set_routed( const bool& _flag )
{
  rt_flags.set( RTD_BIT, _flag );
}

bool GridNode::source( void )
{
  return rt_flags.test( SRC_BIT );
}

void GridNode::set_source( const bool& _flag )
{
  rt_flags.set( SRC_BIT, _flag );
}

bool GridNode::target( void )
{
  return rt_flags.test( TAR_BIT );
}

void GridNode::set_target( const bool& _flag )
{
  rt_flags.set( TAR_BIT, _flag );
}

uint8_t GridNode::direction( void )
{
  return (rt_flags.test(D_2_BIT) << 2) | (rt_flags.test(D_1_BIT) << 1) |
    (rt_flags.test(D_0_BIT));
}

void GridNode::set_direction( const uint8_t& _dir )
{
  rt_flags.set( D_2_BIT, (_dir >> 2) & 1 );
  rt_flags.set( D_1_BIT, (_dir >> 1) & 1 );
  rt_flags.set( D_0_BIT, _dir & 1 );
}

void GridNode::clear_direction( void )
{
  rt_flags.reset( D_2_BIT );
  rt_flags.reset( D_1_BIT );
  rt_flags.reset( D_0_BIT );
}
