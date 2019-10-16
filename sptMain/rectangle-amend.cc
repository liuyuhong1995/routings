/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Amend Inside method
 * private value must be initialized as int type
 */
#include "rectangle-amend.h"
#include "ns3/vector.h"
#include "ns3/assert.h"
#include "ns3/fatal-error.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include "ns3/log.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("RectangleAmend");

Rectangle::Rectangle (double _xMin, double _xMax,      //有参数的构造函数
                      double _yMin, double _yMax)
  : xMin (_xMin),
    xMax (_xMax),
    yMin (_yMin),
    yMax (_yMax)
{
}

Rectangle::Rectangle ()     //无参数的构造函数
  : xMin (0.0),
    xMax (0.0),
    yMin (0.0),
    yMax (0.0)
{
}

bool
Rectangle::IsInside (const Vector &position) const
{
	bool inside = true;
	int32_t intX=nearbyint(position.x);
	int32_t intY=nearbyint(position.y);//To avoid error value caused by clock
	NS_LOG_LOGIC("position.x="<<position.x <<", intX="<<intX
			<<",position.y="<<position.y <<", intY="<<intY);
	NS_LOG_LOGIC(
			"xMax="<< this->xMax<<",xMin="<<this->xMin<<", yMax="<<this->yMax<<", yMin="<<this->yMin);
	inside = intX <= this->xMax && intX >= this->xMin
			&& intY <= this->yMax && intY >= this->yMin;
	NS_LOG_LOGIC("inside="<<inside);
	return inside;
}

Rectangle::Side
Rectangle::GetClosestSide (const Vector &position) const
{
  double xMinDist = std::abs (position.x - this->xMin);
  double xMaxDist = std::abs (this->xMax - position.x);
  double yMinDist = std::abs (position.y - this->yMin);
  double yMaxDist = std::abs (this->yMax - position.y);
  double minX = std::min (xMinDist, xMaxDist);
  double minY = std::min (yMinDist, yMaxDist);
  if (minX < minY)
    {
      if (xMinDist < xMaxDist)
        {
          return LEFT;
        }
      else
        {
          return RIGHT;
        }
    }
  else
    {
      if (yMinDist < yMaxDist)
        {
          return BOTTOM;
        }
      else
        {
          return TOP;
        }
    }
}

Vector
Rectangle::CalculateIntersection (const Vector &current, const Vector &speed) const
{
  NS_ASSERT (IsInside (current));
  double xMaxY = current.y + (this->xMax - current.x) / speed.x * speed.y;
  double xMinY = current.y + (this->xMin - current.x) / speed.x * speed.y;
  double yMaxX = current.x + (this->yMax - current.y) / speed.y * speed.x;
  double yMinX = current.x + (this->yMin - current.y) / speed.y * speed.x;
  bool xMaxYOk = (xMaxY <= this->yMax && xMaxY >= this->yMin);
  bool xMinYOk = (xMinY <= this->yMax && xMinY >= this->yMin);
  bool yMaxXOk = (yMaxX <= this->xMax && yMaxX >= this->xMin);
  bool yMinXOk = (yMinX <= this->xMax && yMinX >= this->xMin);
  if (xMaxYOk && speed.x >= 0)
    {
      return Vector (this->xMax, xMaxY, 0.0);
    }
  else if (xMinYOk && speed.x <= 0)
    {
      return Vector (this->xMin, xMinY, 0.0);
    }
  else if (yMaxXOk && speed.y >= 0)
    {
      return Vector (yMaxX, this->yMax, 0.0);
    }
  else if (yMinXOk && speed.y <= 0)
    {
      return Vector (yMinX, this->yMin, 0.0);
    }
  else
    {
      NS_ASSERT (false);
      // quiet compiler
      return Vector (0.0, 0.0, 0.0);
    }

}

ATTRIBUTE_HELPER_CPP (Rectangle);

/**
 * \brief Stream insertion operator.
 *
 * \param os the stream
 * \param rectangle the rectangle
 * \returns a reference to the stream
 */
std::ostream &
operator << (std::ostream &os, const Rectangle &rectangle)
{
  os << rectangle.xMin << "|" << rectangle.xMax << "|" << rectangle.yMin << "|" << rectangle.yMax;
  return os;
}
/**
 * \brief Stream extraction operator.
 *
 * \param is the stream
 * \param rectangle the rectangle
 * \returns a reference to the stream
 */
std::istream &
operator >> (std::istream &is, Rectangle &rectangle)
{
  char c1, c2, c3;
  is >> rectangle.xMin >> c1 >> rectangle.xMax >> c2 >> rectangle.yMin >> c3 >> rectangle.yMax;
  if (c1 != '|' ||
      c2 != '|' ||
      c3 != '|')
    {
      is.setstate (std::ios_base::failbit);
    }
  return is;
}


} // namespace ns3
