/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx net>
 *         Autonomous Systems Lab <http://www.asl.ethz.ch/>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#include "Sprite.hpp"
#include "numeric.hpp"
#include "util.hpp"
#include "pdebug.hpp"
#include <iostream>


using namespace std;


namespace estar {
  
  
  /** \todo braindead implementation! */  
  Sprite::
  Sprite(double _radius, double _scale)
    : radius(_radius),
      scale(_scale),
      m_x0(0),
      m_y0(0),
      m_x1(0),
      m_y1(0)
  {
    const ssize_t offset(static_cast<ssize_t>(ceil(_radius / _scale)));
    for(ssize_t ix(-offset); ix <= offset; ++ix){
      const double x2(square(ix * _scale));
      for(ssize_t iy(-offset); iy <= offset; ++iy){
	const double rr(sqrt(square(iy * _scale) + x2));
	if(rr <= _radius){
	  m_area.push_back(sindex(ix, iy, rr));
	  if(rr >= _radius - _scale)
	    m_border.push_back(sindex(ix, iy, rr));
	  if (ix < m_x0)
	    m_x0 = ix;
	  if (ix > m_x1)
	    m_x1 = ix;
	  if (iy < m_y0)
	    m_y0 = iy;
	  if (iy > m_y1)
	    m_y1 = iy;
	}
      }
    }
    if(m_area.empty()){
      PVDEBUG("empty area, repairing with center index\n");
      m_area.push_back(sindex(0, 0, 0));
    }
    if(m_border.empty()){
      PVDEBUG("empty border, repairing with center index\n");
      m_border.push_back(sindex(0, 0, 0));
    }
  }
  
  
  void Sprite::
  Dump(std::ostream & os) const
  {
    const ssize_t offset(static_cast<ssize_t>(ceil(radius / scale)));
    const ssize_t dim(2 * offset + 1);
    array<char> sprite(dim, dim, '.'); // '.' means empty
    for(size_t ib(0); ib < m_border.size(); ++ib){
      ssize_t ix(m_border[ib].x + offset);
      ssize_t iy(m_border[ib].y + offset);
      if('.' == sprite[ix][iy])
	sprite[ix][iy] = 'x';	// 'x' means only border error
      else
	sprite[ix][iy] = '#';	// '#' means duplicate border error
    }
    for(size_t ia(0); ia < m_area.size(); ++ia){
      ssize_t ix(m_area[ia].x + offset);
      ssize_t iy(m_area[ia].y + offset);
      if('.' == sprite[ix][iy])
	sprite[ix][iy] = 'o';	// 'o' means only area
      else if('x' == sprite[ix][iy])
	sprite[ix][iy] = '*';	// '*' means border and area
      else
	sprite[ix][iy] = '%';	// '%' means area and duplicate border error
    }
    for(ssize_t iy(dim - 1); iy >= 0; --iy){
      for(ssize_t ix(0); ix < dim; ++ix)
	os << sprite[ix][iy] << ' ';
      os << "\n";
    }
  }
  
  
  void Sprite::
  GetBBox(ssize_t & x0, ssize_t & y0, ssize_t & x1, ssize_t & y1) const
  {
    x0 = m_x0;
    y0 = m_y0;
    x1 = m_x1;
    y1 = m_y1;
  }
  
}
