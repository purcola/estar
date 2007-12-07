/* 
 * Copyright (C) 2005 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef ESTAR_GRID_NODE_HPP
#define ESTAR_GRID_NODE_HPP


#include <estar/CSpace.hpp>
#include <boost/shared_ptr.hpp>


namespace estar {
  
  
  /**
     Holds a C-space vertex and associated data for a node in a Grid.
  */
  class GridNode
  {
  public:
    GridNode() {}
    
    GridNode(ssize_t _ix, ssize_t _iy, vertex_t _vertex)
      : ix(_ix), iy(_iy), vertex(_vertex) {}
    
    friend std::ostream & operator << (std::ostream & os, const GridNode & gn);
    
    ssize_t ix, iy;
    vertex_t vertex;
  };
  
  
  /**
     Utility for translating a grid index into coordinates. This is a
     vestige of graphics-specific distinguishing between HEXGRID and
     orthogonal meshes...
  */
  struct grid_postransform {
    virtual ~grid_postransform() {}
    virtual std::pair<double, double>
    operator()(double ix, double iy) const = 0;
  };
  
  struct grid_postransform_cartesian: public grid_postransform {
    virtual std::pair<double, double> operator()(double ix, double iy) const;
  };
  
  struct grid_postransform_hexgrid: public grid_postransform {
    virtual std::pair<double, double> operator()(double ix, double iy) const;
  };
  
  
  struct grid_bbox_compute {
    virtual ~grid_bbox_compute() {}
    virtual void operator()(double & x0, double & y0,
			    double & x1, double & y1) const = 0;
  };

  struct grid_bbox_compute_fixed: public grid_bbox_compute {
    grid_bbox_compute_fixed(double _x0, double _y0, double _x1, double _y1)
      : x0(_x0), y0(_y0), x1(_x1), y1(_y1) {}
    
    virtual void operator()(double & _x0, double & _y0,
			    double & _x1, double & _y1) const
    { _x0 = x0; _y0 = y0; _x1 = x1; _y1 = y1; }
    
    double x0, y0, x1, y1;
  };
  
  
  /**
     \todo Move this somewhere else?
  */
  class GridCSpace
    : public CustomCSpace<boost::shared_ptr<GridNode> >
  {
  public:
    GridCSpace(boost::shared_ptr<grid_postransform const> postransform,
	       boost::shared_ptr<grid_bbox_compute const> bbox_compute)
      : m_postransform(postransform), m_bbox_compute(bbox_compute) {}
    
    std::pair<double, double> ComputePosition(vertex_t vertex) const {
      boost::shared_ptr<GridNode const> const gg(Lookup(vertex));
      return (*m_postransform)(gg->ix, gg->iy);
    }
    
    void ComputeBBox(double & x0, double & y0, double & x1, double & y1) const
    { (*m_bbox_compute)(x0, y0, x1, y1); }
    
  protected:
    boost::shared_ptr<grid_postransform const> m_postransform;
    boost::shared_ptr<grid_bbox_compute const> m_bbox_compute;
  };
  
} // namespace estar

#endif // ESTAR_GRID_NODE_HPP