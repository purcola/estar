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


#ifndef ESTAR_PROPAGATOR_HPP
#define ESTAR_PROPAGATOR_HPP


#include <estar/base.hpp>
#include <estar/Queue.hpp>
#include <list>


namespace estar {
  
  
  class Upwind;
  class Algorithm;
  
  
  /**
     Propagator set of a node. Used for filtering the neighborhood of
     a node before passing it to a specific Kernel subtype for
     interpolation. To create a Propagator instance, use a
     PropagatorFactory. This allows us to easily experiment with
     alternative formulations of how to compute the propagator set.
  */
  class Propagator
  {
  private:
    friend class PropagatorFactory;
    Propagator(vertex_t target_vertex, double target_meta);
    
  public:
    typedef std::list<vertex_t> backpointer_t;
    typedef backpointer_t::iterator backpointer_it;
    
    double GetTargetMeta() const;
    vertex_t GetTargetVertex() const;
    std::pair<const_queue_it, const_queue_it> GetUpwindNeighbors() const;
    std::size_t GetNUpwindNeighbors() const;
    
    void AddBackpointer(vertex_t vertex);
    std::pair<backpointer_it, backpointer_it> GetBackpointers();
    std::size_t GetNBackpointers() const;
    
  private:
    const vertex_t m_target_vertex;
    const double m_target_meta;
    queue_t m_nbor;
    backpointer_t m_bp;
  };
  
} // namespace estar

#endif // ESTAR_PROPAGATOR_HPP
