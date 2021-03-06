/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx net>
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


#include "Upwind.hpp"
#include "util.hpp"
#include "pdebug.hpp"


namespace estar {
  
  
  bool Upwind::
  HaveEdge(vertex_t from, vertex_t to) const
  {
    const set_t & fs(m_from_to[from]);
    PVDEBUG("from: %lu to: %lu %s\n", from, to,
	    ( ! fs.empty()) && (fs.find(to) != fs.end()) ? "TRUE" : "FALSE");
    return ( ! fs.empty()) && (fs.find(to) != fs.end());
  }
  
  
  void Upwind::
  AddEdge(vertex_t from, vertex_t to)
  {
    if( ! HaveEdge(to, from))
      PVDEBUG("from: %lu to: %lu\n", from, to);
    else{
      PVDEBUG("from: %lu to: %lu REPLACE opposite edge\n", from, to);
      RemoveEdge(to, from);
    }
    m_from_to[from].insert(to);
    m_to_from[to].insert(from);
  }
  
  
  void Upwind::
  RemoveEdge(vertex_t from, vertex_t to)
  {
    PVDEBUG("from: %lu to: %lu\n", from, to);
    m_from_to[from].erase(to);
    m_to_from[to].erase(from);
  }
  
  
  void Upwind::
  RemoveIncoming(vertex_t to)
  {
    set_t & ts(m_to_from[to]);
    if(ts.empty()){
      PVDEBUG("to: %lu EMPTY\n", to);
      return;
    }
    debugos dbg;
    // bugfix from Filip: calling RemoveEdge() directly can invalidate
    // the iterator, so first store them in a temporary area
    std::vector<vertex_t> vertices_to_delete(8); // allocate 'probably enough'
    for(set_t::iterator ifrom(ts.begin()); ifrom != ts.end(); ++ifrom){
      dbg << " " << *ifrom;
      vertices_to_delete.push_back(*ifrom);
    }
    for(std::vector<vertex_t>::iterator it(vertices_to_delete.begin());
	it != vertices_to_delete.end();
	++it){
      RemoveEdge(*it, to);
    }
    PVDEBUG("to: %lu from:%s\n", to, dbg.str().c_str());
  }
  
  
  const Upwind::set_t & Upwind::
  GetDownwind(vertex_t from) const
  {
    return m_from_to[from];
  }
  
} // namespace estar
