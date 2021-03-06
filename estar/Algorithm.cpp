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


#include "Algorithm.hpp"
#include "Kernel.hpp"
#include "Propagator.hpp"
#include "util.hpp"
#include "pdebug.hpp"
#include <boost/assert.hpp>
#include <iostream>


using namespace boost;
using namespace std;


namespace estar {
  
  
  Algorithm::
  Algorithm(shared_ptr<BaseCSpace> cspace,
	    bool check_upwind,
	    bool check_local_consistency,
	    bool check_queue_key,
	    bool auto_reset,
	    bool auto_flush)
    : m_cspace(cspace),
      m_step(0),
      m_last_computed_value(-1),
      m_last_computed_vertex(0),
      m_last_popped_key(-1),
      m_pending_reset(false),
      m_auto_reset(auto_reset),
      m_auto_flush(auto_flush),
      m_cspace_graph(cspace->GetGraph()),
      m_value(cspace->GetValueMap()),
      m_meta(cspace->GetMetaMap()),
      m_rhs(cspace->GetRhsMap()),
      m_flag(cspace->GetFlagMap()), 
      m_propfactory(new PropagatorFactory(m_queue, m_upwind, m_cspace_graph,
					  m_value, m_meta, m_rhs, m_flag,
					  check_upwind,
					  check_local_consistency,
					  check_queue_key))
  {
  }
  
  
  /**
     \todo Shouldn't we test using absval(old-new)<epsilon instead of
     old==new?
  */
  void Algorithm::
  SetMeta(vertex_t vertex, double meta, const Kernel & kernel)
  {
    if (absval(get(m_meta, vertex) - meta) < epsilon)
      return;
    put(m_meta, vertex, meta);
    UpdateVertex(vertex, kernel);
    if (m_auto_reset)
      m_pending_reset = true;
  }
  
  
  void Algorithm::
  ComputeOne(const Kernel & kernel, double slack)
  {
    if (m_auto_flush)
      while (HaveWork())
	DoComputeOne(kernel, slack);
    else
      DoComputeOne(kernel, slack);
  }
  
  
  void Algorithm::
  DoComputeOne(const Kernel & kernel, double slack)
  {
    if(m_pending_reset){
      Reset();
      m_pending_reset = false;
    }
    if(m_queue.IsEmpty())
      return;
    ++m_step;
    
    const double popped_key(m_queue.Get().begin()->first);
    const vertex_t vertex(m_queue.Pop(m_flag));
    const double rhs(get(m_rhs, vertex));
    const double val(get(m_value, vertex));
    
    if(absval(val - rhs) <= slack){
      PVDEBUG("vertex is slack   v: %g   rhs: %g   delta: %g   slack: %g\n",
	      val, rhs, val - rhs, slack);
      return;
    }
    
    m_last_computed_value = rhs;
    m_last_computed_vertex = vertex;
    m_last_popped_key = popped_key;
    
    if(val > rhs){
      PVDEBUG("vertex gets lowered   v: %g   rhs: %g   delta: %g\n",
	      val, rhs, val - rhs);
      put(m_value, vertex, rhs);
      adjacency_it in, nend;
      tie(in, nend) = adjacent_vertices(vertex, m_cspace_graph);
      for(/**/; in != nend; ++in)
	UpdateVertex(*in, kernel);
    }
    
    else{
      PVDEBUG("vertex gets raised   v: %g rhs: %g   delta: %g\n",
	      val, rhs, rhs - val);
      put(m_value, vertex, infinity);
      
#define RE_PROPAGATE_LAST
//#undef RE_PROPAGATE_LAST
#ifndef RE_PROPAGATE_LAST
      PVDEBUG("variant: update raised node before others\n");
      UpdateVertex(vertex, kernel);
#endif // ! RE_PROPAGATE_LAST
      
#define RAISE_DOWNWIND_ONLY
//#undef RAISE_DOWNWIND_ONLY
#ifdef RAISE_DOWNWIND_ONLY
      PVDEBUG("variant: expand only downwind neighbors after raise\n");
      // IMPORTANT: Get a copy, because m_upwind is modified inside
      // the loop by UpdateVertex()!!!
      Upwind::set_t dwnbors(m_upwind.GetDownwind(vertex));
      for(Upwind::set_t::const_iterator id(dwnbors.begin());
	  id != dwnbors.end(); ++id)
	UpdateVertex(*id, kernel);
#else // RAISE_DOWNWIND_ONLY
      PVDEBUG("variant: expand all neighbors after raise\n");
      adjacency_it in, nend;
      tie(in, nend) = adjacent_vertices(vertex, m_cspace_graph);
      for(/**/; in != nend; ++in)
	UpdateVertex(*in, kernel);
#endif // RAISE_DOWNWIND_ONLY
      
#ifdef RE_PROPAGATE_LAST
      PVDEBUG("variant: update raised node after others\n");
      UpdateVertex(vertex, kernel);
#endif // RE_PROPAGATE_LAST
    }
  }
  
  
  bool Algorithm::
  HaveWork() const
  {
    return m_pending_reset || ( ! m_queue.IsEmpty());
  }
  
  
  void Algorithm::
  AddGoal(vertex_t vertex, double value)
  {
    const flag_t flag(get(m_flag, vertex));
    if((flag & GOAL) && (absval(get(m_rhs, vertex) - value) < epsilon)){
      PVDEBUG("no change");
      return;
    }
    put(m_rhs,   vertex, value);
    put(m_flag,  vertex, static_cast<flag_t>(flag | GOAL));
    m_goalset.insert(vertex);
    if(absval(get(m_value, vertex) - value) < epsilon){
      PVDEBUG("same value");
      return;
    }
    PVDEBUG("needs (re)queuing\n");
    put(m_value, vertex, infinity);
    m_queue.Requeue(vertex, m_flag, m_value, m_rhs);
    if (m_auto_reset)
      m_pending_reset = true;
  }
  
  
  void Algorithm::
  RemoveGoal(vertex_t vertex)
  {
    if(get(m_flag, vertex) & GOAL){
      m_goalset.erase(vertex);
      put(m_flag, vertex, NONE);
      m_pending_reset = true;
#ifndef WIN32
# warning "could use m_pending_reset more to avoid work when not needed"
#endif // WIN32
	}
  }
  
  
  void Algorithm::
  RemoveAllGoals()
  {
    if(m_goalset.empty())
      return;
    for(goalset_t::iterator ig(m_goalset.begin()); ig != m_goalset.end(); ++ig)
      put(m_flag, *ig, NONE);
    m_goalset.clear();
    m_pending_reset = true;
  }
  
  
  bool Algorithm::
  IsGoal(vertex_t vertex)
    const
  {
    return get(m_flag, vertex) & GOAL;
  }
  
  
  void Algorithm::
  Reset()
  {
    // Note: obstacle information is not in the flag, but in the meta,
    // which doesn't get touched here.
    m_queue.Clear();
    vertex_it iv, vend;
    tie(iv, vend) = vertices(m_cspace_graph);
    for(/**/; iv != vend; ++iv){

#ifndef WIN32
# warning Is it a waste of time to clear the upwind structure? Does it create or avoid inconsistencies?
#endif // WIN32
		//clear_vertex(get(m_upwind_v, *iv), m_upwind);

      if(get(m_flag, *iv) & GOAL){
	put(m_value, *iv, infinity);
	put(m_flag,  *iv, GOAL);
	
	m_queue.Requeue(*iv, m_flag, m_value, m_rhs);
      }
      else{
	put(m_value, *iv, infinity);
	put(m_rhs,   *iv, infinity);
	put(m_flag,  *iv, NONE);
      }
    }
  }
  

  void Algorithm::
  UpdateVertex(vertex_t vertex, const Kernel & kernel)
  {
    const flag_t flag(get(m_flag, vertex));
    if(flag & GOAL){
      PVDEBUG("i: %lu f: %s special goal handling\n", vertex, flag_name(flag));
      return;
    }
    else{
      scoped_ptr<Propagator> prop(m_propfactory->Create(vertex));
      const double rhs(kernel.Compute(*prop));
      put(m_rhs, vertex, rhs);

      m_upwind.RemoveIncoming(vertex);
      Propagator::backpointer_it ibp, bpend;
      tie(ibp, bpend) = prop->GetBackpointers();
      for(/**/; ibp != bpend; ++ibp)
	m_upwind.AddEdge(*ibp, vertex);
      
      PVDEBUG("i: %lu f: %s v: %g rhs: %g\n",
	      vertex, flag_name(flag), get(m_value, vertex), rhs);

      m_queue.Requeue(vertex, m_flag, m_value, m_rhs);
    }
  }
  
  
  void Algorithm::
  AddVertex(vertex_t vertex, const Kernel & kernel)
  {
    m_cspace->SetValue(vertex, infinity);
    m_cspace->SetRhs(vertex, infinity);
    m_cspace->SetFlag(vertex, NONE);
#define ALWAYS_UPDATE
#ifdef ALWAYS_UPDATE
    UpdateVertex(vertex, kernel);
#else // ALWAYS_UPDATE
    if (m_queue.Get().empty())
      UpdateVertex(vertex, kernel);
    else {
      double const thresh(m_queue.Get().begin()->first);
      for (edge_read_iteration inbor(m_cspace->begin(vertex));
	   inbor.not_at_end(); ++inbor) {
	if (m_cspace->GetValue(*inbor) < thresh) {
	  UpdateVertex(vertex, kernel);
	  return;
	}
      }
    }
#endif // ALWAYS_UPDATE
  }
  
} // namespace estar
