#! /usr/bin/env python3

#  BSD 3-Clause License
#
#  Copyright (c) 2019, David Wuthier - daw@mp.aau.dk
#  Aalborg University
#  Robotics, Vision and Machine Intelligence Laboratory
#  Department of Materials and Production
#  A. C. Meyers Vaenge 15, 2450 Copenhagen SV, Denmark
#  http://rvmi.aau.dk/
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from numpy import array
from numpy import clip

SUCCESS = 1
RUNNING = 0
FAILURE = -1

def success():
  return SUCCESS

def running():
  return RUNNING

def failure():
  return FAILURE

def condition(value, success_id = SUCCESS, failure_id = FAILURE):
  if value:
    return success_id

  return failure_id

def check(value):
  if value:
    return SUCCESS

  return RUNNING

def sequence_binary(functionA, functionB):
  A = functionA()
  if A > 0:
    return functionB()
  else:
    return A

def selector_binary(functionA, functionB):
  A = functionA()
  if A < 0:
    return functionB()
  else:
    return A

def parallel_binary_barrier(functionA, functionB):
  A = functionA()
  B = functionB()

  if A > 0 or (A == 0 and B < 0):
    return B
  else:
    return A

def sequence_n_ary(children):
  for child in children:
    status = child()
    if status <= 0:
      return status
  return status

def selector_n_ary(children):
  for child in children:
    status = child()
    if status >= 0:
      return status
  return status

def parallel_n_ary_barrier(children):
  n = len(children)
  statii = n * [0]
  for i in range(n):
    status = children[i]
    if status < 0:
      return status
    statii[i] = status
  return min(statii)

def parallel_n_ary_preemptive(children):
  for child in children:
    status = child()
    if status != 0:
      return status
  return RUNNING

def sequence(*children):
  return lambda: sequence_n_ary(children)

def selector(*children):
  return lambda: selector_n_ary(children)

def parallel(*children):
  return lambda: parallel_n_ary_barrier(children)

class FiniteStateMachine(object):
  def __init__(self, nodes):
    self.nodes = nodes
    self.token = list(self.nodes.keys())[0]
    self.transitions = {}
    self.markers = {}

  def set_start(self, key):
    self.token = key
    return SUCCESS

  def get_reset(self, key):
    return lambda: self.set_start(key)

  def add_transition(self, source, target, track = '', direct = False, status = SUCCESS):
    if not source in self.transitions:
      self.transitions[source] = {}

    if not track in self.transitions[source]:
      self.transitions[source][track] = {}

    if direct:
      self.transitions[source][track]['direct'] = target
    else:
      self.transitions[source][track][status] = target

  def add_marker(self, key, track = '', direct = False, status = SUCCESS, outcome = SUCCESS):
    if not key in self.markers:
      self.markers[key] = {}

    if not track in self.markers[key]:
      self.markers[key][track] = {}

    if direct:
      self.markers[key][track]['direct'] = outcome
    else:
      self.markers[key][track][status] = outcome

  def get_handle(self, track):
    return lambda: self._update(track)

  def _update(self, track):
    connected = self.token in self.transitions \
                 and track in self.transitions[self.token]

    while connected and 'direct' in self.transitions[self.token][track]:
      self.token = self.transitions[self.token][track]['direct']
      connected = self.token in self.transitions \
                   and track in self.transitions[self.token]

    marked = self.token in self.markers \
              and track in self.markers[self.token]

    if marked and 'direct' in self.markers[self.token][track]:
      return self.markers[self.token][track]['direct']

    if not connected and not marked:
      return FAILURE

    status = self.nodes[self.token]()

    response = RUNNING

    if marked and status in self.markers[self.token][track]:
      response = self.markers[self.token][track][status]

    if connected and status in self.transitions[self.token][track]:
      self.token = self.transitions[self.token][track][status]

    return response

class Node(object):
  def __init__(self, function):
    self.function = function

  def __or__(self, other):
    return Node(lambda: parallel_binary_barrier(self.function, other.function))

  def __mul__(self, other):
    return Node(lambda: sequence_binary(self.function, other.function))

  def __add__(self, other):
    return Node(lambda: selector_binary(self.function, other.function))

  def __lshift__(self, other):
    fsm = FiniteStateMachine({'lhs': self.function, 'rhs': other.function})

    fsm.set_start('lhs')

    fsm.add_transition('lhs', 'rhs', status = FAILURE)
    fsm.add_transition('rhs', 'lhs')
    fsm.add_transition('rhs', 'lhs', status = FAILURE)

    fsm.add_marker('lhs')
    fsm.add_marker('rhs')
    fsm.add_marker('rhs', status = FAILURE, outcome = FAILURE)

    return Node(fsm.get_handle(''))

  def tick(self):
    return self.function()

  def __call__(self):
    return self.function()

  def __invert__(self):
    return Node(lambda: -self.function())

def selector_star(*leaves):
  n = len(leaves)

  nodes = {}

  for i in range(n):
    nodes[str(i)] = leaves[i]

  fsm = FiniteStateMachine(nodes.values(), nodes.keys())

  fsm.set_start(nodes['0'])

  for i in range(1, n):
    fsm.add_transition(nodes[str(i - 1)], nodes[str(i)], '', -1)
    fsm.add_marker(nodes[str(i)], '', 1, 1)
    fsm.add_transition(nodes[str(i)], nodes['0'], '', 1)

  fsm.add_marker(nodes[str(n - 1)], '', -1, -1)
  fsm.add_transition(nodes[str(n - 1)], nodes['0'], '', -1)

  return Node(fsm.get_handle(''))

class SimpleContinuousSystem(object):
  def __init__(self):
    self.p = array(3 * [0.0])
    self.r = array(3 * [0.0])

  def update(self):
    v = clip(0.25 * (self.r - self.p), -0.1, 0.1)
    self.p += v

  def error(self):
    return self.r - self.p

  def x1(self):
    self.r[0] = 1.0
    v = check(abs(self.error()[0]) < 0.05)
    print('x to 1: ' + str(v) + ' | ' + str(self.p))
    return v

  def x2(self):
    self.r[0] = 2.0
    v = check(abs(self.error()[0]) < 0.05)
    print('x to 2: ' + str(v) + ' | ' + str(self.p))
    return v

  def x3(self):
    self.r[0] = 3.0
    v = check(abs(self.error()[0]) < 0.05)
    print('x to 3: ' + str(v) + ' | ' + str(self.p))
    return v

if __name__ == '__main__':
  system = SimpleContinuousSystem()

  fsm = FiniteStateMachine(system.x1, system.x2, system.x3)

  fsm.add_transition(system.x1, system.x2, 0, 1)
  fsm.add_transition(system.x2, system.x3, 0, 1)
  fsm.add_marker(system.x3, 0, 1, 1)

  fsm.add_transition(system.x3, system.x2, 1, 1)
  fsm.add_transition(system.x2, system.x1, 1, 1)
  fsm.add_marker(system.x1, 1, 1, 1)

  forward = fsm.get_handle(0)
  backward = fsm.get_handle(1)

  fsm2 = FiniteStateMachine(forward, backward)

  fsm2.add_transition(forward, backward, 0, 1)
  fsm2.add_marker(backward, 0, 1, 1)

  back_and_forth = fsm2.get_handle(0)

  bt = back_and_forth

  while(not bt()):
    system.update()

