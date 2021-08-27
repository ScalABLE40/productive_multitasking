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

import l3

from l3 import sequence, selector, FiniteStateMachine

class MovementManager(object):
  def __init__(self):
    # Parameters
    self.robot_name = 'arm1'
    self.current_target_id = ''

    nodes = {}

    self.generate_nodes(nodes, 'bins', 'conveyor', 'agv')
    self.generate_nodes(nodes, 'agv', 'bins', 'conveyor')
    self.generate_nodes(nodes, 'conveyor', 'agv', 'bins')

    nodes['approach conveyor'] = lambda: self.move_to(self.ns('conveyor/approach'), 'huge')
    nodes['grasp on conveyor'] = lambda: self.grasp_in(self.ns('conveyor/grasp'), 0.5, False)
    nodes['raise on conveyor'] = lambda: self.move_to(self.ns('conveyor/approach'), 'huge')

    fsm = FiniteStateMachine(nodes)

    fsm.set_start('approach bins')

    self.handles = {}

    self.handles['withdraw'] = fsm.get_handle('withdraw')
    self.handles['preemptive?'] = fsm.get_handle('preemptive?')
    self.handles['bins/pick'] = self.generate_pick(fsm, 'bins', 'conveyor', 'agv')
    self.handles['bins/place'] = self.generate_place(fsm, 'bins', 'conveyor', 'agv')
    self.handles['agv/pick'] = self.generate_pick(fsm, 'agv', 'bins', 'conveyor')
    self.handles['agv/place'] = self.generate_place(fsm, 'agv', 'bins', 'conveyor')
    self.handles['conveyor/pick'] = self.generate_pick(fsm, 'conveyor', 'agv', 'bins')
    self.handles['conveyor/place'] = self.generate_place(fsm, 'conveyor', 'agv', 'bins')

    fsm.add_marker('approach bins', 'preemptive?', direct = True)
    fsm.add_marker('bins to agv', 'preemptive?', direct = True)

  def ns(self, frame_id):
      return '{}/{}'.format(self.robot_name, frame_id)

  def generate_nodes(self, nodes, area, area_before, area_after):
    nodes['{} to {}'.format(area_before, area)] = lambda: self.move_to(self.ns('{}/{}'.format(area, area_before)), 'via point')
    nodes['{} to {}'.format(area_after, area)] = lambda: self.move_to(self.ns('{}/{}'.format(area, area_after)), 'via point')
    nodes['approach {}'.format(area)] = lambda: self.move_to(self.ns('{}/approach'.format(area)), 'fine')
    nodes['enable gripper on {}'.format(area)] = lambda: self.gripper_control(True)
    nodes['grasp on {}'.format(area)] = lambda: self.grasp_in(self.ns('{}/grasp'.format(area)))
    nodes['record grasping pose on {}'.format(area)] = lambda: self.record_grasping_pose()
    nodes['detach approach on {} (pick)'.format(area)] = lambda: self.detach_approach()
    nodes['raise on {}'.format(area)] = lambda: self.move_to(self.ns('{}/approach'.format(area)), 'coarse')
    nodes['disable gripper on {}'.format(area)] = lambda: self.gripper_control(False)
    nodes['put back on {}'.format(area)] = lambda: self.move_to(self.ns('{}/grasp'.format(area)), 'coarse')
    nodes['put on {}'.format(area)] = lambda: self.move_to(self.ns('{}/grasp'.format(area)), 'coarse')
    nodes['release on {}'.format(area)] = lambda: self.gripper_control(False)
    nodes['detach approach on {} (place)'.format(area)] = lambda: self.detach_approach()
    nodes['escape {}'.format(area)] = lambda: self.move_to(self.ns('{}/approach'.format(area)), 'fine')

  def generate_pick(self, fsm, area, area_before, area_after):
    # Transits
    fsm.add_transition('{} to {}'.format(area_before, area_after), 'approach {}'.format(area_before), '{}/pick'.format(area), direct = True)
    fsm.add_transition('approach {}'.format(area_before), '{} to {}'.format(area_before, area), '{}/pick'.format(area))
    fsm.add_transition('{} to {}'.format(area_before, area), 'approach {}'.format(area), '{}/pick'.format(area))
    fsm.add_transition('{} to {}'.format(area, area_before), 'approach {}'.format(area), '{}/pick'.format(area), direct = True)
    fsm.add_transition('{} to {}'.format(area_after, area_before), 'approach {}'.format(area_after), '{}/pick'.format(area), direct = True)
    fsm.add_transition('approach {}'.format(area_after), '{} to {}'.format(area_after, area), '{}/pick'.format(area))
    fsm.add_transition('{} to {}'.format(area_after, area), 'approach {}'.format(area), '{}/pick'.format(area))
    fsm.add_transition('{} to {}'.format(area, area_after), 'approach {}'.format(area), '{}/pick'.format(area), direct = True)
    # Pick
    fsm.add_transition('approach {}'.format(area), 'enable gripper on {}'.format(area), '{}/pick'.format(area))
    fsm.add_transition('enable gripper on {}'.format(area), 'grasp on {}'.format(area), '{}/pick'.format(area))
    fsm.add_transition('grasp on {}'.format(area), 'record grasping pose on {}'.format(area), '{}/pick'.format(area))
    fsm.add_transition('record grasping pose on {}'.format(area), 'detach approach on {} (pick)'.format(area), '{}/pick'.format(area))
    fsm.add_transition('detach approach on {} (pick)'.format(area), 'raise on {}'.format(area), '{}/pick'.format(area))
    fsm.add_transition('raise on {}'.format(area), 'approach {}'.format(area), '{}/pick'.format(area))

    fsm.add_marker('raise on {}'.format(area), '{}/pick'.format(area))

    # Preemptions
    fsm.add_transition('enable gripper on {}'.format(area), 'disable gripper on {}'.format(area), 'withdraw', direct = True)
    fsm.add_transition('disable gripper on {}'.format(area), 'approach {}'.format(area), 'withdraw')
    fsm.add_transition('grasp on {}'.format(area), 'disable gripper on {}'.format(area), 'withdraw', direct = True)
    fsm.add_transition('record grasping pose on {}'.format(area), 'disable gripper on {}'.format(area), 'withdraw', direct = True)
    fsm.add_transition('detach approach on {} (pick)'.format(area), 'disable gripper on {}'.format(area), 'withdraw', direct = True)
    fsm.add_transition('raise on {}'.format(area), 'put back on {}'.format(area), 'withdraw', direct = True)
    fsm.add_transition('put back on {}'.format(area), 'disable gripper on {}'.format(area), 'withdraw')

    fsm.add_marker('approach {}'.format(area), 'withdraw')
    fsm.add_marker('{} to {}'.format(area_before, area), 'withdraw', direct = True)
    fsm.add_marker('{} to {}'.format(area_after, area), 'withdraw', direct = True)

    return fsm.get_handle('{}/pick'.format(area))

  def generate_place(self, fsm, area, area_before, area_after):
    # Transits
    fsm.add_transition('{} to {}'.format(area_before, area_after), 'approach {}'.format(area_before), '{}/place'.format(area), direct = True)
    fsm.add_transition('approach {}'.format(area_before), '{} to {}'.format(area_before, area), '{}/place'.format(area))
    fsm.add_transition('{} to {}'.format(area_before, area), 'approach {}'.format(area), '{}/place'.format(area))
    fsm.add_transition('{} to {}'.format(area, area_before), 'approach {}'.format(area), '{}/place'.format(area), direct = True)
    fsm.add_transition('{} to {}'.format(area_after, area_before), 'approach {}'.format(area_after), '{}/place'.format(area), direct = True)
    fsm.add_transition('approach {}'.format(area_after), '{} to {}'.format(area_after, area), '{}/place'.format(area))
    fsm.add_transition('{} to {}'.format(area_after, area), 'approach {}'.format(area), '{}/place'.format(area))
    fsm.add_transition('{} to {}'.format(area, area_after), 'approach {}'.format(area), '{}/place'.format(area), direct = True)

    # Place
    fsm.add_transition('approach {}'.format(area), 'put on {}'.format(area), '{}/place'.format(area))
    fsm.add_transition('put on {}'.format(area), 'release on {}'.format(area), '{}/place'.format(area))
    fsm.add_transition('release on {}'.format(area), 'detach approach on {} (place)'.format(area), '{}/place'.format(area))
    fsm.add_transition('detach approach on {} (place)'.format(area), 'escape {}'.format(area), '{}/place'.format(area))
    fsm.add_transition('escape {}'.format(area), 'approach {}'.format(area), '{}/place'.format(area))

    fsm.add_marker('escape {}'.format(area), '{}/place'.format(area))

    # Preemptions
    fsm.add_transition('put on {}'.format(area), 'approach {}'.format(area), 'withdraw', direct = True)

    fsm.add_marker('put on {}'.format(area), 'withdraw', direct = True, outcome = l3.FAILURE)
    fsm.add_marker('release on {}'.format(area), 'withdraw', direct = True, outcome = l3.FAILURE)
    fsm.add_marker('detach approach on {} (place)'.format(area), 'withdraw', direct = True, outcome = l3.FAILURE)
    fsm.add_marker('escape {}'.format(area), 'withdraw', direct = True, outcome = l3.FAILURE)

    return fsm.get_handle('{}/place'.format(area))

  def record_grasping_pose(self):
    print('>> record_grasping_pose')
    r = l3.SUCCESS
    print('<< record_grasping_pose returns {}'.format(r))
    return r

  def get_offset(self):
    print('>> get_offset')
    r = l3.SUCCESS
    print('<< get_offset returns {}'.format(r))
    return r

  def attach_approach(self):
    print('>> attach_approach')
    r = l3.SUCCESS
    print('<< attach_approach returns {}'.format(r))
    return r

  def detach_approach(self):
    print('>> detach_approach')
    r = l3.SUCCESS
    print('<< detach_approach returns {}'.format(r))
    return r

  def handle_gripper_state(self, request):
    print('>> handle_gripper_state ({})'.format(request))
    r = l3.SUCCESS
    print('<< handle_gripper_state returns {}'.format(r))
    return r

  def handle_query(self, request):
    print('>> handle_query ({})'.format(request))
    r = l3.SUCCESS
    print('<< handle_query returns {}'.format(r))
    return r

  def gripper_cb(self, msg):
    print('>> gripper_cb ({})'.format(msg))
    r = l3.SUCCESS
    print('<< gripper_cb returns {}'.format(r))
    return r

  def move_to(self, area, tol, speed = 0.5):
    print('>> move_to ({}, {}, {})'.format(area, tol, speed))
    r = l3.SUCCESS
    print('<< move_to returns {}'.format(r))
    return r

  def move_to_height(self, area, tol, speed = 0.5):
    print('>> move_to_height ({}, {}, {})'.format(area, tol, speed))
    r = l3.SUCCESS
    print('<< move_to_height returns {}'.format(r))
    return r

  def grasp_in(self, area, speed = 0.5, shaking = True):
    print('>> grasp_in ({}, {}, {})'.format(area, speed, shaking))
    r = l3.SUCCESS
    print('<< grasp_in returns {}'.format(r))
    return r

  def gripper_control(self, state):
    print('>> gripper_control ({})'.format(state))
    r = l3.SUCCESS
    print('<< gripper_control returns {}'.format(r))
    return r

  def handle_tick(self, request):
    print('> request {}'.format(request))

    [skill, target_id] = request.split()

    if self.current_target_id != target_id:
      if self.handles['withdraw']() == l3.SUCCESS:
        self.current_target_id = target_id
      r = l3.RUNNING
    else:
      r = self.handles[skill]()

    print('< request {} returns {}'.format(request, r))
    print('')

    return r

if __name__ == '__main__':
  mm = MovementManager()

  status = 0
  for i in range(3):
    status = mm.handle_tick('bins/pick gear_1')
  while status != l3.SUCCESS:
    status = mm.handle_tick('conveyor/pick disk_1')
