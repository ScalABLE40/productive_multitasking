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

class Executive(object):
  def __init__(self):

    self._gripper_empty = True
    self._part = ''
    self._location = ''
    self._intermediate_location = ''
    self._available_part = 1
    self._available_intermediate_location = 1
    self._available_location = 1
    self._first_part_in_queue = ''
    self._done = False

    query_queue = l3.Node(self.query_queue)
    queue_empty = l3.Node(self.queue_empty)
    gripper_empty = l3.Node(self.gripper_empty)
    check_intermediate_location = l3.Node(self.check_intermediate_location)
    choose_intermediate_location = l3.Node(self.choose_intermediate_location)
    place_on_intermediate_location = l3.Node(self.place_on_intermediate_location)
    register_gripper_empty = l3.Node(self.register_gripper_empty)
    register_intermediate_placing = l3.Node(self.register_intermediate_placing)
    forget_intermediate_location = l3.Node(self.forget_intermediate_location)
    place_on_location = l3.Node(self.place_on_location)
    register_placing = l3.Node(self.register_placing)
    forget_location = l3.Node(self.forget_location)
    choose_first_part_in_queue = l3.Node(self.choose_first_part_in_queue)
    pick_part = l3.Node(self.pick_part)
    update_queue = l3.Node(self.update_queue)
    register_gripper_not_empty = l3.Node(self.register_gripper_not_empty)
    forget_part = l3.Node(self.forget_part)
    done = l3.Node(self.done)
    choose_location = l3.Node(self.choose_location)
    choose_part = l3.Node(self.choose_part)
    register_picking = l3.Node(self.register_picking)

    behavior_tree =   query_queue \
                    |     ~queue_empty \
                        *     (~gripper_empty \
                            *     (check_intermediate_location \
                                * choose_intermediate_location \
                                * place_on_intermediate_location \
                                * register_gripper_empty \
                                * register_intermediate_placing \
                                * forget_intermediate_location \
                              +   choose_location \
                                * place_on_location \
                                * register_gripper_empty \
                                * register_placing \
                                * forget_location) \
                          +   gripper_empty \
                            * choose_first_part_in_queue \
                            * pick_part \
                            * update_queue \
                            * register_gripper_not_empty \
                            * forget_part) \
                      <<  done \
                      <<  ~gripper_empty \
                        * choose_location \
                        * place_on_location \
                        * register_gripper_empty \
                        * register_placing \
                        * forget_location \
                      <<  gripper_empty \
                        * forget_part \
                        * choose_part \
                        * pick_part \
                        * register_picking \
                        * register_gripper_not_empty \
                        * forget_part

    status = 0

    while not self._done:
      print('>>>>>>')

      status = behavior_tree()

      print('{} <<<<<<'.format(status))

  def query_queue(self):
    r = l3.SUCCESS
    print('query_queue: {}'.format(r))
    return r

  def update_queue(self):
    r = l3.SUCCESS
    print('update_queue: {}'.format(r))
    return r

  def queue_empty(self):
    r = l3.SUCCESS
    print('queue_empty: {}'.format(r))
    return r

  def check_intermediate_location(self):
    r = l3.SUCCESS
    print('check_intermediate_location: {}'.format(r))
    return r

  def choose_intermediate_location(self):
    r = l3.SUCCESS
    print('choose_intermediate_location: {}'.format(r))
    return r

  def done(self):
    r = l3.condition(self._available_location > 1)
    if r == l3.SUCCESS:
      self._done = True
    print('done: {}'.format(r))
    return r

  def gripper_empty(self):
    r = l3.condition(self._gripper_empty)
    print('gripper_empty: {}'.format(r))
    return r

  def choose_location(self):
    r = l3.SUCCESS
    print('choose_location: {}'.format(r))
    return r

  def forget_location(self):
    r = l3.SUCCESS
    print('forget_location: {}'.format(r))
    return r

  def forget_intermediate_location(self):
    r = l3.SUCCESS
    print('forget_intermediate_location: {}'.format(r))
    return r

  def place_on_location(self):
    r = l3.SUCCESS
    print('place_on_location: {}'.format(r))
    return r

  def place_on_intermediate_location(self):
    r = l3.SUCCESS
    print('place_on_intermediate_location: {}'.format(r))
    return r

  def register_gripper_empty(self):
    self._gripper_empty = True
    r = l3.SUCCESS
    print('register_gripper_empty: {}'.format(r))
    return r

  def register_placing(self):
    self._available_location += 1
    r = l3.SUCCESS
    print('register_placing: {}'.format(r))
    return r

  def register_intermediate_placing(self):
    r = l3.SUCCESS
    print('register_intermediate_placing: {}'.format(r))
    return r

  def choose_part(self):
    r = l3.SUCCESS
    print('choose_part: {}'.format(r))
    return r

  def choose_first_part_in_queue(self):
    r = l3.SUCCESS
    print('choose_first_part_in_queue: {}'.format(r))
    return r

  def forget_part(self):
    r = l3.SUCCESS
    print('forget_part: {}'.format(r))
    return r

  def pick_part(self):
    r = l3.SUCCESS
    print('pick_part: {}'.format(r))
    return r

  def register_picking(self):
    r = l3.SUCCESS
    print('register_picking: {}'.format(r))
    return r

  def register_gripper_not_empty(self):
    self._gripper_empty = False
    r = l3.SUCCESS
    print('register_gripper_not_empty: {}'.format(r))
    return r

if __name__ == '__main__':
  e = Executive()
