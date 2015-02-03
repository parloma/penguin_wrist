/* Copyright (C) 2014 Politecnico di Torino


This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

Contributors:
  Ludovico O. Russo (ludovico.russo@polito.it)
*/

#include <ros.h>
#include <penguin_ros/penguin.h>
#include <Servo.h>


ros::NodeHandle node;
penguin_ros::penguin p_msg;

Servo s_up;
Servo s_middle;
Servo s_down;

void penguin_cb(const penguin_ros::penguin & msg) {
  int cmd_up =       (int)msg.up              * 2   + 90 ;
  int cmd_middle =  ((int)msg.middle + 120  ) * 2   + 90 ;
  int cmd_down =    ((int)msg.down   - 120  ) * 2   + 90 ;

  s_up.write(cmd_up);
  s_middle.write(cmd_middle);
  s_down.write(cmd_down);

}

ros::Subscriber<penguin_ros::penguin> sub("penguin_msgs", &penguin_cb);

void setup () {
  s_up.attach(11);
  s_middle.attach(9);
  s_down.attach(10);

  s_up.write(90);
  s_middle.write(90);
  s_down.write(90);

  node.initNode();
  node.subscribe(sub);
}

void loop() {
  node.spinOnce();
  delay(1);
}
