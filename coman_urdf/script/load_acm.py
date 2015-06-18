#!/usr/bin/env python

# Copyright (C) 2015 the Walkman Consortium
#
# Authors: Alessio Rocchi
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import argparse

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os



if __name__ == '__main__':

    parser = argparse.ArgumentParser(usage='save_acm.py <options> srdf_file\nLoad an SRDF file')
    parser.add_argument('srdf_file', type=argparse.FileType('r'), nargs='?',
                        default=None, help='SRDF file. Use - for stdin')
    parser.add_argument('-o', '--output', help='Dump file to SRDF', action='store_true')
    args = parser.parse_args()
    # Extract robot name and directory

    if args.srdf_file is None:
        print("Error! no srdf_file provided")
        exit()
    else:
        args.srdf_file.close()
        srdf = minidom.parse(args.srdf_file.name)
        robot = srdf.getElementsByTagName('robot')
        robot = robot[0]

        acm_file = os.path.splitext(args.srdf_file.name)[0]+'.acm'
        if not os.path.isfile(acm_file):
            raise Exception('ERROR: could not find acm file')

        acm = minidom.parse(acm_file)

        for node in acm.getElementsByTagName('disable_collisions'):
            x = srdf.importNode(node, True)
            robot.appendChild(srdf.createTextNode('    '))
            robot.appendChild(x)
            robot.appendChild(srdf.createTextNode('\n'))
        new_srdf = srdf.toxml()

    if args.output:
        file(args.srdf_file.name, 'w').write(new_srdf)
    else:
        print(new_srdf)
