#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospkg
import shutil
import sys

int_obj_description_path = rospkg.RosPack().get_path("mc_int_obj_description")
mc_scenario_path = rospkg.RosPack().get_path("mc_scenario")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "{} [name]".format(sys.argv[0])
        sys.exit(1)
    name = sys.argv[1]
    urdf = "{}/urdf/{}.urdf".format(int_obj_description_path, name)
    rsdf_dir = "{}/rsdf/{}".format(int_obj_description_path, name)
    scenario = "{}/env/{}.xml".format(mc_scenario_path, name)
    shutil.rmtree(rsdf_dir)
    os.remove(urdf)
    os.remove(scenario)
