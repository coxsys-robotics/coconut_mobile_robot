#!/usr/bin/env python3 
import rospy
from shutil import copyfile
from os.path import expanduser

def main():
    home = expanduser("~")
    lua_name = "coxsys"
    new_lua_name = rospy.get_param('/map_name', 'test')
    
    ##################### SLAM LUA #####################
    luaSlam_path = "{}/coconut_ws/src/coconut_uvc_bringup/config/".format(home)

    luaSlam_filename = "{}_slam.lua".format(lua_name)
    luaSlam_file = luaSlam_path + luaSlam_filename

    new_luaSlam_filename = "{}_slam.lua".format(new_lua_name)
    new_luaSlam_file = luaSlam_path + new_luaSlam_filename

    copy_file(luaSlam_file, new_luaSlam_file)

    ##################### LOCALIZATION LUA #####################
    luaLocalization_path = "{}/coconut_ws/src/coconut_uvc_bringup/config/".format(home)

    luaLocalization_filename = "{}_localization.lua".format(lua_name)
    luaLocalization_file = luaLocalization_path + luaLocalization_filename

    new_luaLocalization_filename = "{}_localization.lua".format(new_lua_name)
    new_luaLocalization_file = luaLocalization_path + new_luaLocalization_filename

    copy_file(luaLocalization_file, new_luaLocalization_file)


def copy_file(old_file, new_file):
    copyfile(old_file, new_file)


if __name__ == '__main__':
    rospy.init_node('create_lua_file_node', anonymous=True)
    # new_lua_name = "test"
    main()