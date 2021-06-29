#!/bin/zsh

# Switch between ROS 1 / ROS 2 workspaces without messing with environment variables
# Originally created by Olivier Kermorgant

# ROS 1 / 2 workspaces are defined in overlay ordering 
# ex: ros1_ws="/opt/ros/noetic $HOME/ros_ws1 $HOME/ros_ws2"

# Takes a path string separated with colons and a list of sub-paths
# Removes path elements containing sub-paths

# replace the custom ROS workspaces started by $HOME
export ros1_ws="/opt/ros/noetic $HOME/catkin_ws"
export ros2_foxy_ws="/opt/ros/foxy $HOME/ros2_ws"
export ros2_galactic_ws="/opt/ros/galactic $HOME/ros2_ws"
export ros2_version="galactic"
export PREFIX_ori=$ZSH_ESSEMBEH_PREFIX

remove_paths()
{
if [[ $# -eq 1 ]]; then
   return ""
fi
IFS=':' read -A -r PATHS <<< $1 
IFS=' ' read -A -r remove_paths <<< $2 

local THISPATH=""
local path
for path in $PATHS; do
   local to_remove=0
   for r_path in $remove_paths; do
      if [[ $path = *$r_path* ]]; then
         to_remove=1
         break
      fi
   done
   if [ $to_remove -eq 0 ]; then
      if [[ $THISPATH = "" ]]; then
         THISPATH=$path
      else
         THISPATH=$THISPATH:$path
      fi
   fi
done
echo $THISPATH
}

# Takes a list of sub-paths
# Updates ROS-related system paths by removing all elements containing sub-paths
remove_all_paths()
{
    export AMENT_PREFIX_PATH=$(remove_paths $AMENT_PREFIX_PATH $@)
    export AMENT_CURRENT_PREFIX=$(remove_paths $AMENT_CURRENT_PREFIX $@)
    export PYTHONPATH=$(remove_paths $PYTHONPATH $@)
    export CMAKE_PREFIX_PATH=$(remove_paths $CMAKE_PREFIX_PATH $@)
    export PATH=$(remove_paths $PATH $@)
    export LD_LIBRARY_PATH=$(remove_paths $LD_LIBRARY_PATH $@)
}

# Register a single ROS 1 / 2 workspace, try to source in order : ws > ws/install > ws/devel
register_ros_workspace()
{
local sub
for sub in "/" "/install/" "/devel/"
do
    if [ -f $1${sub}setup.zsh ]; then        
            source $1${sub}local_setup.zsh
        return
    fi
done
}

# Equivalent of roscd but jumps to the source (also, no completion)
ros2cd()
{
local ros2_workspaces
if [[ $ros2_version = "galactic" ]]; then
    ros2_workspaces=$ros2_galactic_ws
fi
if [[ $ros2_version = "foxy" ]]; then
    ros2_workspaces=$ros2_foxy_ws
fi

IFS=' ' read -A -r ROS2_PATHS <<< $ros2_workspaces
local ws
local prev=""
local sub
local key="<name>$1</name>"
local res
for ws in $ROS2_PATHS
do 
    if [[ $ws = $prev ]]; then
        break
    fi
    # Make it to the source directory if possible
    for sub in "/" "/install/" "/devel/"
    do
    if [ -d ${ws##* }$sub ]; then
        res=$(grep -r --include \*package.xml $key ${ws##* }$sub )
        if [[ $res != "" ]]; then 
        cd ${res%%/package.xml*}
        return
        fi
    fi
    done
    prev=$ws
    ws=${ws% *}
done
echo "Could not find package $1"
}

# Activate ROS 1 ws
ros1ws()
{
# Clean ROS 2 paths
remove_all_paths $ros2_foxy_ws
remove_all_paths $ros2_galactic_ws
unset ROS_DISTRO

# register ROS 1 workspaces
IFS=' ' read -A -r ROS1_PATHS <<< $ros1_ws 
local ws
for ws in $ROS1_PATHS
do
    register_ros_workspace $ws
done
# change prompt 
export ZSH_ESSEMBEH_PREFIX="%{$fg[blue]%}[ROS1]%{$reset_color%}$PREFIX_ori"
source /usr/share/gazebo/setup.sh
}

# Activate ROS 2 ws
ros2ws()
{
# Clean ROS 1 paths
remove_all_paths $ros1_ws
unset ROS_DISTRO

local ros2_workspaces
# switch to galactic by default
if [[ $# -eq 0 ]]; then
   ros2_workspaces=$ros2_galactic_ws
   export ros2_version="galactic"
else
   if [[ $@ = "galactic" ]]; then
       ros2_workspaces=$ros2_galactic_ws
       export ros2_version="galactic"
   fi
   if [[ $@ = "foxy" ]]; then
       ros2_workspaces=$ros2_foxy_ws
       export ros2_version="foxy"
   fi
fi
# register ROS 2 workspaces
IFS=' ' read -A -r ROS2_PATHS <<< $ros2_workspaces 
local ws
for ws in $ROS2_PATHS
do
    register_ros_workspace $ws
done
# change prompt 
export ZSH_ESSEMBEH_PREFIX="%{$fg[yellow]%}[ROS2-$ros2_version]%{$reset_color%}$PREFIX_ori"
source /usr/share/gazebo/setup.sh
}

# some shortcuts
colbuild()
{
# Clean ROS 1 paths
remove_all_paths $ros1_ws
unset ROS_DISTRO
# source ROS 2 ws up to this one
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH

local ros2_workspaces
if [[ $ros2_version = "galactic" ]]; then
    ros2_workspaces=$ros2_galactic_ws
fi
if [[ $ros2_version = "foxy" ]]; then
    ros2_workspaces=$ros2_foxy_ws
fi
IFS=' ' read -A -r ROS2_PATHS <<< $ros2_workspaces 
local ws
for ws in $ROS2_PATHS; do
    #if [[ $ws = $PWD* ]]; then
    #  break
    #fi
    register_ros_workspace $ws
done

cd $ws
local cmd
if [[ $# -eq 0 ]]; then
    cmd="colcon build --symlink-install --continue-on-error $@"
    if [ -d "src/ros1_bridge" ]; then
        cmd="$cmd  --packages-skip ros1_bridge"
    fi
else
    cmd="colcon build --packages-select $@"
fi
eval $cmd
ros2ws
}

# shortcut to be sure where we are
alias rosd='echo $ROS_DISTRO'

# shortcut to build ros1_bridge without messing system paths
# assumes we are in our ROS 2 workspace directory / recompiles ros1_bridge
# make sure it is worth it, this is quite long...
ros1bridge_recompile()
{
if [ ! -d "src/ros1_bridge" ]; then
    echo "ros1_bridge is not in this workspace - is it actually a ROS 2 workspace?"
    return
fi
# clean environment variables
remove_all_paths $ros1_ws $ros2_ws
unset ROS_DISTRO
# register ROS 2 overlays before the ros1_bridge overlay
colbuild

# register base ROS 1 installation
unset ROS_DISTRO
local ros1_base=${ros1_ws% *}
register_ros_workspace $ros1_base
# register base ROS 2 installation
unset ROS_DISTRO
local ros2_base=${ros2_galactic_ws% *}
register_ros_workspace $ros2_base

# register ROS 1 overlays
unset ROS_DISTRO
for ws in $ros1_ws; do
    if [[ $ws != $ros1_base ]]; then
        register_ros_workspace $ws
    fi
done

# register ROS 2 overlays up to the ros1_bridge overlay
unset ROS_DISTRO
for ws in $ros2_workspaces; do
    if [[ $ws != $ros2_base ]]; then
       register_ros_workspace $ws
       if [[ $ws = $bridge_overlay* ]]; then
       break
       fi
    fi
done
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --continue-on-error
}
