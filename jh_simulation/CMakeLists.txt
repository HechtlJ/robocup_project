cmake_minimum_required(VERSION 3.0.2)
project(jh_simulation)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

foreach(dir config launch Media models scripts worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

