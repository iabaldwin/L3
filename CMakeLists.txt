cmake_minimum_required(VERSION 2.6.4)
project(L3)

find_package( BOOST )
find_package( HDF5 )
find_package( GLUT )
find_package( OpenGL )

# Core
add_subdirectory( core )

# Graphics
add_subdirectory( renderer )
