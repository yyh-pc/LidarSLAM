# because pcl doesn't come with a modern cmake yet (it doesn't provide targets)
# we construct our own pcl target
add_library(PCL::PCL INTERFACE IMPORTED GLOBAL)
set_target_properties(PCL::PCL PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
    INTERFACE_COMPILE_DEFINITIONS "${PCL_DEFINITIONS}"
)
