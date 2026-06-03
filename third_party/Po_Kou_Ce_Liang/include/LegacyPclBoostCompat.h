#pragma once

// PCL 1.8.0 and its bundled Boost 1.59 predate VS2022's STL.
// This forced-include header keeps the old headers on their portable paths.
#if defined(_MSC_VER) && _MSC_VER >= 1930
#  define log2f pcl_log2f_vs2022_compat
#  define BOOST_IOSTREAMS_DETAIL_CONFIG_FPOS_HPP_INCLUDED
#  define BOOST_LIB_TOOLSET "vc120"
#endif
