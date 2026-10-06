# Boost 1.74.0 subset

pimax_slam.pi.dll was built against Boost 1.74 (`C:\Boost\include\boost-1_74\...` assert paths; archive library
version 18) with boost::serialization (binary and the portable-binary example archives) and boost::filesystem
linked statically. This directory holds only what the build needs: every header reached by the project and
library translation units (collected with clang-cl `/showIncludes`), the complete `config/`, `archive/`,
`serialization/`, `filesystem/` and `system/` header trees (other compilers/configurations select different
config headers), and the library sources `libs/serialization/src` and `libs/filesystem/src`. Unmodified.
Full release: https://archives.boost.io/release/1.74.0/source/boost_1_74_0.tar.bz2
