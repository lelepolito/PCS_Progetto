list(APPEND PolyhedralMesh_headers ${CMAKE_CURRENT_SOURCE_DIR}/Utils.hpp)
list(APPEND PolyhedralMesh_sources ${CMAKE_CURRENT_SOURCE_DIR}/Utils.cpp)

list(APPEND PolyhedralMesh_headers ${CMAKE_CURRENT_SOURCE_DIR}/struttura.hpp)

list(APPEND PolyhedralMesh_includes ${CMAKE_CURRENT_SOURCE_DIR})

set(PolyhedralMesh_sources ${PolyhedralMesh_sources} PARENT_SCOPE)
set(PolyhedralMesh_headers ${PolyhedralMesh_headers} PARENT_SCOPE)
set(PolyhedralMesh_includes ${PolyhedralMesh_includes} PARENT_SCOPE)