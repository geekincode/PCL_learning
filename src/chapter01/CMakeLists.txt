cmake_minimum_required(VERSION 3.8)
project(chapter01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable(point_convert src/point_convert.cpp)
# target_link_libraries(point_convert ${PCL_LIBRARIES})

# add_executable(create_point src/create_point.cpp)
# target_link_libraries(create_point ${PCL_LIBRARIES})

# add_executable(create_point_color src/create_point_color.cpp)
# target_link_libraries(create_point_color ${PCL_LIBRARIES})

# 查找 src 目录下的所有 .cpp 源文件
file(GLOB SOURCE_FILES "./src/*.cpp")
message("--------------------------------------------------")
message("SOURCE_FILES:")
foreach(SRC_FILE ${SOURCE_FILES})
    message("  ${SRC_FILE}")
endforeach()
message("--------------------------------------------------")

# 循环处理每个源文件
foreach(SRC_FILE ${SOURCE_FILES})
    # 提取源文件名（不含扩展名）作为目标名称
    get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)
    message("TARGET_NAME: ${TARGET_NAME}")
    
    # 添加可执行文件
    add_executable(${TARGET_NAME} ${SRC_FILE})
    
    # 链接 PCL 库
    target_link_libraries(${TARGET_NAME} ${PCL_LIBRARIES})
    
    # 将目标名称加入列表，供后续安装使用
    list(APPEND EXECUTABLE_TARGETS ${TARGET_NAME})
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${EXECUTABLE_TARGETS}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# message("--------------------------------------------------")
# message(${EXECUTABLE_TARGETS})

install(DIRECTORY pcd
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
