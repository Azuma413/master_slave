cmake_minimum_required(VERSION 3.8)
project(dynamixel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(yaml_cpp_vendor REQUIRED) # ament_cmake_autoが対応していないライブラリ

#xmlから依存を見つける
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
src/dynamixel_workbench_toolbox/dynamixel_driver.cpp include/dynamixel_driver.h
src/dynamixel_workbench_toolbox/dynamixel_item.cpp include/dynamixel_item.h
src/dynamixel_workbench_toolbox/dynamixel_tool.cpp include/dynamixel_tool.h
src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp include/dynamixel_workbench.h
)

#実行ファイルの生成
ament_auto_add_executable(
dynamixel_node src/dynamixel_node.cpp
)

target_link_libraries(dynamixel_node yaml-cpp)
#target_link_libraries(dynamixel_node dynamixel_sdk)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

#構文チェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#ライブラリのビルド
ament_auto_package()
