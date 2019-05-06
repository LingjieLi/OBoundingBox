 cmake_minimum_required(VERSION 3.10)
# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS
aux_source_directory(. DIR_LIB_SRCS)

# 生成链接库
add_library(OBoundingBox ${DIR_LIB_SRCS})


#中间文件生成目录
# SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../lib)


#指定 OBoundingBox 库的安装路径
SET(CMAKE_INSTALL_PREFIX "..") #安装根目录
install(TARGETS OBoundingBox DESTINATION lib)
install(FILES oboundingbox.h DESTINATION include)

