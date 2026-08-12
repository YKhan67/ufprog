cmake_minimum_required(VERSION 3.13)


project(ufsnandprog)


set(ufsnandprog_common_src
    ufsnand-common.c
)


if(NOT (WIN32 OR MINGW))
    add_link_options(-Wl,--rpath=.,--disable-new-dtags)
endif()


add_executable(ufsnandprog ufsnandprog.c ${ufsnandprog_common_src})
target_compile_definitions(ufsnandprog PRIVATE UFP_VERSION=\"${UFPROG_VERSION_MAJOR}.${UFPROG_VERSION_MINOR}\")
target_link_libraries(ufsnandprog PRIVATE ufprog_spi_nand ufprog_nand_core ufprog_spi ufprog_common)


add_executable(ufsnandtest ufsnandtest.c ${ufsnandprog_common_src})
target_compile_definitions(ufsnandtest PRIVATE UFP_VERSION=\"${UFPROG_VERSION_MAJOR}.${UFPROG_VERSION_MINOR}\")
target_link_libraries(ufsnandtest PRIVATE ufprog_spi_nand ufprog_nand_core ufprog_spi ufprog_common)

add_executable(ch341-recover
    ch341-recover.c
)
target_link_libraries(ch341-recover
    PRIVATE
        ${CMAKE_BINARY_DIR}/flash/nand/spi-nand/libufprog-spi-nand.dll.a
        ${CMAKE_BINARY_DIR}/flash/nand/core/libufprog-nand-core.dll.a
        ${CMAKE_BINARY_DIR}/interface/spi/libufprog-spi.dll.a
        ${CMAKE_BINARY_DIR}/controller/core/libufprog-controller.dll.a
        ${CMAKE_BINARY_DIR}/common/libufprog-common.dll.a
)

add_executable(ch341-nand-recover
    ch341-recover.c
)
target_link_libraries(ch341-nand-recover
    PRIVATE
        ${CMAKE_BINARY_DIR}/flash/nand/spi-nand/libufprog-spi-nand.dll.a
        ${CMAKE_BINARY_DIR}/flash/nand/core/libufprog-nand-core.dll.a
        ${CMAKE_BINARY_DIR}/interface/spi/libufprog-spi.dll.a
        ${CMAKE_BINARY_DIR}/controller/core/libufprog-controller.dll.a
        ${CMAKE_BINARY_DIR}/common/libufprog-common.dll.a
)

add_executable(ch341-nand-id-test
    ch341-nand-id-test.c
)
target_link_libraries(ch341-nand-id-test
    PRIVATE
        ${CMAKE_BINARY_DIR}/flash/nand/spi-nand/libufprog-spi-nand.dll.a
        ${CMAKE_BINARY_DIR}/flash/nand/core/libufprog-nand-core.dll.a
        ${CMAKE_BINARY_DIR}/interface/spi/libufprog-spi.dll.a
        ${CMAKE_BINARY_DIR}/controller/core/libufprog-controller.dll.a
        ${CMAKE_BINARY_DIR}/common/libufprog-common.dll.a
)

add_executable(ch341-nand-program
    ch341-nand-program.c
)
target_link_libraries(ch341-nand-program
    PRIVATE
        ${CMAKE_BINARY_DIR}/flash/nand/spi-nand/libufprog-spi-nand.dll.a
        ${CMAKE_BINARY_DIR}/flash/nand/core/libufprog-nand-core.dll.a
        ${CMAKE_BINARY_DIR}/interface/spi/libufprog-spi.dll.a
        ${CMAKE_BINARY_DIR}/controller/core/libufprog-controller.dll.a
        ${CMAKE_BINARY_DIR}/common/libufprog-common.dll.a
)

add_executable(ch341-nand-parameter-test
    ch341-nand-parameter-test.c
)
target_link_libraries(ch341-nand-parameter-test
    PRIVATE
        ${CMAKE_BINARY_DIR}/flash/nand/spi-nand/libufprog-spi-nand.dll.a
        ${CMAKE_BINARY_DIR}/flash/nand/core/libufprog-nand-core.dll.a
        ${CMAKE_BINARY_DIR}/interface/spi/libufprog-spi.dll.a
        ${CMAKE_BINARY_DIR}/controller/core/libufprog-controller.dll.a
        ${CMAKE_BINARY_DIR}/common/libufprog-common.dll.a
)

add_executable(ch341-nand-write
    ch341-nand-write.c
)

target_link_libraries(ch341-nand-write
    PRIVATE
        ${CMAKE_BINARY_DIR}/flash/nand/spi-nand/libufprog-spi-nand.dll.a
        ${CMAKE_BINARY_DIR}/flash/nand/core/libufprog-nand-core.dll.a
        ${CMAKE_BINARY_DIR}/interface/spi/libufprog-spi.dll.a
        ${CMAKE_BINARY_DIR}/controller/core/libufprog-controller.dll.a
        ${CMAKE_BINARY_DIR}/common/libufprog-common.dll.a
)

include_directories(${ufprog_common_SOURCE_DIR}/include)
include_directories(${ufprog_controller_SOURCE_DIR}/include)
include_directories(${ufprog_spi_SOURCE_DIR}/include)
include_directories(${ufprog_nand_core_SOURCE_DIR}/include)
include_directories(${ufprog_spi_nand_SOURCE_DIR}/include)


install(TARGETS ufsnandprog ufsnandtest
    RUNTIME DESTINATION ${EXE_DIR}
)