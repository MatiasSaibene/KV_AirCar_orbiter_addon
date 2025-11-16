set(CMAKE_CXX_STANDARD 20)
cmake_minimum_required(VERSION 3.16)
project(AirCar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS ON)

# Directorio que contiene las bibliotecas
set(ORBITER_ROOT "C:/Users/Matias_Saibene/Desktop/OpenOrbiter2024")
set(ORBITER_SDK "${ORBITER_ROOT}/Orbitersdk")
set(ORBITER_INCLUDES "${ORBITER_SDK}/include")
set(ORBITER_LIBS "${ORBITER_SDK}/lib")
set(XRSOUND_DIRECTORY "${ORBITER_SDK}/XRSound")

file(GLOB LIBRARIES "${ORBITER_LIBS}/*.lib")
file(GLOB XRSoundLIBS "${XRSOUND_DIRECTORY}/*.lib")

# Definiciones de los archivos de código fuente
set(SOURCE_FILES
    AirCar.cpp
)

# Definiciones de los archivos de encabezado
set(HEADER_FILES
    AirCar.h
    KleinVision_AirCar.h
)

# Agregar los archivos de código fuente y encabezado al proyecto
add_library(${PROJECT_NAME} SHARED
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

# Especificar directorios de inclusión
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ORBITER_INCLUDES}
    ${XRSOUND_DIRECTORY}
)

# Enlazar con todas las bibliotecas encontradas en el directorio
target_link_libraries(AirCar ${LIBRARIES} ${XRSoundLIBS})

# Opciones de compilador para MSVC
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /MD)

    # Librerías de tiempo de ejecución multi-threaded
    target_compile_options(${PROJECT_NAME} PRIVATE /MD)

    # Opciones de optimización y depuración
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        target_compile_options(${PROJECT_NAME} PRIVATE /O2)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE /Zi /Od)
        target_link_options(${PROJECT_NAME} PRIVATE /DEBUG)
    endif()
endif()
