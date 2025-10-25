@echo off
echo Compiling N-Body Simulator...

:: Create object directory if it doesn't exist
if not exist ".\obj" mkdir ".\obj"

:: Define compiler and flags
:: We add -I "include" and -I "entt" so the compiler
:: knows where to find your .hpp files.
set COMPILER=g++
set FLAGS=-std=c++17 -I "include" -I "entt" -O3
set SRC_DIR=.\src
set OBJ_DIR=.\obj

:: Compile source files into object files
echo Compiling systems...
%COMPILER% -c %SRC_DIR%\systems.cpp -o %OBJ_DIR%\systems.o %FLAGS%
if %errorlevel% neq 0 (
    echo systems.cpp failed to compile.
    exit /b %errorlevel%
)

echo Compiling integrators...
%COMPILER% -c %SRC_DIR%\integrators.cpp -o %OBJ_DIR%\integrators.o %FLAGS%
if %errorlevel% neq 0 (
    echo integrators.cpp failed to compile.
    exit /b %errorlevel%
)

echo Compiling main...
%COMPILER% -c %SRC_DIR%\main.cpp -o %OBJ_DIR%\main.o %FLAGS%
if %errorlevel% neq 0 (
    echo main.cpp failed to compile.
    exit /b %errorlevel%
)

:: Link object files into final executable
echo Linking...
%COMPILER% %OBJ_DIR%\main.o %OBJ_DIR%\systems.o %OBJ_DIR%\integrators.o -o main.exe %FLAGS%
if %errorlevel% neq 0 (
    echo Linking failed.
    exit /b %errorlevel%
)

echo Compilation successful: main.exe