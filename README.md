# Serial Communication Library

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)
![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)
![macOS](https://img.shields.io/badge/mac%20os-000000?style=for-the-badge&logo=macos&logoColor=F0F0F0)

Basic C++ library to communicate with serial porst in POSIX-compliant operating systems.

## Prerequisites

Make sure you have the following installed on your system:
- Git
- CMake (version 3.10 or later)
- A C++ compiler (e.g., GCC, Clang, or MSVC)

## Getting Started

Follow these steps to clone the repository, build the project, and run an example.

### 1. Clone the Repository

Start by cloning the project repository from GitHub.

```bash
git clone git@github.com:uOttawaCAV/serial_comm.git
cd serial_comm
```

### 2. Create a Build Directory

If build directory does not already exist, one must be created.

```bash
mkdir build && cd build
```

### 3. Run CMake and Make

CMake will configure the build environment based on the project files. Running *make* will create the executables in the *example* directory. Run the following commands inside the build directory:

```bash
cmake ..
make
```

### 4. Running an example

The example *read_counter* is used to read counter values from an encoder connected to the HDC2460 motor controller. Ensure you are in the root of the project and not in the build directory by entering the following commands.

```bash
# Change directory out of build into the root.
cd ..

# Run the read_counter binary
./read_counter
```

## Sources 
- https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
