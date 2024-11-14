# Serial Communication Library

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