# Maximum Flow algorithm with Electrical Flows

This project is an implementation of the algorithm $\tilde{O}(m^{\frac{10}{7}}U^\frac{1}{7})$-time algorithm, proposed by **Aleksander Mądry** in [*Computing maximum flow with augmenting electrical flows*](https://dblp.org/rec/journals/corr/Madry16). It is a part of the thesis which could be treated as an implementation guide. The thesis contains a theory overview with all important references and a pretty deep description of the implementation.

Code is written in **C++** language using [CMake](https://cmake.org/) tool for building. The project could be used both as code that is part of another program and as a standalone application.

## Dependencies

This project depend on:

* [CMake](https://cmake.org/) as a building tool,
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) as used linear system solver.

Eigen dependency could be easily installed in your system using the below command.
```console
> sudo apt install libeigen3-dev
```

A user can use any solver instead. For example, a very efficient [Laplacians.jl](https://github.com/danspielman/Laplacians.jl) mentioned in the work of **Madry**. Details could be found in the mentioned thesis.

This work depends also on **dynamic trees** implementation from [Dinic's Algorithm with Link-Cut-Tree](https://github.com/sergmiller/Dinic-s-Algorithm-with-Link-Cut-Tree). The slightly modified code of this structure is attached to sources in *community-code* directory.

## Usage

Before you start above dependencies need to be installed.

In project directory run:

```console
// create a directory for build artifacts.
> mkdir build

// in build directory run command to generate a native build system.
> cd build/
> cmake ..

// run a native build system (GNU Make).
> make

// run program.
> ./electrical-flows 
```

Program takes input from *stdin* in format:

* The first line is an integer **t**, indicating the number of test cases.
* The second line contains integers **n** and **m**, separated by space, indicating the number of nodes and edges respectively.
* The next *m* consecutive lines consist of three numbers **a**, **b** and **u** sparated by space where **a** and **b** are begin and end respectively of the directed edge and *u* is a capacity.
* Last line contains integers **s** and **t**, separated by space, indicating the number of source and destination node respectively.

Input examples can be found in the **test** directory in files with prefix **in**. Expected outputs can be found in the corresponding **out** file.

## License
[MIT](https://choosealicense.com/licenses/mit/)