# Gurobi_Cpp_TSP

A header only factory of TSP models built using Gurobi 12 in C++.

# Requirements

The code was developed using:
* C++ 20;
* Gurobi 12 C++ API;
* Windows 11 OS running Visual Studio 22 IDE;

Not so older versions of Gurobi will probably work too. 
Tests on Linux and MacOS machines still need to be performed. Since the code 
is not so complex, it should compile and work fine in both systems.

# Usage

The main goal of this code is to offer a simple and straightforward way to build 
models for the Traveling Salesman Problem (TSP), solve them using Gurobi and 
analyze the solution obtained.

In the `Gurobi_Cpp_TSP.cpp` file is presented an example on how to read 
an instance saved in **.csv** format, build three different models, solve 
them all and print the solutions on the terminal. We present below a summary on
how things are done using this code.

## Example file step-by-step

### Necessary #includes
```c++
#include "gurobi_c++.h"
#include "include/Gurobi_Cpp_TSP.hpp"
#include "third_party/rapidcsv.h"
```

The first lines of the example file are, as expected, the includes needed. 
Here we are using only three includes:
* The Gurobi C++ API;
* The factory main header;
* A third-party code to read **.csv** files.

**Note:** Make sure to have Gurobi installed in your machine, and that you have 
a valid license to use it, before trying to compile the code. If you don't know
how to install Gurobi on your computer, take a look at the getting started 
tutorials at [Gurobi Documentation](https://docs.gurobi.com/current/). If you
need help on how to compile a project using Gurobi in C++, there are some
directions on [Gurobi Help Center](https://support.gurobi.com/hc/en-us) about
building projects on Windows using Visual Studio, and on Linux and MacOS
using CMake.

### Reading an instance
```c++
const std::string INSTANCE_NAME{ "burma14.csv" };

const std::string INSTANCE_PATH{ "./Instances/" + INSTANCE_NAME };

std::vector<Point> points{ readInstanceFile(INSTANCE_PATH) };
```

The first lines inside the main function are used to define which instance 
will be solved, define where it is located, and read it into a 
`vector` of `Points` called `points`.

### Building the factory

```c++
GRBEnv* env{ buildGurobiEnv() };

TSPInput data{ TSPInput(points, ObjectiveType::MIN_DIST, distanceFunction_Euclidean) };

TSPCreator factory{ TSPCreator(env, data) };
```

To keep things a few more general, the code needs to receive a Gurobi env 
pointer as its first argument. The second one is an `TSPInput` called `data`, 
in which are defined:
* The points to be used to build the models;
* The objective type ([more details below](#objective-types));
* The distance function to be used in the objective ([more details below](#distance-functions)).

After that, the TSP models factory can be imediately created.

### Building and solving the models

```c++
ATSPModel* dfjModel{ factory.build(ATSP_ModelType::DFJ) };
dfjModel->solve();
dfjModel->printSolution();

ATSPModel* mtzModel{ factory.build(ATSP_ModelType::MTZ) };
mtzModel->solve();
mtzModel->printSolution();

ATSPModel* ggModel{ factory.build(ATSP_ModelType::GG) };
ggModel->solve();
ggModel->printSolution();
```

In this last part we see the three models being built, solved and the 
solutions found being printed on the screen. 

# Features available

As presented above, it is possible to set some parts of the inputs to define 
different models. These features are:

## Objective types

There are four types of objective types available, namely:

* **Minimize total route cost**: Represented by the `MIN_DIST` value on the
`ObjectiveType` enum. It is the standard objective on the classical TSP,
and is probably the most common objective in its variants;
* **Maximize total route cost**: Represented by the `MAX_DIST` value on the
`ObjectiveType` enum. It is basically the opposite of the `MIN_DIST` case. 
* **Minimize the greatest arc included in a route**: Represented by the 
`MINMAX_EDGE` value on the `ObjectiveType` enum. It can be used to define
a $\min \max$ objective over the arcs included in the route. Different of the
`MIN_DIST` case, here we are **not** intenting to produce the smallest 
possible route, but to guarantee that the arcs have the least superior limit.
* **Maximize the smallest arc included in a route**: Represented by the
`MAXMIN_EDGE` value on the `ObjectiveType` enum. It is basically the opposite
of the `MINMAX_EDGE`, imposing the goal of finding the greatest inferior
limit on the ars using a linearized $\max \min$ approach.

## Distance functions

* Euclidean distance: Represented by the `distanceFunction_Euclidean` value on
the `ATSP_ModelType` enum. It is used to impose the most common objective
function for the TSP - to minimize the euclidean distance between two points.
* Manhattan distance: Represented by the `distanceFunction_Manhattan` value on
the `ATSP_ModelType` enum. It is used to impose the Manhattan distance, or the
Taxi Cab distance, between two points, as the distance function to be used to
build the objective.
* Maximum distance: Represented by the `distanceFunction_Max` value on the
`ATSP_ModelType` enum. It allows to minimize the $\max$ distance between the
points.

## Models

There are three different classical models available to be used, namely:
* The Dantzig, Fulkerson and Jhonson (1954), denoted as **DFJ**;
* The Miller, Tucker and Zemlin (1960), denoted by **MTZ**;
* The Gavish and Graves (1978), denoted by **GG**.

The type of the model to be built is passed as an argument to the factory method 
`build()` using the `ATSP_ModelType` enum.

# About the models

## Base model

The three models available are based in the same common Assignment Problem 
structure described below:
$$\sum_{i=1}^n \sum_{j=1, j \neq i}^n c_{ij} x_{ij}.$$
$$\sum_{i=1, i \neq j}^n x_{ij}, \quad j = 1, ..., n.$$
$$\sum_{j=1, j \neq i}^n x_{ij}, \quad i = 1, ..., n.$$
The standard objective function is designed to minimize the total route cost, 
but it can be customized in the input data, as described above. The constraints
means that every city must be visited and left once.

## DFJ Model

Based on the work by Dantzig, Fulkerson and Jhonson (1954), this is one of the 
most classical formulations for the TSP. It uses no additional variables 
(*natural formulation*), and needs only one extra set of constraints to 
represent the subtour elimination condition, imposing literally that no subtour 
should be included in a solution.

The constraint set that defines the DFJ formulation is:

$$ \sum_{i \in Q} \sum_{j \in Q, j \in i} x_{ij} \leqslant |Q| - 1, \quad
\forall Q \subsetneq V, |Q| \geqslant 2. $$

It is important to notice that, despite the fact that it is just one extra set 
of constraints, the condition is imposed among almost all possible subsets of 
$V$, that is, it must be defined to almost all elements in the power set of 
$V$, and so the number of constraints is of order $O(2^n)$.

In summary, the DFJ model is an interesting formulation in the theoretical 
sense, **but impracticable for most applications**.

## MTZ Model

Proposed by Miller, Tucker and Zemlin in 1960, it is another classical approach
on the subject formulations for the TSP. As in the DFJ case, this subtour 
elimination constraints set can be obtained by adding just an 
extra constraints set to the (AP), but in this case it will be necessary
to add a new set of variables to.

The new variables and constraints are:
$$u_i - u_j + 1 \leqslant (n-1) (1 - x_{ij}), \quad i, j = 2, ..., n; i \neq j. $$
$$2 \leqslant u_i \leqslant n, \quad i = 2, ..., n. $$

An advantage of this formulation is the small number of variables and
constraints needed to define it, making it a *compact formulation*. On the
other hand, in the mathematical sense, the MTZ formulation is a very weak
formulation, and tends to take a long time to be solved in many cases.

## GG Model

Finally, the factory was coded to build the model by Gavish and Graves (1978) 
too. This is the earliest single commodity flow formulation, in which we need
to add a new set of variables $g_{ij}$, for all $i, j \in V, i \neq j$. The new
constraints sets are given by:
$$\sum_{j=1}^n g_{ji} - \sum_{j=2}^n g_{ij} = 1, \quad i = 2, ..., n. $$
$$0 \leqslant g_{ij} \leqslant (n-1) x_{ij}, \quad i = 1,...,n; \quad j = 2, ..., n.$$

As in the MTZ formulation, here we are adding a set of $n^2$ variables, and just
one set of $n^2$ constraints, leading to a *compact formulation*. It is possible
to prove that the GG formulation is stronger then the MTZ, but weaker then the
DFJ.

# Contact

Critics or suggestions can be sent directly to the e-mail [eduardo281.dev@gmail.com](mailto:eduardo281.dev@gmail.com).

Some details about the implementation of the code will be included in this 
`README` file in the future, but right now the information presented should
be sufficient to allow interested users to run the code and test different
options on distance functions, objective functions and results for different
instances. 

New models and variants of the problem will be included in the future.

Ideas on how to improve the code or the documentation are totally welcome!
