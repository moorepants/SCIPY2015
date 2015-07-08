==================================================================================================
Optimal Control and Parameter Identification of Dynamcal Systems with Direct Collocation and SymPy
==================================================================================================

:Author: Jason K. Moore

Abstract
========

There are variety of techniques for approaching the optimal control and
parameter identification problems of dynamical systems. Traditionally, discrete
methods for linear systems have been utilized and/or various shooting
optimization techniques for non-linear systems. But more recently the direct
collocation method has been used to formulate these two problems in terms of a
non-linear programming (NLP) problem where large scale sparse optimizer can be
utilized to find the optimal solution. The methods have proved valuable because
the computation time can be reduced by many orders of magnitude relative to
shooting, local minima are less of a problem, and unstable systems can easily
be dealt with.

The translation of an optimal control or parameter identification problem into
a non-linear programming problem is not trivial. I will present a lightweight
Python package that translates high level symbolic descriptions of a dynamic
system and the optimization objectives to an efficient implementation of a NLP
problem which can then be passed to a variety of solvers, such as the open
source IPOPT. This package, opty, allows the user to define a problem in very
few lines of code which directly mirrors the math that defines the high level
description of the problem. opty can be used to solve a wide variety of
problems and I will demonstrate its effectiveness and ease of use on both
classic problems and some research grade problems in the biomechanics and
vehicle dynamics domains.

Description
===========

Introduction
------------

The translation of the optimal control problem into a non-linear programming
problem of begins with the system description. First the equations of motion
(continuous ordinary non-linear differential equations) are defined::

   0 = f(x''(t), x'(t), x(t), u(t), p, t)

where ``x(t)`` are the states, ``u(t)`` are the exogenous inputs, ``p`` are the
model constants, and ``t`` is time. In addition various various quantities of
interest, i.e. outputs needed for the objective evaluation, are defined as::

   y = g(x(t), u(t), p, t)

The outputs, ``y``, are used to define a scalar objective function that depends
on the unknown inputs, ``u*``, and/or unknown parameters, ``p*``::

   Q(u*, p*)

There may also be additional constraints::

   b(u*, p*)

This is in contrast to a typical non-linear programming problem that is defined
as::

   min      J(w)
   w E R^n

   cl <= c(w) <= cu
   wl <= w <= wu

where an objective function, ``J(w)``, is a function of the optimization
variables ``w`` and is subject to the general non-linear constraints ``c(w)``
and bounds on the optimization variables.

The construction of ``c(w)`` and its sparse Jacobian, ``dc(w)/dw``, for a
complex ``f()`` and ``b()`` is a time consuming and error prone process. opty
allows the user to specify the optimal control problem as ``f()``, ``g()``,
``Q()``, and ``b()`` and it automatically transforms the equations of motion
into efficient numerical implementations of ``J()`` and ``c()``.

Usage
-----

The user defines the optimization problem using SymPy::

   f = Matrix([theta(t).diff() - omega(t),
               I * omega(t).diff() + m * g * d * sin(theta(t)) - T(t)])
   Q = Integral(T * theta(t), (t, 0, 1))
   b = [theta(0),
        theta(1) - pi,
        omega(0),
        omega(1)]

This describes a simple pendulum in which the objective to move the pendulum
from its initial hanging position to a vertical position with the input torque
``T`` but in an energy minimal fashion, i.e. minimal work.

Next the user provides this information to opty via the ``Problem`` class::

   p = Problem(Q,  # objective
               f,  # equations of motion
               [theta(t), omega(t)],  # states
               100,  # number of collocation nodes
               0.01,  # node interval in seconds
               known_parameter_map={I: 1.0, m: 1.0, d: 1.0, g: 9.8},
               instance_constraints=b, bounds={T: (-1.5, 1.5)})

When the problem is initialized SymPy's code generation features are used
behind the scenes to implement efficient numerical functions for the objective,
the gradient of the objective, the constraints, and the Jacobian of the
constraints. The problem can then be solved with IPOPT via the ``.solve()``
method::

   solution, info = p.solve(np.zeros(prob.num_free))

At this point opty hands off the functions to IPOPT through the cyipopt Cython
bindings for the actual NLP solution. IPOPT's multitude of configuration
options can be set with the ``.add_option`` method, for example::

   p.add_option('linear_solver', 'ma57')

Implementation
--------------

The transformation of the optimal control specification to the NLP problem
follows these steps:

1. Identify the known and unknown parameters and trajectories.
2. Construct the optimization parameter list ``w()`` from the states at each
   collocation node, the unknown trajectories, and unknown parameters.
3. Construct the symbolic ``c(w)`` vector by combining the equation of motion
   constraints and instance constraints and introducing the backward Euler or
   midpoint discretization.
4. Symbolically differentiate ``c(w)`` with respect to ``w`` to form a matrix
   that specifies the non-zero entries of the Jacobian of ``c(w)``.
5. Generate wrapped C/Cython based vectorized implementations of ``c`` and
   ``dc/dw`` that evaluate the matrices given array inputs. The sparse Jacobian
   is provided in triplet form.
6. Symbolically differentiate the objective function ``J(w)`` with respect to
   ``w`` to define the gradient.
7. Generate a numerical functions that evaluate the objective and its gradient.
8. Setup all the functions and parameters for hand-off to IPOPT in the
   ``Problem`` class.

The translation from symbolics to numerics is handled by SymPy's code
generation facilities which identifies common sub-expressions before
compilation, among other things, for optimized C code. All of the above is
handle with a few classes spread across a few modules in the opty package.

Examples
--------

After the introduction to the methods and software, I will demonstrate several
example problems with the software that range from optimal control of a human
balancing and directing a bicycle, optimal jumping of the Pixar lamp logo, and
several classical difficult optimization problems.

Conclusion
----------

SymPy excels at providing a way to expressively describe mathematical
constructs in a high level way and has the ability to covert those constructs
to fast numerical codes. opty makes use of these facilities to implement a user
friendly and efficient framework for solving general optimal control and
parameter identification problems with direct collocation. The use cases are
wide and the solutions play an important role in understanding the trajectory
evolution of dynamical systems that can be described by continuous ordinary
differential equations.

References
----------

- Personal website: http://moorepants.info
- opty source repository: https://github.com/csu-hmc/opty
- SymPy: http://www.sympy.org
- Paper draft for an opty research application:
  https://github.com/csu-hmc/inverted-pendulum-sys-id-paper
- An example of public speaking: https://youtu.be/H9AK65ZY-Vw
