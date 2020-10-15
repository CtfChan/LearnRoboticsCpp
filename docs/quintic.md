ntroduction
Quintic splines are a common path parametrization in robotics. A quintic polynomial is a 5th order polynomial. Each dimension $(x, y)$ is represented as a quintic polynomial. The coefficients $a_i$ and $b_i$ can be solved for provided we give the robot's start position, velocity and acceleration ($x_s$, $v_s$, $a_s$) as well as the end position, velocity and acceleration ($x_f$, $v_f$, $a_f$); these are also known the boundary conditions.

$$x(t) = a_o + a_1 t + a_2 t ^2 + a_3 t ^ 3 + a_4 t ^ 4 + a_5 t^5   $$

$$y(t) = b_o + b_1 t + b_2 t ^2 + b_3 t ^ 3 + b_4 t ^ 4 + b_5 t^5   $$

## Issues
There are discontinuities in the curvature paramatrization of quintic splines, which makes the curvature hard to constrain. From the equation of curvature $k(t)$ below, we can see that the denominator of the equation could be undefined.

$$k(t) = \frac{x'(t) y''(t) - y'(t)x''(t)}{ (x'(t)^2 + y'(t)^2)^{3/2}  }$$

## Solving the Equation
Let us consider only `x(t)`. We can solve for $a_i$ provided that we are given the start position, velocity and acceleration ($x_s$, $v_s$, $a_s$) as well as the end position, velocity and acceleration ($x_f$, $v_f$, $a_f$); these are also known the boundary conditions of $x(t)$'s zeroth, first and second derivative.

First derivative:
$x'(t) = a_1 + 2 a_2 t  + 3 a_3 t ^ 2 + 4 a_4 t ^ 3 + 5 a_5 t^ 4   $

Second derivative:
$x''(t) = 2 a_2 t  +  6 a_3 t + 12 a_4 t ^ 2 + 20 a_5 t^ 3    $

Using our initial boundary conditions for $t = 0$ we can easily solve for $a_0$, $a_1$, $a_2$.

$x(0) = x_s = a_0 $

$x'(0) = v_s = a_1 $

$x''(0) = a_s = 2 a_2 $

Now lets solve for $a_3$, $a_4$, $a_5$ using our boundary conditions of $t = T$.

Eqn 1:  $x(T) = x_e = a_o + a_1 T + a_2 T ^2 + a_3 T ^ 3 + a_4 T ^ 4 + a_5 T^5   $

Eqn 2: $x'(T) = v_e = a_1 + 2 a_2 T  + 3 a_3 T ^ 2 + 4 a_4 T ^ 3 + 5 a_5 T^ 4   $

Eqn 3: $x''(T) = a_e = 2 a_2 T  +  6 a_3 T + 12 a_4 T ^ 2 + 20 a_5 T^ 3    $

Since we have already solved for $a_0$, $a_1$, $a_2$, we can rearrange (1), (2) and (3) to have all of our unknwons on the left and known variables on the right side of the equality.

Eqn 1:  $ a_3 T ^ 3 + a_4 T ^ 4 + a_5 T^5  = x_e - a_0 - a_1 T - a_2 T ^2  $

Eqn 2:  $ 3 a_3 T ^ 2 + 4 a_4 T ^ 3 + 5 a_5 T^4  = v_e - a_1 - a_1 T - 2 a_2 T $

Eqn 3:  $ 6 a_3 T + 12 a_4 T ^ 2 + 20 a_t T^3 = a_e - 2 a_2 $

The three equations above can now be conveniently factored into a system of linear equations $Ax=b$.

$$
\begin{pmatrix}
T^3 & T^4 & T^5 \\
3T^2 & 4 T ^3 & 5 T ^ 4 \\
6T & 12 T ^2 & 20 T^3
\end{pmatrix}
\begin{pmatrix}
a_3 \\
a_4 \\
a_5
\end{pmatrix}
=\space
\begin{pmatrix}
x_e - a_0 - a_1 T - a_2 T ^2 \\
v_e - a_1 - a_1 T - 2 a_2 T \\
a_e - 2 a_2
\end{pmatrix}
$$

Since $T \geq 0$ our A matrix will be invertible and we can solve for $a_3$, $a_4$, $a_5$ using $x = A^{-1} b$.



