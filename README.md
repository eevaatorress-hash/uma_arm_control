# uma_arm_control
En este repositorio se estudiarán los resultados de Lab2, Lab3 y Lab4.

# Lab2
En esta práctica se añadirán las ecuaciones dinámicas a la cadena cinemática de nuestro manipulador para poder observar su comportamiento. Será en Laboratorios posteriores cuando,  además de la dinámica natural del manipulador, se incorporarán controladores para contrarrestar dichos efectos.

Se prestará especial atención al cálculo de las aceleraciones articulares puesto que estas son las que permiten observar los cambios en fuerzas y torques al realizarse los movimientos.

## Fundamento teórico
La cadena cinemática del manipulador se define con la siguiente ecuación:

$$
\boldsymbol{M(q)\ddot{q} + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = \tau + \tau_{ext}}
$$

donde

- $\boldsymbol{q \in \mathbb{R}^{n\times1}}$ es el vector de las posiciones articulares (`joint_positions_`).

- $\boldsymbol{\dot{q} \in \mathbb{R}^{n\times1}}$ es el vector de las velocidades articulares (`joint_velocities_`).

- $\boldsymbol{\ddot{q} \in \mathbb{R}^{n\times1}}$ es el vector de las aceleraciones articulares (`joint_accelerations_`).

- $\boldsymbol{M(q) \in \mathbb{R}^{n\times n}}$ es la matriz de inercia.

- $\boldsymbol{C(q,\dot{q}) \in \mathbb{R}^{n\times n}}$ es la matriz de fuerzas centrífugas y de coriolis.

- $\boldsymbol{F_b \in \mathbb{R}^{n\times n}}$ es la matriz de fricción viscosa.

- $\boldsymbol{g \in \mathbb{R}^{n\times1}}$ es el vector de gravedad.

- $\boldsymbol{\tau \in \mathbb{R}^{n\times1}}$ es el vector de torques articulares de control (`joint_torques_`).

- $\boldsymbol{\tau_{ext} \in \mathbb{R}^{n\times1}}$ es el vector de torques articulares provocados por fuerzas externas.

En nuestro caso, al contar con dos grados de libertad, n = 2. Si se despeja las aceleraciones articulares de la ecuación anterior se obtiene:

$$
\boldsymbol{\ddot{q} = M^{-1}(q)\left[\tau + \tau_{ext} - C(q,\dot{q})\dot{q} - F_b\dot{q} - g(q)\right]}
$$

Para calcular las aceleraciones articulares es necesario evaluar las matrices anteriores. Para ello, se aplicarán las formulaciones de Lagrange o las de Newton-Euler. Las matrices quedan definidas como aparece a continuación:

$$
\boldsymbol{M(q)=
\begin{bmatrix}
m_1l_1^2 + m_2\left(l_1^2 + 2l_1l_2\cos(q_2) + l_2^2\right) &
m_2\left(l_1l_2\cos(q_2)+l_2^2\right)
\\
m_2\left(l_1l_2\cos(q_2)+l_2^2\right) &
m_2l_2^2
\end{bmatrix}}
$$

$$
\boldsymbol{C(q,\dot{q})\dot{q}=
\begin{bmatrix}
-m_2l_1l_2\sin(q_2)\left(2\dot{q}_1\dot{q}_2+\dot{q}_2^2\right)
\\
m_2l_1l_2\dot{q}_1^2\sin(q_2)
\end{bmatrix}}
$$

$$
\boldsymbol{F_b=
\begin{bmatrix}
b_1 & 0 \\
0 & b_2
\end{bmatrix}}
$$

$$
\boldsymbol{g(q)=
\begin{bmatrix}
(m_1+m_2)l_1g\cos(q_1)+m_2gl_2\cos(q_1+q_2)
\\
m_2gl_2\cos(q_1+q_2)
\end{bmatrix}}
$$

Para poder tener en cuenta las fuerzas externas aplicadas sobre el efector final, se calcula el siguiente jacobiano:

$$
\boldsymbol{J(q)=
\begin{bmatrix}
-l_1\sin(q_1)-l_2\sin(q_1+q_2) &
-l_2\sin(q_1+q_2)
\\
l_1\cos(q_1)+l_2\cos(q_1+q_2) &
l_2\cos(q_1+q_2)
\end{bmatrix}}
$$

Se calcula también el vector de torques articulares provocados por fuerzas externas:

$$
\boldsymbol{\tau_{ext}=J(q)^T \cdot F_{ext}}
$$

Al tratarse de un sistema discreto, hará falta discretizar las ecuaciones anteriores:

$$
\boldsymbol{\ddot{q}_{k+1}=M^{-1}(q_k)\left[\tau_k+\tau_{ext_k}-C(q_k,\dot{q}_k)\dot{q}_k-F_b\dot{q}_k-g(q_k)\right]}
$$

$$
\boldsymbol{\dot{q}=\int\ddot{q}\,dt \Longrightarrow \dot{q}_{k+1}=\dot{q}_k+\ddot{q}_{k+1}\Delta t}
$$

$$
\boldsymbol{q=\int\dot{q}\,dt \Longrightarrow q_{k+1}=q_k+\dot{q}_{k+1}\Delta t}
$$

## Aplicación práctica

Para poder aplicar lo visto en el fundamento teórico se creará el nodo de `uma_arm_dynamics.cpp`, donde se añade el siguiente código para la calculación de las aceleraciones articulares:

~~~
// Initialize M, C, Fb, g_vec, J, and tau_ext
Eigen::MatrixXd M(2, 2);
Eigen::VectorXd C(2);
Eigen::MatrixXd Fb(2, 2);
Eigen::VectorXd g_vec(2);
Eigen::MatrixXd J(2, 2);
Eigen::VectorXd tau_ext(2);

// Initialize q1, q2, q_dot1, and q_dot2
double q1 = joint_positions_(0);
double q2 = joint_positions_(1);
double q_dot1 = joint_velocities_(0);
double q_dot2 = joint_velocities_(1);

// Placeholder calculations for M, C, Fb, g, and tau_ext
// Calculate matrix M
M(0, 0) = m1_ * pow(l1_, 2) + m2_ * (pow(l1_, 2) + 2 * l1_ * l2_ * cos(q2) + pow(l2_, 2));
M(0, 1) = m2_ * (l1_ * l2_ * cos(q2) + pow(l2_, 2));
M(1, 0) = M(0, 1);
M(1, 1) = m2_ * pow(l2_, 2);

// Calculate vector C (C is 2x1 because it already includes q_dot)
C << -m2_ * l1_ * l2_ * sin(q2) * (2 * q_dot1 * q_dot2 + pow(q_dot2, 2)),
m2_ * l1_ * l2_ * pow(q_dot1, 2) * sin(q2);

// Calculate Fb matrix
Fb << b1_, 0.0,
0.0, b2_;

// Calculate g_vect
g_vec << (m1_ + m2_) * l1_ * g_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2),
    m2_ * g_ * l2_ * cos(q1 + q2);

// Calculate J
J << -l1_ * sin(q1) - l2_ * sin(q1 + q2), -l2_ * sin(q1 + q2),
    l1_ * cos(q1) + l2_ * cos(q1 + q2), l2_ * cos(q1 + q2);

// Calculate tau_ext
tau_ext << J.transpose() * external_wrenches_;

// Calculate joint acceleration using the dynamic model: M * q_ddot = torque - C * q_dot - Fb * joint_velocities_ - g + tau_ext
Eigen::VectorXd q_ddot(2);
q_ddot << M.inverse() * (joint_torques_ - C - Fb * joint_velocities_ - g_vec + tau_ext);

return q_ddot;
~~~

Para las posiciones y velocidades articulares:

~~~
// Method to calculate joint velocity
Eigen::VectorXd calculate_velocity()
{
    // Placeholder for velocity calculation
    // Integrate velocity over the time step (elapsed_time_)
    Eigen::VectorXd q_dot = joint_velocities_ + joint_accelerations_ * elapsed_time_;

    return q_dot;
}

// Method to calculate joint position
Eigen::VectorXd calculate_position()
{
    // Placeholder for position calculation
    // Integrate position over the time step (elapsed_time_)
    Eigen::VectorXd q = joint_positions_ + joint_velocities_ * elapsed_time_;

    return q;
}
~~~

## Resultados
Los resultados del laboratorio se obtienen al lanzar los siguientes comandos en diferentes terminales: `ros2 launch uma_arm_description uma_arm_visualization.launch.py`, `ros2 launch uma_arm_control uma_arm_dynamics_launch.py`. Sin embargo, si además se quieren observar las fluctuaciones de posición, velocidad y aceleración articulares, hace falta grabar los experimentos y reproducirlos utilizando `ros2 bag record --all -o experiment3` y `plotjuggler`.

![Resultados del Lab2](/images/Resultados_Lab2.png)

Para entender más aún los efectos de los distintos elementos necesarios para llevar a cabo los cálculos anteriores, se realizan los siguientes experimentos:

- Aumento de las masas de las articulaciones. El manipulador presenta una respuesta más oscilatoria y mayores desviaciones respecto a la posición de equilibrio debido al incremento de la inercia del sistema.

![Resultados del Lab2 al cambiar las masas](/images/Resultados_cambio_masa.png)

- Aumento de los coeficientes de fricción viscosa. El sistema presenta un mayor amortiguamiento puesto que se ayuda a disipar energía del sistema, reduciendo progresivamente las oscilaciones y haciendo que las velocidades tiendan a cero con mayor rapidez.

![Resultados del Lab2 al cambiar los coeficientes de fricción](/images/Resultados_cambio_friccion.png)

- Aumento de la gravedad. Aumentan las fuerzas gravitacionales que actúan sobre el manipulador, generando una respuesta dinámica similar al caso de aumento de masas.

![Resultados del Lab2 al cambiar la gravedad](/images/Resultados_cambio_gravedad.png)