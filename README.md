# uma_arm_control
En este repositorio se estudiarán los resultados de Lab2, Lab3 y Lab4 de la asignatura Ampliación de Robótica de la parte de manipualdores.
Para reproducir los resultados, también es necesario hacer uso del paquete `uma_arm_description`, el cual puede obtenerse al lanzar este comando dentro de `your_ros2_ws/src`:

~~~
git clone https://github.com/jmgandarias/uma_arm_description.git
~~~

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

En nuestro caso, al contar con dos grados de libertad, n = 2. Si se despejan las aceleraciones articulares de la ecuación anterior se obtiene:

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
Los resultados del laboratorio se obtienen al lanzar los siguientes comandos en diferentes terminales: `ros2 launch uma_arm_description uma_arm_visualization.launch.py`, `ros2 launch uma_arm_control uma_arm_dynamics_launch.py`. Sin embargo, si además se quieren observar las fluctuaciones de posición, velocidad y aceleración articulares, se pueden grabar los experimentos con `ros2 bag record --all -o experiment` y reproducirlos utilizando `ros2 bag play` y `plotjuggler`. El resultado es el de la figura 2.1.

<p align="center">
    <img src="/images/Resultados_Lab2.png">
    <br>
    <em>Figura 2.1: Resultados del Lab2.</em>
</p>

Para entender más aún los efectos de los distintos elementos necesarios para llevar a cabo los cálculos anteriores, se realizan los siguientes experimentos:

- Aumento de las masas de las articulaciones. El manipulador presenta una respuesta más oscilatoria y mayores desviaciones respecto a la posición de equilibrio debido al incremento de la inercia del sistema. Este efecto puede ser observado en la figura 2.2.

<p align="center">
    <img src="/images/Resultados_cambio_masa.png">
    <br>
    <em>Figura 2.2: Resultados del Lab2 al cambiar las masas.</em>
</p>

- Aumento de los coeficientes de fricción viscosa. El sistema presenta un mayor amortiguamiento puesto que se ayuda a disipar energía del sistema, reduciendo progresivamente las oscilaciones y haciendo que las velocidades tiendan a cero con mayor rapidez. Este efecto puede ser observado en la figura 2.3.

<p align="center">
    <img src="/images/Resultados_cambio_friccion.png">
    <br>
    <em>Figura 2.3: Resultados del Lab2 al cambiar los coeficientes de fricción.</em>
</p>

- Aumento de la gravedad. Aumentan las fuerzas gravitacionales que actúan sobre el manipulador, generando una respuesta dinámica similar al caso de aumento de masa. Este efecto puede ser observado en la figura 2.4.

<p align="center">
    <img src="/images/Resultados_cambio_gravedad.png">
    <br>
    <em>Figura 2.4: Resultados del Lab2 al cambiar la gravedad.</em>
</p>

# Lab 3

En esta práctica se realizarán tres tareas. La primera consiste en compensación de la gravedad para permitir que el brazo permanezca en una pose fija sin caerse debido a la gravedad. En la segunda tarea se desarrolla un controlador que compensa toda la dinámica no lineal del manipulador usando un modelo inverso de la dinámica. La última tarea tiene como objetivo desarrollar un controlador PD en espacio articular con compensación de dinámica no lineal.

## Compensación de la gravedad

### Fundamento teórico

Se realizará un controlador centralizado que generará en los motores un torque que compense el producido por la gravedad. Para ello, el torque será el siguiente:

$$
\boldsymbol{\tau=g(q)} = 
\begin{bmatrix}
(m_1+m_2) \cdot l_1 \cdot g \cdot \cos(q_1)+m_2 \cdot g \cdot l_2 \cdot \cos(q_1+q_2)
\\
m_2 \cdot g \cdot l_2 \cdot \cos(q_1+q_2)
\end{bmatrix}
$$

### Aplicación práctica

Se ha creado un nodo llamado `gravity_compensation.cpp` y su archivo launch correspondiente llamado `gravity_compensation_launch.py`. El código para obtener el torque deseado es el siguiente:
~~~
    // Method to calculate the desired joint torques
    Eigen::VectorXd gravity_compensation()
    {
        // Placeholder for calculate the commanded torques
        // Calculate the control torque to compensate only for gravity effects: tau = g(q)
        Eigen::VectorXd g_vec(2);
        double q1 = joint_positions_(0);
        double q2 = joint_positions_(1);

        // Calculate g_vect
        g_vec << (m1_ + m2_) * l1_ * g_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2),
        m2_ * g_ * l2_ * cos(q1 + q2);

        // Calculate desired torque
        Eigen::VectorXd torque(2);
        torque = g_vec;

       return torque;
    }
~~~

### Resultados
Para simular el comportamiento del brazo con un controlador que compensa la gravedad se lanzan estos 4 comandos, cada uno en una terminal diferente:  
`ros2 launch uma_arm_description uma_arm_visualization.launch.py`  
`ros2 launch uma_arm_control gravity_compensation_launch.py`  
`ros2 launch uma_arm_control uma_arm_dynamics_launch.py`  
`python3 wrench_trackbar_publisher.py`  
Inicialmente se puede observar como el brazo se mantiene fijo en la posición inicial mostrada en la figura 3.1.1.

<p align="center">
    <img src="/images/gc_pose_inicial.png">
    <br>
    <em>Figura 3.1.1: Posición inicial del brazo.</em>
</p>


Se le aplica una fuerza puntual positiva en el eje $x$ y luego en el eje $y$, obteniendo el comportamiento mostrado en la figura 3.1.2.

<p align="center">
    <img src="/images/gc_grafica_movimiento.png">
    <br>
    <em>Figura 3.1.2: Gráfica de los resultados de compensación de la gravedad.</em>
</p>

Se observa que las posiciones se mantienen constantes hasta la aplicación de una primera fuerza, momento en el que ambas articulaciones comienzan a presentar velocidad y aceleración. Al cesar la fuerza aplicada, la velocidad empieza a disminuir y la aceleración cambia de signo, ya que el sistema pasa de moverse a frenarse debido a las fuerzas viscosas del modelo. Una vez desaparece el efecto de la fuerza externa, el controlador desarrollado permite que el brazo mantenga una nueva posición estable. Este mismo proceso se repite al aplicar una segunda fuerza. Finalmente el brazo se queda fijo en la posición mostrada en la figura 3.1.3.

<p align="center">
    <img src="/images/gc_pose_final.png">
    <br>
    <em>Figura 3.1.3: Posición final del brazo.</em>
</p>

En el caso de que los valores $m1$, $m2$, $l1$ o $l2$ sean incorrectos, la compensación deja de funcionar correctamente, ya que el controlador estima de forma errónea el par asociado a la gravedad. Por ejemplo, si alguno de estos valores es menor que el real, el par calculado por gravedad será inferior al real, por lo que el robot aplicará menos esfuerzo del necesario y terminará cediendo progresivamente hasta caer. Este efecto puede ser observado en la figura 3.1.4.

<p align="center">
    <img src="/images/yamlreducido_gc.png">
    <br>
    <em>Figura 3.1.4: Simulación del comportamiento al disminuir m1, m2, l1, l2.</em>
</p>

En el caso de aumentar estos valores, ocurre el efecto contrario, se cree que el par generado por la gravedad es mayor, por lo que aplica más esfuerzo del necesario y el brazo sube. Este efecto puede ser observado en la figura 3.1.5.

<p align="center">
    <img src="/images/yamlaumentadas_gc.png">
    <br>
    <em>Figura 3.1.5: Simulación del comportamiento al aumentar m1, m2, l1, l2.</em>
</p>

## Compensación de la dinámica

### Fundamento teórico

Para compensar la dinámica no lineal del manipulador, es necesario calcular la cancelación de dicha dinámica a partir de las aceleraciones articulares deseadas ($\boldsymbol{\ddot{q}}_d$) y del estado actual de las articulaciones ($\boldsymbol{q},\boldsymbol{\dot{q}}$). Se usará el controlador de la figura 3.2.1.

<p align="center">
    <img src="/images/dc_esquema.png">
    <br>
    <em>Figura 3.2.1: Controlador de compensación de la dinámica.</em>
</p>

Los pares articulares vienen dados por:

$$
\boldsymbol{\tau = M(q)\ddot{q}}_d + \boldsymbol{n(q,\dot{q}) = M(q)\ddot{q}}_d + \boldsymbol{C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q)}
$$

Evaluando cada matriz de la misma forma que en el Lab 2:

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

### Aplicación práctica

Se ha creado un nodo llamado `dynamics_cancellation.cpp` y su archivo launch correspondiente llamado `dynamics_cancellation_launch.py`. El código para obtener el torque deseado es el siguiente:

~~~
// Method to calculate joint torques
Eigen::VectorXd cancel_dynamics()
{
    // Initialize M, C, Fb, g_vec
    Eigen::MatrixXd M(2, 2);
    Eigen::VectorXd C(2);
    Eigen::MatrixXd Fb(2, 2);
    Eigen::VectorXd g_vec(2);

    // Initialize q1, q2, q_dot1 and q_dot2
    double q1 = joint_positions_(0);
    double q2 = joint_positions_(1);
    double q_dot1 = joint_velocities_(0);
    double q_dot2 = joint_velocities_(1);

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

    // Calculate g_vec
    g_vec << (m1_ + m2_) * l1_ * g_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2),
        m2_ * g_ * l2_ * cos(q1 + q2);

    // Calculate control torque using the dynamic model: torque = M * q_ddot + C  + Fb * q_dot + g
    Eigen::VectorXd torque(2);
    torque = M * desired_joint_accelerations_ + C + Fb * joint_velocities_ + g_vec;

    return torque;
}
~~~

### Resultados
Para ver el comportamiento del brazo con un controlador que compensa la dinámica al mandar una trayectoria cúbica se lanzan estos 4 comandos, cada uno en una terminal diferente:  
`ros2 launch uma_arm_description uma_arm_visualization.launch.py`  
`ros2 launch uma_arm_control dynamics_cancellation_launch.py`  
`ros2 launch uma_arm_control uma_arm_dynamics_launch.py`  
`python3 cubic_trajectory.py`

Se obtiene el comportamiento de la figura 3.2.2.

<p align="center">
    <img src="/images/dynamics_cancellation.png">
    <br>
    <em>Figura 3.2.2: Simulación de compensación de la dinámica.</em>
</p>

La aceleración es lineal, la velocidad parabólica y la trayectoria cúbica tal y como se esperaba.

Si alguno de los parámetros $m1$,  $m2$,  $l1$,  $l2$,  $b1$ o $b2$ es incluso ligeramente incorrecto, el sistema pierde estabilidad, llegando a ser incapaz de mantenerse en reposo en la misma posición. Esto ocurre porque la cancelación de dinámica depende de un modelo muy preciso, cualquier error provoca comportamientos muy inesperados al ser un sistema complejo y no lineal.

## Controlador PD en espacio articular con compensación de dinámica no lineal

### Fundamento teórico
Una vez compensada la dinámica no lineal del manipulador mediante el controlador de dinámica inversa, es posible diseñar un controlador lineal para regular su posición articular. Para ello, se implementa un controlador PD en espacio articular que calcula las aceleraciones articulares deseadas a partir del error de posición y velocidad entre el estado actual del robot y una configuración de referencia. El esquema del nuevo controlador es el mostrado en la figura 3.3.1.

<p align="center">
    <img src="/images/pd_esquema.png">
    <br>
    <em>Figura 3.3.1: Esquema de un controlador pd compensador de la dinámica.</em>
</p>

Donde:

- $\boldsymbol{K}_P = diag\{ \omega ²_{n_1},...,  \omega ²_{n_n}\}$ es la matriz diagonal de ganancias proporcionales (`KP`).
- $\boldsymbol{K}_D = diag\{ 2 \zeta _1 \omega ²_{n_1},..., 2 \zeta _n \omega ²_{n_n}\}$ es la matriz diagonal de ganancias derivativas (`KD`).
- $\boldsymbol{y}$ es la aceleración deseada en un instante.

Se establece $\boldsymbol{\dot{q}}_d = \boldsymbol{0}$, $\boldsymbol{\ddot{q}}_d = \boldsymbol{0}$ y $\boldsymbol{q}_d$ cualquier posición dentro del espacio articular. La aceleración y velocidad deseadas son nulas ya que una vez alcanzada la posición final, el brazo debe detenerse. De esta forma se obtiene al siguiente ecuación:

$$
\boldsymbol{y} = 
\boldsymbol{K}_P (\boldsymbol{q}_d - \boldsymbol{q}) - \boldsymbol{K}_D \boldsymbol{\dot{q}} 
$$

### Aplicación práctica

Se ha creado un nuevo nodo denominado `pd_controller.cpp`, en el que se implementa el control lineal estabilizante. En este nodo se definen las matrices $\boldsymbol{K}_P$ y $\boldsymbol{K}_D$, así como la posición articular deseada a la que se quiere alcanzar el sistema. El nodo se suscribe a las señales de posición y velocidad articular actuales, y a partir de esta información calcula y publica la aceleración articular deseada.

~~~
// Method to calculate desired joint acceleration
Eigen::VectorXd compute_pd_acceleration()
{
    Eigen::Vector2d qddot_desired = - KD * joint_velocities_ + KP * ( q_ref  - joint_positions_);
    return qddot_desired;
}
~~~

Una vez publicada la aceleración, ésta es leída por el compensador de la dinámica permitiendo así que el robot alcance dicha posición. 

Para obtener un movimiento con pocas oscilaciones, se toma $\zeta = 2$. Por otro lado, se selecciona una frecuencia natural $\omega_n = 1$ para poder visualizar el movimiento del robot.

### Resultados
Para ver el comportamiento del brazo con el controlador pd lineal se lanzan estos 4 comandos, cada uno en una terminal diferente:  
`ros2 launch uma_arm_description uma_arm_visualization.launch.py`  
`ros2 launch uma_arm_control dynamics_cancellation_launch.py`  
`ros2 launch uma_arm_control uma_arm_dynamics_launch.py`  
`ros2 run uma_arm_control pd_controller`

Con los valores descirtos anteriormente y una posición de $[0.0, \ 3.14/2]$ se consigue el siguiente resultado mostrado en las figuras 3.3.2 y 3.3.3.

<p align="center">
    <img src="/images/pd1.png">
    <br>
    <em>Figura 3.3.2: Simulacion con control pd 1.</em>
</p>

<p align="center">
    <img src="/images/pd1_grafica.png">
    <br>
    <em>Figura 3.3.3: Simulacion con control pd 1 grafica.</em>
</p>

El manipulador alcanza de forma correcta la posición final con velocidad ya celeración nulas. En las posición se puede observar una pequeña sobreoscilación debida a la inercia. Si se quiere alcanzar la posición final de forma más rápida, se puede aumentar la frecuencia antural a $\omega_n = 15$ por ejemplo. Este efecto se puede observar en la figura 3.3.4.

<p align="center">
    <img src="/images/pd2_grafica.png">
    <br>
    <em>Figura 3.3.4: Simulacion con control pd 2 grafica.</em>
</p>

En el caso de que se quiera un movimiento más suave y más controlado, se pueden establecer estos valores: $\omega_n = 0.5$ y $\zeta = 4$. De esta forma se obtiene un resultado sin sobreoscilación. Este efecto se puede observar en la figura 3.3.5.

<p align="center">
    <img src="/images/pd3_grafica.png">
    <br>
    <em>Figura 3.3.5: Simulacion con control pd 3 grafica.</em>
</p>

# Lab 4
En esta práctica se implementará en el paquete de `uma_arm_control` el esquema del controlador de impedancia mostrado en la figura 4.1.

<p align="center">
    <img src="/images/Esquema_modelo.png">
    <br>
    <em>Figura 4.1: Esquema modelo con controlador de impedancia.</em>
</p>

## Fundamento teórico
Este controlador permite al manipulador ajustar su fuerza y movimiento al interactuar con el entorno en vez de mantener una posición rígida.
Como tal la compensación a nivel articular la realiza el nodo diseñado en el laboratorio anterior, `dynamics_cancellation`, por eso en este nodo se pasarán esas variables del espacio operacional al cartesiano.

Se irán realizando los siguientes procedimientos:
- Obtener la posición cartesiana a partir de la articular.

$$
\mathbf{x} =
\begin{bmatrix}
l_1 \cos(q_1) + l_2 \cos(q_1 + q_2) \\
l_1 \sin(q_1) + l_2 \sin(q_1 + q_2)
\end{bmatrix}
$$

- Obtener los jacobianos para poder obtener la relación entre movimientos articulares y datos cartesianos. La derivada del jacobiano permitirá corregir el error en el movimiento.

$$
\mathbf{J(q)} =
\begin{bmatrix}
-l_1\sin(q_1) - l_2\sin(q_1+q_2) & -l_2\sin(q_1+q_2) \\
l_1\cos(q_1) + l_2\cos(q_1+q_2) & l_2\cos(q_1+q_2)
\end{bmatrix}
$$

$$
\mathbf{\dot{J}}(\mathbf{q},\dot{\mathbf{q}}) =
\begin{bmatrix}
-l_1\cos(q_1)\dot{q}_1 - l_2\cos(q_1+q_2)\dot{q}_1 &
-l_2\cos(q_1+q_2)\dot{q}_2 \\
-l_1\sin(q_1)\dot{q}_1 - l_2\sin(q_1+q_2)\dot{q}_1 &
-l_2\sin(q_1+q_2)\dot{q}_2
\end{bmatrix}
$$

- Obtener las velocidades cartesianas.
  
$$
\dot{\mathbf{x}} = \mathbf{J(q)} \dot{\mathbf{q}}
$$

- Obtener las aceleraciones cartesianas deseadas.

$$
\mathbf{M}\ddot{\tilde{\mathbf{x}}} + \mathbf{B}\dot{\tilde{\mathbf{x}}} + \mathbf{K}\tilde{\mathbf{x}} = \mathbf{f}_{ext}
$$

$$
\ddot{\mathbf{x}}_d = \mathbf{M}^{-1} \left( -\mathbf{B}\dot{\tilde{\mathbf{x}}} -\mathbf{K}\tilde{\mathbf{x}} +\mathbf{f}_{ext} \right)
$$

$$
\tilde{\mathbf{x}} = \mathbf{x} - \mathbf{x_d}
$$

$$
\dot{\tilde{\mathbf{x}}} = \dot{\mathbf{x}} - \dot{\mathbf{x}}_d
$$

- Obtener las aceleraciones deseadas articulares.

$$
\ddot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\ddot{\mathbf{q}} + \dot{\mathbf{J}}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}}
$$

$$
\ddot{\mathbf{q}} = \mathbf{J}(\mathbf{q})^{-1} \left[ \ddot{\mathbf{x}} - \dot{\mathbf{J}}(\mathbf{q},\dot{\mathbf{q}}) \dot{\mathbf{q}} \right]
$$

## Aplicación práctica
Para aplicar las ecuaciones vistas en el fundamento teórico de esta práctica se irán diseñando las siguientes funciones:

- Para la posición cartesiana, `forward_kinematics()`.

~~~
Eigen::VectorXd forward_kinematics()
    {
        // Placeholder for forward kinematics x = [l1 * cos(q1) + l2 * cos(q1 + q2), l1 * sin(q1) + l2 * sin(q1 + q2)]
        // Initialize q1, q2, q_dot1 and q_dot2
        double q1 = joint_positions_(0);
        double q2 = joint_positions_(1);

        Eigen::VectorXd x(2);
        x << l1_*cos(q1) + l2_*cos(q1 + q2), l1_*sin(q1) + l2_ * sin(q1 + q2);

        return x;
    }
~~~

- Para los Jacobianos, `update_jacobians()`.
~~~
void update_jacobians()
    {
        // Placeholder for jacobian and jacobian_derivative matrices
        // Initialize q1, q2, q_dot1 and q_dot2
        double q1 = joint_positions_(0);
        double q2 = joint_positions_(1);
        double q_dot1 = joint_velocities_(0);
        double q_dot2 = joint_velocities_(1);

        // Calculate J(q)
        jacobian_ <<-l1_ * sin(q1) - l2_ * sin(q1 + q2), -l2_ * sin(q1 + q2),
        l1_ * cos(q1) + l2_ * cos(q1 + q2), l2_ * cos(q1 + q2);

        // Calculate J'(q,q')
        jacobian_derivative_ << -l1_*cos(q1)*q_dot1 - l2_*cos(q1 + q2)*q_dot1, -l2_*cos(q1+q2)*q_dot2,
        -l1_*sin(q1)*q_dot1 - l2_*sin(q1 + q2)*q_dot1, -l2_*sin(q1+q2)*q_dot2;

        RCLCPP_INFO(this->get_logger(), "Jacobian:\n[%.3f, %.3f]\n[%.3f, %.3f]",
                jacobian_(0, 0), jacobian_(0, 1),
                jacobian_(1, 0), jacobian_(1, 1));

        double det = jacobian_.determinant();
        RCLCPP_INFO(this->get_logger(), "Jacobian determinant: %.6f", det);
        }
~~~

- Para las velocidades cartesianas, `differential_kinematics()`.
~~~
Eigen::MatrixXd differential_kinematics()
    {
        // Placeholder for first-order differential kinematics

        Eigen::VectorXd x_dot(2);
        x_dot = jacobian_ * joint_velocities_;

        return x_dot;
    }
~~~

- Para las aceleraciones cartesianas deseadas, `impedance_controller()`.
~~~
Eigen::VectorXd impedance_controller()
    {
        // Placeholder for impedance controller calculation
        Eigen::VectorXd x_dot_d = Eigen::VectorXd::Zero(2); // We assume desired cartesian velocity = 0


        // Calculate Cartesian errors
        Eigen::VectorXd x_error(2);
        cartesian_pose_ = forward_kinematics();
        x_error = cartesian_pose_ - equilibrium_pose_; 

        Eigen::VectorXd x_dot_error(2);
        x_dot_error = cartesian_velocities_;

        // Replace with actual impedance controller equation: x'' = M^(-1)[F_ext - k x_error - B x'_error]
        Eigen::VectorXd x_ddot(2);
        x_ddot = mass_matrix_.inverse() * (-damping_matrix_*x_dot_error - stiffness_matrix_ * x_error + external_wrenches_ );

            
            return x_ddot;
        }
~~~

- Para las aceleraciones deseadas articulares, 
~~~
Eigen::VectorXd calculate_desired_joint_accelerations()
    {
        // Placeholder for the second-order differential kinematics
        // q'' = J(q)^(-1)[x'' - J'(q,q')q']

        RCLCPP_INFO(this->get_logger(), "x_ddot: [%.3f, %.3f]",
            desired_cartesian_accelerations_(0), desired_cartesian_accelerations_(1));
            

        Eigen::VectorXd q_ddot;
        q_ddot = jacobian_.inverse() * (desired_cartesian_accelerations_ - jacobian_derivative_ * joint_velocities_); //cuidado con lo de xddot

        return q_ddot;
    }
~~~

Además de este controlador, se añadirá un nodo (`equilibrium_pose_publisher.py`) que nos permita cambiar la posición de equilibrio del manipulador.

## Resultados
Los resultados del laboratorio se obtienen al lanzar los siguientes comandos en diferentes terminales: 
`ros2 launch uma_arm_description uma_arm_visualization.launch.py`
`ros2 launch uma_arm_control impedance_controller_launch.py`
`ros2 launch uma_arm_control dynamics_cancellation_external_forces_launch.py`
`ros2 launch uma_arm_control uma_arm_dynamics_launch.py`
`python3 wrench_trackbar_publisher.py`

Mandando fuerzas externas sobre el eje y, se obtiene la figura 4.2.

<p align="center">
    <img src="/images/Respuesta_ejey.png">
    <br>
    <em>Figura 4.2: Respuesta del manipulador ante fuerzas externas sobre el eje y.</em>
</p>

Si se aumentasen los valores de las matrices utilizadas para esta práctica, utilizando el mismo experimento con el que se obtuvo la figura anterior, se alcanzan los siguientes resultados:

- Cambios sobre la matriz de masas (M). Presenta una respuesta más lenta ante fuerzas externas, ya que para la misma fuerza se generan aceleraciones menores. Este efecto se puede observar en la figura 4.3.

<p align="center">
    <img src="/images/Respuesta_cambioM.png">
    <br>
    <em>Figura 4.3: Respuesta del manipulador ante cambios en M.</em>
</p>

- Cambios sobre la matriz de coeficientes de fricción viscosa (B). Incrementa la disipación de energía del sistema, reduciendo oscilaciones y mejorando la estabilidad. Este efecto se puede observar en la figura 4.4.

<p align="center">
    <img src="/images/Respuesta_cambioB.png">
    <br>
    <em>Figura 4.4: Respuesta del manipulador ante cambios en B.</em>
</p>

- Cambios sobre la matriz de rigidez (K). El robot se resiste más al desplazamiento respecto a su podición de equilibrio. Este efecto se puede observar en la figura 4.5.

<p align="center">
    <img src="/images/Respuesta_cambioK.png">
    <br>
    <em>Figura 4.5: Respuesta del manipulador ante cambios en K.</em>
</p>

Se puede concluir que, si en el eje X la impedancia fuese grande, el movimiento  sería más rígido, preciso y resistente a perturbaciones en esa dirección. Por el contrario, si en el eje Y la impedancia fuese pequeña, el comportamiento en esa dirección sería más flexible y fácil de perturbar permitiendo mayores desplazamientos ante la aplicación de fuerzas externas.

No obstante, la aplicación de fuerzas en un eje no produce efectos solo en el, sino que también perturba al otro como se observa en la siguiente figura. Esto se debe a la transformación entre cartesiano y articular mediante a los jacobianos no preserva la independencia entre los ejes cartesianos, cada articulación contribuye simultáneamente a ambos ejes. Para evitar estas secuelas se podrían aumentar los coeficientes de fricción y de rígidez para intentar que el robot resista mejor a movimientos indeseados. Este efecto se puede observar en la figura 4.6.

<p align="center">
    <img src="/images/Fuerza_x_y.png">
    <br>
    <em>Figura 4.6: Respuesta del manipulador ante esfuerzos en X e Y.</em>
</p>

Por último, se observan los efectos de cambios en la posición de equilibrio. Para poder modificar esta posición se lanza la siguiente terminal `python3 equilibrium_pose_publisher.py`.

Ante cambios pequeños en la posición de equilibrio, el robot se aproximará a la nueva posición sin problema. Sin embargo, si cambiamos esta posición a regiones fuera de la zona de trabajo del robot este dejará de responder. Este efecto se puede observar en las figuras 4.7 y 4.8.

<p align="center">
    <img src="/images/Equilibrio.png">
    <br>
    <em>Figura 4.7: Respuesta del manipulador ante cambios en posición de equilibrio.</em>
</p>

<p align="center">
    <img src="/images/Equilibrio2.png">
    <br>
    <em>Figura 4.8: Respuesta del manipulador ante cambios en posición de equilibrio.</em>
</p>
