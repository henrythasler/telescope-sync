This is rendered correctly:

$$\begin{aligned}
\mathbf{M} &= \mathbf{R} \times \mathbf{S}^{-1} \\
&= \begin{bmatrix}
         R_{1_{Az}} & R_{2_{Az}} & R_{3_{Az}}\\
         R_{1_{Alt}} & R_{2_{Alt}} & R_{3_{Alt}}\\
         1 & 1 & 1
     \end{bmatrix} \times
    \begin{bmatrix}
         S_{1_{Az}} & S_{2_{Az}} & S_{3_{Az}}\\
         S_{1_{Alt}} & S_{2_{Alt}} & S_{3_{Alt}}\\
         1 & 1 & 1
     \end{bmatrix}^{-1}
\end{aligned}$$

This does not render at all:

$$\begin{aligned}
\mathbf{A} &= \mathbf{M} \times \mathbf{P}_S\\
\begin{pmatrix}
         A_{Az}\\
         A_{Alt}\\ 
         A_z 
     \end{pmatrix} &=      
    \begin{bmatrix}
         M_{11} & M_{12} & M_{13}\\
         M_{21} & M_{22} & M_{23}\\ 
         M_{31} & M_{32} & M_{33} 
     \end{bmatrix} 
     \times 
    \begin{pmatrix}
         P_{Az}\\
         P_{Alt}\\ 
         1 
     \end{pmatrix}
\end{aligned}$$

Removing the *\mathbf* statement from the second equation is a workaround:

$$\begin{aligned}
A &= M \times P_S\\
\begin{pmatrix}
         A_{Az}\\
         A_{Alt}\\ 
         A_z 
     \end{pmatrix} &=      
    \begin{bmatrix}
         M_{11} & M_{12} & M_{13}\\
         M_{21} & M_{22} & M_{23}\\ 
         M_{31} & M_{32} & M_{33} 
     \end{bmatrix} 
     \times 
    \begin{pmatrix}
         P_{Az}\\
         P_{Alt}\\ 
         1 
     \end{pmatrix}
\end{aligned}$$
