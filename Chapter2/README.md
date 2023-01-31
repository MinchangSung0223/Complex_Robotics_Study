# Chapter 2. Kinematic manipulability of general mechanical systems
## 2.1 Introduction
 - **General kinematic model** wich considers all degree of freedom and then imposes the constraints as **algebraic conditions**
 - **General static force balance model** which can be considered as a dual of **the differential kinematics**
 - **Manipulability Ellipsoid / Force Ellipsoid**
 - **Manipulability / Grasp Stability**
    + 과연 bracing on manipulability가  어떤 효과를 주는가?
    + 어디를 잡도록 kinematic optimization을 수행해야할까??
 - Terminology and Notation: 
    + Spatial Force( Spatial Wrench )  : $\mathcal{F}_s = (m,f) = (torque , force ) \in \mathbf{R}^{6 \times 1}$
    + Spatial Velocity( Spatial Twist )  : $\mathcal{V}_s = (w,v) = (\text{angular velocity} , \text{linear velocity} ) \in \mathbf{R}^{6 \times 1}$
    + $\tilde{G}$ : the annihilator of $G$, $G\tilde{G} = \tilde{G}G = 0$

## 2.2 Differential kinematics and static force model
### 2.2.1 Differential kinematics
이번 섹션에서는 kinematic constraints를 갖는 general mechanism에 대해서 다룬다. constraints가 없는 generalized coordinate를 $\theta$라고 하고, Active joints' angle(or actuated joints' angle)을 $\theta_a$ Passive joints' angle을 $\theta_p$라 한다.

$$\theta = \begin{bmatrix}\theta_a \\\theta_p\end{bmatrix}$$

$$\dot{\theta} = \begin{bmatrix}\dot{\theta}_a \\\dot{\theta}_p\end{bmatrix}$$ 

이제 joint velocity vector에 대해서 general constraint에 대해서

$$J_C(\theta)\dot{\theta} = 0 ~~~~~~~~~~~ (2.1) $$ 

task frame의 spatil velocity에 대해서는

$$v_T = J_T(\theta)\dot{\theta} ~~~~~~~~~~~ (2.2)$$

위와 같이 기술할 수 있다.
$J_C(\theta)$가 full rank라고 가정하면, $\dot{\theta} = \tilde{J}_C\zeta$ 이며, task velocity는 다음과 같이 작성된다.
$$ v_T = J_T \tilde{J}_C \zeta ~~~~~~~~~~~ (2.3)$$
만약 $J_T\tilde{J}_C\zeta$ 행렬이 rank가 떨어지면 **'mechanism이 singular이다.'**라고 한다. 다시말해서 어떤 task velocity $v_T$는 constraint에 의해서 어떤 방향으로는 움직일 수 없다.

  + Example: multiple fingers grasping a rigid payload
![image](https://user-images.githubusercontent.com/53217819/215644958-ad73c4b0-93a2-4f12-9f7e-652c04b861d3.png)



### 2.2.2 2.2.2 Force balance 

## 2.3 Velocity and force manipulability ellipsoids 
### 2.3.1 Serial manipulators 
### 2.3.2 Velocity ellipsoid
### 2.3.3 Force ellipsoid 
### 2.3.4 Configuration stability and manipulability

## 2.4 Illustrative example
### 2.4.1 Simple two-arm example 
### 2.4.2 Planar Stewart platform example 
### 2.4.3 Six-DOF Stewart platform example

## 2.5 Effects of arm posture and bracing on manipulability 
### 2.5.1 Effect of arm posture 
### 2.5.2 Effect of bracing
### 2.5.3 Effect of brace location 
### 2.5.4 Effect of brace contact type 

## 2.6 Comparison of manipulability ellipsoids 
## 2.7 Conclusions 
![ezgif com-gif-maker (4)](https://user-images.githubusercontent.com/53217819/215256910-d79964ee-8f47-403f-8d47-e8953f9ae88e.gif)
  
